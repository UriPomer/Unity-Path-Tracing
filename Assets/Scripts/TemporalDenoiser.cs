using System;
using UnityEngine;

// SVGF reconstruction owns image history; it never feeds filtered radiance to
// the path tracer or ReSTIR reservoirs. All inputs and histories are linear HDR.
public sealed class TemporalDenoiser : IDisposable
{
    readonly ComputeShader _shader;
    readonly int _prepareGeometry, _reproject, _temporal, _variance, _atrous, _combine;
    // Each signal has its own ping-pong history and moments; geometry and
    // scratch images are shared. Direct light is never filtered with GI variance.
    readonly RenderTexture[] _history = new RenderTexture[6];
    readonly RenderTexture[] _moments = new RenderTexture[6];
    readonly RenderTexture[] _filter = new RenderTexture[2];
    readonly RenderTexture[] _motion = new RenderTexture[2];
    RenderTexture _direct, _directFiltered, _diffuse, _pathDiffuse, _indirectFiltered;
    ComputeBuffer _previousTransforms, _geometry;
    int _index, _width, _height;
    int _transformRevision = -1;
    bool _valid;
    Matrix4x4 _previousVP, _previousView, _projection;
    Vector3 _cameraPosition;
    Quaternion _cameraRotation;

    public RenderTexture Output { get; private set; }
    // RGB: direct diffuse estimate; A: paired-observation uncertainty after demodulation.
    public RenderTexture DirectInput => _direct;
    public RenderTexture DiffuseInput => _diffuse;
    // RGB: BSDF diffuse fraction; A: PT observation, then its variance after shadow.
    public RenderTexture PathDiffuseWeights => _pathDiffuse;
    public RenderTexture MotionVectors => _motion[1 - _index];
    public int GeometryRevision { get; private set; } = -1;

    public TemporalDenoiser()
    {
        _shader = UnityEngine.Object.Instantiate(Resources.Load<ComputeShader>("TemporalDenoise"));
        _prepareGeometry = _shader.FindKernel("PrepareGeometry");
        _reproject = _shader.FindKernel("Reproject");
        _temporal = _shader.FindKernel("Temporal");
        _variance = _shader.FindKernel("Variance");
        _atrous = _shader.FindKernel("Atrous");
        _combine = _shader.FindKernel("Combine");
    }

    public void Reset() => _valid = false;

    public void Prepare(Camera camera, int width, int height, ComputeShader tracer, int traceKernel)
    {
        if (width != _width || height != _height)
        {
            ReleaseTextures(); _width = width; _height = height;
            _geometry?.Release(); _geometry = new ComputeBuffer(width*height,48);
            _shader.SetBuffer(_prepareGeometry, "_Geometry", _geometry);
            _shader.SetBuffer(_temporal, "_Geometry", _geometry);
            _shader.SetBuffer(_variance, "_Geometry", _geometry);
            _shader.SetBuffer(_atrous, "_Geometry", _geometry);
            for (int i = 0; i < 2; i++)
            {
                _filter[i] = Create("SVGF filter"); _motion[i] = Create("Motion vectors");
            }
            for (int i = 0; i < 6; i++)
            {
                string signal = i < 2 ? "Direct diffuse" : i < 4 ? "Indirect diffuse" : "Specular";
                _history[i] = Create(signal + " history");
                _moments[i] = Create(signal + " moments");
            }
            _direct = Create("Raw primary direct light");
            _directFiltered = Create("Reconstructed direct light");
            _diffuse = Create("Raw primary diffuse light");
            _pathDiffuse = Create("Primary BSDF diffuse fractions");
            _indirectFiltered = Create("Reconstructed indirect diffuse light");
            _valid = false; _index = 0;
        }
        var transforms = BVHBuilder.GetTransforms();
        int count = Mathf.Max(transforms.Count, 1);
        if (_previousTransforms == null || _previousTransforms.count != count)
        {
            _previousTransforms?.Release();
            _previousTransforms = new ComputeBuffer(count, 64);
            _transformRevision = -1;
            _valid = false;
        }
        bool cut = _projection != camera.projectionMatrix ||
            Quaternion.Angle(_cameraRotation, camera.transform.rotation) > 45f ||
            Vector3.Distance(_cameraPosition, camera.transform.position) > 10f;
        if (cut || GeometryRevision != BVHBuilder.GeometryRevision) _valid = false;
        if (!_valid)
        {
            _previousVP = camera.projectionMatrix * camera.worldToCameraMatrix;
            _previousView = camera.worldToCameraMatrix;
            CaptureTransforms();
        }
        GeometryRevision = BVHBuilder.GeometryRevision;
        tracer.SetTexture(traceKernel, "_DenoiseMotion", _motion[_index]);
        tracer.SetBuffer(traceKernel, "_DenoisePreviousTransforms", _previousTransforms);
        tracer.SetMatrix("_DenoisePreviousVP", _previousVP);
        tracer.SetMatrix("_DenoiseCurrentVP", camera.projectionMatrix * camera.worldToCameraMatrix);
        tracer.SetMatrix("_DenoisePreviousView", _previousView);
    }

    public RenderTexture Reconstruct(Camera camera, RenderTexture noisy, ComputeBuffer currentSurface, ComputeBuffer previousSurface)
    {
        _shader.SetInts("_Size", _width, _height);
        _shader.SetInt("_HistoryValid", _valid ? 1 : 0);
        _shader.SetFloat("_PixelWorldScale", 2f * Mathf.Tan(camera.fieldOfView * Mathf.Deg2Rad * 0.5f) / _height);
        _shader.SetMatrix("_View", camera.worldToCameraMatrix);
        _shader.SetVector("_CameraPosition", camera.transform.position);
        _shader.SetVector("_PreviousCameraPosition", _cameraPosition);
        _shader.SetBuffer(_reproject, "_CurrentSurface", currentSurface);
        _shader.SetBuffer(_reproject, "_PreviousSurface", previousSurface);
        _shader.SetBuffer(_reproject, "_CurrentTransforms", BVHBuilder.TransformBuffer);
        _shader.SetBuffer(_reproject, "_PreviousTransforms", _previousTransforms);
        _shader.SetBuffer(_prepareGeometry, "_CurrentSurface", currentSurface);
        _shader.SetTexture(_prepareGeometry, "_Motion", _motion[_index]);
        Dispatch(_prepareGeometry);
        _shader.SetBuffer(_temporal, "_CurrentSurface", currentSurface);
        _shader.SetTexture(_temporal, "_Motion", _motion[_index]);
        _shader.SetTexture(_temporal, "_Noisy", noisy);
        _shader.SetTexture(_temporal, "_Direct", _direct);
        _shader.SetTexture(_temporal, "_Diffuse", _diffuse);
        _shader.SetTexture(_reproject, "_Motion", _motion[_index]);
        _shader.SetTexture(_reproject, "_PreviousMotion", _motion[1 - _index]);
        _shader.SetBuffer(_variance, "_CurrentSurface", currentSurface);
        _shader.SetTexture(_variance, "_Motion", _motion[_index]);
        _shader.SetBuffer(_atrous, "_CurrentSurface", currentSurface);
        _shader.SetTexture(_atrous, "_Motion", _motion[_index]);
        // Keep direct-light reconstruction at the finest wavelet scale. A
        // second, dilated pass widens physical penumbrae even with stable history.
        ReconstructSignal(0, 1, _directFiltered);
        ReconstructSignal(1, 5, _indirectFiltered);
        RenderTexture specular = ReconstructSignal(2, 3, null);
        _shader.SetBuffer(_combine, "_CurrentSurface", currentSurface);
        _shader.SetTexture(_combine, "_Noisy", noisy);
        _shader.SetTexture(_combine, "_DirectFiltered", _directFiltered);
        _shader.SetTexture(_combine, "_IndirectFiltered", _indirectFiltered);
        _shader.SetTexture(_combine, "_Input", specular);
        _shader.SetTexture(_combine, "_Out", _filter[0]);
        Dispatch(_combine);
        Output = _filter[0];
        _previousVP = camera.projectionMatrix * camera.worldToCameraMatrix;
        _previousView = camera.worldToCameraMatrix;
        _projection = camera.projectionMatrix;
        _cameraPosition = camera.transform.position; _cameraRotation = camera.transform.rotation;
        CaptureTransforms();
        _index = 1 - _index; _valid = true;
        return Output;
    }

    RenderTexture ReconstructSignal(int signal, int passes, RenderTexture finalOutput)
    {
        int current = signal * 2 + _index, previous = signal * 2 + 1 - _index;
        _shader.SetInt("_Signal", signal);
        _shader.SetTexture(_reproject, "_History", _history[previous]);
        _shader.SetTexture(_reproject, "_PreviousMoments", _moments[previous]);
        _shader.SetTexture(_reproject, "_Out", _filter[0]);
        _shader.SetTexture(_reproject, "_MomentsOut", _filter[1]);
        Dispatch(_reproject);
        _shader.SetTexture(_temporal, "_Reprojected", _filter[0]);
        _shader.SetTexture(_temporal, "_ReprojectedMoments", _filter[1]);
        _shader.SetTexture(_temporal, "_MomentsOut", _moments[current]);
        RenderTexture temporal = _history[current];
        _shader.SetTexture(_temporal, "_Out", temporal);
        Dispatch(_temporal);
        RenderTexture input = temporal;
        if (signal != 0)
        {
            _shader.SetTexture(_variance, "_Input", temporal);
            _shader.SetTexture(_variance, "_Moments", _moments[current]);
            _shader.SetTexture(_variance, "_Out", _filter[1]);
            Dispatch(_variance); input = _filter[1];
        }
        for (int pass = 0; pass < passes; pass++)
        {
            // GI feeds back the first wavelet level. Direct light retains the
            // temporal estimate before spatial filtering: repeated filtering
            // of slowly averaged shadow history would keep widening penumbrae.
            RenderTexture output = pass == passes - 1 && finalOutput != null ? finalOutput :
                pass == 0 ? (signal == 0 ? _filter[0] : _history[current]) : _filter[(pass - 1) & 1];
            _shader.SetInt("_Step", 1 << pass);
            _shader.SetInt("_Final", pass == passes - 1 ? 1 : 0);
            _shader.SetTexture(_atrous, "_Input", input);
            _shader.SetTexture(_atrous, "_Out", output);
            Dispatch(_atrous); input = output;
        }
        return input;
    }

    void Dispatch(int kernel) => _shader.Dispatch(kernel, (_width + 7) / 8, (_height + 7) / 8, 1);

    void CaptureTransforms()
    {
        if (_transformRevision == BVHBuilder.TransformRevision) return;
        var transforms = BVHBuilder.GetTransforms();
        if (transforms.Count > 0) _previousTransforms.SetData(transforms);
        _transformRevision = BVHBuilder.TransformRevision;
    }

    RenderTexture Create(string name)
    {
        var texture = new RenderTexture(_width, _height, 0, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear) {
            name = name, enableRandomWrite = true, filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp
        };
        texture.Create(); return texture;
    }

    void ReleaseTextures()
    {
        foreach (var array in new[] { _history, _moments, _filter, _motion })
            for (int i = 0; i < array.Length; i++)
                Release(ref array[i]);
        Release(ref _direct); Release(ref _directFiltered);
        Release(ref _diffuse); Release(ref _pathDiffuse); Release(ref _indirectFiltered);
        Output = null;
    }

    static void Release(ref RenderTexture texture)
    {
        if (texture == null) return;
        if (RenderTexture.active == texture) RenderTexture.active = null;
        texture.Release(); UnityEngine.Object.Destroy(texture); texture = null;
    }

    public void Dispose()
    {
        ReleaseTextures(); _geometry?.Release(); _geometry = null;
        _previousTransforms?.Release(); _previousTransforms = null;
        UnityEngine.Object.Destroy(_shader);
    }
}
