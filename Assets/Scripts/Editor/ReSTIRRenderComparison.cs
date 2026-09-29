using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

// End-to-end scene -> Camera.Render -> production wavefront/ReSTIR -> linear HDR.
// Run in a separate Editor process; scene/material changes are never saved.
[InitializeOnLoad]
public static class ReSTIRRenderComparison
{
    const string ActiveKey = "ReSTIRRenderComparison.Active";
    const BindingFlags Fields = BindingFlags.Instance | BindingFlags.NonPublic;
    static readonly string[] Modes = { "pt", "di", "gi", "di_gi", "restart" };
    static Tracing tracing;
    static Camera camera;
    static RenderTexture output;
    static Report report;
    static int mode, replicate, frames, size, repeats, warmup, batch;
    static string directory;
    static double started, warmupSeconds;
    static string error;
    static readonly List<string> editorErrors = new List<string>();

    [Serializable] sealed class Capture
    {
        public string mode, file;
        public int seed, samples;
        public double meanLuminance, maxLuminance, wallSeconds, warmupSeconds;
        public int nonfinitePixels;
        public long gpuBufferBytes;
    }
    [Serializable] sealed class Report
    {
        public string scene, unity, device, command, sourceRevision, diagnosticsDirectory;
        public int width, height, frames, repeats, warmupFrames, batchFrames;
        public List<Capture> captures = new List<Capture>();
        public string failure;
        public List<string> editorErrors;
        public float analyticExpected = -1;
        public string[] materialSnapshot;
    }

    static ReSTIRRenderComparison()
    {
        EditorApplication.update += Tick;
        Application.logMessageReceived += (message, stack, type) =>
        {
            if (SessionState.GetBool(ActiveKey, false) &&
                (type == LogType.Error || type == LogType.Exception || type == LogType.Assert))
            {
                string detail = message + "\n" + stack;
                if (stack.Contains("Tracing") || stack.Contains("BVHBuilder") ||
                    message.Contains("Compute shader") || message.Contains("Shader error"))
                    error = detail;
                else editorErrors.Add(detail);
            }
        };
    }

    public static void Run()
    {
        string scene = Arg("-renderScene", "Assets/Scenes/CornellBox.unity");
        if (scene == "analytic" || scene == "mixed" || scene == "alpha" || scene == "glass") BuildAnalyticScene(scene);
        else EditorSceneManager.OpenScene(scene);
        var target = Object.FindAnyObjectByType<Tracing>();
        if (target == null) throw new InvalidOperationException("Scene has no Tracing camera.");
        Set(target, "WriteReSTIRGIDiagnostics", Arg("-renderDiagnostics", "false") == "true");
        Set(target, "FrameLimit", 0);
        SessionState.SetBool(ActiveKey, true);
        EditorApplication.isPlaying = true;
    }

    static void Tick()
    {
        if (!SessionState.GetBool(ActiveKey, false) || !EditorApplication.isPlaying) return;
        try
        {
            if (error != null) throw new InvalidOperationException(error);
            if (tracing == null)
            {
                tracing = Object.FindAnyObjectByType<Tracing>();
                if (tracing == null || !(bool)Get(tracing, "_runtimeStarted"))
                {
                    tracing = null;
                    return;
                }
                Configure();
            }
            if (SampleCount() == 0) started = EditorApplication.timeSinceStartup;
            for (int i = 0; i < batch && SampleCount() < frames; i++) camera.Render();
            if (SampleCount() != frames)
            {
                if (EditorApplication.timeSinceStartup - started > 300)
                    throw new TimeoutException("Renderer did not reach the requested sample count.");
                return;
            }
            SaveCapture();
            if (mode == 0 && replicate == 0 && Arg("-renderCheckAccumulation", "false") == "true")
                CheckAccumulationSwitch();
            if (++replicate == repeats) { replicate = 0; mode++; }
            if (mode == Modes.Length) { Finish(null); return; }
            BeginCase();
        }
        catch (Exception ex) { Finish(ex.ToString()); }
    }

    static void BuildAnalyticScene(string kind)
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var parent = new GameObject("Analytic geometry");
        parent.AddComponent<SceneObject>();
        var diffuse = new Material(Shader.Find("Standard"));
        diffuse.color = Color.white * 0.5f;
        diffuse.SetFloat("_Glossiness", 0);
        var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.transform.SetParent(parent.transform);
        floor.transform.position = new Vector3(0, -0.05f, 0);
        floor.transform.localScale = new Vector3(100, 0.1f, 100);
        floor.GetComponent<Renderer>().sharedMaterial = diffuse;
        if (kind == "glass")
        {
            diffuse.color = Color.white;
            diffuse.SetFloat("_Glossiness", 1);
            diffuse.SetFloat("_Mode", 3);
        }
        if (kind == "mixed" || kind == "alpha")
        {
            var emitter = new Material(Shader.Find("Standard"));
            emitter.color = Color.black;
            emitter.SetFloat("_Metallic", 1);
            emitter.SetFloat("_Glossiness", 0.7f);
            emitter.SetColor("_EmissionColor", Color.white);
            emitter.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
            MaterialEditor.FixupEmissiveFlag(emitter);
            emitter.EnableKeyword("_EMISSION");
            var roof = GameObject.CreatePrimitive(PrimitiveType.Cube);
            roof.transform.SetParent(parent.transform);
            roof.transform.position = new Vector3(50, 1, 0);
            roof.transform.localScale = new Vector3(100, 0.1f, 100);
            roof.GetComponent<Renderer>().sharedMaterial = emitter;
            roof.AddComponent<Emission>().Intensity = 1;
            if (kind == "alpha")
            {
                // One thin sheet, alpha=1/2, blocks half the overhead sun.
                Object.DestroyImmediate(roof.GetComponent<Collider>());
                roof.GetComponent<MeshFilter>().sharedMesh = new Mesh {
                    vertices = new[] { new Vector3(-1,0,-1), new Vector3(1,0,-1), new Vector3(1,0,1), new Vector3(-1,0,1) },
                    triangles = new[] { 0, 2, 1, 0, 3, 2 },
                    normals = new[] { Vector3.up, Vector3.up, Vector3.up, Vector3.up }
                };
                roof.transform.position = new Vector3(0, 1, 0);
                emitter.color = new Color(0, 0, 0, 0.5f);
                emitter.SetFloat("_Mode", 2);
                emitter.SetColor("_EmissionColor", Color.black);
                emitter.globalIlluminationFlags = MaterialGlobalIlluminationFlags.EmissiveIsBlack;
                emitter.DisableKeyword("_EMISSION");
            }
        }
        var go = new GameObject("Analytic camera");
        go.tag = "MainCamera";
        var cam = go.AddComponent<Camera>();
        cam.transform.position = new Vector3(0, 0.5f, 0);
        cam.transform.rotation = Quaternion.Euler(90, 0, 0);
        var renderer = go.AddComponent<Tracing>();
        renderer.tracingShader = AssetDatabase.LoadAssetAtPath<ComputeShader>("Assets/ComputeShader/main/Tracing.compute");
        var sky = new Texture2D(2, 2, TextureFormat.RGBAFloat, false, true);
        sky.SetPixels(new[] { Color.white, Color.white, Color.white, Color.white }); sky.Apply();
        Set(renderer, "skyboxTexture", sky);
        Set(renderer, "TraceDepth", kind == "glass" ? 8 : 2);
        var sun = new GameObject("Disabled sun").AddComponent<Light>();
        sun.type = LightType.Directional; sun.intensity = kind == "alpha" ? 1 : 0;
        sun.transform.rotation = Quaternion.Euler(90, 0, 0);
        if (kind == "alpha") { Set(renderer, "SkyboxIntensity", 0f); Set(renderer, "SunAngularRadius", 0f); }
        var lights = go.GetComponent<LightManager>();
        lights.DirectionalLight = sun;
        var settings = new SerializedObject(lights);
        settings.FindProperty("PointLights").arraySize = 0;
        settings.ApplyModifiedPropertiesWithoutUndo();
    }

    static void Configure()
    {
        frames = int.Parse(Arg("-renderFrames", "256"));
        repeats = int.Parse(Arg("-renderRepeats", "4"));
        size = int.Parse(Arg("-renderSize", "128"));
        warmup = int.Parse(Arg("-renderWarmup", "0"));
        batch = int.Parse(Arg("-renderBatch", "8"));
        if (frames < 1 || repeats < 2 || size < 4 || warmup < 0 || batch < 1)
            throw new ArgumentOutOfRangeException("Invalid render comparison dimensions or counts.");
        directory = Path.GetFullPath(Arg("-renderOutput", "Tools/Output/render-comparison"));
        Directory.CreateDirectory(directory);
        foreach (var move in Object.FindObjectsByType<CameraMove>()) move.enabled = false;
        foreach (var c in Object.FindObjectsByType<Camera>()) c.enabled = false;
        camera = tracing.GetComponent<Camera>();
        output = new RenderTexture(size, size, 24, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
        output.Create();
        camera.targetTexture = output;
        camera.aspect = 1;
        camera.transform.hasChanged = false;
        Set(tracing, "FrameLimit", frames);
        Set(tracing, "ToneMap", false);
        Set(tracing, "AccumulateFrames", true);
        Set(tracing, "OnlyDrawAlbedo", false);
        Set(tracing, "OnlyDrawNormals", false);
        Set(tracing, "OnlyDrawDepth", false);
        Set(tracing, "targetFrameRate", 240);
        report = new Report {
            scene = Arg("-renderScene", "Assets/Scenes/CornellBox.unity"),
            unity = Application.unityVersion, device = SystemInfo.graphicsDeviceName,
            command = string.Join(" ", Environment.GetCommandLineArgs()),
            sourceRevision = Arg("-renderRevision", "working-tree"),
            width = size, height = size, frames = frames, repeats = repeats,
            warmupFrames = warmup, batchFrames = batch,
            editorErrors = editorErrors
        };
        if (report.scene == "analytic" || report.scene == "mixed") report.analyticExpected = 0.5f;
        if (report.scene == "alpha") report.analyticExpected = 0.25f / Mathf.PI;
        if (report.scene == "glass") report.analyticExpected = 1f;
        BeginCase();
    }

    static void BeginCase()
    {
        Set(tracing, "UseReSTIRDI", mode == 1 || mode == 3);
        Set(tracing, "UseReSTIRGI", mode == 2 || mode == 3);
        if (mode == 4 && replicate == 0)
        {
            tracing.enabled = false;
            tracing.enabled = true;
        }
        tracing.SendMessage("Update");
        double warmupStarted = EditorApplication.timeSinceStartup;
        if (warmup > 0)
        {
            Set(tracing, "FrameLimit", 0);
            for (int i = 0; i < warmup; i++) camera.Render();
            // Drain GPU work before timing; readback is confined to this E2E tool.
            var sync = new Texture2D(1, 1, TextureFormat.RGBAFloat, false, true);
            var previous = RenderTexture.active;
            RenderTexture.active = (RenderTexture)Get(tracing, "frameConverged");
            sync.ReadPixels(new Rect(0, 0, 1, 1), 0, 0);
            RenderTexture.active = previous;
            Object.DestroyImmediate(sync);
            Set(tracing, "FrameLimit", frames);
        }
        warmupSeconds = EditorApplication.timeSinceStartup - warmupStarted;
        typeof(Tracing).GetMethod("ResetSampleCount", Fields).Invoke(tracing, new object[] { "render_comparison" });
        uint seed = (uint)(replicate * 100003);
        Set(tracing, "frameId", seed);
        UnityEngine.Random.InitState((int)seed);
        started = EditorApplication.timeSinceStartup;
        Debug.Log($"[RenderComparison] {Modes[mode]} replicate={replicate} frames={frames}");
    }

    static void SaveCapture()
    {
        if (report.diagnosticsDirectory == null)
            report.diagnosticsDirectory = ((ReSTIRDiagnosticsSession)Get(tracing, "_restirDiagnostics"))?.OutputDirectory;
        if (report.materialSnapshot == null)
        {
            var materials = new List<string>();
            foreach (var m in BVHBuilder.GetMaterials())
                materials.Add($"color={m.Color} emission={m.Emission} intensity={m.EmissionIntensity} metal={m.Metallic} smooth={m.Smoothness} mode={m.RenderMode} emitTexture={m.EmitIdx}");
            report.materialSnapshot = materials.ToArray();
        }
        var hdr = (RenderTexture)Get(tracing, "frameConverged");
        var image = new Texture2D(size, size, TextureFormat.RGBAFloat, false, true);
        var previous = RenderTexture.active;
        RenderTexture.active = hdr;
        image.ReadPixels(new Rect(0, 0, size, size), 0, 0);
        double completedSeconds = EditorApplication.timeSinceStartup - started;
        image.Apply();
        RenderTexture.active = previous;
        string name = $"{Modes[mode]}_{replicate}";
        File.WriteAllBytes(Path.Combine(directory, name + ".exr"), image.EncodeToEXR(Texture2D.EXRFlags.OutputAsFloat));
        var pixels = image.GetPixels();
        var capture = new Capture { mode = Modes[mode], seed = replicate * 100003,
            samples = SampleCount(), file = name + ".rgba32f", wallSeconds = completedSeconds,
            warmupSeconds = warmupSeconds };
        using (var writer = new BinaryWriter(File.Create(Path.Combine(directory, capture.file))))
        {
            foreach (var p in pixels)
            {
                writer.Write(p.r); writer.Write(p.g); writer.Write(p.b); writer.Write(p.a);
                double lum = p.r * 0.2126 + p.g * 0.7152 + p.b * 0.0722;
                if (double.IsNaN(lum) || double.IsInfinity(lum)) capture.nonfinitePixels++;
                else { capture.meanLuminance += lum / pixels.Length; capture.maxLuminance = Math.Max(capture.maxLuminance, lum); }
            }
        }
        foreach (var field in typeof(Tracing).GetFields(Fields))
            if (field.GetValue(tracing) is ComputeBuffer buffer)
                capture.gpuBufferBytes += (long)buffer.count * buffer.stride;
        // Preview uses one fixed display curve; numerical comparisons use linear HDR.
        for (int i = 0; i < pixels.Length; i++)
        {
            Color p = pixels[i];
            pixels[i] = new Color(p.r / (1 + p.r), p.g / (1 + p.g), p.b / (1 + p.b), 1).gamma;
        }
        var preview = new Texture2D(size, size, TextureFormat.RGBA32, false);
        preview.SetPixels(pixels); preview.Apply();
        File.WriteAllBytes(Path.Combine(directory, name + ".png"), preview.EncodeToPNG());
        Object.DestroyImmediate(image); Object.DestroyImmediate(preview);
        report.captures.Add(capture);
        File.WriteAllText(Path.Combine(directory, "manifest.json"), JsonUtility.ToJson(report, true));
        if (capture.nonfinitePixels > 0) throw new InvalidDataException("Nonfinite HDR output.");
    }

    static void Finish(string failure)
    {
        SessionState.EraseBool(ActiveKey);
        if (report != null)
        {
            report.failure = failure;
            File.WriteAllText(Path.Combine(directory, "manifest.json"), JsonUtility.ToJson(report, true));
        }
        if (failure != null) Debug.LogError(failure);
        EditorApplication.Exit(failure == null ? 0 : 1);
    }

    // Output contract: off shows this frame; on shows the arithmetic mean since reset.
    // Also catches a missing Inspector property, stale averages after toggling, and FrameLimit bypass.
    static void CheckAccumulationSwitch()
    {
        var settings = new SerializedObject(tracing);
        var accumulation = settings.FindProperty("AccumulateFrames");
        if (accumulation == null) throw new InvalidOperationException("Missing accumulation Inspector switch.");
        Set(tracing, "UseReSTIRDI", true);
        Set(tracing, "UseReSTIRGI", true);
        Set(tracing, "FrameLimit", 4);
        double worstError = 0;
        double worstDisplayError = 0;
        for (int pass = 0; pass < 3; pass++)
        {
            bool enabled = pass != 1;
            settings.Update();
            accumulation.boolValue = enabled;
            settings.ApplyModifiedPropertiesWithoutUndo();
            tracing.SendMessage("Update");
            if (pass == 0)
                typeof(Tracing).GetMethod("ResetSampleCount", Fields).Invoke(tracing, new object[] { "output_comparison" });
            UnityEngine.Random.InitState(0);
            var sum = new Color[size * size];
            Color[] displayed = null;
            for (int frame = 1; frame <= 4; frame++)
            {
                camera.Render();
                var raw = ReadOutput((RenderTexture)Get(tracing, "target"));
                var accumulated = enabled ? ReadOutput((RenderTexture)Get(tracing, "frameConverged")) : raw;
                displayed = ReadOutput(output);
                for (int i = 0; i < raw.Length; i++)
                {
                    sum[i] += raw[i];
                    Color expected = enabled ? sum[i] / frame : raw[i];
                    for (int channel = 0; channel < 3; channel++)
                    {
                        double delta = Math.Abs(accumulated[i][channel] - expected[channel]);
                        if (double.IsNaN(delta) || delta > 1e-5 * Math.Max(1, Math.Abs(expected[channel])))
                            throw new InvalidDataException($"Accumulation={enabled}, frame={frame}, pixel={i}: {delta}");
                        worstError = Math.Max(worstError, delta);
                        // Camera.Render includes a half-precision presentation copy on D3D11.
                        // The linear HDR average above is checked separately at float precision.
                        double displayDelta = Math.Abs(displayed[i][channel] - expected[channel]);
                        if (double.IsNaN(displayDelta) || displayDelta > Math.Max(1.0 / 16777216, Math.Abs(expected[channel]) / 1024))
                            throw new InvalidDataException($"Display output mismatch: {displayDelta}");
                        worstDisplayError = Math.Max(worstDisplayError, displayDelta);
                    }
                }
            }
            camera.Render();
            var frozen = ReadOutput(output);
            if (SampleCount() != 4) throw new InvalidDataException("Frame limit ignored.");
            for (int i = 0; i < frozen.Length; i++)
                if (frozen[i] != displayed[i]) throw new InvalidDataException("Frozen output changed.");
            var image = new Texture2D(size, size, TextureFormat.RGBAFloat, false, true);
            image.SetPixels(displayed); image.Apply();
            File.WriteAllBytes(Path.Combine(directory, $"accumulation_{pass}_{enabled}.exr"),
                image.EncodeToEXR(Texture2D.EXRFlags.OutputAsFloat));
            Object.DestroyImmediate(image);
        }
        Set(tracing, "FrameLimit", frames);
        File.WriteAllText(Path.Combine(directory, "accumulation-check.json"),
            "{\"passed\":true,\"sequence\":\"on-off-on\",\"framesPerCase\":4,\"maxAbsoluteError\":" +
            worstError.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ",\"maxDisplayError\":" +
            worstDisplayError.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "}");
    }

    static Color[] ReadOutput(RenderTexture texture)
    {
        var image = new Texture2D(texture.width, texture.height, TextureFormat.RGBAFloat, false, true);
        var previous = RenderTexture.active;
        RenderTexture.active = texture;
        image.ReadPixels(new Rect(0, 0, texture.width, texture.height), 0, 0); image.Apply();
        RenderTexture.active = previous;
        var pixels = image.GetPixels();
        Object.DestroyImmediate(image);
        return pixels;
    }

    static int SampleCount() => (int)Get(tracing, "sampleCount");
    static object Get(Tracing t, string field) => typeof(Tracing).GetField(field, Fields).GetValue(t);
    static void Set(Tracing t, string field, object value) => typeof(Tracing).GetField(field, Fields).SetValue(t, value);
    static string Arg(string key, string fallback)
    {
        var args = Environment.GetCommandLineArgs();
        int index = Array.IndexOf(args, key);
        return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
    }
}
