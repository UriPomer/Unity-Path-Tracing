using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;
using Debug = UnityEngine.Debug;

public static class ReSTIRTelemetryLayout
{
    public const uint Magic = 0x52535452u;
    public const uint SchemaVersion = 1u;
    public const int PacketWordCount = 1024;

    public const int HeaderMagic = 0;
    public const int HeaderSchema = 1;
    public const int HeaderFrame = 2;
    public const int HeaderSample = 3;
    public const int HeaderGeneration = 4;
    public const int HeaderWidth = 5;
    public const int HeaderHeight = 6;
    public const int HeaderModeFlags = 7;
    public const int HeaderDirectInitialSlot = 8;
    public const int HeaderDirectFinalSlot = 9;
    public const int HeaderIndirectInitialSlot = 10;
    public const int HeaderIndirectFinalSlot = 11;
    public const int HeaderSelectedPixel = 12;
    public const int HeaderWordCount = 32;

    public const int CounterBase = HeaderWordCount;
    public const int CounterCount = 96;
    public const int RecordBase = CounterBase + CounterCount;
    public const int RecordWordCount = 56;
    public const int RecordCount = 16;
    public const int RecordPayloadBase = 4;
    public const int RecordPayloadWordCount = 24;
    public const int RecordGeneration = 28;
    public const int RecordSample = 29;

    public static bool ShouldCaptureSample(int sample, int steadyInterval)
    {
        if (sample == 1 || sample == 2 || sample == 4 || sample == 8 || sample == 16)
            return true;

        return steadyInterval > 0 && sample > 16 && sample % steadyInterval == 0;
    }
}

public enum ReSTIRTelemetryCounter
{
    SampledPixels = 0,
    DIInitialAccepted = 1,
    DIInitialInvalidSurface = 2,
    DIInitialInvalidProposal = 3,
    DITemporalInvalidCurrent = 4,
    DITemporalNoHistory = 5,
    DITemporalReprojectionOutOfBounds = 6,
    DITemporalInvalidHistory = 7,
    DITemporalIncompatibleSurface = 8,
    DITemporalReevaluationRejected = 9,
    DITemporalHistoryCombined = 10,
    DITemporalHistorySelected = 11,
    DITemporalNonFiniteOutput = 12,
    DIShadeInvalidReservoir = 13,
    DIShadeVisibilityRejected = 14,
    DIShadePositiveContribution = 15,

    GIInitialPrimaryMiss = 16,
    GIInitialInvalidSecondary = 17,
    GIInitialInvalidProposal = 18,
    GIInitialZeroThroughput = 19,
    GIInitialZeroTarget = 20,
    GIInitialNonFinite = 21,
    GIInitialAccepted = 22,

    GITemporalInvalidCurrent = 32,
    GITemporalNoHistory = 33,
    GITemporalReprojectionOutOfBounds = 34,
    GITemporalInvalidHistory = 35,
    GITemporalIncompatibleSurface = 36,
    GITemporalReevaluationRejected = 37,
    GITemporalJacobianRejected = 38,
    GITemporalHistoryCombined = 39,
    GITemporalHistorySelected = 40,
    GITemporalMCapped = 41,
    GITemporalNonFiniteOutput = 42,

    GISpatialInvalidCurrent = 48,
    GISpatialInvalidNeighbor = 49,
    GISpatialIncompatibleNeighbor = 50,
    GISpatialReevaluationRejected = 51,
    GISpatialJacobianRejected = 52,
    GISpatialNeighborCombined = 53,
    GISpatialNeighborSelected = 54,
    GISpatialZeroNormalization = 55,
    GISpatialMCapped = 56,
    GISpatialNonFiniteOutput = 57,

    GIFinalInvalidPrimary = 64,
    GIFinalInvalidReservoir = 65,
    GIFinalVisibilityRejected = 66,
    GIFinalZeroRadiance = 67,
    GIFinalZeroTarget = 68,
    GIFinalNonFiniteWeight = 69,
    GIFinalNonFiniteContribution = 70,
    GIFinalExcessiveWeight = 71,
    GIFinalExcessiveContribution = 72,
    GIFinalPositiveContribution = 73,

    CriticalNonFinite = 80,
    CriticalOutOfRange = 81,
    CriticalBufferContract = 82
}

public enum ReSTIRTelemetryStage
{
    None = 0,
    DIInitial = 1,
    DITemporal = 2,
    DIShade = 3,
    GIInitial = 4,
    GITemporal = 5,
    GISpatial = 6,
    GIFinal = 7
}

public enum ReSTIRTelemetryReason
{
    None = 0,
    PrimaryMiss = 1,
    InvalidSurface = 2,
    InvalidProposalPdf = 3,
    ZeroThroughput = 4,
    ZeroTarget = 5,
    ReprojectionOutOfBounds = 6,
    InvalidHistory = 7,
    IncompatibleSurface = 8,
    ReevaluationRejected = 9,
    JacobianRejected = 10,
    ZeroNormalization = 11,
    VisibilityRejected = 12,
    NonFiniteReservoir = 13,
    NonFiniteWeight = 14,
    NonFiniteContribution = 15,
    ExcessiveWeight = 16,
    ExcessiveContribution = 17
}

public readonly struct ReSTIRTelemetryRecord
{
    private readonly uint[] _words;
    private readonly int _baseWord;

    internal ReSTIRTelemetryRecord(uint[] words, int baseWord)
    {
        _words = words;
        _baseWord = baseWord;
    }

    public ReSTIRTelemetryStage Stage => (ReSTIRTelemetryStage)_words[_baseWord + 1];
    public ReSTIRTelemetryReason Reason => (ReSTIRTelemetryReason)_words[_baseWord + 2];
    public int PixelIndex => unchecked((int)_words[_baseWord + 3]);
    public int Generation => unchecked((int)_words[_baseWord + ReSTIRTelemetryLayout.RecordGeneration]);
    public int Sample => unchecked((int)_words[_baseWord + ReSTIRTelemetryLayout.RecordSample]);

    public uint GetPayloadWord(int index)
    {
        if (index < 0 || index >= ReSTIRTelemetryLayout.RecordPayloadWordCount)
            throw new ArgumentOutOfRangeException(nameof(index));

        return _words[_baseWord + ReSTIRTelemetryLayout.RecordPayloadBase + index];
    }
}

public readonly struct ReSTIRTelemetryPacket
{
    private readonly uint[] _words;

    private ReSTIRTelemetryPacket(uint[] words)
    {
        _words = words;
    }

    public int Frame => (int)_words[ReSTIRTelemetryLayout.HeaderFrame];
    public int Sample => (int)_words[ReSTIRTelemetryLayout.HeaderSample];
    public int Generation => (int)_words[ReSTIRTelemetryLayout.HeaderGeneration];
    public int Width => (int)_words[ReSTIRTelemetryLayout.HeaderWidth];
    public int Height => (int)_words[ReSTIRTelemetryLayout.HeaderHeight];
    public uint ModeFlags => _words[ReSTIRTelemetryLayout.HeaderModeFlags];
    public int DirectInitialSlot => unchecked((int)_words[ReSTIRTelemetryLayout.HeaderDirectInitialSlot]);
    public int DirectFinalSlot => unchecked((int)_words[ReSTIRTelemetryLayout.HeaderDirectFinalSlot]);
    public int IndirectInitialSlot => unchecked((int)_words[ReSTIRTelemetryLayout.HeaderIndirectInitialSlot]);
    public int IndirectFinalSlot => unchecked((int)_words[ReSTIRTelemetryLayout.HeaderIndirectFinalSlot]);
    public int SelectedPixel => unchecked((int)_words[ReSTIRTelemetryLayout.HeaderSelectedPixel]);

    public uint GetCounter(ReSTIRTelemetryCounter counter)
    {
        int index = (int)counter;
        if (index < 0 || index >= ReSTIRTelemetryLayout.CounterCount)
            return 0;

        return _words[ReSTIRTelemetryLayout.CounterBase + index];
    }

    public bool TryGetRecord(int index, out ReSTIRTelemetryRecord record)
    {
        record = default;
        if (index < 0 || index >= ReSTIRTelemetryLayout.RecordCount)
            return false;

        int baseWord = ReSTIRTelemetryLayout.RecordBase + index * ReSTIRTelemetryLayout.RecordWordCount;
        if (_words[baseWord] == 0)
            return false;

        record = new ReSTIRTelemetryRecord(_words, baseWord);
        return true;
    }

    public static bool TryDecode(
        uint[] words,
        int expectedGeneration,
        out ReSTIRTelemetryPacket packet,
        out string error)
    {
        packet = default;
        error = null;

        if (words == null || words.Length != ReSTIRTelemetryLayout.PacketWordCount)
        {
            error = $"packet length must be {ReSTIRTelemetryLayout.PacketWordCount} words";
            return false;
        }

        if (words[ReSTIRTelemetryLayout.HeaderMagic] != ReSTIRTelemetryLayout.Magic)
        {
            error = "packet magic mismatch";
            return false;
        }

        if (words[ReSTIRTelemetryLayout.HeaderSchema] != ReSTIRTelemetryLayout.SchemaVersion)
        {
            error = "packet schema mismatch";
            return false;
        }

        if (words[ReSTIRTelemetryLayout.HeaderGeneration] != (uint)expectedGeneration)
        {
            error = "packet generation is stale";
            return false;
        }

        packet = new ReSTIRTelemetryPacket(words);
        return true;
    }

    public static uint[] CreateFixture(int frame, int sample, int generation)
    {
        uint[] words = new uint[ReSTIRTelemetryLayout.PacketWordCount];
        words[ReSTIRTelemetryLayout.HeaderMagic] = ReSTIRTelemetryLayout.Magic;
        words[ReSTIRTelemetryLayout.HeaderSchema] = ReSTIRTelemetryLayout.SchemaVersion;
        words[ReSTIRTelemetryLayout.HeaderFrame] = (uint)frame;
        words[ReSTIRTelemetryLayout.HeaderSample] = (uint)sample;
        words[ReSTIRTelemetryLayout.HeaderGeneration] = (uint)generation;
        words[ReSTIRTelemetryLayout.HeaderSelectedPixel] = uint.MaxValue;
        return words;
    }
}

public readonly struct ReSTIRDiagnosticsSettings
{
    public readonly string OutputRoot;
    public readonly string SceneName;
    public readonly int Width;
    public readonly int Height;
    public readonly bool EnableGpuTelemetry;
    public readonly int SteadyCaptureInterval;
    public readonly bool UseReSTIRDI;
    public readonly bool UseReSTIRGI;
    public readonly bool Denoise;

    public ReSTIRDiagnosticsSettings(
        string outputRoot,
        string sceneName,
        int width,
        int height,
        bool enableGpuTelemetry,
        int steadyCaptureInterval,
        bool useReSTIRDI,
        bool useReSTIRGI,
        bool denoise)
    {
        OutputRoot = outputRoot;
        SceneName = sceneName ?? string.Empty;
        Width = width;
        Height = height;
        EnableGpuTelemetry = enableGpuTelemetry;
        SteadyCaptureInterval = steadyCaptureInterval;
        UseReSTIRDI = useReSTIRDI;
        UseReSTIRGI = useReSTIRGI;
        Denoise = denoise;
    }
}

public sealed class ReSTIRDiagnosticsSession : IDisposable
{
    private const int MaxConsecutiveReadbackErrors = 4;
    private const int MaxReadbackBackoffFrames = 64;

    private sealed class JsonlSink : IDisposable
    {
        private readonly string _fileName;
        private StreamWriter _writer;
        private bool _reportedFailure;

        public JsonlSink(string path)
        {
            _fileName = Path.GetFileName(path);
            try
            {
                _writer = new StreamWriter(path, false, new UTF8Encoding(false), 8192)
                {
                    AutoFlush = false
                };
            }
            catch (Exception ex)
            {
                Disable(ex);
            }
        }

        public void Write(string value)
        {
            if (_writer == null)
                return;
            try { _writer.Write(value); }
            catch (Exception ex) { Disable(ex); }
        }

        public void Write(char value)
        {
            if (_writer == null)
                return;
            try { _writer.Write(value); }
            catch (Exception ex) { Disable(ex); }
        }

        public void WriteLine(string value)
        {
            if (_writer == null)
                return;
            try { _writer.WriteLine(value); }
            catch (Exception ex) { Disable(ex); }
        }

        public void Flush()
        {
            if (_writer == null)
                return;
            try { _writer.Flush(); }
            catch (Exception ex) { Disable(ex); }
        }

        public void Dispose()
        {
            StreamWriter writer = _writer;
            _writer = null;
            if (writer == null)
                return;
            try { writer.Dispose(); }
            catch (Exception ex) { ReportFailure(ex); }
        }

        private void Disable(Exception exception)
        {
            StreamWriter writer = _writer;
            _writer = null;
            try { writer?.Dispose(); }
            catch { }
            ReportFailure(exception);
        }

        private void ReportFailure(Exception exception)
        {
            if (_reportedFailure)
                return;
            _reportedFailure = true;
            Debug.LogError($"[ReSTIR][Diagnostics] disabled sink {_fileName}: {exception.Message}");
        }
    }

    private readonly struct CounterField
    {
        public readonly ReSTIRTelemetryCounter Counter;
        public readonly string JsonName;

        public CounterField(ReSTIRTelemetryCounter counter, string jsonName)
        {
            Counter = counter;
            JsonName = jsonName;
        }
    }

    [StructLayout(LayoutKind.Explicit)]
    private struct FloatWord
    {
        [FieldOffset(0)] public uint UInt32;
        [FieldOffset(0)] public float Single;
    }

    private readonly struct CaptureContext
    {
        public readonly int Frame;
        public readonly int Sample;
        public readonly int Generation;

        public CaptureContext(int frame, int sample, int generation)
        {
            Frame = frame;
            Sample = sample;
            Generation = generation;
        }
    }

    private static readonly CounterField[] CounterFields =
    {
        new CounterField(ReSTIRTelemetryCounter.SampledPixels, "sampledPixels"),
        new CounterField(ReSTIRTelemetryCounter.DIInitialAccepted, "diInitialAccepted"),
        new CounterField(ReSTIRTelemetryCounter.DIInitialInvalidSurface, "diInitialInvalidSurface"),
        new CounterField(ReSTIRTelemetryCounter.DIInitialInvalidProposal, "diInitialInvalidProposal"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalInvalidCurrent, "diTemporalInvalidCurrent"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalNoHistory, "diTemporalNoHistory"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalReprojectionOutOfBounds, "diTemporalReprojectionOutOfBounds"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalInvalidHistory, "diTemporalInvalidHistory"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalIncompatibleSurface, "diTemporalIncompatibleSurface"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalReevaluationRejected, "diTemporalReevaluationRejected"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalHistoryCombined, "diTemporalHistoryCombined"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalHistorySelected, "diTemporalHistorySelected"),
        new CounterField(ReSTIRTelemetryCounter.DITemporalNonFiniteOutput, "diTemporalNonFiniteOutput"),
        new CounterField(ReSTIRTelemetryCounter.DIShadeInvalidReservoir, "diShadeInvalidReservoir"),
        new CounterField(ReSTIRTelemetryCounter.DIShadeVisibilityRejected, "diShadeVisibilityRejected"),
        new CounterField(ReSTIRTelemetryCounter.DIShadePositiveContribution, "diShadePositiveContribution"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialPrimaryMiss, "giInitialPrimaryMiss"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialInvalidSecondary, "giInitialInvalidSecondary"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialInvalidProposal, "giInitialInvalidProposal"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialZeroThroughput, "giInitialZeroThroughput"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialZeroTarget, "giInitialZeroTarget"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialNonFinite, "giInitialNonFinite"),
        new CounterField(ReSTIRTelemetryCounter.GIInitialAccepted, "giInitialAccepted"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalInvalidCurrent, "giTemporalInvalidCurrent"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalNoHistory, "giTemporalNoHistory"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalReprojectionOutOfBounds, "giTemporalReprojectionOutOfBounds"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalInvalidHistory, "giTemporalInvalidHistory"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalIncompatibleSurface, "giTemporalIncompatibleSurface"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalReevaluationRejected, "giTemporalReevaluationRejected"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalJacobianRejected, "giTemporalJacobianRejected"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalHistoryCombined, "giTemporalHistoryCombined"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalHistorySelected, "giTemporalHistorySelected"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalMCapped, "giTemporalMCapped"),
        new CounterField(ReSTIRTelemetryCounter.GITemporalNonFiniteOutput, "giTemporalNonFiniteOutput"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialInvalidCurrent, "giSpatialInvalidCurrent"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialInvalidNeighbor, "giSpatialInvalidNeighbor"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialIncompatibleNeighbor, "giSpatialIncompatibleNeighbor"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialReevaluationRejected, "giSpatialReevaluationRejected"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialJacobianRejected, "giSpatialJacobianRejected"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialNeighborCombined, "giSpatialNeighborCombined"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialNeighborSelected, "giSpatialNeighborSelected"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialZeroNormalization, "giSpatialZeroNormalization"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialMCapped, "giSpatialMCapped"),
        new CounterField(ReSTIRTelemetryCounter.GISpatialNonFiniteOutput, "giSpatialNonFiniteOutput"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalInvalidPrimary, "giFinalInvalidPrimary"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalInvalidReservoir, "giFinalInvalidReservoir"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalVisibilityRejected, "giFinalVisibilityRejected"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalZeroRadiance, "giFinalZeroRadiance"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalZeroTarget, "giFinalZeroTarget"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalNonFiniteWeight, "giFinalNonFiniteWeight"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalNonFiniteContribution, "giFinalNonFiniteContribution"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalExcessiveWeight, "giFinalExcessiveWeight"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalExcessiveContribution, "giFinalExcessiveContribution"),
        new CounterField(ReSTIRTelemetryCounter.GIFinalPositiveContribution, "giFinalPositiveContribution"),
        new CounterField(ReSTIRTelemetryCounter.CriticalNonFinite, "criticalNonFinite"),
        new CounterField(ReSTIRTelemetryCounter.CriticalOutOfRange, "criticalOutOfRange"),
        new CounterField(ReSTIRTelemetryCounter.CriticalBufferContract, "criticalBufferContract")
    };

    private readonly ReSTIRDiagnosticsSettings _settings;
    private readonly JsonlSink _sessionWriter;
    private readonly JsonlSink _eventWriter;
    private readonly JsonlSink _performanceWriter;
    private readonly JsonlSink _telemetryStatsWriter;
    private readonly JsonlSink _diStatsWriter;
    private readonly JsonlSink _giProbeWriter;
    private readonly JsonlSink _giTemporalWriter;
    private readonly JsonlSink _giSpatialWriter;
    private readonly JsonlSink _giFinalWriter;
    private readonly uint[] _packetWords = new uint[ReSTIRTelemetryLayout.PacketWordCount];
    private readonly Action<AsyncGPUReadbackRequest> _readbackCallback;
    private bool _readbackPending;
    private bool _captureArmed;
    private bool _forceNextCapture;
    private bool _disposed;
    private int _acceptedCaptures;
    private int _droppedCaptures;
    private int _readbackErrors;
    private int _consecutiveReadbackErrors;
    private int _nextReadbackFrame;
    private bool _gpuTelemetryDisabled;
    private int _rowsSinceFlush;
    private bool _loggedFirstIssue;
    private CaptureContext _captureContext;

    private ReSTIRDiagnosticsSession(ReSTIRDiagnosticsSettings settings)
    {
        _settings = settings;
        _readbackCallback = OnReadback;
        SessionId = Guid.NewGuid().ToString("N");
        Generation = 1;

        string timestamp = DateTime.UtcNow.ToString("yyyy-MM-dd_HHmmss_fff", CultureInfo.InvariantCulture);
        string root = Path.GetFullPath(settings.OutputRoot);
        OutputDirectory = Path.Combine(root, timestamp + "_" + SessionId.Substring(0, 8));
        Directory.CreateDirectory(OutputDirectory);

        _sessionWriter = OpenWriter("restir_session.jsonl");
        _eventWriter = OpenWriter("restir_events.jsonl");
        _performanceWriter = OpenWriter("restir_performance.jsonl");
        _telemetryStatsWriter = OpenWriter("restir_telemetry_stats.jsonl");
        _diStatsWriter = OpenWriter("restir_di_stats.jsonl");
        _giProbeWriter = OpenWriter("restir_gi_probe.jsonl");
        _giTemporalWriter = OpenWriter("restir_gi_temporal_stats.jsonl");
        _giSpatialWriter = OpenWriter("restir_gi_spatial_stats.jsonl");
        _giFinalWriter = OpenWriter("restir_gi_final_stats.jsonl");

        WriteSessionRecord("session_start");
        FlushWriters();
        Debug.Log($"[ReSTIR][Session] start id={SessionId} output={OutputDirectory}");
    }

    public string SessionId { get; }
    public string OutputDirectory { get; }
    public int Generation { get; private set; }
    public bool ReadbackPending => _readbackPending;
    public bool GpuTelemetryEnabled =>
        _settings.EnableGpuTelemetry && SystemInfo.supportsAsyncGPUReadback && !_gpuTelemetryDisabled;

    public static ReSTIRDiagnosticsSession Start(ReSTIRDiagnosticsSettings settings)
    {
        return new ReSTIRDiagnosticsSession(settings);
    }

    public void ScheduleResetCapture(string reason, int sample = -1)
    {
        if (_disposed)
            return;

        Generation++;
        _forceNextCapture = true;
        RecordStateChange(reason, sample);
    }

    public bool BeginCapture(
        int frame,
        int sample,
        int width,
        int height,
        bool useDI,
        bool useGI,
        int directInitial,
        int directFinal,
        int indirectInitial,
        int indirectFinal)
    {
        if (_disposed || !GpuTelemetryEnabled)
            return false;

        if (Time.frameCount < _nextReadbackFrame)
            return false;

        bool scheduled = _forceNextCapture ||
            ReSTIRTelemetryLayout.ShouldCaptureSample(sample, _settings.SteadyCaptureInterval);
        if (!scheduled)
            return false;

        if (_readbackPending)
        {
            _droppedCaptures++;
            return false;
        }

        _forceNextCapture = false;
        _captureArmed = true;
        _captureContext = new CaptureContext(frame, sample, Generation);
        return true;
    }

    public void RequestCapture(ComputeBuffer packetBuffer)
    {
        if (_disposed || !_captureArmed || _readbackPending || packetBuffer == null)
            return;

        _captureArmed = false;
        _readbackPending = true;
        try
        {
            AsyncGPUReadback.Request(packetBuffer, _readbackCallback);
        }
        catch (Exception ex)
        {
            _readbackPending = false;
            RegisterReadbackFailure("readback_request_failed", _captureContext, ex.Message);
        }
    }

    public void RecordStateChange(string reason, int sample)
    {
        if (_disposed)
            return;

        _eventWriter.Write("{\"sessionId\":\"");
        _eventWriter.Write(SessionId);
        _eventWriter.Write("\",\"event\":\"state_change\",\"generation\":");
        _eventWriter.Write(Generation.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"sampleCount\":");
        _eventWriter.Write(sample.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"reason\":\"");
        _eventWriter.Write(EscapeJson(reason));
        _eventWriter.WriteLine("\"}");
        FlushIfNeeded();
    }

    private void OnReadback(AsyncGPUReadbackRequest request)
    {
        CaptureContext context = _captureContext;
        Stopwatch stopwatch = Stopwatch.StartNew();
        try
        {
            if (_disposed)
                return;

            if (request.hasError)
            {
                RegisterReadbackFailure("readback_failed", context, "AsyncGPUReadbackRequest.hasError");
                return;
            }

            var data = request.GetData<uint>();
            if (data.Length != ReSTIRTelemetryLayout.PacketWordCount)
            {
                RegisterReadbackFailure(
                    "readback_size_mismatch",
                    context,
                    data.Length.ToString(CultureInfo.InvariantCulture));
                return;
            }

            data.CopyTo(_packetWords);
            if (_packetWords[ReSTIRTelemetryLayout.HeaderFrame] != (uint)context.Frame ||
                _packetWords[ReSTIRTelemetryLayout.HeaderSample] != (uint)context.Sample)
            {
                RegisterReadbackFailure(
                    "packet_correlation_mismatch",
                    context,
                    "frame or sample header does not match the capture request");
                return;
            }

            if (!ProcessPacketWords(_packetWords, Generation, out string error))
            {
                if (error == "packet generation is stale")
                    WriteWarning("packet_stale", context, error);
                else
                    RegisterReadbackFailure("packet_rejected", context, error);
                return;
            }

            _consecutiveReadbackErrors = 0;
        }
        catch (Exception ex)
        {
            RegisterReadbackFailure("readback_callback_failed", context, ex.Message);
        }
        finally
        {
            stopwatch.Stop();
            _readbackPending = false;
            if (!_disposed)
                WritePerformance(context, stopwatch.Elapsed.TotalMilliseconds);
        }
    }

    private bool ProcessPacketWords(uint[] words, int expectedGeneration, out string error)
    {
        if (!ReSTIRTelemetryPacket.TryDecode(
                words,
                expectedGeneration,
                out ReSTIRTelemetryPacket packet,
                out error))
        {
            return false;
        }

        if (!ValidatePacket(packet, out error))
            return false;

        _acceptedCaptures++;
        WritePacketSummary(packet);
        WritePacketCounters(packet);
        for (int i = 0; i < ReSTIRTelemetryLayout.RecordCount; i++)
        {
            if (packet.TryGetRecord(i, out ReSTIRTelemetryRecord record))
                WriteStageRecord(packet, record);
        }

        WriteCriticalCounterEvent(packet);
        error = null;
        return true;
    }

    private static bool ValidatePacket(ReSTIRTelemetryPacket packet, out string error)
    {
        if (packet.Width <= 0 || packet.Height <= 0)
        {
            error = "packet dimensions must be positive";
            return false;
        }

        long pixelCount = (long)packet.Width * packet.Height;
        if (packet.SelectedPixel != -1 && (packet.SelectedPixel < 0 || packet.SelectedPixel >= pixelCount))
        {
            error = "selected pixel is outside packet dimensions";
            return false;
        }

        if (!IsReservoirSlotValid(packet.DirectInitialSlot) ||
            !IsReservoirSlotValid(packet.DirectFinalSlot) ||
            !IsReservoirSlotValid(packet.IndirectInitialSlot) ||
            !IsReservoirSlotValid(packet.IndirectFinalSlot))
        {
            error = "reservoir slot is outside [-1, 2]";
            return false;
        }

        for (int i = 0; i < ReSTIRTelemetryLayout.RecordCount; i++)
        {
            if (!packet.TryGetRecord(i, out ReSTIRTelemetryRecord record))
                continue;

            int stage = (int)record.Stage;
            if (stage < (int)ReSTIRTelemetryStage.DIInitial || stage > (int)ReSTIRTelemetryStage.GIFinal)
            {
                error = "record stage is invalid";
                return false;
            }

            int reason = (int)record.Reason;
            if (reason < (int)ReSTIRTelemetryReason.None ||
                reason > (int)ReSTIRTelemetryReason.ExcessiveContribution)
            {
                error = "record reason is invalid";
                return false;
            }

            if (record.PixelIndex < 0 || record.PixelIndex >= pixelCount)
            {
                error = "record pixel is outside packet dimensions";
                return false;
            }

            if (record.Generation != packet.Generation || record.Sample != packet.Sample)
            {
                error = "record correlation does not match packet header";
                return false;
            }
        }

        error = null;
        return true;
    }

    private static bool IsReservoirSlotValid(int slot)
    {
        return slot >= -1 && slot <= 2;
    }

    private void WritePacketSummary(ReSTIRTelemetryPacket packet)
    {
        _eventWriter.Write("{\"sessionId\":\"");
        _eventWriter.Write(SessionId);
        _eventWriter.Write("\",\"event\":\"telemetry_packet\",\"severity\":\"info\",\"frameIndex\":");
        _eventWriter.Write(packet.Frame.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"sampleCount\":");
        _eventWriter.Write(packet.Sample.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"generation\":");
        _eventWriter.Write(packet.Generation.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"selectedPixel\":");
        _eventWriter.Write(packet.SelectedPixel.ToString(CultureInfo.InvariantCulture));
        _eventWriter.WriteLine("}");
        FlushIfNeeded();
    }

    private void WritePacketCounters(ReSTIRTelemetryPacket packet)
    {
        WriteCorrelationPrefix(_telemetryStatsWriter, packet);
        _telemetryStatsWriter.Write(",\"renderWidth\":");
        _telemetryStatsWriter.Write(packet.Width.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"renderHeight\":");
        _telemetryStatsWriter.Write(packet.Height.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"modeFlags\":");
        _telemetryStatsWriter.Write(packet.ModeFlags.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"directInitialSlot\":");
        _telemetryStatsWriter.Write(packet.DirectInitialSlot.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"directFinalSlot\":");
        _telemetryStatsWriter.Write(packet.DirectFinalSlot.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"indirectInitialSlot\":");
        _telemetryStatsWriter.Write(packet.IndirectInitialSlot.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"indirectFinalSlot\":");
        _telemetryStatsWriter.Write(packet.IndirectFinalSlot.ToString(CultureInfo.InvariantCulture));
        _telemetryStatsWriter.Write(",\"selectedPixel\":");
        _telemetryStatsWriter.Write(packet.SelectedPixel.ToString(CultureInfo.InvariantCulture));
        for (int i = 0; i < CounterFields.Length; i++)
        {
            CounterField field = CounterFields[i];
            _telemetryStatsWriter.Write(",\"");
            _telemetryStatsWriter.Write(field.JsonName);
            _telemetryStatsWriter.Write("\":");
            _telemetryStatsWriter.Write(packet.GetCounter(field.Counter).ToString(CultureInfo.InvariantCulture));
        }
        _telemetryStatsWriter.WriteLine("}");
        FlushIfNeeded();
    }

    private void WriteStageRecord(ReSTIRTelemetryPacket packet, ReSTIRTelemetryRecord record)
    {
        JsonlSink writer = GetStageWriter(record.Stage);
        WriteCorrelationPrefix(writer, packet);
        writer.Write(",\"stage\":\"");
        writer.Write(StageName(record.Stage));
        writer.Write("\",\"reason\":\"");
        writer.Write(ReasonName(record.Reason));
        writer.Write("\",\"reasonCode\":");
        writer.Write(((int)record.Reason).ToString(CultureInfo.InvariantCulture));
        writer.Write(",\"severity\":\"");
        string severity = SeverityName(record.Reason);
        writer.Write(severity);
        writer.Write("\",\"pixelIndex\":");
        writer.Write(record.PixelIndex.ToString(CultureInfo.InvariantCulture));
        writer.Write(",\"pixelX\":");
        writer.Write((record.PixelIndex % packet.Width).ToString(CultureInfo.InvariantCulture));
        writer.Write(",\"pixelY\":");
        writer.Write((record.PixelIndex / packet.Width).ToString(CultureInfo.InvariantCulture));
        writer.Write(",\"payload\":[");
        for (int i = 0; i < ReSTIRTelemetryLayout.RecordPayloadWordCount; i++)
        {
            if (i != 0)
                writer.Write(',');
            writer.Write(DecodeFloat(record.GetPayloadWord(i)).ToString("R", CultureInfo.InvariantCulture));
        }
        writer.WriteLine("]}");
        FlushIfNeeded();

        if (severity != "info")
            WriteStageIssueEvent(packet, record, severity);
    }

    private void WriteStageIssueEvent(
        ReSTIRTelemetryPacket packet,
        ReSTIRTelemetryRecord record,
        string severity)
    {
        WriteCorrelationPrefix(_eventWriter, packet);
        _eventWriter.Write(",\"event\":\"stage_invariant\",\"severity\":\"");
        _eventWriter.Write(severity);
        _eventWriter.Write("\",\"stage\":\"");
        _eventWriter.Write(StageName(record.Stage));
        _eventWriter.Write("\",\"reason\":\"");
        _eventWriter.Write(ReasonName(record.Reason));
        _eventWriter.Write("\",\"pixelIndex\":");
        _eventWriter.Write(record.PixelIndex.ToString(CultureInfo.InvariantCulture));
        _eventWriter.WriteLine("}");
        FlushIfNeeded();

        if (_loggedFirstIssue)
            return;

        _loggedFirstIssue = true;
        Debug.LogWarning(
            $"[ReSTIR][Diagnostics] {severity} stage={StageName(record.Stage)} " +
            $"reason={ReasonName(record.Reason)} pixel={record.PixelIndex} " +
            $"frame={packet.Frame} session={SessionId}");
    }

    private void WriteCriticalCounterEvent(ReSTIRTelemetryPacket packet)
    {
        uint nonFinite = packet.GetCounter(ReSTIRTelemetryCounter.CriticalNonFinite);
        uint outOfRange = packet.GetCounter(ReSTIRTelemetryCounter.CriticalOutOfRange);
        uint bufferContract = packet.GetCounter(ReSTIRTelemetryCounter.CriticalBufferContract);
        if (nonFinite == 0 && outOfRange == 0 && bufferContract == 0)
            return;

        WriteCorrelationPrefix(_eventWriter, packet);
        _eventWriter.Write(",\"event\":\"critical_counter\",\"severity\":\"error\",\"criticalNonFinite\":");
        _eventWriter.Write(nonFinite.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"criticalOutOfRange\":");
        _eventWriter.Write(outOfRange.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"criticalBufferContract\":");
        _eventWriter.Write(bufferContract.ToString(CultureInfo.InvariantCulture));
        _eventWriter.WriteLine("}");
        _eventWriter.Flush();
    }

    private void WriteCorrelationPrefix(JsonlSink writer, ReSTIRTelemetryPacket packet)
    {
        writer.Write("{\"sessionId\":\"");
        writer.Write(SessionId);
        writer.Write("\",\"frameIndex\":");
        writer.Write(packet.Frame.ToString(CultureInfo.InvariantCulture));
        writer.Write(",\"sampleCount\":");
        writer.Write(packet.Sample.ToString(CultureInfo.InvariantCulture));
        writer.Write(",\"generation\":");
        writer.Write(packet.Generation.ToString(CultureInfo.InvariantCulture));
    }

    private JsonlSink GetStageWriter(ReSTIRTelemetryStage stage)
    {
        switch (stage)
        {
            case ReSTIRTelemetryStage.DIInitial:
            case ReSTIRTelemetryStage.DITemporal:
            case ReSTIRTelemetryStage.DIShade:
                return _diStatsWriter;
            case ReSTIRTelemetryStage.GIInitial:
                return _giProbeWriter;
            case ReSTIRTelemetryStage.GITemporal:
                return _giTemporalWriter;
            case ReSTIRTelemetryStage.GISpatial:
                return _giSpatialWriter;
            case ReSTIRTelemetryStage.GIFinal:
                return _giFinalWriter;
            default:
                throw new ArgumentOutOfRangeException(nameof(stage), stage, "Unsupported telemetry stage");
        }
    }

    private static float DecodeFloat(uint value)
    {
        FloatWord word = default;
        word.UInt32 = value;
        return word.Single;
    }

    private static string StageName(ReSTIRTelemetryStage stage)
    {
        switch (stage)
        {
            case ReSTIRTelemetryStage.DIInitial: return "di_initial";
            case ReSTIRTelemetryStage.DITemporal: return "di_temporal";
            case ReSTIRTelemetryStage.DIShade: return "di_shade";
            case ReSTIRTelemetryStage.GIInitial: return "gi_initial";
            case ReSTIRTelemetryStage.GITemporal: return "gi_temporal";
            case ReSTIRTelemetryStage.GISpatial: return "gi_spatial";
            case ReSTIRTelemetryStage.GIFinal: return "gi_final";
            default: return "none";
        }
    }

    private static string ReasonName(ReSTIRTelemetryReason reason)
    {
        switch (reason)
        {
            case ReSTIRTelemetryReason.None: return "none";
            case ReSTIRTelemetryReason.PrimaryMiss: return "primary_miss";
            case ReSTIRTelemetryReason.InvalidSurface: return "invalid_surface";
            case ReSTIRTelemetryReason.InvalidProposalPdf: return "invalid_proposal_pdf";
            case ReSTIRTelemetryReason.ZeroThroughput: return "zero_throughput";
            case ReSTIRTelemetryReason.ZeroTarget: return "zero_target";
            case ReSTIRTelemetryReason.ReprojectionOutOfBounds: return "reprojection_out_of_bounds";
            case ReSTIRTelemetryReason.InvalidHistory: return "invalid_history";
            case ReSTIRTelemetryReason.IncompatibleSurface: return "incompatible_surface";
            case ReSTIRTelemetryReason.ReevaluationRejected: return "reevaluation_rejected";
            case ReSTIRTelemetryReason.JacobianRejected: return "jacobian_rejected";
            case ReSTIRTelemetryReason.ZeroNormalization: return "zero_normalization";
            case ReSTIRTelemetryReason.VisibilityRejected: return "visibility_rejected";
            case ReSTIRTelemetryReason.NonFiniteReservoir: return "nonfinite_reservoir";
            case ReSTIRTelemetryReason.NonFiniteWeight: return "nonfinite_weight";
            case ReSTIRTelemetryReason.NonFiniteContribution: return "nonfinite_contribution";
            case ReSTIRTelemetryReason.ExcessiveWeight: return "excessive_weight";
            case ReSTIRTelemetryReason.ExcessiveContribution: return "excessive_contribution";
            default: return "unknown";
        }
    }

    private static string SeverityName(ReSTIRTelemetryReason reason)
    {
        switch (reason)
        {
            case ReSTIRTelemetryReason.NonFiniteReservoir:
            case ReSTIRTelemetryReason.NonFiniteWeight:
            case ReSTIRTelemetryReason.NonFiniteContribution:
            case ReSTIRTelemetryReason.ExcessiveWeight:
            case ReSTIRTelemetryReason.ExcessiveContribution:
                return "error";
            case ReSTIRTelemetryReason.ZeroNormalization:
                return "warning";
            default:
                return "info";
        }
    }

    private void RegisterReadbackFailure(string eventName, CaptureContext context, string message)
    {
        _readbackErrors++;
        _consecutiveReadbackErrors++;
        int backoffFrames = 1 << Math.Min(_consecutiveReadbackErrors - 1, 6);
        _nextReadbackFrame = Time.frameCount + Math.Min(backoffFrames, MaxReadbackBackoffFrames);
        if (_consecutiveReadbackErrors >= MaxConsecutiveReadbackErrors)
        {
            _gpuTelemetryDisabled = true;
            message += "; GPU telemetry disabled for this session";
        }

        WriteError(eventName, context, message);
    }

    private void WriteError(string eventName, CaptureContext context, string message)
    {
        WriteIssue(eventName, "error", context, message);
        Debug.LogError($"[ReSTIR][Diagnostics] {eventName}: {message} session={SessionId}");
    }

    private void WriteWarning(string eventName, CaptureContext context, string message)
    {
        WriteIssue(eventName, "warning", context, message);
        Debug.LogWarning($"[ReSTIR][Diagnostics] {eventName}: {message} session={SessionId}");
    }

    private void WriteIssue(string eventName, string severity, CaptureContext context, string message)
    {
        _eventWriter.Write("{\"sessionId\":\"");
        _eventWriter.Write(SessionId);
        _eventWriter.Write("\",\"event\":\"");
        _eventWriter.Write(eventName);
        _eventWriter.Write("\",\"severity\":\"");
        _eventWriter.Write(severity);
        _eventWriter.Write("\",\"frameIndex\":");
        _eventWriter.Write(context.Frame.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"sampleCount\":");
        _eventWriter.Write(context.Sample.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"generation\":");
        _eventWriter.Write(context.Generation.ToString(CultureInfo.InvariantCulture));
        _eventWriter.Write(",\"message\":\"");
        _eventWriter.Write(EscapeJson(message));
        _eventWriter.WriteLine("\"}");
        _eventWriter.Flush();
    }

    private void WritePerformance(CaptureContext context, double callbackMilliseconds)
    {
        _performanceWriter.Write("{\"sessionId\":\"");
        _performanceWriter.Write(SessionId);
        _performanceWriter.Write("\",\"frameIndex\":");
        _performanceWriter.Write(context.Frame.ToString(CultureInfo.InvariantCulture));
        _performanceWriter.Write(",\"sampleCount\":");
        _performanceWriter.Write(context.Sample.ToString(CultureInfo.InvariantCulture));
        _performanceWriter.Write(",\"generation\":");
        _performanceWriter.Write(context.Generation.ToString(CultureInfo.InvariantCulture));
        _performanceWriter.Write(",\"callbackMilliseconds\":");
        _performanceWriter.Write(callbackMilliseconds.ToString("R", CultureInfo.InvariantCulture));
        _performanceWriter.WriteLine("}");
        FlushIfNeeded();
    }

    private void WriteSessionRecord(string eventName)
    {
        _sessionWriter.Write("{\"sessionId\":\"");
        _sessionWriter.Write(SessionId);
        _sessionWriter.Write("\",\"event\":\"");
        _sessionWriter.Write(eventName);
        _sessionWriter.Write("\",\"schemaVersion\":");
        _sessionWriter.Write(ReSTIRTelemetryLayout.SchemaVersion.ToString(CultureInfo.InvariantCulture));
        _sessionWriter.Write(",\"sceneName\":\"");
        _sessionWriter.Write(EscapeJson(_settings.SceneName));
        _sessionWriter.Write("\",\"renderWidth\":");
        _sessionWriter.Write(_settings.Width.ToString(CultureInfo.InvariantCulture));
        _sessionWriter.Write(",\"renderHeight\":");
        _sessionWriter.Write(_settings.Height.ToString(CultureInfo.InvariantCulture));
        _sessionWriter.Write(",\"gpuTelemetryEnabled\":");
        _sessionWriter.Write(GpuTelemetryEnabled ? "true" : "false");
        _sessionWriter.Write(",\"useReSTIRDI\":");
        _sessionWriter.Write(_settings.UseReSTIRDI ? "true" : "false");
        _sessionWriter.Write(",\"useReSTIRGI\":");
        _sessionWriter.Write(_settings.UseReSTIRGI ? "true" : "false");
        _sessionWriter.Write(",\"denoise\":");
        _sessionWriter.Write(_settings.Denoise ? "true" : "false");
        _sessionWriter.Write(",\"asyncGpuReadbackSupported\":");
        _sessionWriter.Write(SystemInfo.supportsAsyncGPUReadback ? "true" : "false");
        _sessionWriter.Write(",\"unityVersion\":\"");
        _sessionWriter.Write(EscapeJson(Application.unityVersion));
        _sessionWriter.Write("\",\"graphicsDevice\":\"");
        _sessionWriter.Write(EscapeJson(SystemInfo.graphicsDeviceName));
        _sessionWriter.Write("\",\"acceptedCaptures\":");
        _sessionWriter.Write(_acceptedCaptures.ToString(CultureInfo.InvariantCulture));
        _sessionWriter.Write(",\"droppedCaptures\":");
        _sessionWriter.Write(_droppedCaptures.ToString(CultureInfo.InvariantCulture));
        _sessionWriter.Write(",\"readbackErrors\":");
        _sessionWriter.Write(_readbackErrors.ToString(CultureInfo.InvariantCulture));
        _sessionWriter.WriteLine("}");
    }

    private JsonlSink OpenWriter(string fileName)
    {
        return new JsonlSink(Path.Combine(OutputDirectory, fileName));
    }

    private void FlushIfNeeded()
    {
        _rowsSinceFlush++;
        if (_rowsSinceFlush < 8)
            return;

        FlushWriters();
    }

    private void FlushWriters()
    {
        _rowsSinceFlush = 0;
        _sessionWriter.Flush();
        _eventWriter.Flush();
        _performanceWriter.Flush();
        _telemetryStatsWriter.Flush();
        _diStatsWriter.Flush();
        _giProbeWriter.Flush();
        _giTemporalWriter.Flush();
        _giSpatialWriter.Flush();
        _giFinalWriter.Flush();
    }

    private static string EscapeJson(string value)
    {
        if (string.IsNullOrEmpty(value))
            return string.Empty;

        return value
            .Replace("\\", "\\\\")
            .Replace("\"", "\\\"")
            .Replace("\r", "\\r")
            .Replace("\n", "\\n")
            .Replace("\t", "\\t");
    }

    public void Dispose()
    {
        if (_disposed)
            return;

        WriteSessionRecord("session_end");
        FlushWriters();
        _disposed = true;
        _sessionWriter.Dispose();
        _eventWriter.Dispose();
        _performanceWriter.Dispose();
        _telemetryStatsWriter.Dispose();
        _diStatsWriter.Dispose();
        _giProbeWriter.Dispose();
        _giTemporalWriter.Dispose();
        _giSpatialWriter.Dispose();
        _giFinalWriter.Dispose();
        Debug.Log($"[ReSTIR][Session] end id={SessionId} output={OutputDirectory}");
    }
}
