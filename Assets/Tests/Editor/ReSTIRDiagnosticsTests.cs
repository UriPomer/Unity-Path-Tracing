using System;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public class ReSTIRDiagnosticsTests
{
    [Test]
    public void Diagnostics_Use_One_Compact_Async_Readback()
    {
        string path = ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs");
        Assert.That(File.Exists(path), Is.True, $"Missing diagnostics session: {path}");

        string source = File.ReadAllText(path);
        StringAssert.Contains("PacketWordCount = 1024", source);
        StringAssert.Contains("AsyncGPUReadback.Request", source);
        StringAssert.Contains("_readbackPending", source);
        StringAssert.DoesNotContain(".GetData(", source);
        StringAssert.DoesNotContain("File.AppendAllText", source);
    }

    [Test]
    public void Diagnostics_Isolate_File_Sinks_And_Back_Off_Repeated_Readback_Failures()
    {
        string source = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));

        StringAssert.Contains("private sealed class JsonlSink", source);
        StringAssert.Contains("disabled sink", source);
        StringAssert.Contains("MaxConsecutiveReadbackErrors", source);
        StringAssert.Contains("_nextReadbackFrame", source);
        StringAssert.Contains("_gpuTelemetryDisabled = true", source);
        StringAssert.Contains("WriteWarning(\"packet_stale\"", source);
    }

    [Test]
    public void Tracing_Does_Not_Synchronously_Read_Diagnostic_Buffers()
    {
        string source = File.ReadAllText(ProjectFile("Assets", "Scripts", "Tracing.cs"));
        StringAssert.DoesNotContain(".GetData(", source);
        StringAssert.DoesNotContain("_globalColorFrameBeforeReadback", source);
        StringAssert.DoesNotContain("_globalColorFrameAfterReadback", source);
        StringAssert.DoesNotContain("WriteReSTIRGIProbeIfNeeded", source);
        StringAssert.Contains("ReSTIRDiagnosticsSession", source);
        StringAssert.Contains("RequestCapture", source);
    }

    [Test]
    public void Telemetry_Capture_Uses_The_Exact_Shader_Frame_Id()
    {
        string source = File.ReadAllText(ProjectFile("Assets", "Scripts", "Tracing.cs"));

        StringAssert.Contains("_restirDiagnostics.BeginCapture(\n            (int)frameId,", source);
        StringAssert.Contains("SetInt(\"_FrameCount\", (int)frameId)", source);
        StringAssert.DoesNotContain("_currentShaderFrameId", source);
    }

    [Test]
    public void Runtime_Proof_Uses_DI_And_GI_With_Denoise_Disabled()
    {
        string selfTest = File.ReadAllText(ProjectFile("Assets", "Scripts", "Editor", "SelfTest.cs"));
        string tracing = File.ReadAllText(ProjectFile("Assets", "Scripts", "Tracing.cs"));
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string verifier = File.ReadAllText(ProjectFile("Tests", "Verify-LatestReSTIRGILogs.ps1"));

        StringAssert.Contains("SetPrivateField(tracing, \"UseReSTIRDI\", true);", selfTest);
        StringAssert.Contains("SetPrivateField(tracing, \"UseReSTIRGI\", true);", selfTest);
        StringAssert.Contains("SetPrivateField(tracing, \"Denoise\", false);", selfTest);
        StringAssert.Contains("(Denoise ? 16 : 0)", tracing);
        StringAssert.Contains("useReSTIRDI", session);
        StringAssert.Contains("useReSTIRGI", session);
        StringAssert.Contains("denoise", session);
        StringAssert.Contains("restir_di_stats.jsonl", verifier);
        StringAssert.Contains("DI initial/temporal/shade rows", verifier);
    }

    [Test]
    public void Telemetry_Instrumentation_Does_Not_Change_GI_Estimator_Behavior()
    {
        string giInitial = RestirShader("gi_initial.hlsl");
        string giShade = RestirShader("gi_shade.hlsl");

        StringAssert.DoesNotContain("IsDirectLightSampleVisible", giInitial);
        StringAssert.Contains("if (!IsGISecondaryBypass(sampleFlags))\n        data.throughput = 1.0;", giInitial);
        StringAssert.DoesNotContain("ApplyGIContributionFence", giShade);
        StringAssert.DoesNotContain("RESTIR_GI_FIRE_FLY_LUM_LIMIT", giShade);
        StringAssert.Contains("weightedReflectedRadiance = reflectedRadiance * res.weightSum;", giShade);
    }

    [Test]
    public void Direct_Light_Cdf_Is_Built_On_Gpu_Without_Readback()
    {
        string tracing = File.ReadAllText(ProjectFile("Assets", "Scripts", "Tracing.cs"));
        string compute = File.ReadAllText(ProjectFile("Assets", "ComputeShader", "main", "Tracing.compute"));
        string prepare = RestirShader("prepare_lights.hlsl");

        StringAssert.DoesNotContain(".GetData(", tracing);
        StringAssert.Contains("FindKernel(\"kernel_build_light_cdf\")", tracing);
        StringAssert.Contains("Dispatch(kernelBuildLightCdf, 1, 1, 1)", tracing);
        StringAssert.Contains("#pragma kernel kernel_build_light_cdf", compute);
        StringAssert.Contains("void kernel_build_light_cdf", prepare);
    }

    [TestCase(1, true)]
    [TestCase(2, true)]
    [TestCase(3, false)]
    [TestCase(4, true)]
    [TestCase(8, true)]
    [TestCase(16, true)]
    [TestCase(17, false)]
    [TestCase(60, true)]
    public void Capture_Schedule_Is_Deterministic(int sample, bool expected)
    {
        Type layout = RuntimeType("ReSTIRTelemetryLayout");

        MethodInfo method = layout.GetMethod("ShouldCaptureSample", BindingFlags.Public | BindingFlags.Static);
        Assert.That(method, Is.Not.Null, "ShouldCaptureSample method not found");
        Assert.That(method.Invoke(null, new object[] { sample, 60 }), Is.EqualTo(expected));
    }

    [Test]
    public void Decoder_Rejects_Bad_Magic_And_Stale_Generation()
    {
        Type layout = RuntimeType("ReSTIRTelemetryLayout");
        Type packetType = RuntimeType("ReSTIRTelemetryPacket");
        int packetWordCount = Constant<int>(layout, "PacketWordCount");
        uint[] words = new uint[packetWordCount];

        AssertDecodeFails(packetType, words, 3, "magic");

        words[Constant<int>(layout, "HeaderMagic")] = Constant<uint>(layout, "Magic");
        words[Constant<int>(layout, "HeaderSchema")] = Constant<uint>(layout, "SchemaVersion");
        words[Constant<int>(layout, "HeaderGeneration")] = 2;
        AssertDecodeFails(packetType, words, 3, "generation");
    }

    [Test]
    public void Decoder_Preserves_Correlation_And_Counters()
    {
        Type layout = RuntimeType("ReSTIRTelemetryLayout");
        Type packetType = RuntimeType("ReSTIRTelemetryPacket");
        Type counterType = RuntimeType("ReSTIRTelemetryCounter");

        MethodInfo createFixture = packetType.GetMethod("CreateFixture", BindingFlags.Public | BindingFlags.Static);
        Assert.That(createFixture, Is.Not.Null);
        uint[] words = (uint[])createFixture.Invoke(null, new object[] { 12, 8, 4 });

        object acceptedCounter = Enum.Parse(counterType, "GIInitialAccepted");
        int counterIndex = Convert.ToInt32(acceptedCounter);
        words[Constant<int>(layout, "CounterBase") + counterIndex] = 19;

        object[] arguments = { words, 4, null, null };
        MethodInfo tryDecode = packetType.GetMethod("TryDecode", BindingFlags.Public | BindingFlags.Static);
        Assert.That((bool)tryDecode.Invoke(null, arguments), Is.True, arguments[3] as string);

        object packet = arguments[2];
        Assert.That(packetType.GetProperty("Frame").GetValue(packet), Is.EqualTo(12));
        Assert.That(packetType.GetProperty("Sample").GetValue(packet), Is.EqualTo(8));
        Assert.That(packetType.GetProperty("Generation").GetValue(packet), Is.EqualTo(4));
        Assert.That(packetType.GetMethod("GetCounter").Invoke(packet, new[] { acceptedCounter }), Is.EqualTo(19u));
    }

    [Test]
    public void Session_Writes_Start_State_And_End_With_One_Id()
    {
        string root = Path.Combine(Path.GetTempPath(), "restir-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        try
        {
            Type settingsType = RuntimeType("ReSTIRDiagnosticsSettings");
            object settings = Activator.CreateInstance(settingsType, new object[]
            {
                root, "TestScene", 640, 360, false, 60, true, true, false
            });

            Type sessionType = RuntimeType("ReSTIRDiagnosticsSession");
            object session = sessionType.GetMethod("Start", BindingFlags.Public | BindingFlags.Static)
                .Invoke(null, new[] { settings });
            string outputDirectory = (string)sessionType.GetProperty("OutputDirectory").GetValue(session);
            string sessionId = (string)sessionType.GetProperty("SessionId").GetValue(session);

            sessionType.GetMethod("RecordStateChange").Invoke(session, new object[] { "restir_gi_toggled", 1 });
            ((IDisposable)session).Dispose();

            string sessionLog = File.ReadAllText(Path.Combine(outputDirectory, "restir_session.jsonl"));
            string eventsLog = File.ReadAllText(Path.Combine(outputDirectory, "restir_events.jsonl"));
            StringAssert.Contains("\"event\":\"session_start\"", sessionLog);
            StringAssert.Contains("\"event\":\"session_end\"", sessionLog);
            StringAssert.Contains("\"sessionId\":\"" + sessionId + "\"", sessionLog);
            StringAssert.Contains("\"reason\":\"restir_gi_toggled\"", eventsLog);
            StringAssert.Contains("\"sessionId\":\"" + sessionId + "\"", eventsLog);
        }
        finally
        {
            Directory.Delete(root, true);
        }
    }

    [Test]
    public void Session_Decodes_Gpu_Record_Into_Correlated_Stage_And_Counter_Logs()
    {
        string root = Path.Combine(Path.GetTempPath(), "restir-packet-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        try
        {
            Type settingsType = RuntimeType("ReSTIRDiagnosticsSettings");
            object settings = Activator.CreateInstance(settingsType, new object[]
            {
                root, "PacketScene", 320, 180, false, 60, true, true, false
            });
            Type sessionType = RuntimeType("ReSTIRDiagnosticsSession");
            object session = sessionType.GetMethod("Start", BindingFlags.Public | BindingFlags.Static)
                .Invoke(null, new[] { settings });
            int generation = (int)sessionType.GetProperty("Generation").GetValue(session);
            string outputDirectory = (string)sessionType.GetProperty("OutputDirectory").GetValue(session);

            Type layout = RuntimeType("ReSTIRTelemetryLayout");
            Type packetType = RuntimeType("ReSTIRTelemetryPacket");
            uint[] words = (uint[])packetType.GetMethod("CreateFixture", BindingFlags.Public | BindingFlags.Static)
                .Invoke(null, new object[] { 21, 8, generation });
            words[Constant<int>(layout, "HeaderWidth")] = 320;
            words[Constant<int>(layout, "HeaderHeight")] = 180;
            Type counterType = RuntimeType("ReSTIRTelemetryCounter");
            int acceptedCounter = Convert.ToInt32(Enum.Parse(counterType, "GIInitialAccepted"));
            words[Constant<int>(layout, "CounterBase") + acceptedCounter] = 23;

            int recordBase = Constant<int>(layout, "RecordBase");
            words[recordBase] = 1;
            words[recordBase + 1] = 4; // GIInitial
            words[recordBase + 2] = 0;
            words[recordBase + 3] = 37;
            words[recordBase + 4] = FloatBits(1.0f);
            words[recordBase + 5] = FloatBits(2.0f);
            words[recordBase + 6] = FloatBits(3.0f);
            words[recordBase + 7] = FloatBits(0.25f);
            words[recordBase + Constant<int>(layout, "RecordGeneration")] = (uint)generation;
            words[recordBase + Constant<int>(layout, "RecordSample")] = 8;

            MethodInfo process = sessionType.GetMethod("ProcessPacketWords", BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.That(process, Is.Not.Null, "ProcessPacketWords method not found");
            object[] processArgs = { words, generation, null };
            Assert.That((bool)process.Invoke(session, processArgs), Is.True, processArgs[2] as string);
            ((IDisposable)session).Dispose();

            string probe = File.ReadAllText(Path.Combine(outputDirectory, "restir_gi_probe.jsonl"));
            string counters = File.ReadAllText(Path.Combine(outputDirectory, "restir_telemetry_stats.jsonl"));
            StringAssert.Contains("\"sessionId\":", probe);
            StringAssert.Contains("\"frameIndex\":21", probe);
            StringAssert.Contains("\"pixelIndex\":37", probe);
            StringAssert.Contains("\"stage\":\"gi_initial\"", probe);
            StringAssert.Contains("\"giInitialAccepted\":23", counters);
        }
        finally
        {
            Directory.Delete(root, true);
        }
    }

    [Test]
    public void Editor_Bridge_Validates_Only_A_New_Completed_Session_Without_UI_Or_Exit()
    {
        string bridgePath = ProjectFile("Assets", "Scripts", "Editor", "ReSTIRDiagnosticsEditorBridge.cs");
        Assert.That(File.Exists(bridgePath), Is.True, $"Missing Editor diagnostics bridge: {bridgePath}");

        string bridge = File.ReadAllText(bridgePath);
        StringAssert.Contains("[InitializeOnLoad]", bridge);
        StringAssert.Contains("EditorApplication.playModeStateChanged", bridge);
        StringAssert.Contains("PlayModeStateChange.EnteredEditMode", bridge);
        StringAssert.Contains("EditorApplication.delayCall", bridge);
        StringAssert.Contains("session_end", bridge);
        StringAssert.Contains("VerifyLatestGILogsNonInteractive", bridge);
        StringAssert.DoesNotContain("DisplayDialog", bridge);
        StringAssert.DoesNotContain("EditorApplication.Exit", bridge);

        string selfTest = File.ReadAllText(ProjectFile("Assets", "Scripts", "Editor", "SelfTest.cs"));
        StringAssert.Contains("public static bool VerifyLatestGILogsNonInteractive(out string report)", selfTest);
    }

    [Test]
    public void Telemetry_Hlsl_Layout_Matches_CSharp_And_Has_Clear_Kernel()
    {
        Type layout = RuntimeType("ReSTIRTelemetryLayout");
        int packetWords = Constant<int>(layout, "PacketWordCount");
        int recordEnd = Constant<int>(layout, "RecordBase") +
            Constant<int>(layout, "RecordWordCount") * Constant<int>(layout, "RecordCount");
        Assert.That(recordEnd, Is.EqualTo(packetWords), "Record region must end at the packet boundary");

        string telemetryPath = ProjectFile("Assets", "ComputeShader", "main", "restir", "telemetry.hlsl");
        Assert.That(File.Exists(telemetryPath), Is.True, $"Missing telemetry include: {telemetryPath}");
        string telemetry = File.ReadAllText(telemetryPath);

        StringAssert.Contains("#define RESTIR_TELEMETRY_MAGIC 0x52535452u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_SCHEMA 1u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_PACKET_WORDS 1024u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_COUNTER_BASE 32u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_COUNTER_COUNT 96u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_RECORD_BASE 128u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_RECORD_WORDS 56u", telemetry);
        StringAssert.Contains("#define RESTIR_TELEMETRY_RECORD_COUNT 16u", telemetry);
        StringAssert.Contains("RWByteAddressBuffer ReSTIRTelemetry", telemetry);
        StringAssert.Contains("InterlockedCompareExchange", telemetry);
        StringAssert.Contains("_RestirTelemetrySampleStride", telemetry);

        string compute = File.ReadAllText(ProjectFile("Assets", "ComputeShader", "main", "Tracing.compute"));
        StringAssert.Contains("#pragma kernel kernel_clear_restir_telemetry", compute);
        StringAssert.Contains("void kernel_clear_restir_telemetry", compute);
    }

    [Test]
    public void ReSTIR_Stages_Emit_Bounded_Telemetry_Without_Replacing_Estimator_Formulas()
    {
        string diInitial = RestirShader("generate_initial.hlsl");
        StringAssert.Contains("RESTIR_COUNTER_DI_INITIAL_INVALID_SURFACE", diInitial);
        StringAssert.Contains("RESTIR_COUNTER_DI_INITIAL_INVALID_PROPOSAL", diInitial);
        StringAssert.Contains("RESTIR_COUNTER_DI_INITIAL_ACCEPTED", diInitial);

        string diTemporal = RestirShader("temporal_resampling.hlsl");
        StringAssert.Contains("RESTIR_COUNTER_DI_TEMPORAL_INVALID_CURRENT", diTemporal);
        StringAssert.Contains("RESTIR_COUNTER_DI_TEMPORAL_REPROJECTION_OOB", diTemporal);
        StringAssert.Contains("RESTIR_COUNTER_DI_TEMPORAL_HISTORY_COMBINED", diTemporal);
        StringAssert.Contains("RESTIR_COUNTER_DI_TEMPORAL_HISTORY_SELECTED", diTemporal);
        StringAssert.Contains("float prevW = prev.selectedWeight * prevSample.targetLum", diTemporal);

        string giInitial = RestirShader("gi_initial.hlsl");
        StringAssert.Contains("RESTIR_COUNTER_GI_INITIAL_PRIMARY_MISS", giInitial);
        StringAssert.Contains("RESTIR_COUNTER_GI_INITIAL_INVALID_SECONDARY", giInitial);
        StringAssert.Contains("RESTIR_COUNTER_GI_INITIAL_ZERO_TARGET", giInitial);
        StringAssert.Contains("RESTIR_COUNTER_GI_INITIAL_ACCEPTED", giInitial);
        StringAssert.Contains("RestirTelemetryClaimSelectedPixel", giInitial);
        StringAssert.Contains("InitializeIndirectReservoirSample(", giInitial);

        string giTemporal = RestirShader("gi_temporal.hlsl");
        StringAssert.Contains("RESTIR_COUNTER_GI_TEMPORAL_INVALID_CURRENT", giTemporal);
        StringAssert.Contains("RESTIR_COUNTER_GI_TEMPORAL_REPROJECTION_OOB", giTemporal);
        StringAssert.Contains("RESTIR_COUNTER_GI_TEMPORAL_HISTORY_COMBINED", giTemporal);
        StringAssert.Contains("RESTIR_COUNTER_GI_TEMPORAL_HISTORY_SELECTED", giTemporal);
        StringAssert.Contains("FinalizeIndirectReservoir(outR, normalizationNumerator, normalizationDenominator);", giTemporal);

        string giSpatial = RestirShader("gi_spatial.hlsl");
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_INVALID_CURRENT", giSpatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_INVALID_NEIGHBOR", giSpatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_REEVALUATION_REJECTED", giSpatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_JACOBIAN_REJECTED", giSpatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_COMBINED", giSpatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_SELECTED", giSpatial);
        StringAssert.Contains("FinalizeIndirectReservoir(outR, normalizationNumerator, normalizationDenominator);", giSpatial);

        string giFinal = RestirShader("gi_shade.hlsl");
        StringAssert.Contains("RESTIR_COUNTER_GI_FINAL_INVALID_PRIMARY", giFinal);
        StringAssert.Contains("RESTIR_COUNTER_GI_FINAL_INVALID_RESERVOIR", giFinal);
        StringAssert.Contains("RESTIR_COUNTER_GI_FINAL_VISIBILITY_REJECTED", giFinal);
        StringAssert.Contains("RESTIR_COUNTER_GI_FINAL_NONFINITE_CONTRIBUTION", giFinal);
        StringAssert.Contains("RESTIR_COUNTER_GI_FINAL_POSITIVE_CONTRIBUTION", giFinal);
        StringAssert.Contains("GlobalColors[id.x].L += max(gi, 0.0);", giFinal);
    }

    private static void AssertDecodeFails(Type packetType, uint[] words, int expectedGeneration, string expectedMessage)
    {
        MethodInfo tryDecode = packetType.GetMethod("TryDecode", BindingFlags.Public | BindingFlags.Static);
        Assert.That(tryDecode, Is.Not.Null);
        object[] arguments = { words, expectedGeneration, null, null };
        Assert.That((bool)tryDecode.Invoke(null, arguments), Is.False);
        StringAssert.Contains(expectedMessage, ((string)arguments[3]).ToLowerInvariant());
    }

    private static Type RuntimeType(string name)
    {
        Type type = Type.GetType(name + ", Assembly-CSharp");
        Assert.That(type, Is.Not.Null, name + " type not found in Assembly-CSharp");
        return type;
    }

    private static T Constant<T>(Type type, string fieldName)
    {
        FieldInfo field = type.GetField(fieldName, BindingFlags.Public | BindingFlags.Static);
        Assert.That(field, Is.Not.Null, fieldName + " constant not found on " + type.Name);
        return (T)field.GetRawConstantValue();
    }

    private static uint FloatBits(float value)
    {
        return BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
    }

    private static string ProjectFile(params string[] parts)
    {
        string projectRoot = Path.GetDirectoryName(Application.dataPath);
        Assert.That(projectRoot, Is.Not.Null.And.Not.Empty);
        return Path.GetFullPath(Path.Combine(projectRoot, Path.Combine(parts)));
    }

    private static string RestirShader(string fileName)
    {
        return File.ReadAllText(ProjectFile("Assets", "ComputeShader", "main", "restir", fileName));
    }
}
