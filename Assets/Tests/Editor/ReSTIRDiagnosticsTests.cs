using System;
using System.IO;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;

public class ReSTIRDiagnosticsTests
{
    [Test]
    public void Indirect_Reservoir_Stride_Matches_Shader_Layout()
    {
        Type tracing = RuntimeType("Tracing");
        FieldInfo field = tracing.GetField("IndirectReservoirStride", BindingFlags.NonPublic | BindingFlags.Static);
        Assert.That(field, Is.Not.Null);
        string shader = File.ReadAllText(Path.Combine(Path.GetDirectoryName(UnityEngine.Application.dataPath),
            "Assets", "ComputeShader", "main", "global.hlsl"));
        Match definition = Regex.Match(shader, @"struct IndirectReservoirData\s*\{(?<fields>.*?)\};", RegexOptions.Singleline);
        Assert.That(definition.Success, Is.True);
        string fields = Regex.Replace(definition.Groups["fields"].Value, @"//[^\r\n]*", "");
        int bytes = 0;
        foreach (Match member in Regex.Matches(fields, @"\b(?:float|uint)(?<count>[234]?)\s+\w+\s*;"))
        {
            string count = member.Groups["count"].Value;
            bytes += 4 * (count.Length == 0 ? 1 : int.Parse(count));
        }
        Assert.That((int)field.GetValue(null), Is.EqualTo(bytes));
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
                root, "TestScene", 640, 360, false, 60, false, false, false
            });

            Type sessionType = RuntimeType("ReSTIRDiagnosticsSession");
            object session = sessionType.GetMethod("Start", BindingFlags.Public | BindingFlags.Static)
                .Invoke(null, new[] { settings });
            string outputDirectory = (string)sessionType.GetProperty("OutputDirectory").GetValue(session);
            string sessionId = (string)sessionType.GetProperty("SessionId").GetValue(session);

            sessionType.GetMethod("RecordRenderModes").Invoke(session, new object[] { true, true });
            sessionType.GetMethod("RecordStateChange").Invoke(session, new object[] { "restir_gi_toggled", 1 });
            ((IDisposable)session).Dispose();

            string[] sessionLog = File.ReadAllLines(Path.Combine(outputDirectory, "restir_session.jsonl"));
            string eventsLog = File.ReadAllText(Path.Combine(outputDirectory, "restir_events.jsonl"));
            Assert.That(sessionLog, Has.Length.EqualTo(2));
            StringAssert.Contains("\"event\":\"session_start\"", sessionLog[0]);
            StringAssert.Contains("\"restirModeSeen\":false", sessionLog[0]);
            StringAssert.Contains("\"event\":\"session_end\"", sessionLog[1]);
            StringAssert.Contains("\"restirModeSeen\":true", sessionLog[1]);
            StringAssert.Contains("\"sessionId\":\"" + sessionId + "\"", sessionLog[1]);
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
    public void Explicit_Validation_Rejects_Zero_Captures()
    {
        string root = Path.Combine(Path.GetTempPath(), "restir-verifier-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        try
        {
            File.WriteAllLines(Path.Combine(root, "restir_session.jsonl"), new[]
            {
                "{\"sessionId\":\"test\",\"event\":\"session_start\",\"useReSTIRDI\":true,\"useReSTIRGI\":true}",
                "{\"sessionId\":\"test\",\"event\":\"session_end\",\"acceptedCaptures\":0,\"readbackErrors\":0}"
            });
            Type verifier = Type.GetType("SelfTest, Assembly-CSharp-Editor");
            Assert.That(verifier, Is.Not.Null);
            MethodInfo verify = verifier.GetMethod("VerifyGILogsInDirectory", new[]
            {
                typeof(string), typeof(string).MakeByRefType()
            });
            Assert.That(verify, Is.Not.Null);

            object[] arguments = { root, null };
            Assert.That((bool)verify.Invoke(null, arguments), Is.False);
            StringAssert.Contains("acceptedCaptures=0, readbackErrors=0", (string)arguments[1]);
        }
        finally
        {
            Directory.Delete(root, true);
        }
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
}
