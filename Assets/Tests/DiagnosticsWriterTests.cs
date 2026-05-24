using NUnit.Framework;
using UnityEngine;
using System.IO;

/// <summary>
/// 测试 DiagnosticsWriter 的核心行为：
/// - 文件输出：写 JSON 文件，内容可反序列化、数值正确
/// - 错误处理：路径无效时不抛异常、关闭输出时不创建文件
/// </summary>
public class DiagnosticsWriterTests
{
    private string _tempDir;

    [SetUp]
    public void SetUp()
    {
        _tempDir = Path.Combine(Application.temporaryCachePath, "TTTest_" + System.Guid.NewGuid().ToString("N").Substring(0, 8));
        Directory.CreateDirectory(_tempDir);
        DiagnosticsWriter.OutputDirectory = _tempDir;
        DiagnosticsWriter.EnableFileOutput = true;
        DiagnosticsWriter.EnableConsoleOutput = false;
    }

    [TearDown]
    public void TearDown()
    {
        if (Directory.Exists(_tempDir))
            Directory.Delete(_tempDir, true);
        DiagnosticsWriter.ResetFrameCounter();
    }

    // ---- File Output Tests ----

    [Test]
    public void WriteFrameData_CreatesJsonFile()
    {
        var slots = CreateTestSlots(targetLum: 6.29f, sampleCount: 8, payloadValid: 1);

        DiagnosticsWriter.WriteFrameData(0, 1, new Vector2Int(64, 48), slots,
            "CornellBox", 128, 128, true, false, 8);

        string[] files = Directory.GetFiles(_tempDir, "*.json");
        Assert.AreEqual(1, files.Length, "应在 temp 目录生成一个 JSON 文件");

        string content = File.ReadAllText(files[0]);
        StringAssert.Contains("CornellBox", content);
        StringAssert.Contains("\"frameIndex\":0", content);
    }

    [Test]
    public void WriteFrameData_JsonIsValidAndContainsCorrectValues()
    {
        var slots = CreateTestSlots(targetLum: 6.29f, sampleCount: 8, payloadValid: 1);
        slots[1] = new Vector4(5.99f, 1.25f, 8f, 0f); // current reservoir
        slots[6] = new Vector4(1f, 1f, 228.8f, 0f);    // firstDraw

        DiagnosticsWriter.WriteFrameData(5, 30, new Vector2Int(64, 48), slots,
            "CornellBox", 256, 256, true, false, 8);

        string[] files = Directory.GetFiles(_tempDir, "*.json");
        string json = File.ReadAllText(files[0]);

        // 读取第一行（每个写入是一个 JSON 对象，一行一个）
        string[] lines = json.Split('\n');
        string firstLine = lines[0];

        Assert.IsTrue(firstLine.Contains("\"frameIndex\":5"));
        Assert.IsTrue(firstLine.Contains("\"sampleCount\":30"));
        Assert.IsTrue(firstLine.Contains("\"sceneName\":\"CornellBox\""));
        Assert.IsTrue(firstLine.Contains("\"useRIS\":true"));
        Assert.IsTrue(firstLine.Contains("\"bounceCount\":8"));
    }

    [Test]
    public void WriteFrameData_MultipleFrames_AppendsToSameFile()
    {
        var slots = CreateTestSlots(targetLum: 1.0f, sampleCount: 1, payloadValid: 1);

        DiagnosticsWriter.WriteFrameData(0, 1, Vector2Int.zero, slots,
            "Test", 64, 64, false, false, 1);
        DiagnosticsWriter.WriteFrameData(1, 2, Vector2Int.zero, slots,
            "Test", 64, 64, false, false, 1);
        DiagnosticsWriter.WriteFrameData(2, 3, Vector2Int.zero, slots,
            "Test", 64, 64, false, false, 1);

        string[] files = Directory.GetFiles(_tempDir, "*.json");
        Assert.AreEqual(1, files.Length, "多帧应写入同一个 JSON 文件");

        string[] lines = File.ReadAllText(files[0]).TrimEnd('\n').Split('\n');
        Assert.AreEqual(3, lines.Length, "3 帧 = 3 行 JSON");
    }

    // ---- Error Handling Tests ----

    [Test]
    public void WriteFrameData_InvalidDirectory_DoesNotThrow()
    {
        DiagnosticsWriter.EnableFileOutput = true;
        DiagnosticsWriter.OutputDirectory = "Z:\\NonExistentDrive\\Path\\ShouldNotExist";

        var slots = CreateTestSlots(targetLum: 1f, sampleCount: 1, payloadValid: 1);

        Assert.DoesNotThrow(() =>
        {
            DiagnosticsWriter.WriteFrameData(0, 1, Vector2Int.zero, slots,
                "Test", 64, 64, false, false, 4);
        }, "无效路径不应抛异常");
    }

    [Test]
    public void WriteFrameData_DisabledFileOutput_DoesNotCreateFile()
    {
        DiagnosticsWriter.EnableFileOutput = false;
        var slots = CreateTestSlots(targetLum: 1f, sampleCount: 1, payloadValid: 1);

        DiagnosticsWriter.WriteFrameData(0, 1, Vector2Int.zero, slots,
            "Test", 64, 64, false, false, 4);

        string[] files = Directory.GetFiles(_tempDir, "*.json");
        Assert.AreEqual(0, files.Length, "File output 关闭时应不创建文件");
    }

    // ---- Data Integrity Test ----

    [Test]
    public void WriteFrameData_SlotsRoundTrip_Correctly()
    {
        var original = CreateRandomSlots();
        DiagnosticsWriter.WriteFrameData(0, 1, new Vector2Int(64, 48), original,
            "Test", 128, 128, false, false, 4);

        string[] files = Directory.GetFiles(_tempDir, "*.json");
        string line = File.ReadAllText(files[0]).TrimEnd('\n');

        // 验证 JSON 包含 slot 数据
        Assert.IsTrue(line.Contains("\"slots\":["));
    }

    // ---- Helpers ----

    private static Vector4[] CreateTestSlots(float targetLum, float sampleCount, float payloadValid)
    {
        var slots = new Vector4[22];
        for (int i = 0; i < 22; i++) slots[i] = Vector4.zero;
        slots[0] = new Vector4(1, 0, 0, 0);                             // flags: PrimaryHit=1
        slots[1] = new Vector4(targetLum, 1.0f, sampleCount, 0);        // current reservoir
        slots[6] = new Vector4(1, payloadValid, 63.66f, targetLum);     // firstDraw
        return slots;
    }

    private static Vector4[] CreateRandomSlots()
    {
        var slots = new Vector4[22];
        for (int i = 0; i < 22; i++)
            slots[i] = new Vector4(i * 1.1f, i * 2.2f, i * 3.3f, i * 4.4f);
        return slots;
    }
}
