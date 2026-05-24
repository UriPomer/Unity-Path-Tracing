using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;

/// <summary>
/// Batchmode 自测试入口。
/// 使用方式：Unity.exe -batchmode -quit -projectPath ... -executeMethod SelfTest.Run -sceneName CornellBox -frameCount 60
/// 渲染 N 帧后读取 DiagnosticsWriter 的 JSON 日志，做简单断言，输出通过/失败摘要。
/// </summary>
public static class SelfTest
{
    private static string s_sceneName;
    private static int s_targetFrames;
    private static int s_warmupFrames = 10;
    private static int s_exitCode;
    private static Tracing s_tracing;

    public static void Run()
    {
        s_sceneName = GetArg("-sceneName") ?? "CornellBox";
        s_targetFrames = int.TryParse(GetArg("-frameCount"), out int f) ? f : 60;
        s_exitCode = 0;

        Debug.Log($"[SelfTest] 开始: scene={s_sceneName} frames={s_targetFrames}");

        // 加载场景
        string scenePath = $"Assets/Scenes/{s_sceneName}.unity";
        if (!File.Exists(scenePath))
        {
            // 可能在子目录
            var found = AssetDatabase.FindAssets($"{s_sceneName} t:Scene");
            if (found.Length > 0)
                scenePath = AssetDatabase.GUIDToAssetPath(found[0]);
            else
            {
                Fail($"场景未找到: {s_sceneName}");
                return;
            }
        }

        EditorSceneManager.OpenScene(scenePath);

        // 找 Tracing 组件
        s_tracing = Object.FindAnyObjectByType<Tracing>();
        if (s_tracing == null)
        {
            Fail("场景中未找到 Tracing 组件");
            return;
        }

        // 设置 FrameLimit（private 字段，用反射）
        var frameLimitField = typeof(Tracing).GetField("FrameLimit",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        frameLimitField?.SetValue(s_tracing, s_targetFrames + s_warmupFrames);
        DiagnosticsWriter.EnableFileOutput = true;
        DiagnosticsWriter.EnableConsoleOutput = true;
        DiagnosticsWriter.CloseCurrentFile();

        // 进入 Play Mode，在下一帧开始监控
        EditorApplication.update += OnEditorUpdate;
        EditorApplication.isPlaying = true;
    }

    private static void OnEditorUpdate()
    {
        if (s_tracing == null || !EditorApplication.isPlaying)
        {
            // Play Mode 退出了
            EditorApplication.update -= OnEditorUpdate;
            FinalizeTest();
            return;
        }

        int currentFrame = GetSampleCount(s_tracing);

        if (currentFrame >= s_targetFrames + s_warmupFrames)
        {
            EditorApplication.isPlaying = false;
            EditorApplication.update -= OnEditorUpdate;
        }
    }

    private static void FinalizeTest()
    {
        // 等一小段时间让 DiagnosticsWriter 写完文件
        DiagnosticsWriter.CloseCurrentFile();

        string outputDir = Path.Combine(Application.dataPath, "..", DiagnosticsWriter.OutputDirectory);
        if (!Directory.Exists(outputDir))
        {
            Fail($"输出目录不存在: {outputDir}");
            EditorApplication.Exit(s_exitCode);
            return;
        }

        var jsonFiles = Directory.GetFiles(outputDir, $"{s_sceneName}*.json")
            .OrderByDescending(f => f)
            .ToArray();

        if (jsonFiles.Length == 0)
        {
            Fail("未找到诊断 JSON 文件");
            EditorApplication.Exit(s_exitCode);
            return;
        }

        string latestJson = jsonFiles[0];
        string[] lines = File.ReadAllText(latestJson).TrimEnd('\n').Split('\n');

        if (lines.Length < s_warmupFrames)
        {
            Fail($"JSON 只有 {lines.Length} 帧数据，不足 warmup={s_warmupFrames} 帧");
            EditorApplication.Exit(s_exitCode);
            return;
        }

        // 取最后 10 帧做断言
        var lastLines = lines.Skip(System.Math.Max(0, lines.Length - 10)).ToArray();
        var summary = new System.Text.StringBuilder();
        summary.AppendLine($"=== SelfTest Summary ===");
        summary.AppendLine($"Scene: {s_sceneName}");
        summary.AppendLine($"Target Frames: {s_targetFrames}");
        summary.AppendLine($"Actual Frames in JSON: {lines.Length}");
        summary.AppendLine($"Warmup Skipped: {s_warmupFrames}");
        summary.AppendLine($"Output: {latestJson}");
        summary.AppendLine();

        // 断言每条有效帧
        bool allPassed = true;
        int testedFrames = 0;
        foreach (string line in lastLines)
        {
            int frameIdx = ExtractInt(line, "frameIndex");
            int sampleCount = ExtractInt(line, "sampleCount");
            int primaryHit = ExtractIntFromSlots(line, 0, 0);   // slot 0, comp 0 = PrimaryHit
            float targetLum = ExtractFloatFromSlots(line, 1, 0); // slot 1, comp 0 = targetLum
            float payloadValid = ExtractFloatFromSlots(line, 6, 1); // slot 6, comp 1 = payloadValid

            // 只检测渲染已稳定的帧
            if (sampleCount < s_warmupFrames) continue;
            testedFrames++;

            bool framePassed = true;
            string errors = "";

            if (primaryHit == 0) { framePassed = false; errors += " PrimaryHit=0"; }
            if (targetLum <= 0) { framePassed = false; errors += $" targetLum={targetLum}"; }
            if (payloadValid < 0.5f && primaryHit == 1) { framePassed = false; errors += $" payloadValid={payloadValid}"; }

            // NaN 检查
            if (float.IsNaN(targetLum)) { framePassed = false; errors += " targetLum=NaN"; }

            string status = framePassed ? "PASS" : "FAIL";
            summary.AppendLine($"{status} frame={frameIdx} sampleCount={sampleCount} primaryHit={primaryHit} targetLum={targetLum:F4} payloadValid={payloadValid} {errors}");

            if (!framePassed) allPassed = false;
        }

        if (testedFrames == 0)
        {
            summary.AppendLine("FAIL: 没有足够的稳定帧用于测试");
            allPassed = false;
        }

        summary.AppendLine();
        summary.AppendLine(allPassed ? "RESULT: PASS" : "RESULT: FAIL");

        string summaryPath = Path.Combine(outputDir, $"{s_sceneName}_summary.txt");
        File.WriteAllText(summaryPath, summary.ToString());

        Debug.Log(summary.ToString());

        if (!allPassed) s_exitCode = 1;

        EditorApplication.Exit(s_exitCode);
    }

    // --- 辅助方法 ---

    private static int GetSampleCount(Tracing t)
    {
        // 用反射或 public 属性读取
        var field = typeof(Tracing).GetField("sampleCount",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        return (int)(field?.GetValue(t) ?? 0);
    }

    private static int ExtractInt(string jsonLine, string key)
    {
        var match = Regex.Match(jsonLine, $"\"{key}\":(-?\\d+)");
        return match.Success ? int.Parse(match.Groups[1].Value) : -1;
    }

    private static int ExtractIntFromSlots(string jsonLine, int slotIndex, int componentIndex)
    {
        // slots:[[x,y,z,w],[x,y,z,w],...]  取 slots[slotIndex][componentIndex]
        var slotsMatch = Regex.Match(jsonLine, "\"slots\":\\[(.+)\\]");
        if (!slotsMatch.Success) return -1;

        string allSlots = slotsMatch.Groups[1].Value;
        var slotMatches = Regex.Matches(allSlots, "\\[([^\\]]+)\\]");
        if (slotIndex >= slotMatches.Count) return -1;

        string comps = slotMatches[slotIndex].Groups[1].Value;
        var vals = comps.Split(',');
        if (componentIndex >= vals.Length) return -1;

        return (int)float.Parse(vals[componentIndex].Trim(),
            System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture);
    }

    private static float ExtractFloatFromSlots(string jsonLine, int slotIndex, int componentIndex)
    {
        var slotsMatch = Regex.Match(jsonLine, "\"slots\":\\[(.+)\\]");
        if (!slotsMatch.Success) return float.NaN;

        string allSlots = slotsMatch.Groups[1].Value;
        var slotMatches = Regex.Matches(allSlots, "\\[([^\\]]+)\\]");
        if (slotIndex >= slotMatches.Count) return float.NaN;

        string comps = slotMatches[slotIndex].Groups[1].Value;
        var vals = comps.Split(',');
        if (componentIndex >= vals.Length) return float.NaN;

        return float.Parse(vals[componentIndex].Trim(),
            System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture);
    }

    private static string GetArg(string name)
    {
        var args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i].ToLower() == name.ToLower())
                return args[i + 1];
        }
        return null;
    }

    private static void Fail(string reason)
    {
        s_exitCode = 1;
        Debug.LogError($"[SelfTest] FAIL: {reason}");
    }
}
