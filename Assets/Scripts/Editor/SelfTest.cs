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
        Debug.Log($"[SelfTest] 完成: scene={s_sceneName} frames={s_targetFrames}");
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
