using UnityEngine;
using System.IO;
using System.Text;

/// <summary>
/// 唯一诊断出口。将渲染帧的诊断数据同时输出到：
/// - Console (Debug.Log，[TT-DIAG] 前缀，给 MCP read_console)
/// - 文件 (JSON Lines 格式，给 batchmode 回归)
/// </summary>
public static class DiagnosticsWriter
{
    public static bool EnableFileOutput = true;
    public static bool EnableConsoleOutput = true;
    public static string OutputDirectory = "Tools/Output";
    public static int ConsoleOutputIntervalFrames = 30;

    private static int _frameCounter;
    private static string _currentFilePath;

    public static int CurrentFileFrameCount { get; private set; }

    public static void ResetFrameCounter()
    {
        _frameCounter = 0;
    }

    /// <summary>
    /// 写一帧的诊断数据。
    /// </summary>
    public static void WriteFrameData(
        int frameIndex,
        int sampleCount,
        Vector2Int diagnosticPixel,
        Vector4[] diagnosticsSlots,
        string sceneName,
        int renderWidth,
        int renderHeight,
        bool useRIS,
        bool useNeighborReuse,
        int bounceCount)
    {
        if (diagnosticsSlots == null || diagnosticsSlots.Length < 15) return;

        Vector4 flags = diagnosticsSlots[0];
        Vector4 firstDraw = diagnosticsSlots[6];
        float targetLum = firstDraw.w;
        float payloadValid = firstDraw.y;

        bool hasError = !(targetLum > 0 && payloadValid > 0.5f);

        // Console 输出
        if (EnableConsoleOutput)
        {
            bool shouldLog = _frameCounter == 0
                          || _frameCounter <= 5
                          || (_frameCounter % ConsoleOutputIntervalFrames == 0)
                          || hasError;

            if (shouldLog)
            {
                string prefix = hasError ? "[TT-DIAG-ERR]" : "[TT-DIAG]";
                Debug.Log($"{prefix} frame={frameIndex} targetLum={targetLum:F3} sampleCount={sampleCount} " +
                          $"currentPdf={diagnosticsSlots[3].x:F3} selectedWeight={diagnosticsSlots[9].z:F6} " +
                          $"payloadValid={(int)payloadValid} primaryHit={(int)flags.x} " +
                          $"pixel={diagnosticPixel.x},{diagnosticPixel.y}");
            }
        }

        // 文件输出
        if (EnableFileOutput)
        {
            try
            {
                if (_currentFilePath == null)
                    _currentFilePath = ResolveOutputPath(sceneName);

                string jsonLine = BuildJsonLine(frameIndex, sampleCount, diagnosticPixel,
                    diagnosticsSlots, sceneName, renderWidth, renderHeight, useRIS, useNeighborReuse, bounceCount);

                // 追加写入（不保持文件句柄，避免 Unity crash 时文件损坏）
                File.AppendAllText(_currentFilePath, jsonLine + "\n", Encoding.UTF8);
                CurrentFileFrameCount++;
            }
            catch (System.Exception ex)
            {
                // 文件出错时 fallback 到 Console
                Debug.LogWarning($"[TT-DIAG] 文件写入失败: {ex.Message}");
            }
        }

        _frameCounter++;
    }

    /// <summary>
    /// 写配置快照行。应在首帧之前调用。
    /// </summary>
    public static void WriteConfigLine(
        string sceneName,
        int renderWidth,
        int renderHeight,
        bool useRIS,
        bool useNeighborReuse,
        int bounceCount)
    {
        if (EnableConsoleOutput)
        {
            Debug.Log($"[TT-DIAG] CONFIG useRIS={useRIS} useNeighbor={useNeighborReuse} " +
                      $"bounceCount={bounceCount} scene={sceneName} res={renderWidth}x{renderHeight}");
        }
    }

    /// <summary>
    /// 关闭当前文件（写文件尾）。应在测试结束时调用。
    /// </summary>
    public static void CloseCurrentFile()
    {
        _currentFilePath = null;
        CurrentFileFrameCount = 0;
    }

    // --- private ---

    private static string ResolveOutputPath(string sceneName)
    {
        string dir = OutputDirectory;
        if (!Path.IsPathRooted(dir))
            dir = Path.Combine(Application.dataPath, "..", dir);

        try { Directory.CreateDirectory(dir); } catch { }

        string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string safeSceneName = string.Join("_", sceneName.Split(Path.GetInvalidFileNameChars()));
        return Path.Combine(dir, $"{safeSceneName}_{timestamp}.json");
    }

    private static string BuildJsonLine(
        int frameIndex, int sampleCount, Vector2Int pixel,
        Vector4[] slots, string sceneName, int w, int h,
        bool useRIS, bool useNeighborReuse, int bounceCount)
    {
        var sb = new StringBuilder();
        sb.Append('{');

        sb.Append($"\"frameIndex\":{frameIndex}");
        sb.Append($",\"sampleCount\":{sampleCount}");
        sb.Append($",\"pixelX\":{pixel.x},\"pixelY\":{pixel.y}");
        sb.Append($",\"sceneName\":\"{EscapeJson(sceneName)}\"");
        sb.Append($",\"renderWidth\":{w},\"renderHeight\":{h}");
        sb.Append($",\"useRIS\":{(useRIS ? "true" : "false")}");
        sb.Append($",\"useNeighborReuse\":{(useNeighborReuse ? "true" : "false")}");
        sb.Append($",\"bounceCount\":{bounceCount}");
        sb.Append(",\"slots\":[");

        for (int i = 0; i < slots.Length; i++)
        {
            if (i > 0) sb.Append(',');
            var s = slots[i];
            sb.Append($"[{s.x:R},{s.y:R},{s.z:R},{s.w:R}]");
        }

        sb.Append("]}");
        return sb.ToString();
    }

    private static string EscapeJson(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"");
    }
}
