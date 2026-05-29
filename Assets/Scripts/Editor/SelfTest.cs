using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Text.RegularExpressions;

/// <summary>
/// Batchmode 自测试入口。
/// 使用方式：Unity.exe -batchmode -projectPath ... -executeMethod SelfTest.Run -sceneName CornellBox -frameCount 60
/// 渲染 N 帧后读取 DiagnosticsWriter 的 JSON 日志，做简单断言，输出通过/失败摘要。
/// </summary>
public static class SelfTest
{
    private const string SessionKeyActive = "SelfTest.Active";
    private const string SessionKeySceneName = "SelfTest.SceneName";
    private const string SessionKeyTargetFrames = "SelfTest.TargetFrames";
    private const string SessionKeyWarmupFrames = "SelfTest.WarmupFrames";
    private const string SessionKeyEnableReSTIRGI = "SelfTest.EnableReSTIRGI";
    private const string SessionKeyGiProbePath = "SelfTest.GIProbePath";
    private const string SessionKeyGiTemporalStatsPath = "SelfTest.GITemporalStatsPath";
    private const string SessionKeyGiFinalStatsPath = "SelfTest.GIFinalStatsPath";
    private const string SessionKeyGiSpatialStatsPath = "SelfTest.GISpatialStatsPath";
    private const string SessionKeyExitCode = "SelfTest.ExitCode";
    private const string SessionKeyStartTicks = "SelfTest.StartTicks";
    private const string SessionKeyLastSampleCount = "SelfTest.LastSampleCount";
    private const string SessionKeyLogPath = "SelfTest.LogPath";

    private static string s_sceneName;
    private static int s_targetFrames;
    private static int s_warmupFrames = 10;
    private static int s_exitCode;
    private static Tracing s_tracing;
    private static string s_giProbePath;
    private static string s_giTemporalStatsPath;
    private static string s_giFinalStatsPath;
    private static string s_giSpatialStatsPath;
    private static bool s_enableReSTIRGI;
    private static long s_startTicks;
    private static int s_lastSampleCount;
    private static string s_logPath;

    [InitializeOnLoadMethod]
    private static void Initialize()
    {
        EditorApplication.update -= OnEditorUpdate;
        EditorApplication.update += OnEditorUpdate;
    }

    public static void Run()
    {
        s_sceneName = GetArg("-sceneName") ?? "CornellBox";
        s_targetFrames = int.TryParse(GetArg("-frameCount"), out int f) ? f : 60;
        s_enableReSTIRGI = GetBoolArg("-useRestirGI", true);
        s_exitCode = 0;
        s_giProbePath = Path.GetFullPath(Path.Combine("Tools", "Output", "restir_gi_probe.jsonl"));
        s_giTemporalStatsPath = Path.GetFullPath(Path.Combine("Tools", "Output", "restir_gi_temporal_stats.jsonl"));
        s_giFinalStatsPath = Path.GetFullPath(Path.Combine("Tools", "Output", "restir_gi_final_stats.jsonl"));
        s_giSpatialStatsPath = Path.GetFullPath(Path.Combine("Tools", "Output", "restir_gi_spatial_stats.jsonl"));
        s_logPath = GetArg("-logFile");
        s_startTicks = System.DateTime.UtcNow.Ticks;
        s_lastSampleCount = -1;
        PersistState();

        Debug.Log($"[SelfTest] 开始: scene={s_sceneName} frames={s_targetFrames} useReSTIRGI={s_enableReSTIRGI}");

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
        s_tracing = UnityEngine.Object.FindAnyObjectByType<Tracing>();
        if (s_tracing == null)
        {
            Fail("场景中未找到 Tracing 组件");
            return;
        }

        PrepareReSTIRGIValidation(s_tracing);

        // 设置 FrameLimit（private 字段，用反射）
        var frameLimitField = typeof(Tracing).GetField("FrameLimit",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        frameLimitField?.SetValue(s_tracing, s_targetFrames + s_warmupFrames);

        // 进入 Play Mode，在下一帧开始监控
        EditorApplication.isPlaying = true;
    }

    private static void OnEditorUpdate()
    {
        if (!SessionState.GetBool(SessionKeyActive, false))
            return;

        RestoreState();

        if (s_tracing == null || !EditorApplication.isPlaying)
        {
            if (EditorApplication.isPlaying)
                s_tracing = UnityEngine.Object.FindAnyObjectByType<Tracing>();

            if (s_tracing == null || !EditorApplication.isPlaying)
                FinalizeTest();
            return;
        }

        ForceCameraRenderIfNeeded();

        int currentFrame = GetSampleCount(s_tracing);
        if (currentFrame != s_lastSampleCount)
        {
            s_lastSampleCount = currentFrame;
            SessionState.SetInt(SessionKeyLastSampleCount, s_lastSampleCount);
            Debug.Log($"[SelfTest] sampleCount={currentFrame}");
        }

        if (GetElapsedSeconds() > 60.0)
        {
            Fail($"Timed out waiting for sampleCount to reach {s_targetFrames + s_warmupFrames}, current={currentFrame}");
            EditorApplication.isPlaying = false;
            return;
        }

        if (currentFrame >= s_targetFrames + s_warmupFrames)
        {
            EditorApplication.isPlaying = false;
        }
    }

    private static void FinalizeTest()
    {
        if (s_exitCode == 0 && s_enableReSTIRGI)
            ValidateReSTIRGIProbe();

        Debug.Log($"[SelfTest] 完成: scene={s_sceneName} frames={s_targetFrames}");
        SessionState.EraseBool(SessionKeyActive);
        SessionState.EraseString(SessionKeySceneName);
        SessionState.EraseInt(SessionKeyTargetFrames);
        SessionState.EraseInt(SessionKeyWarmupFrames);
        SessionState.EraseBool(SessionKeyEnableReSTIRGI);
        SessionState.EraseString(SessionKeyGiProbePath);
        SessionState.EraseString(SessionKeyGiTemporalStatsPath);
        SessionState.EraseString(SessionKeyGiFinalStatsPath);
        SessionState.EraseString(SessionKeyGiSpatialStatsPath);
        SessionState.EraseInt(SessionKeyExitCode);
        SessionState.EraseString(SessionKeyStartTicks);
        SessionState.EraseInt(SessionKeyLastSampleCount);
        SessionState.EraseString(SessionKeyLogPath);
        EditorApplication.Exit(s_exitCode);
    }

    private static void PrepareReSTIRGIValidation(Tracing tracing)
    {
        if (!s_enableReSTIRGI)
            return;

        DeleteIfExists(s_giProbePath);
        DeleteIfExists(s_giTemporalStatsPath);
        DeleteIfExists(s_giFinalStatsPath);
        DeleteIfExists(s_giSpatialStatsPath);
        SetPrivateField(tracing, "UseReSTIRGI", true);
        SetPrivateField(tracing, "WriteReSTIRGIDiagnostics", true);
        SetPrivateField(tracing, "WriteReSTIRGIDiagnosticDetails", false);
        SetPrivateField(tracing, "ReSTIRGIDiagnosticFrameInterval", 1);
        SetPrivateField(tracing, "Denoise", false);
        SetPrivateField(tracing, "FrameLimit", s_targetFrames + s_warmupFrames);
    }

    private static void ValidateReSTIRGIProbe()
    {
        if (!File.Exists(s_giProbePath))
        {
            Fail($"GI probe file not found: {s_giProbePath}");
            return;
        }

        string[] lines = File.ReadAllLines(s_giProbePath)
            .Where(line => !string.IsNullOrWhiteSpace(line))
            .ToArray();
        if (lines.Length == 0)
        {
            Fail("GI probe file is empty");
            return;
        }

        string[] primaryHitLines = lines.Where(line => ExtractBool(line, "primaryHit")).ToArray();
        if (primaryHitLines.Length == 0)
        {
            Fail("No GI probe sample hit a primary surface");
            return;
        }

        bool sawReusableProbe = primaryHitLines.Any(line => ExtractString(line, "probeClass") == "reservoir_reusable");
        if (!sawReusableProbe)
        {
            Fail("GI probe file never reported a reservoir_reusable probe on a primary hit");
            return;
        }

        bool sawReusableActiveProbe = primaryHitLines.Any(line =>
            ExtractString(line, "probeClass") == "reservoir_reusable" &&
            ExtractBool(line, "activeValid"));
        if (!sawReusableActiveProbe)
        {
            Fail("Reusable GI probe never produced an active reservoir; expected temporal/spatial GI reuse to feed the active pipeline");
            return;
        }

        if (SceneRequiresMirrorBypassSemantics())
        {
            bool sawBypassProbe = primaryHitLines.Any(line => ExtractString(line, "probeClass") == "bypass");
            if (!sawBypassProbe)
            {
                Fail("GI probe file never reported a bypass probe on a primary hit for a mirror-bypass validation scene");
                return;
            }

            bool centerProbeRegressedToInvalidStage1 = primaryHitLines.Any(line =>
                ExtractInt(line, "probeId") == 0 &&
                ExtractString(line, "probeClass") == "invalid_stage1");
            if (centerProbeRegressedToInvalidStage1)
            {
                Fail("Center GI probe regressed to invalid_stage1; expected reference-style bypass behavior for the mirror probe");
                return;
            }

            bool sawPerfectMetalBypassProbe = primaryHitLines.Any(line =>
                ExtractInt(line, "probeId") == 2 &&
                ExtractString(line, "probeClass") == "bypass");
            if (!sawPerfectMetalBypassProbe)
            {
                Fail("Perfect-metal GI probe never reported bypass semantics; expected reference-style non-reusable mirror behavior");
                return;
            }
        }

        string[] ggxBackfacingInvalidLines = primaryHitLines
            .Where(line => ExtractString(line, "stage1InvalidReason") == "ggx_specular_backfacing")
            .ToArray();
        int primaryFrameCount = primaryHitLines
            .Select(line => ExtractInt(line, "frameIndex"))
            .Distinct()
            .Count();
        int ggxBackfacingFrameCount = ggxBackfacingInvalidLines
            .Select(line => ExtractInt(line, "frameIndex"))
            .Distinct()
            .Count();
        int maxAllowedGGXBackfacingCount = Mathf.Max(8, Mathf.CeilToInt(primaryHitLines.Length * 0.05f));
        int maxAllowedGGXBackfacingFrames = Mathf.Max(8, Mathf.CeilToInt(primaryFrameCount * 0.2f));

        bool invalidGGXBackfacingLine = ggxBackfacingInvalidLines.Any(line =>
            ExtractString(line, "probeClass") != "invalid_stage1" ||
            ExtractBool(line, "initialValid") ||
            ExtractBool(line, "activeValid"));
        if (invalidGGXBackfacingLine)
        {
            Fail("GI probe reported ggx_specular_backfacing outside the expected stage1-invalid empty-reservoir path");
            return;
        }

        int maxGGXBackfacingPerFrame = primaryHitLines
            .GroupBy(line => ExtractInt(line, "frameIndex"))
            .Select(group => group.Count(line => ExtractString(line, "stage1InvalidReason") == "ggx_specular_backfacing"))
            .DefaultIfEmpty(0)
            .Max();
        int maxPrimaryHitsPerFrame = primaryHitLines
            .GroupBy(line => ExtractInt(line, "frameIndex"))
            .Select(group => group.Count())
            .DefaultIfEmpty(0)
            .Max();
        int maxAllowedGGXBackfacingPerFrame = Mathf.Max(1, Mathf.CeilToInt(maxPrimaryHitsPerFrame * 0.6f));
        if (maxGGXBackfacingPerFrame > maxAllowedGGXBackfacingPerFrame)
        {
            Fail($"GI ggx_specular_backfacing tail exceeded per-frame allowance: maxPerFrame={maxGGXBackfacingPerFrame} maxAllowedPerFrame={maxAllowedGGXBackfacingPerFrame}");
            return;
        }

        if (ggxBackfacingInvalidLines.Length > maxAllowedGGXBackfacingCount)
        {
            Fail($"GI ggx_specular_backfacing tail exceeded total allowance: invalidCount={ggxBackfacingInvalidLines.Length} maxAllowed={maxAllowedGGXBackfacingCount}");
            return;
        }

        if (ggxBackfacingFrameCount > maxAllowedGGXBackfacingFrames)
        {
            Fail($"GI ggx_specular_backfacing tail exceeded frame allowance: invalidFrames={ggxBackfacingFrameCount} maxAllowedFrames={maxAllowedGGXBackfacingFrames}");
            return;
        }

        if (!File.Exists(s_giTemporalStatsPath))
        {
            Fail($"GI temporal stats file not found: {s_giTemporalStatsPath}");
            return;
        }

        string[] temporalSummaryLines = File.ReadAllLines(s_giTemporalStatsPath)
            .Where(line => !string.IsNullOrWhiteSpace(line))
            .ToArray();
        if (temporalSummaryLines.Length == 0)
        {
            Fail("GI temporal stats file is empty");
            return;
        }

        string invalidTemporalProbeLine = temporalSummaryLines.FirstOrDefault(line =>
        {
            int selectedProbeId = ExtractInt(line, "selectedProbeId");
            int selectedFrameIndex = ExtractInt(line, "frameIndex");
            return !primaryHitLines.Any(probeLine =>
                ExtractInt(probeLine, "frameIndex") == selectedFrameIndex &&
                ExtractInt(probeLine, "probeId") == selectedProbeId &&
                ExtractString(probeLine, "probeClass") == "reservoir_reusable" &&
                ExtractBool(probeLine, "activeValid"));
        });
        if (invalidTemporalProbeLine != null)
        {
            Fail($"GI temporal diagnostics selected a probe that was not reusable+active: {invalidTemporalProbeLine}");
            return;
        }

        bool sawValidTemporalSummary = temporalSummaryLines.Any(line =>
            IsFinitePositive(ExtractFloat(line, "proposalPdf")) &&
            IsFinitePositive(ExtractFloat(line, "targetLum")) &&
            IsFinitePositiveBounded(ExtractFloat(line, "weightSum"), MaxValidGIWeightSum) &&
            IsFinitePositiveBounded(ExtractFloat(line, "selectedWeight"), MaxValidGISelectedWeight) &&
            IsFinitePositive(ExtractFloat(line, "sampleCountM")));
        if (!sawValidTemporalSummary)
        {
            Fail("GI temporal diagnostics never reported a valid temporal reservoir summary (with weightSum<=1e6, selectedWeight<=1e6 bound)");
            return;
        }

        // Hard upper-bound sweep: catch any frame where the runaway-RIS tail
        // returned (regression detector for the baseline_pre_fix scenario).
        string runawayTemporalLine = temporalSummaryLines.FirstOrDefault(line =>
            ExtractFloat(line, "weightSum") > MaxValidGIWeightSum ||
            ExtractFloat(line, "selectedWeight") > MaxValidGISelectedWeight);
        if (runawayTemporalLine != null)
        {
            Fail($"GI temporal weightSum/selectedWeight exceeded firefly bound (>1e6): {runawayTemporalLine}");
            return;
        }

        if (!File.Exists(s_giFinalStatsPath))
        {
            Fail($"GI final stats file not found: {s_giFinalStatsPath}");
            return;
        }

        string[] finalSummaryLines = File.ReadAllLines(s_giFinalStatsPath)
            .Where(line => !string.IsNullOrWhiteSpace(line))
            .ToArray();
        if (finalSummaryLines.Length == 0)
        {
            Fail("GI final stats file is empty");
            return;
        }

        string invalidFinalProbeLine = finalSummaryLines.FirstOrDefault(line =>
        {
            int selectedProbeId = ExtractInt(line, "selectedProbeId");
            int selectedFrameIndex = ExtractInt(line, "frameIndex");
            return !primaryHitLines.Any(probeLine =>
                ExtractInt(probeLine, "frameIndex") == selectedFrameIndex &&
                ExtractInt(probeLine, "probeId") == selectedProbeId &&
                ExtractString(probeLine, "probeClass") == "reservoir_reusable" &&
                ExtractBool(probeLine, "activeValid"));
        });
        if (invalidFinalProbeLine != null)
        {
            Fail($"GI final diagnostics selected a probe that was not reusable+active: {invalidFinalProbeLine}");
            return;
        }

        bool sawValidFinalSummary = finalSummaryLines.Any(line =>
            ExtractBool(line, "finalContributionFinite") &&
            ExtractBool(line, "finalContributionPositive") &&
            IsFinitePositiveBounded(ExtractFloat(line, "finalContributionLum"), MaxValidGIFinalContributionLum) &&
            IsFinitePositive(ExtractFloat(line, "globalLightDeltaLum")));
        if (!sawValidFinalSummary)
        {
            Fail("GI final diagnostics never reported a finite positive final GI contribution within bound (<=1e4)");
            return;
        }

        string runawayFinalLine = finalSummaryLines.FirstOrDefault(line =>
            ExtractFloat(line, "finalContributionLum") > MaxValidGIFinalContributionLum);
        if (runawayFinalLine != null)
        {
            Fail($"GI finalContributionLum exceeded firefly bound (>1e4): {runawayFinalLine}");
            return;
        }

        if (!File.Exists(s_giSpatialStatsPath))
        {
            Fail($"GI spatial stats file not found: {s_giSpatialStatsPath}");
            return;
        }

        string[] spatialSummaryLines = File.ReadAllLines(s_giSpatialStatsPath)
            .Where(line => !string.IsNullOrWhiteSpace(line))
            .ToArray();

        bool sawCombinedSelectedProbeNeighbors = spatialSummaryLines.Any(line =>
            ExtractFloat(line, "spatialShaderCombinedNeighbors") > 0.0f);
        if (!sawCombinedSelectedProbeNeighbors)
        {
            Fail("GI spatial diagnostics never reported a combined selected-probe neighbor set");
            return;
        }

        bool sawSelectedProbeZeroTargetFailure = spatialSummaryLines.Any(line =>
            ExtractFloat(line, "spatialShaderReevaluateFailZeroTarget") > 0.0f);
        if (sawSelectedProbeZeroTargetFailure)
        {
            Fail("GI spatial diagnostics reported zero-target failures on the selected probe");
            return;
        }

        string invalidSelectedProbeLine = spatialSummaryLines.FirstOrDefault(line =>
        {
            int selectedProbeId = ExtractInt(line, "selectedProbeId");
            int selectedFrameIndex = ExtractInt(line, "frameIndex");
            return !primaryHitLines.Any(probeLine =>
                ExtractInt(probeLine, "frameIndex") == selectedFrameIndex &&
                ExtractInt(probeLine, "probeId") == selectedProbeId &&
                ExtractString(probeLine, "probeClass") == "reservoir_reusable" &&
                ExtractBool(probeLine, "activeValid"));
        });
        if (invalidSelectedProbeLine != null)
        {
            Fail($"GI spatial diagnostics selected a probe that was not reusable+active: {invalidSelectedProbeLine}");
            return;
        }

        string validLine = primaryHitLines.LastOrDefault(line =>
            ExtractBool(line, "activeValid") || ExtractBool(line, "initialValid"));
        if (validLine == null)
        {
            string latestPrimaryHit = primaryHitLines[^1];
            float primaryDistance = ExtractFloat(latestPrimaryHit, "primaryDistance");
            int probeId = ExtractInt(latestPrimaryHit, "probeId");
            string primaryDistanceText = primaryDistance.ToString("R", CultureInfo.InvariantCulture);
            Fail($"GI probe never produced a valid reservoir on a primary hit. Latest probeId={probeId} primaryDistance={primaryDistanceText} line={latestPrimaryHit}");
            return;
        }

        bool useActiveReservoir = ExtractBool(validLine, "activeValid");
        string fieldPrefix = useActiveReservoir ? "active" : "initial";

        float proposalPdf = ExtractFloat(validLine, fieldPrefix + "ProposalPdf");
        float targetLum = ExtractFloat(validLine, fieldPrefix + "TargetLum");
        float weightSum = ExtractFloat(validLine, fieldPrefix + "WeightSum");
        float selectedWeight = ExtractFloat(validLine, fieldPrefix + "SelectedWeight");
        float sampleCountM = ExtractFloat(validLine, fieldPrefix + "SampleCountM");

        if (!IsFinitePositive(proposalPdf))
        {
            Fail($"Invalid GI proposalPdf: {proposalPdf.ToString("R", CultureInfo.InvariantCulture)}");
            return;
        }

        if (!IsFinitePositive(targetLum))
        {
            Fail($"Invalid GI targetLum: {targetLum.ToString("R", CultureInfo.InvariantCulture)}");
            return;
        }

        if (!IsFinitePositiveBounded(weightSum, MaxValidGIWeightSum))
        {
            Fail($"GI weightSum out of bound (must be 0<x<=1e6): {weightSum.ToString("R", CultureInfo.InvariantCulture)}");
            return;
        }

        if (!IsFinitePositiveBounded(selectedWeight, MaxValidGISelectedWeight))
        {
            Fail($"GI selectedWeight out of bound (must be 0<x<=1e6): {selectedWeight.ToString("R", CultureInfo.InvariantCulture)}");
            return;
        }

        if (!IsFinitePositive(sampleCountM))
        {
            Fail($"Invalid GI sampleCountM: {sampleCountM.ToString("R", CultureInfo.InvariantCulture)}");
            return;
        }

        Debug.Log($"[SelfTest] GI probe OK ({fieldPrefix}): pdf={proposalPdf.ToString("R", CultureInfo.InvariantCulture)} targetLum={targetLum.ToString("R", CultureInfo.InvariantCulture)} weightSum={weightSum.ToString("R", CultureInfo.InvariantCulture)} selectedWeight={selectedWeight.ToString("R", CultureInfo.InvariantCulture)} m={sampleCountM.ToString("R", CultureInfo.InvariantCulture)}");
    }

    private static bool SceneRequiresMirrorBypassSemantics()
    {
        return string.Equals(s_sceneName, "CornellBox", StringComparison.OrdinalIgnoreCase);
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

    private static float ExtractFloat(string jsonLine, string key)
    {
        var match = Regex.Match(jsonLine, $"\"{key}\":([-+0-9.eE]+)");
        return match.Success
            ? float.Parse(match.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture)
            : float.NaN;
    }

    private static bool ExtractBool(string jsonLine, string key)
    {
        var match = Regex.Match(jsonLine, $"\"{key}\":(true|false)");
        return match.Success && bool.Parse(match.Groups[1].Value);
    }

    private static string ExtractString(string jsonLine, string key)
    {
        var match = Regex.Match(jsonLine, $"\"{key}\":\"([^\"]*)\"");
        return match.Success ? match.Groups[1].Value : string.Empty;
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

    private static bool GetBoolArg(string name, bool defaultValue)
    {
        string value = GetArg(name);
        if (string.IsNullOrEmpty(value))
            return defaultValue;

        if (bool.TryParse(value, out bool parsed))
            return parsed;

        return defaultValue;
    }

    private static void SetPrivateField<T>(Tracing tracing, string fieldName, T value)
    {
        var field = typeof(Tracing).GetField(fieldName,
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        field?.SetValue(tracing, value);
    }

    private static void DeleteIfExists(string path)
    {
        if (File.Exists(path))
            File.Delete(path);
    }

    private const float MaxValidGIWeightSum = 1e6f;
    private const float MaxValidGISelectedWeight = 1e6f;
    private const float MaxValidGIFinalContributionLum = 1e4f;

    private static bool IsFinitePositiveBounded(float v, float upperBound)
    {
        return IsFinitePositive(v) && v <= upperBound;
    }

    private static bool IsFinitePositive(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value) && value > 0.0f;
    }

    private static void ForceCameraRenderIfNeeded()
    {
        if (s_tracing == null)
            return;

        Camera targetCamera = s_tracing.GetComponent<Camera>();
        if (targetCamera == null || !targetCamera.isActiveAndEnabled)
            return;

        targetCamera.Render();
        EditorApplication.QueuePlayerLoopUpdate();
    }

    private static double GetElapsedSeconds()
    {
        if (s_startTicks <= 0)
            return 0.0;

        return new System.TimeSpan(System.DateTime.UtcNow.Ticks - s_startTicks).TotalSeconds;
    }

    private static void PersistState()
    {
        SessionState.SetBool(SessionKeyActive, true);
        SessionState.SetString(SessionKeySceneName, s_sceneName ?? string.Empty);
        SessionState.SetInt(SessionKeyTargetFrames, s_targetFrames);
        SessionState.SetInt(SessionKeyWarmupFrames, s_warmupFrames);
        SessionState.SetBool(SessionKeyEnableReSTIRGI, s_enableReSTIRGI);
        SessionState.SetString(SessionKeyGiProbePath, s_giProbePath ?? string.Empty);
        SessionState.SetString(SessionKeyGiTemporalStatsPath, s_giTemporalStatsPath ?? string.Empty);
        SessionState.SetString(SessionKeyGiFinalStatsPath, s_giFinalStatsPath ?? string.Empty);
        SessionState.SetString(SessionKeyGiSpatialStatsPath, s_giSpatialStatsPath ?? string.Empty);
        SessionState.SetInt(SessionKeyExitCode, s_exitCode);
        SessionState.SetString(SessionKeyStartTicks, s_startTicks.ToString());
        SessionState.SetInt(SessionKeyLastSampleCount, s_lastSampleCount);
        SessionState.SetString(SessionKeyLogPath, s_logPath ?? string.Empty);
    }

    private static void RestoreState()
    {
        s_sceneName = SessionState.GetString(SessionKeySceneName, s_sceneName ?? string.Empty);
        s_targetFrames = SessionState.GetInt(SessionKeyTargetFrames, s_targetFrames);
        s_warmupFrames = SessionState.GetInt(SessionKeyWarmupFrames, s_warmupFrames);
        s_enableReSTIRGI = SessionState.GetBool(SessionKeyEnableReSTIRGI, s_enableReSTIRGI);
        s_giProbePath = SessionState.GetString(SessionKeyGiProbePath, s_giProbePath ?? string.Empty);
        s_giTemporalStatsPath = SessionState.GetString(SessionKeyGiTemporalStatsPath, s_giTemporalStatsPath ?? string.Empty);
        s_giFinalStatsPath = SessionState.GetString(SessionKeyGiFinalStatsPath, s_giFinalStatsPath ?? string.Empty);
        s_giSpatialStatsPath = SessionState.GetString(SessionKeyGiSpatialStatsPath, s_giSpatialStatsPath ?? string.Empty);
        s_exitCode = SessionState.GetInt(SessionKeyExitCode, s_exitCode);
        long.TryParse(SessionState.GetString(SessionKeyStartTicks, "0"), out s_startTicks);
        s_lastSampleCount = SessionState.GetInt(SessionKeyLastSampleCount, s_lastSampleCount);
        s_logPath = SessionState.GetString(SessionKeyLogPath, s_logPath ?? string.Empty);
    }

    private static void Fail(string reason)
    {
        s_exitCode = 1;
        SessionState.SetInt(SessionKeyExitCode, s_exitCode);
        Debug.LogError($"[SelfTest] FAIL: {reason}");
    }
}
