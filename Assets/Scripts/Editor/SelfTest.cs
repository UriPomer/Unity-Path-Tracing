using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Text.RegularExpressions;

/// <summary>
/// ReSTIR GI 日志校验与历史自测试入口。
/// 当前推荐流程是 Editor 里手动 Play，结束后由工具读取最新的 Tools/Output/<timestamp>/ 日志目录执行断言。
/// </summary>
public static class SelfTest
{
    [Serializable]
    private sealed class TelemetrySessionRow
    {
        public string sessionId;
        public string @event;
        public int acceptedCaptures;
        public int readbackErrors;
        public bool useReSTIRDI;
        public bool useReSTIRGI;
        public bool denoise;
    }

    [Serializable]
    private sealed class TelemetryStatsRow
    {
        public string sessionId;
        public int frameIndex;
        public int sampleCount;
        public int generation;
        public int renderWidth;
        public int renderHeight;
        public int modeFlags;
        public int criticalNonFinite;
        public int criticalOutOfRange;
        public int criticalBufferContract;
    }

    [Serializable]
    private sealed class TelemetryStageRow
    {
        public string sessionId;
        public int frameIndex;
        public int sampleCount;
        public int generation;
        public string stage;
        public string severity;
        public int pixelIndex;
        public float[] payload;
    }

    [Serializable]
    private sealed class TelemetryEventRow
    {
        public string sessionId;
        public string severity;
    }

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
    // Failure routing: batchmode wants exit(1) on Fail(), the editor menu wants
    // a dialog + Console error so the editor stays alive. Default to ExitOnFail
    // for backwards compatibility with the existing Run() batchmode entrypoint.
    private enum FailMode { ExitOnFail, LogOnFail }
    private static FailMode s_failMode = FailMode.ExitOnFail;
    private static string s_lastFailReason;

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
        {
            if (TryUseLatestReSTIRGILogPaths(out string latestDir))
            {
                Debug.Log($"[SelfTest] Validating GI logs in {latestDir}");
                ValidateLatestReSTIRLogs();
            }
            else
            {
                Fail("No Tools/Output/<timestamp>/ subdirectory found after batchmode run");
            }
        }

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
        SetPrivateField(tracing, "UseReSTIRDI", true);
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
            // 2026-06-01: a temporal row with weightSum==0 means the kernel selected a
            // prev candidate but reevaluation collapsed it to an empty reservoir. That
            // pixel never contributes to GlobalColors, so it is not a regression even
            // if the corresponding probe ends up classified non-reusable. Only flag
            // probes whose temporal kernel actually wrote a non-zero reservoir.
            float weightSum = ExtractFloat(line, "weightSum");
            if (!IsFinitePositive(weightSum))
                return false;

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

        // Hard upper-bound sweep FIRST: a regression that produces 100% runaway frames
        // would still pass the Any() check below (no valid frame -> "never reported"
        // message) and the engineer would chase the wrong cause. Run the sweep up front
        // so any out-of-bound entry surfaces with the correct firefly diagnostic.
        string runawayTemporalLine = temporalSummaryLines.FirstOrDefault(line =>
            ExtractFloat(line, "weightSum") > MaxValidGIRISWeight ||
            ExtractFloat(line, "selectedWeight") > MaxValidGIRISWeight);
        if (runawayTemporalLine != null)
        {
            Fail($"GI temporal weightSum/selectedWeight exceeded firefly bound (>1e6): {runawayTemporalLine}");
            return;
        }

        bool sawValidTemporalSummary = temporalSummaryLines.Any(line =>
            IsFinitePositive(ExtractFloat(line, "proposalPdf")) &&
            IsFinitePositive(ExtractFloat(line, "targetLum")) &&
            IsFinitePositiveBounded(ExtractFloat(line, "weightSum"), MaxValidGIRISWeight) &&
            IsFinitePositiveBounded(ExtractFloat(line, "selectedWeight"), MaxValidGIRISWeight) &&
            IsFinitePositive(ExtractFloat(line, "sampleCountM")));
        if (!sawValidTemporalSummary)
        {
            Fail("GI temporal diagnostics never reported a valid temporal reservoir summary (with weightSum<=1e6, selectedWeight<=1e6 bound)");
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
            // Skip rows where the final pipeline produced zero contribution: those
            // pixels never feed GlobalColors so they are not regressions even if
            // the probe is non-reusable. See temporal block above for full rationale.
            if (!ExtractBool(line, "finalContributionPositive"))
                return false;

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

        // Sweep first; see temporal-summary block above for why ordering matters.
        string runawayFinalLine = finalSummaryLines.FirstOrDefault(line =>
            ExtractFloat(line, "finalContributionLum") > MaxValidGIFinalContributionLum);
        if (runawayFinalLine != null)
        {
            Fail($"GI finalContributionLum exceeded firefly bound (>1e4): {runawayFinalLine}");
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

        // Sweep first; same ordering rationale as temporal/final blocks above.
        // spatialNormalizationDenominator = selectedTargetPdf * piSum, where piSum
        // sums targetPdf*M across up to 8 neighbors. With the M-cap fence in
        // gi_spatial.hlsl, this stays well below 1e8; without it the 2026-05-31
        // Sponza regression hit 7.3e6 with 7/8 active neighbors and 36k piSum.
        string runawaySpatialLine = spatialSummaryLines.FirstOrDefault(line =>
            ExtractFloat(line, "spatialNormalizationDenominator") > MaxValidGISpatialDenom);
        if (runawaySpatialLine != null)
        {
            Fail($"GI spatial normalizationDenominator exceeded firefly bound (>1e8): {runawaySpatialLine}");
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
            // Skip rows where spatial selected nothing or pi was zero — those pixels
            // pass through the bias correction without contributing energy. Same
            // rationale as the temporal/final relaxations above. Use selectedTargetPdf
            // (the pdf at the selected sample) rather than spatialPi: the latter falls
            // back to currentTargetPdf when no neighbor was picked, which can be
            // non-zero even when no contribution flowed through (frame 145 case).
            float selectedTargetPdf = ExtractFloat(line, "spatialSelectedTargetPdf");
            if (!IsFinitePositive(selectedTargetPdf))
                return false;

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

        if (!IsFinitePositiveBounded(weightSum, MaxValidGIRISWeight))
        {
            Fail($"GI weightSum out of bound (must be 0<x<=1e6): {weightSum.ToString("R", CultureInfo.InvariantCulture)}");
            return;
        }

        if (!IsFinitePositiveBounded(selectedWeight, MaxValidGIRISWeight))
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

    // Mirrors RESTIR_GI_FINITE_LIMIT (gi_reservoir.hlsl). HLSL only enforces this on
    // selectedWeight via IsIndirectReservoirValid; the C# test extends the bound to
    // weightSum as well for stricter regression detection. Both reservoir scalars share
    // the same bound because in the post-fix RTXDI semantics selectedWeight mirrors
    // weightSum (Task 3).
    private const float MaxValidGIRISWeight = 1e6f;
    private const float MaxValidGIFinalContributionLum = 1e4f;
    // Spatial denom = selectedTargetPdf * piSum (piSum aggregates targetPdf*M over 8 neighbors).
    // With M-capped to RESTIR_GI_MAX_RESERVOIR_SAMPLES, the product is naturally one
    // order of magnitude looser than per-reservoir weightSum/selectedWeight (1e6).
    private const float MaxValidGISpatialDenom = 1e8f;

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

    private static void ValidateLatestReSTIRLogs()
    {
        string outputDirectory = Path.GetDirectoryName(s_giProbePath);
        string sessionPath = outputDirectory == null
            ? null
            : Path.Combine(outputDirectory, "restir_session.jsonl");
        if (!string.IsNullOrEmpty(sessionPath) && File.Exists(sessionPath))
        {
            ValidateTelemetrySession(outputDirectory, sessionPath);
            return;
        }

        ValidateReSTIRGIProbe();
    }

    private static void ValidateTelemetrySession(string outputDirectory, string sessionPath)
    {
        TelemetrySessionRow[] sessionRows = ReadJsonLines<TelemetrySessionRow>(sessionPath);
        TelemetrySessionRow start = sessionRows.FirstOrDefault(row => row.@event == "session_start");
        TelemetrySessionRow end = sessionRows.LastOrDefault(row => row.@event == "session_end");
        if (start == null || end == null || string.IsNullOrEmpty(start.sessionId) || start.sessionId != end.sessionId)
        {
            Fail("Telemetry session is missing correlated session_start/session_end rows");
            return;
        }

        if (!start.useReSTIRDI || !start.useReSTIRGI || start.denoise)
        {
            Fail("Runtime proof requires useReSTIRDI=true, useReSTIRGI=true, denoise=false");
            return;
        }

        if (end.acceptedCaptures <= 0 || end.readbackErrors != 0)
        {
            Fail($"Telemetry session ended with acceptedCaptures={end.acceptedCaptures}, readbackErrors={end.readbackErrors}");
            return;
        }

        TelemetryStatsRow[] stats = ReadJsonLines<TelemetryStatsRow>(
            Path.Combine(outputDirectory, "restir_telemetry_stats.jsonl"));
        if (stats.Length == 0 || stats.Any(row =>
                row.sessionId != start.sessionId || row.frameIndex < 0 || row.sampleCount < 0 ||
                row.generation <= 0 || row.renderWidth <= 0 || row.renderHeight <= 0 ||
                (row.modeFlags & 3) != 3 || (row.modeFlags & 16) != 0))
        {
            Fail("Telemetry stats are missing or contain invalid correlation fields");
            return;
        }

        TelemetryStatsRow critical = stats.FirstOrDefault(row =>
            row.criticalNonFinite != 0 || row.criticalOutOfRange != 0 || row.criticalBufferContract != 0);
        if (critical != null)
        {
            Fail($"Telemetry critical counter failed at frame {critical.frameIndex}");
            return;
        }

        TelemetryEventRow[] events = ReadJsonLines<TelemetryEventRow>(
            Path.Combine(outputDirectory, "restir_events.jsonl"));
        if (events.Any(row => row.sessionId != start.sessionId || row.severity == "error"))
        {
            Fail("Telemetry events contain an error or mismatched session ID");
            return;
        }

        if (!stats.Any(row => (row.modeFlags & 2) != 0))
            return;

        ValidateTelemetryDIFile(outputDirectory, start.sessionId);
        ValidateTelemetryStageFile(outputDirectory, "restir_gi_probe.jsonl", start.sessionId, "gi_initial");
        ValidateTelemetryStageFile(outputDirectory, "restir_gi_final_stats.jsonl", start.sessionId, "gi_final");
    }

    private static void ValidateTelemetryDIFile(string outputDirectory, string sessionId)
    {
        string path = Path.Combine(outputDirectory, "restir_di_stats.jsonl");
        TelemetryStageRow[] rows = ReadJsonLines<TelemetryStageRow>(path);
        bool hasInitial = rows.Any(row => row.stage == "di_initial");
        bool hasTemporal = rows.Any(row => row.stage == "di_temporal");
        bool hasShade = rows.Any(row => row.stage == "di_shade");
        bool invalid = rows.Any(row =>
            row.sessionId != sessionId ||
            (row.stage != "di_initial" && row.stage != "di_temporal" && row.stage != "di_shade") ||
            row.frameIndex < 0 || row.sampleCount < 0 || row.generation <= 0 || row.pixelIndex < 0 ||
            row.payload == null || row.payload.Length != ReSTIRTelemetryLayout.RecordPayloadWordCount ||
            row.payload.Any(value => float.IsNaN(value) || float.IsInfinity(value)) ||
            row.severity == "error");
        if (!hasInitial || !hasTemporal || !hasShade || invalid)
            Fail("DI telemetry stage file is missing initial/temporal/shade rows or contains invalid data");
    }

    private static void ValidateTelemetryStageFile(
        string outputDirectory,
        string fileName,
        string sessionId,
        string expectedStage)
    {
        if (s_exitCode != 0)
            return;

        TelemetryStageRow[] rows = ReadJsonLines<TelemetryStageRow>(Path.Combine(outputDirectory, fileName));
        bool invalid = rows.Length == 0 || rows.Any(row =>
            row.sessionId != sessionId || row.stage != expectedStage || row.frameIndex < 0 ||
            row.sampleCount < 0 || row.generation <= 0 || row.pixelIndex < 0 ||
            row.payload == null || row.payload.Length != ReSTIRTelemetryLayout.RecordPayloadWordCount ||
            row.payload.Any(value => float.IsNaN(value) || float.IsInfinity(value)) ||
            row.severity == "error");
        if (invalid)
            Fail($"Telemetry stage file is missing or invalid: {fileName}");
    }

    private static T[] ReadJsonLines<T>(string path) where T : class
    {
        if (!File.Exists(path))
            return Array.Empty<T>();

        return File.ReadLines(path)
            .Where(line => !string.IsNullOrWhiteSpace(line))
            .Select(JsonUtility.FromJson<T>)
            .Where(row => row != null)
            .ToArray();
    }

    private static void Fail(string reason)
    {
        s_exitCode = 1;
        s_lastFailReason = reason;
        // SessionState is only meaningful for the batchmode Run() path, which
        // survives domain reloads. The editor menu runs synchronously inside
        // one editor frame, so writing to SessionState there would just leak
        // state into a future batchmode run.
        if (s_failMode == FailMode.ExitOnFail)
            SessionState.SetInt(SessionKeyExitCode, s_exitCode);
        Debug.LogError($"[SelfTest] FAIL: {reason}");
    }

    public static bool VerifyLatestGILogs(out string report)
    {
        s_failMode = FailMode.LogOnFail;
        s_exitCode = 0;
        s_lastFailReason = null;

        if (!TryUseLatestReSTIRGILogPaths(out string latestDir))
        {
            string msg = "No Tools/Output/<timestamp>/ subdirectory found. Play the scene with WriteReSTIRGIDiagnostics enabled, then verify the latest logs.";
            report = $"FAIL: {msg}";
            Debug.LogError($"[SelfTest] {msg}");
            s_failMode = FailMode.ExitOnFail;
            return false;
        }

        s_sceneName = "(editor-play)";
        s_enableReSTIRGI = true;

        Debug.Log($"[SelfTest] Verifying GI logs in {latestDir}");
        ValidateLatestReSTIRLogs();

        report = s_exitCode == 0 ? $"PASS\n\nLogs verified: {latestDir}" : $"FAIL: {s_lastFailReason}\n\nLogs: {latestDir}";
        if (s_exitCode == 0)
            Debug.Log($"[SelfTest] PASS — {latestDir}");
        else
            Debug.LogError($"[SelfTest] {report}");

        // Restore default so any subsequent batchmode Run() in the same editor
        // session still exits on failure as designed.
        s_failMode = FailMode.ExitOnFail;
        return s_exitCode == 0;
    }

    public static bool VerifyLatestGILogsNonInteractive(out string report)
    {
        return VerifyLatestGILogs(out report);
    }

    private static bool TryUseLatestReSTIRGILogPaths(out string latestDir)
    {
        latestDir = FindLatestDiagnosticOutputDir();
        if (latestDir == null)
            return false;

        s_giProbePath = Path.Combine(latestDir, "restir_gi_probe.jsonl");
        s_giTemporalStatsPath = Path.Combine(latestDir, "restir_gi_temporal_stats.jsonl");
        s_giFinalStatsPath = Path.Combine(latestDir, "restir_gi_final_stats.jsonl");
        s_giSpatialStatsPath = Path.Combine(latestDir, "restir_gi_spatial_stats.jsonl");
        return true;
    }

    // Editor-only convenience entrypoint that surfaces the same latest-log
    // verification result in a modal dialog for manual use.
    [MenuItem("Tools/Verify GI Logs")]
    public static void VerifyGILogsFromEditor()
    {
        VerifyLatestGILogs(out string report);
        EditorUtility.DisplayDialog("Verify GI Logs", report, "OK");
    }

    private static string FindLatestDiagnosticOutputDir()
    {
        string rootOutputDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Output"));
        if (!Directory.Exists(rootOutputDir))
            return null;

        // Tracing.GetDiagnosticOutputPath uses "yyyy-MM-dd_HHmmss" with optional
        // "_N" collision suffix. Lexicographic sort matches chronological order
        // for the timestamp portion; collision suffixes preserve relative order
        // because they are appended to identical timestamps.
        var subdirs = Directory.GetDirectories(rootOutputDir);
        if (subdirs.Length == 0)
            return null;

        Array.Sort(subdirs, StringComparer.Ordinal);
        // Walk from most-recent to oldest and pick the first one that actually
        // contains the GI probe log; older runs might be empty or partial.
        for (int i = subdirs.Length - 1; i >= 0; i--)
        {
            if (File.Exists(Path.Combine(subdirs[i], "restir_gi_probe.jsonl")))
                return subdirs[i];
        }
        return null;
    }
}
