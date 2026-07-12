using System;
using System.Reflection;
using System.Text.RegularExpressions;
using System.IO;
using NUnit.Framework;

public class TracingContractsTests
{
    [Test]
    public void Tracing_Exposes_ReSTIR_GI_Toggle()
    {
        FieldInfo field = TracingType().GetField("UseReSTIRGI", BindingFlags.NonPublic | BindingFlags.Instance);
        Assert.That(field, Is.Not.Null);
        Assert.That(field.FieldType, Is.EqualTo(typeof(bool)));
    }

    [Test]
    public void Tracing_Defines_IndirectReservoirStride_As_80_Bytes()
    {
        FieldInfo field = TracingType().GetField("IndirectReservoirStride", BindingFlags.NonPublic | BindingFlags.Static);
        Assert.That(field, Is.Not.Null);
        Assert.That((int)field.GetValue(null), Is.EqualTo(80));
    }

    [Test]
    public void Tracing_Exposes_ReSTIR_GI_Diagnostic_Detail_Toggle()
    {
        FieldInfo field = TracingType().GetField("WriteReSTIRGIDiagnosticDetails", BindingFlags.NonPublic | BindingFlags.Instance);
        Assert.That(field, Is.Not.Null);
        Assert.That(field.FieldType, Is.EqualTo(typeof(bool)));
    }

    [Test]
    public void Tracing_Uses_Separate_Direct_And_Indirect_History_State()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string diBody = ExtractMethodBody(source, "DispatchReSTIRDI");
        string giBody = ExtractMethodBody(source, "DispatchReSTIRGI");

        StringAssert.Contains("_lastDirectReservoirOutputIdx", diBody);
        StringAssert.Contains("_hasDirectRestirHistory", diBody);
        StringAssert.DoesNotContain("_lastIndirectReservoirOutputIdx", diBody);
        StringAssert.DoesNotContain("_hasIndirectRestirHistory", diBody);

        StringAssert.Contains("_lastIndirectReservoirOutputIdx", giBody);
        StringAssert.Contains("_hasIndirectRestirHistory", giBody);
        StringAssert.DoesNotContain("_lastDirectReservoirOutputIdx", giBody);
        StringAssert.DoesNotContain("_hasDirectRestirHistory", giBody);
    }

    [Test]
    public void Tracing_Defines_ReSTIR_GI_Probe_Output_Contract()
    {
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string telemetry = RestirShader("telemetry.hlsl");

        StringAssert.Contains("restir_gi_probe.jsonl", session);
        StringAssert.Contains("WriteStageRecord", session);
        StringAssert.Contains("frameIndex", session);
        StringAssert.Contains("sampleCount", session);
        StringAssert.Contains("generation", session);
        StringAssert.Contains("pixelIndex", session);
        StringAssert.Contains("payload", session);
        StringAssert.Contains("RESTIR_STAGE_GI_INITIAL", telemetry);
        StringAssert.Contains("RESTIR_REASON_NONFINITE_RESERVOIR", telemetry);
    }

    [Test]
    public void Tracing_Classifies_ReSTIR_GI_Probe_Sample_Types_For_Runtime_Diagnostics()
    {
        string telemetry = RestirShader("telemetry.hlsl");
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));

        StringAssert.Contains("RESTIR_REASON_PRIMARY_MISS", telemetry);
        StringAssert.Contains("RESTIR_REASON_INVALID_PROPOSAL_PDF", telemetry);
        StringAssert.Contains("RESTIR_REASON_ZERO_THROUGHPUT", telemetry);
        StringAssert.Contains("RESTIR_REASON_ZERO_TARGET", telemetry);
        StringAssert.Contains("RESTIR_REASON_NONFINITE_RESERVOIR", telemetry);
        StringAssert.Contains("ReasonName", session);
        StringAssert.Contains("SeverityName", session);
    }

    [Test]
    public void RestirGI_BXDF_Diagnostics_Differentiate_Delta_And_GGX_Zero_Throughput_Cases()
    {
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(bxdfSourcePath), Is.True, $"BXDF source not found: {bxdfSourcePath}");
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string bxdfSource = File.ReadAllText(bxdfSourcePath);
        string giInitialSource = File.ReadAllText(giInitialSourcePath);

        StringAssert.Contains("out float zeroReasonCode", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 0.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 1.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 2.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 3.0;", bxdfSource);
        StringAssert.Contains("EvaluateBXDFWithDotAndPDFDetailed(hit, ray, f_brdf, sampledSpecular, zeroReasonCode);", bxdfSource);
        StringAssert.Contains("float throughputZeroReason;", giInitialSource);
        StringAssert.Contains("data.reserved = throughputZeroReason;", giInitialSource);
    }

    [Test]
    public void RestirGI_RoughGGX_Specular_Backfacing_Remains_A_Diagnostic_Invalid_In_Stage1()
    {
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
        Assert.That(File.Exists(bxdfSourcePath), Is.True, $"BXDF source not found: {bxdfSourcePath}");

        string bxdfSource = File.ReadAllText(bxdfSourcePath);

        StringAssert.Contains("float3 H = SampleGGXVNDF(hit.normal, V, alpha, xi);", bxdfSource);
        StringAssert.Contains("rayOutDir = normalize(reflect(-V, H));", bxdfSource);
        StringAssert.Contains("float NdotL = saturate(dot(hit.normal, rayOutDir));", bxdfSource);
        StringAssert.Contains("if (NdotL <= 0.0)", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 2.0;", bxdfSource);
        StringAssert.Contains("return;", bxdfSource);
    }

    [Test]
    public void RestirGI_PerfectMetal_Opaque_Samples_Force_Specular_Lobe_Selection()
    {
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
        Assert.That(File.Exists(bxdfSourcePath), Is.True, $"BXDF source not found: {bxdfSourcePath}");

        string bxdfSource = File.ReadAllText(bxdfSourcePath);

        StringAssert.Contains("bool isPerfectMetal = hit.material.metallic >= 0.999 && hit.material.roughness < 1e-4;", bxdfSource);
        StringAssert.Contains("if (isPerfectMetal)", bxdfSource);
        StringAssert.Contains("specProb = 1.0;", bxdfSource);
        StringAssert.Contains("diffProb = 0.0;", bxdfSource);
        StringAssert.DoesNotContain("perfect_metal_specular_not_selected", bxdfSource, "Perfect-metal samples should no longer remain on the non-selected diagnostic path after forcing specular-lobe selection.");
    }

    [Test]
    public void RestirGI_Rough_Dielectrics_Bias_Opaque_Lobe_Selection_Toward_Diffuse()
    {
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
        Assert.That(File.Exists(bxdfSourcePath), Is.True, $"BXDF source not found: {bxdfSourcePath}");

        string bxdfSource = File.ReadAllText(bxdfSourcePath);

        StringAssert.Contains("if (mat.metallic < 0.001 && mat.roughness >= 0.999)", bxdfSource);
        StringAssert.Contains("specProb = 0.0;", bxdfSource);
        StringAssert.Contains("diffProb = 1.0;", bxdfSource);
        StringAssert.Contains("if (mat.metallic < 0.999)", bxdfSource);
        StringAssert.Contains("float roughDiffuseBoost = saturate(mat.roughness * mat.roughness);", bxdfSource);
        StringAssert.Contains("diffProb = lerp(diffProb, 1.0, roughDiffuseBoost);", bxdfSource);
        StringAssert.Contains("specProb = 1.0 - diffProb;", bxdfSource);
    }

    [Test]
    public void Tracing_Uses_Current_Frame_GlobalHits_For_Restir_Inputs_And_GI_Probe()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string diBody = ExtractMethodBody(source, "DispatchReSTIRDI");
        string giBody = ExtractMethodBody(source, "DispatchReSTIRGI");
        string captureBody = ExtractMethodBody(source, "BeginReSTIRTelemetryCapture");

        StringAssert.Contains("_RestirGbuffer\", _globalHits", diBody);
        StringAssert.DoesNotContain("_RestirGbuffer\", _primarySurfaceHistory", diBody);

        StringAssert.Contains("_RestirGbuffer\", _globalHits", giBody);
        StringAssert.DoesNotContain("_RestirGbuffer\", _primarySurfaceHistory", giBody);

        StringAssert.Contains("_currentRenderWidth", captureBody);
        StringAssert.Contains("_currentRenderHeight", captureBody);
        StringAssert.Contains("indirectInitialSlot", captureBody);
        StringAssert.Contains("indirectFinalSlot", captureBody);
        StringAssert.DoesNotContain(".GetData(", source);
    }

    [Test]
    public void RestirGI_Stages_Secondary_Surface_Payload_For_Shading()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        string globalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "global.hlsl"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");
        Assert.That(File.Exists(globalSourcePath), Is.True, $"Global source not found: {globalSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string giInitialSource = File.ReadAllText(giInitialSourcePath);
        string globalSource = File.ReadAllText(globalSourcePath);
        string giBody = ExtractMethodBody(tracingSource, "DispatchReSTIRGI");

        StringAssert.Contains("SetBuffer(kernelGenerateGISecondarySurfaces, \"SecondarySurfaces\", _secondarySurfaces)", giBody);
        StringAssert.Contains("SetBuffer(kernelShadeGISecondarySurfaces, \"SecondarySurfaces\", _secondarySurfaces)", giBody);
        StringAssert.Contains("struct SecondarySurfaceData", globalSource);
        StringAssert.Contains("float3 albedo;             float roughness;", globalSource);
        StringAssert.Contains("float3 emissionRadiance;   float metallic;", globalSource);
        StringAssert.Contains("data.albedo =", giInitialSource);
        StringAssert.Contains("data.roughness =", giInitialSource);
        StringAssert.Contains("data.metallic =", giInitialSource);
        StringAssert.Contains("if (!IsGISecondaryBypass(sampleFlags))", giInitialSource);
        StringAssert.Contains("data.throughput = 1.0;", giInitialSource);
        StringAssert.Contains("secondaryHit.material.albedo = secondary.albedo;", giInitialSource);
        StringAssert.Contains("secondaryHit.material.roughness = secondary.roughness;", giInitialSource);
    }

    [Test]
    public void RestirGI_Uses_Staged_Secondary_Surface_Kernels_Instead_Of_Legacy_Initial_Kernel()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string computeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "Tracing.compute"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");
        Assert.That(File.Exists(computeSourcePath), Is.True, $"Tracing compute source not found: {computeSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string computeSource = File.ReadAllText(computeSourcePath);
        string startBody = ExtractMethodBody(tracingSource, "Start");
        string giBody = ExtractMethodBody(tracingSource, "DispatchReSTIRGI");

        StringAssert.DoesNotContain("kernel_generate_gi_initial", startBody);
        StringAssert.Contains("kernel_generate_gi_secondary_surfaces", startBody);
        StringAssert.Contains("kernel_shade_gi_secondary_surfaces", startBody);
        StringAssert.DoesNotContain("#pragma kernel kernel_generate_gi_initial", computeSource);
        StringAssert.Contains("#pragma kernel kernel_generate_gi_secondary_surfaces", computeSource);
        StringAssert.Contains("#pragma kernel kernel_shade_gi_secondary_surfaces", computeSource);
        StringAssert.Contains("kernelGenerateGISecondarySurfaces", giBody);
        StringAssert.Contains("kernelShadeGISecondarySurfaces", giBody);
        StringAssert.Contains("SetBuffer(kernelShadeGISecondarySurfaces, \"_RestirGbuffer\", _globalHits)", giBody);
    }

    [Test]
    public void SelfTest_Consumes_ReSTIR_GI_Probe_Contract()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"SelfTest source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string methodBody = ExtractMethodBody(source, "ValidateReSTIRGIProbe");

        StringAssert.Contains("restir_gi_probe.jsonl", source);
        StringAssert.Contains("restir_gi_temporal_stats.jsonl", source);
        StringAssert.Contains("restir_gi_final_stats.jsonl", source);
        StringAssert.Contains("activeValid", methodBody);
        StringAssert.Contains("initialValid", methodBody);
        StringAssert.Contains("fieldPrefix + \"ProposalPdf\"", methodBody);
        StringAssert.Contains("fieldPrefix + \"TargetLum\"", methodBody);
        StringAssert.Contains("fieldPrefix + \"WeightSum\"", methodBody);
        StringAssert.Contains("fieldPrefix + \"SelectedWeight\"", methodBody);
        StringAssert.Contains("fieldPrefix + \"SampleCountM\"", methodBody);
        StringAssert.Contains("primaryHit", methodBody);
        StringAssert.Contains("probeId", methodBody);
        StringAssert.Contains("ExtractString", source);
        StringAssert.Contains("probeClass", methodBody);
        StringAssert.Contains("reservoir_reusable", methodBody);
        StringAssert.Contains("invalid_stage1", methodBody);
        StringAssert.Contains("activeValid", methodBody);
        StringAssert.Contains("Reusable GI probe never produced an active reservoir", methodBody);
        StringAssert.Contains("SceneRequiresMirrorBypassSemantics()", methodBody);
        StringAssert.Contains("string.Equals(s_sceneName, \"CornellBox\"", source);
        StringAssert.Contains("GI probe file never reported a bypass probe on a primary hit for a mirror-bypass validation scene", methodBody);
        StringAssert.Contains("Perfect-metal GI probe never reported bypass semantics", methodBody);
        StringAssert.Contains("ggx_specular_backfacing", methodBody);
        StringAssert.Contains("probeClass\") != \"invalid_stage1\"", methodBody);
        StringAssert.Contains("ExtractBool(line, \"initialValid\")", methodBody);
        StringAssert.Contains("ExtractBool(line, \"activeValid\")", methodBody);
        StringAssert.Contains("GI probe reported ggx_specular_backfacing outside the expected stage1-invalid empty-reservoir path", methodBody);
        StringAssert.Contains("maxPrimaryHitsPerFrame", methodBody);
        StringAssert.Contains("maxAllowedGGXBackfacingPerFrame", methodBody);
        StringAssert.Contains("GI ggx_specular_backfacing tail exceeded per-frame allowance", methodBody);
        StringAssert.Contains("GI ggx_specular_backfacing tail exceeded total allowance", methodBody);
        StringAssert.Contains("GI ggx_specular_backfacing tail exceeded frame allowance", methodBody);
        StringAssert.Contains("GI temporal stats file not found", methodBody);
        StringAssert.Contains("GI temporal stats file is empty", methodBody);
        StringAssert.Contains("GI temporal diagnostics selected a probe that was not reusable+active", methodBody);
        StringAssert.Contains("GI temporal diagnostics never reported a valid temporal reservoir summary", methodBody);
        StringAssert.Contains("GI final stats file not found", methodBody);
        StringAssert.Contains("GI final stats file is empty", methodBody);
        StringAssert.Contains("GI final diagnostics selected a probe that was not reusable+active", methodBody);
        StringAssert.Contains("GI final diagnostics never reported a finite positive final GI contribution", methodBody);
        StringAssert.Contains("restir_gi_spatial_stats.jsonl", source);
        StringAssert.Contains("GI spatial stats file not found", methodBody);
        StringAssert.Contains("spatialShaderCombinedNeighbors", methodBody);
        StringAssert.Contains("spatialShaderReevaluateFailZeroTarget", methodBody);
        StringAssert.Contains("selectedProbeId", methodBody);
        StringAssert.Contains("selectedFrameIndex", methodBody);
        StringAssert.Contains("ExtractInt(probeLine, \"frameIndex\") == selectedFrameIndex", methodBody);
        StringAssert.Contains("GI spatial diagnostics selected a probe that was not reusable+active", methodBody);
        StringAssert.Contains("GI spatial diagnostics never reported a combined selected-probe neighbor set", methodBody);
        StringAssert.Contains("GI spatial diagnostics reported zero-target failures on the selected probe", methodBody);
        StringAssert.DoesNotContain("File.ReadAllLines(s_logPath)", methodBody);
    }

    [Test]
    public void RestirGI_Temporal_Diagnostics_Are_Persisted_To_Independent_Jsonl()
    {
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string temporal = RestirShader("gi_temporal.hlsl");

        StringAssert.Contains("restir_gi_temporal_stats.jsonl", session);
        StringAssert.Contains("case ReSTIRTelemetryStage.GITemporal", session);
        StringAssert.Contains("RESTIR_STAGE_GI_TEMPORAL", temporal);
        StringAssert.Contains("float4(outR.radiance, outR.weightSum)", temporal);
        StringAssert.Contains("float4(outR.contribution, outR.selectedWeight)", temporal);
        StringAssert.Contains("float4(outR.primaryNormal, outR.sampleCount)", temporal);
    }

    [Test]
    public void RestirGI_Final_Diagnostics_Are_Persisted_To_Independent_Jsonl()
    {
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string finalShade = RestirShader("gi_shade.hlsl");

        StringAssert.Contains("restir_gi_final_stats.jsonl", session);
        StringAssert.Contains("case ReSTIRTelemetryStage.GIFinal", session);
        StringAssert.Contains("RESTIR_STAGE_GI_FINAL", finalShade);
        StringAssert.Contains("float4(finalRes.radiance, finalRes.weightSum)", finalShade);
        StringAssert.Contains("float4(finalRes.contribution, finalRes.selectedWeight)", finalShade);
        StringAssert.Contains("float4(gi, rawLum)", finalShade);
    }

    [Test]
    public void RestirGI_Temporal_Uses_Same_RW_Buffer_Read_Pattern_As_DI_Temporal()
    {
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        string diTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "temporal_resampling.hlsl"));
        Assert.That(File.Exists(giTemporalSourcePath), Is.True, $"GI temporal source not found: {giTemporalSourcePath}");
        Assert.That(File.Exists(diTemporalSourcePath), Is.True, $"DI temporal source not found: {diTemporalSourcePath}");

        string giTemporalSource = File.ReadAllText(giTemporalSourcePath);
        string diTemporalSource = File.ReadAllText(diTemporalSourcePath);

        StringAssert.Contains("DirectLightReservoirs[curIdx]", diTemporalSource);
        StringAssert.Contains("DirectLightReservoirs[_RestirPrevReservoirOffset + prevPxIdx]", diTemporalSource);
        StringAssert.Contains("IndirectReservoirs[curIdx]", giTemporalSource);
        StringAssert.Contains("IndirectReservoirs[_RestirPrevReservoirOffset + prevPxIdx]", giTemporalSource);
        StringAssert.DoesNotContain("IndirectReservoirsRead[curIdx]", giTemporalSource);
        StringAssert.DoesNotContain("IndirectReservoirsRead[_RestirPrevReservoirOffset + prevPxIdx]", giTemporalSource);
    }

    [Test]
    public void RestirGI_Uses_Spatial_Reuse_Pass_After_Temporal()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string computeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "Tracing.compute"));
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");
        Assert.That(File.Exists(computeSourcePath), Is.True, $"Tracing compute source not found: {computeSourcePath}");
        Assert.That(File.Exists(giSpatialSourcePath), Is.True, $"GI spatial source not found: {giSpatialSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string computeSource = File.ReadAllText(computeSourcePath);
        string giSpatialSource = File.ReadAllText(giSpatialSourcePath);
        string startBody = ExtractMethodBody(tracingSource, "Start");
        string giBody = ExtractMethodBody(tracingSource, "DispatchReSTIRGI");

        StringAssert.Contains("kernelSpatialGIResampling", tracingSource);
        StringAssert.Contains("#pragma kernel kernel_spatial_gi_resampling", computeSource);
        StringAssert.Contains("#include \"restir/gi_spatial.hlsl\"", computeSource);
        StringAssert.Contains("kernel_spatial_gi_resampling", startBody);
        StringAssert.Contains("kernelSpatialGIResampling", giBody);
        StringAssert.Contains("SetInt(\"_RestirSpatialReservoirOffset\"", giBody);
        StringAssert.Contains("IndirectReservoirs[outIdx] = outR;", giSpatialSource);
    }

    [Test]
    public void RestirGI_Reuses_Reservoirs_With_Combine_And_Finalize_Semantics()
    {
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(giTemporalSourcePath), Is.True, $"GI temporal source not found: {giTemporalSourcePath}");
        Assert.That(File.Exists(giSpatialSourcePath), Is.True, $"GI spatial source not found: {giSpatialSourcePath}");
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string giTemporalSource = File.ReadAllText(giTemporalSourcePath);
        string giSpatialSource = File.ReadAllText(giSpatialSourcePath);
        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);
        string giInitialSource = File.ReadAllText(giInitialSourcePath);

        StringAssert.Contains("InitializeIndirectReservoirSample", giInitialSource);
        StringAssert.Contains("CombineIndirectReservoirs", giTemporalSource);
        StringAssert.Contains("CombineIndirectReservoirs", giSpatialSource);
        StringAssert.Contains("FinalizeIndirectReservoir", giTemporalSource);
        StringAssert.Contains("FinalizeIndirectReservoir", giSpatialSource);
        StringAssert.Contains("CalculateIndirectJacobian", giTemporalSource);
        StringAssert.Contains("CalculateIndirectJacobian", giSpatialSource);
        StringAssert.DoesNotContain("prev.weightSum * (prevTargetLumCur / max(prev.targetLum, 1e-6))", giTemporalSource);
        StringAssert.DoesNotContain("neighbor.weightSum * (neighborTargetLumCur / max(neighbor.targetLum, 1e-6))", giSpatialSource);
        StringAssert.Contains("bool CombineIndirectReservoirs(", giReservoirSource);
        StringAssert.Contains("void FinalizeIndirectReservoir(", giReservoirSource);
        StringAssert.Contains("for (int sampleIdx = 0;", giTemporalSource);
        StringAssert.Contains("static const int2 kTemporalOffsets", giTemporalSource);
        StringAssert.Contains("WrapTemporalOffsetIndex", giTemporalSource);
        StringAssert.DoesNotContain("% 5", giTemporalSource);
        StringAssert.Contains("for (int neighborSampleIdx = 0;", giSpatialSource);
        StringAssert.Contains("for (int cachedSampleIdx = 0;", giSpatialSource);
        StringAssert.Contains("static const int2 kNeighborOffsets", giSpatialSource);
        StringAssert.Contains("WrapNeighborOffsetIndex", giSpatialSource);
        StringAssert.DoesNotContain("% 8", giSpatialSource);
    }

    [Test]
    public void RestirGI_Initial_Invalid_Secondary_Surfaces_Write_Empty_Reservoirs()
    {
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string giInitialSource = File.ReadAllText(giInitialSourcePath);

        StringAssert.Contains("IndirectReservoirData reservoir = EmptyIndirectReservoir();", giInitialSource);
        StringAssert.DoesNotContain("sampleCount = -10.0", giInitialSource);
        StringAssert.DoesNotContain("sampleCount = -11.0", giInitialSource);
        StringAssert.DoesNotContain("sampleCount = -12.0", giInitialSource);
        StringAssert.DoesNotContain("sampleCount = -13.0", giInitialSource);
        StringAssert.DoesNotContain("IndirectReservoirData debugReservoir", giInitialSource);
    }

    [Test]
    public void RestirGI_Temporal_Reuse_Selects_At_Most_One_Reprojected_Candidate()
    {
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        Assert.That(File.Exists(giTemporalSourcePath), Is.True, $"GI temporal source not found: {giTemporalSourcePath}");

        string giTemporalSource = File.ReadAllText(giTemporalSourcePath);

        StringAssert.Contains("float prevRISWeight = GetIndirectReservoirRISWeight(prevCandidate, prevTargetLumCur);",
            giTemporalSource,
            "GI temporal reuse must distinguish candidate streaming from candidate selection.");
        StringAssert.Contains("combinedPrevious = true;", giTemporalSource,
            "GI temporal reuse must record that a history candidate entered the RIS stream even when current remains selected.");
        Assert.That(
            Regex.IsMatch(
                giTemporalSource,
                @"bool candidateSelected\s*=\s*CombineIndirectReservoirs\(outR,\s*prevCandidate,\s*RNG_Next\(rng\),\s*prevTargetLumCur\).*?combinedPrevious\s*=\s*true\s*;.*?if\s*\(candidateSelected\).*?selectedPrevious\s*=\s*true\s*;.*?}\s*break\s*;",
                RegexOptions.Singleline),
            Is.True,
            "GI temporal reuse should stop after the first valid history candidate enters the stream, not only after history wins selection.");
    }

    [Test]
    public void RestirGI_Temporal_And_Spatial_Clear_Debug_Data_Before_Early_Returns()
    {
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
        Assert.That(File.Exists(giTemporalSourcePath), Is.True, $"GI temporal source not found: {giTemporalSourcePath}");
        Assert.That(File.Exists(giSpatialSourcePath), Is.True, $"GI spatial source not found: {giSpatialSourcePath}");

        string giTemporalSource = File.ReadAllText(giTemporalSourcePath);
        string giSpatialSource = File.ReadAllText(giSpatialSourcePath);

        StringAssert.Contains("for (int debugSlot = 0; debugSlot < 5; debugSlot++)", giTemporalSource,
            "GI temporal pass must clear debug slots before any early return so JSONL stats cannot inherit stale data.");
        StringAssert.Contains("ReSTIRDebugData[debugSlot] = 0.0;", giTemporalSource,
            "GI temporal pass must zero ReSTIRDebugData for the selected debug pixel.");
        StringAssert.Contains("for (int debugSlot = 0; debugSlot < 5; debugSlot++)", giSpatialSource,
            "GI spatial pass must clear debug slots before any early return so JSONL stats cannot inherit stale data.");
        StringAssert.Contains("ReSTIRDebugData[debugSlot] = 0.0;", giSpatialSource,
            "GI spatial pass must zero ReSTIRDebugData for the selected debug pixel.");
    }

    [Test]
    public void RestirGI_Spatial_Uses_Two_Pass_Bias_Correction_For_MultiNeighborReuse()
    {
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
        Assert.That(File.Exists(giSpatialSourcePath), Is.True, $"GI spatial source not found: {giSpatialSourcePath}");

        string giSpatialSource = File.ReadAllText(giSpatialSourcePath);

        Assert.That(
            Regex.Matches(giSpatialSource, @"for\s*\(int (neighborSampleIdx|cachedSampleIdx) = 0;\s*(neighborSampleIdx|cachedSampleIdx) < 8;\s*(neighborSampleIdx|cachedSampleIdx)\+\+\)").Count,
            Is.GreaterThanOrEqualTo(2),
            "GI spatial reuse should perform a second pass over cached neighbors for normalization.");

        StringAssert.Contains("uint cachedResult = 0u;", giSpatialSource);
        StringAssert.Contains("int selected = -1;", giSpatialSource);
        StringAssert.Contains("cachedResult |= (1u << uint(neighborSampleIdx));", giSpatialSource);
        StringAssert.Contains("(cachedResult & (1u << uint(cachedSampleIdx))) == 0", giSpatialSource);
        StringAssert.Contains("piSum += neighborP * max(neighborSampleCount, 0.0);", giSpatialSource);
        StringAssert.Contains("float normalizationDenominator = selectedTargetPdf * piSum;", giSpatialSource);
        Assert.That(
            Regex.IsMatch(
                giSpatialSource,
                @"for\s*\(int cachedSampleIdx = 0;.*?\(cachedResult & \(1u << uint\(cachedSampleIdx\)\)\) == 0.*?ReevaluateIndirectReservoirAtSurface\(hdNeighbor,\s*outR,.*?neighborP\).*?pi = selected == cachedSampleIdx \? neighborP : pi;.*?piSum \+= neighborP \* max\(neighborSampleCount, 0\.0\);",
                RegexOptions.Singleline),
            Is.True,
            "GI spatial reuse must re-evaluate the selected reservoir in every cached neighbor domain and use the selected neighbor-domain target as pi.");
        StringAssert.DoesNotContain("break;", giSpatialSource, "GI spatial reuse should not early-break after the first selected neighbor when using multi-neighbor bias correction.");
        Assert.That(
            Regex.IsMatch(
                giSpatialSource,
                @"if\s*\(candidateSelected\)\s*\{.*?selected\s*=\s*neighborSampleIdx\s*;\s*selectedTargetPdf\s*=\s*neighborTargetLumCur\s*;.*?\}",
                RegexOptions.Singleline),
            Is.True,
            "GI spatial reuse should keep track of the selected neighbor index for the second normalization pass.");
    }

    [Test]
    public void RestirGI_RuntimeDiagnostics_Report_SelectedProbeSpatialNeighborStats()
    {
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string spatial = RestirShader("gi_spatial.hlsl");

        StringAssert.Contains("restir_gi_spatial_stats.jsonl", session);
        StringAssert.Contains("case ReSTIRTelemetryStage.GISpatial", session);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_INVALID_NEIGHBOR", spatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_INCOMPATIBLE_NEIGHBOR", spatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_REEVALUATION_REJECTED", spatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_JACOBIAN_REJECTED", spatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_COMBINED", spatial);
        StringAssert.Contains("RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_SELECTED", spatial);
        StringAssert.Contains("RESTIR_STAGE_GI_SPATIAL", spatial);
    }

    [Test]
    public void RestirGI_FinalShading_Uses_CurrentSurfaceAndVisibility()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string giShadeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_shade.hlsl"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");
        Assert.That(File.Exists(giShadeSourcePath), Is.True, $"GI shade source not found: {giShadeSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string giShadeSource = File.ReadAllText(giShadeSourcePath);
        string giBody = ExtractMethodBody(tracingSource, "DispatchReSTIRGI");

        StringAssert.Contains("SetBuffer(kernelShadeGISamples, \"_RestirGbuffer\", _globalHits)", giBody);
        StringAssert.Contains("SetInt(\"_RestirInitialReservoirOffset\"", giBody);
        StringAssert.Contains("HitData hd = _RestirGbuffer[id.x];", giShadeSource);
        StringAssert.Contains("IntersectTlasFast(shadowRay, tMax)", giShadeSource);
        StringAssert.Contains("GetDirectLightSurfaceNormal", giShadeSource);
        StringAssert.Contains("IndirectReservoirsRead[_RestirInitialReservoirOffset + id.x]", giShadeSource);
        StringAssert.Contains("weightedReflectedRadiance = reflectedRadiance * res.weightSum;", giShadeSource);
    }

    [Test]
    public void RestirGI_Reevaluation_Uses_TargetContribution_Not_BRDF_Proposal_Pdf_Gating()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");

        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("EvaluateIndirectSampleAtSurface(", giReservoirSource);
        StringAssert.Contains("GetDirectLightSurfaceNormal(primaryHit, V);", giReservoirSource);
        StringAssert.Contains("EvaluateBXDF_GivenDir(primaryHit, V, L, f_brdf, proposalPdf);", giReservoirSource);
        StringAssert.DoesNotContain("if (proposalPdf <= 1e-6 || !all(isfinite(f_brdf)))", giReservoirSource);
        StringAssert.Contains("if (!IsFiniteIndirectFloat3(f_brdf))", giReservoirSource);
        StringAssert.Contains("float3 reflected = max(f_brdf * NdotL * radiance, 0.0);", giReservoirSource);
        StringAssert.Contains("targetLum = max(reflectedRadiance.x, max(reflectedRadiance.y, reflectedRadiance.z));", giReservoirSource);
    }

    [Test]
    public void RestirGI_FinalShading_And_Reevaluation_Share_The_Same_Reflected_Radiance_Helper()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        string giShadeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_shade.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        Assert.That(File.Exists(giShadeSourcePath), Is.True, $"GI shade source not found: {giShadeSourcePath}");

        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);
        string giShadeSource = File.ReadAllText(giShadeSourcePath);

        // Helper signature: thin wrapper that returns reflectedRadiance only.
        StringAssert.Contains("bool EvaluateIndirectSampleAtSurface(", giReservoirSource);
        StringAssert.Contains("EvaluateIndirectSampleAtSurface(hd, res, reflectedRadiance)", giShadeSource);
        // Final shading multiplies reflectedRadiance by reservoir.weightSum (RTXDI FinalShading.hlsl:66 parity).
        StringAssert.Contains("reflectedRadiance * res.weightSum", giShadeSource);
        StringAssert.DoesNotContain("ApplyGIContributionFence", giShadeSource);
        // Old dead out-param patterns must be gone.
        StringAssert.DoesNotContain("trueBrdf, weightedRadiance", giShadeSource);
        StringAssert.DoesNotContain("weightedRadiance = sample.radiance * sample.selectedWeight", giReservoirSource);
        // Pre-fix shading-by-selectedWeight pattern must be gone.
        StringAssert.DoesNotContain("weightedReflectedRadiance = reflectedRadiance * res.selectedWeight", giShadeSource);
    }

    [Test]
    public void RestirGI_ReservoirValidity_Uses_Explicit_Finite_Guards()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");

        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("RESTIR_GI_FINITE_LIMIT", giReservoirSource);
        StringAssert.Contains("bool IsFiniteIndirectScalar(float v)", giReservoirSource);
        StringAssert.Contains("bool IsFiniteIndirectFloat3(float3 v)", giReservoirSource);
        StringAssert.Contains("IsFiniteIndirectFloat3(r.secondaryPosition)", giReservoirSource);
        StringAssert.Contains("IsFiniteIndirectScalar(r.selectedWeight)", giReservoirSource);
        StringAssert.DoesNotContain("all(isfinite(r.secondaryPosition))", giReservoirSource);
        StringAssert.DoesNotContain("isfinite(r.selectedWeight)", giReservoirSource);
    }

    [Test]
    public void RestirGI_Initial_Reservoir_Uses_Secondary_Radiance_Not_Baked_Primary_Contribution()
    {
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string giInitialSource = File.ReadAllText(giInitialSourcePath);

        StringAssert.Contains("float3 throughputSample;", giInitialSource);
        StringAssert.Contains("EvaluateBXDFWithDotAndPDFDetailed(primaryHit, bounceRay, throughputSample, sampledSpecular, throughputZeroReason);", giInitialSource);
        StringAssert.Contains("if (!IsGISecondaryBypass(sampleFlags))", giInitialSource);
        StringAssert.Contains("data.throughput = 1.0;", giInitialSource);
        StringAssert.Contains("data.throughput = throughputSample;", giInitialSource);
        StringAssert.Contains("EvaluateIndirectRadianceAtSurface(_RestirGbuffer[id.x], secondary.position, secondaryRadiance, brdfAtPrimary, contribution)", giInitialSource);
    }

    [Test]
    public void RestirGI_DeltaSpecular_Samples_Bypass_Reservoir_And_Shade_Directly()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");
        Assert.That(File.Exists(bxdfSourcePath), Is.True, $"BXDF source not found: {bxdfSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string giInitialSource = File.ReadAllText(giInitialSourcePath);
        string bxdfSource = File.ReadAllText(bxdfSourcePath);

        StringAssert.Contains("RESTIR_GI_STAGE1_FLAG_BYPASS", giInitialSource);
        StringAssert.Contains("sampledSpecular && isDeltaSurface", giInitialSource);
        StringAssert.Contains("bool bypassReservoir = IsGISecondaryBypass(secondary.flags);", giInitialSource);
        StringAssert.Contains("float3 contribution = max(secondary.throughput * secondaryRadiance, 0.0);", giInitialSource);
        StringAssert.Contains("GlobalColors[id.x].L += contribution;", giInitialSource);
        StringAssert.Contains("SetBuffer(kernelShadeGISecondarySurfaces, \"GlobalColors\", _globalColors)", tracingSource);
        StringAssert.Contains("EvaluateBXDFWithDotAndPDFDetailed", giInitialSource);
        StringAssert.Contains("void EvaluateBXDFWithDotAndPDFDetailed(", bxdfSource);
        StringAssert.Contains("out bool sampledSpecular", bxdfSource);
    }

    [Test]
    public void SelfTest_Disables_Verbose_ReSTIR_GI_Detail_Logs()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"SelfTest source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string methodBody = ExtractMethodBody(source, "PrepareReSTIRGIValidation");

        StringAssert.Contains("SetPrivateField(tracing, \"WriteReSTIRGIDiagnosticDetails\", false);", methodBody);
    }

    [Test]
    public void Tracing_Warns_When_ReSTIR_GI_Is_Enabled_Without_ReSTIR_DI()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);

        StringAssert.Contains("WarnIfReSTIRDirectLightingConfigurationIsRisky", source);
        StringAssert.Contains("UseReSTIRGI && !UseReSTIRDI", source);
        StringAssert.Contains("ReSTIR GI is enabled while ReSTIR DI is disabled", source);
    }

    [Test]
    public void Tracing_Defines_ReSTIR_DI_Diagnostic_Output_Contract()
    {
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string diInitial = RestirShader("generate_initial.hlsl");
        string diTemporal = RestirShader("temporal_resampling.hlsl");
        string diShade = RestirShader("shade_di.hlsl");

        StringAssert.Contains("restir_di_stats.jsonl", session);
        StringAssert.Contains("case ReSTIRTelemetryStage.DIInitial", session);
        StringAssert.Contains("case ReSTIRTelemetryStage.DITemporal", session);
        StringAssert.Contains("RESTIR_STAGE_DI_INITIAL", diInitial);
        StringAssert.Contains("RESTIR_STAGE_DI_TEMPORAL", diTemporal);
        StringAssert.Contains("RESTIR_STAGE_DI_SHADE", diShade);
        StringAssert.Contains("RESTIR_COUNTER_DI_SHADE_POSITIVE_CONTRIBUTION", diShade);
        StringAssert.Contains("float4(r.direction, r.targetLum)", diInitial);
        StringAssert.Contains("float4(r.surfaceNormal, r.proposalPdf)", diInitial);
        StringAssert.Contains("r.selectedWeight", diInitial);
    }

    [Test]
    public void RestirDI_Temporal_Reuse_Does_Not_Multiply_WeightSum_By_SampleCount_Twice()
    {
        string diTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "temporal_resampling.hlsl"));
        Assert.That(File.Exists(diTemporalSourcePath), Is.True, $"DI temporal source not found: {diTemporalSourcePath}");

        string diTemporalSource = File.ReadAllText(diTemporalSourcePath);

        StringAssert.Contains("float curW = cur.weightSum;", diTemporalSource);
        StringAssert.Contains("float prevW = prev.selectedWeight * prevSample.targetLum * max((float)prev.sampleCount, 1.0);", diTemporalSource);
        StringAssert.DoesNotContain("cur.weightSum * (float)max(cur.sampleCount, 1u)", diTemporalSource);
        StringAssert.DoesNotContain("prev.weightSum * (float)max(prev.sampleCount, 1u)", diTemporalSource);
    }

    [Test]
    public void RestirGI_SelectedWeight_Mirrors_WeightSum_For_Stride_Compat()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");

        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        // ComputeIndirectProposalInversePdf is the inverse-PDF helper still in use.
        StringAssert.Contains("float ComputeIndirectProposalInversePdf(float proposalPdf)", giReservoirSource);
        // Stage2 init now writes weightSum = 1/p (RTXDI_MakeGIReservoir parity).
        StringAssert.Contains("reservoir.weightSum = ComputeIndirectProposalInversePdf(reservoir.proposalPdf);", giReservoirSource);
        // selectedWeight is now a mirror of weightSum at init.
        StringAssert.Contains("reservoir.selectedWeight = reservoir.weightSum;", giReservoirSource);
        // RIS streaming weight no longer multiplies by sampleCount (M-double-count fix).
        StringAssert.Contains("return max(targetPdf, 0.0) * max(candidate.weightSum, 0.0);", giReservoirSource);
        StringAssert.DoesNotContain("max(candidate.sampleCount, 0.0)", giReservoirSource);
        // Old stage2 form (targetLum * ...) must be gone.
        StringAssert.DoesNotContain("reservoir.weightSum = targetLum * ComputeIndirectProposalInversePdf", giReservoirSource);
    }

    [Test]
    public void RestirGI_Finalize_Does_Not_Divide_By_TargetLum_Times_M()
    {
        // RTXDI parity: FinalizeGIResampling sets weightSum = (wsum*num)/denom and
        // does NOT divide by targetLum*M. Old division was the source of the
        // baseline 1e+29 selectedWeight blowup.
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("// In RTXDI semantics, weightSum AFTER FinalizeIndirectReservoir already encodes",
            giReservoirSource,
            "ComputeIndirectMISWeight comment must mark RTXDI-parity intent so it isn't reverted to a /(targetLum*M) form.");
        StringAssert.DoesNotContain("weightSum / max(targetLum * sampleCount",
            giReservoirSource,
            "Old MIS divisor must be removed");
    }

    [Test]
    public void RestirGI_Formula_Map_Tracks_UCW_Reservoir_Semantics()
    {
        string formulaMapPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "memory", "unity-path-tracing", "RESTIR_GI_FORMULA_MAP.md"));
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(formulaMapPath), Is.True, $"Formula map not found: {formulaMapPath}");
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");

        string formulaMap = File.ReadAllText(formulaMapPath);
        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("unbiased contribution weight `W` / UCW", formulaMap);
        StringAssert.Contains("final estimator multiplier for the selected sample", formulaMap);
        StringAssert.Contains("`weightSum = 1 / proposalPdf`", formulaMap);
        StringAssert.Contains("`GetIndirectReservoirRISWeight(...)` must be `targetPdf * candidate.weightSum`", formulaMap);
        StringAssert.Contains("RIS unbiased contribution weight W", giReservoirSource);
        StringAssert.Contains("selected sample's final estimator multiplier", giReservoirSource);
        StringAssert.Contains("not targetLum/p", giReservoirSource);
        StringAssert.DoesNotContain("W = 1/p_hat", giReservoirSource,
            "GI comments must not teach the shorthand that W is simply 1/p_hat; W is the finalized RIS estimator multiplier.");
    }

    [Test]
    public void RestirGI_Worked_Example_Documents_Formula_To_Code_Numerics()
    {
        string memoryRootPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "memory"));
        string memoryIndexPath = Path.Combine(memoryRootPath, "MEMORY.md");
        string formulaMapPath = Path.Combine(memoryRootPath, "unity-path-tracing", "RESTIR_GI_FORMULA_MAP.md");
        string workedExamplePath = Path.Combine(memoryRootPath, "unity-path-tracing", "RESTIR_GI_WORKED_EXAMPLE.md");
        Assert.That(File.Exists(memoryIndexPath), Is.True, $"Memory index not found: {memoryIndexPath}");
        Assert.That(File.Exists(formulaMapPath), Is.True, $"Formula map not found: {formulaMapPath}");
        Assert.That(File.Exists(workedExamplePath), Is.True, $"Worked example not found: {workedExamplePath}");

        string memoryIndex = File.ReadAllText(memoryIndexPath);
        string formulaMap = File.ReadAllText(formulaMapPath);
        string workedExample = File.ReadAllText(workedExamplePath);

        StringAssert.Contains("RESTIR_GI_WORKED_EXAMPLE.md", memoryIndex,
            "Memory index must route new sessions to the ReSTIR GI numeric walkthrough.");
        StringAssert.Contains("RESTIR_GI_WORKED_EXAMPLE.md", formulaMap,
            "Formula map must point readers to the worked numeric example.");
        StringAssert.Contains("weightSum = 1 / proposalPdf = 1 / 0.25 = 4", workedExample,
            "Worked example must show fresh reservoir inverse-proposal initialization.");
        StringAssert.Contains("risWeight = targetPdf * candidate.weightSum = 0.8 * 4 = 3.2", workedExample,
            "Worked example must show RIS stream weight without folding M into the candidate weight.");
        StringAssert.Contains("final weightSum = sumWeights * pi / normalizationDenominator", workedExample,
            "Worked example must show Finalize converting stream weight to UCW.");
        StringAssert.Contains("selectedPrevReuseProposalPdf = max(0.5 / 2.0, RESTIR_GI_MIN_PROPOSAL_PDF)", workedExample,
            "Worked example must show temporal history proposal PDF transformed by the Jacobian.");
        StringAssert.Contains("spatialSelectedNeighborReuseProposalPdf = max(0.4 / 0.5, RESTIR_GI_MIN_PROPOSAL_PDF)", workedExample,
            "Worked example must show spatial selected-neighbor proposal PDF transformed by the Jacobian.");
        StringAssert.Contains("weightedReflectedRadiance = raw reflectedRadiance * weightSum", workedExample,
            "Worked example must connect final shading to raw reflected radiance times finalized UCW.");
        StringAssert.Contains("fence scale = 128 / 200 = 0.64", workedExample,
            "Worked example must show the low-sample fence scale after reflected radiance is multiplied by UCW.");
        StringAssert.Contains("finalReflectedRadianceSource = \"shaderRaw\"", workedExample,
            "Worked example must explain why final schema v2 is the reliable shader-raw proof path.");
    }

    [Test]
    public void RestirGI_RIS_Stream_Weight_Has_No_Extra_M_Multiplier()
    {
        // GetIndirectReservoirRISWeight must NOT multiply by candidate.sampleCount;
        // CombineIndirectReservoirs already adds candidate.M into reservoir.M.
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.DoesNotContain("max(candidate.sampleCount, 0.0)",
            giReservoirSource,
            "candidate.sampleCount must not be folded into RIS streaming weight (double-counts M).");
        StringAssert.Contains("return max(targetPdf, 0.0) * max(candidate.weightSum, 0.0);",
            giReservoirSource,
            "RIS weight must be targetPdf * candidate.weightSum (RTXDI form).");
    }

    [Test]
    public void RestirGI_Combine_And_Finalize_Keep_RIS_Stream_And_M_Separate()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");

        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("float risWeight = GetIndirectReservoirRISWeight(candidate, targetPdf);",
            giReservoirSource,
            "Combine must compute the RIS stream weight from targetPdf and candidate.weightSum.");
        StringAssert.Contains("reservoir.sampleCount += candidate.sampleCount;",
            giReservoirSource,
            "Combine must accumulate M/sampleCount separately from RIS stream weight.");
        StringAssert.Contains("reservoir.weightSum += risWeight;",
            giReservoirSource,
            "Combine must accumulate the RIS stream weight into weightSum before finalization.");
        StringAssert.Contains("bool selectSample = random * reservoir.weightSum <= risWeight;",
            giReservoirSource,
            "Reservoir selection probability must be proportional to the new candidate RIS weight over the accumulated stream weight.");
        StringAssert.Contains("reservoir.targetLum = targetPdf;",
            giReservoirSource,
            "When a candidate wins, the reservoir target must store the targetPdf used for that selected sample.");
        StringAssert.Contains("reservoir.weightSum = normalizationDenominator <= 0.0",
            giReservoirSource,
            "Finalize must guard zero normalization denominators explicitly.");
        StringAssert.Contains(": (reservoir.weightSum * normalizationNumerator) / normalizationDenominator;",
            giReservoirSource,
            "Finalize must convert accumulated stream weight into UCW with sumWeights * numerator / denominator.");
        StringAssert.Contains("reservoir.selectedWeight = ComputeIndirectMISWeight(reservoir.weightSum, reservoir.targetLum, reservoir.sampleCount);",
            giReservoirSource,
            "After Finalize, selectedWeight must mirror the finalized UCW for readback compatibility.");
    }

    [Test]
    public void RestirGI_FinalShading_Uses_WeightSum_Not_SelectedWeight()
    {
        string giShadeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_shade.hlsl"));
        Assert.That(File.Exists(giShadeSourcePath), Is.True, $"GI shade source not found: {giShadeSourcePath}");

        string giShadeSource = File.ReadAllText(giShadeSourcePath);
        StringAssert.Contains("reflectedRadiance * res.weightSum",
            giShadeSource,
            "EvaluateVisibleGISample must use radiance * weightSum (RTXDI FinalShading parity).");
        StringAssert.DoesNotContain("ApplyGIContributionFence",
            giShadeSource,
            "Diagnostics must not clamp or otherwise change the GI estimator.");
        StringAssert.DoesNotContain("RESTIR_GI_LOW_SAMPLE_FIRE_FLY_MAX_M",
            giShadeSource,
            "GI firefly fencing must not allow high-M temporal/spatial reservoirs to bypass clamping.");
    }

    [Test]
    public void RestirGI_Finite_Limit_Tightened_For_Firefly_Containment()
    {
        // Old 1e30 allowed 1e+27 reservoirs to spread across frames. 1e6 keeps a
        // healthy headroom over typical 1/p_hat (1..1e3) but kills the runaway tail.
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("static const float RESTIR_GI_FINITE_LIMIT = 1e6;",
            giReservoirSource,
            "RESTIR_GI_FINITE_LIMIT must be 1e6 to contain firefly bubble propagation.");
    }

    [Test]
    public void RestirGI_Spatial_MCap_Scales_WeightSum_Proportionally()
    {
        // 2026-05-31 Sponza regression showed spatialNormalizationDenominator hitting
        // 7.3e6 with 7/8 active neighbors. Cause: spatial RIS streaming weight had
        // sampleCount clamped without proportional Wsum scaling, so streamed weight
        // stayed at full magnitude while M was capped. Fence required at BOTH the
        // per-neighbor candidate site AND the post-loop outR site, mirroring gi_temporal.hlsl.
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
        Assert.That(File.Exists(giSpatialSourcePath), Is.True, $"GI spatial source not found: {giSpatialSourcePath}");

        string giSpatialSource = File.ReadAllText(giSpatialSourcePath);

        StringAssert.Contains("neighborCandidate.weightSum *= neighborSampleCountClamped / neighborCandidate.sampleCount;",
            giSpatialSource,
            "Spatial per-neighbor candidate must scale weightSum by clamped/sampleCount when M-capping (TrueTrace fence).");

        StringAssert.Contains("outR.weightSum *= outSampleCountClamped / outR.sampleCount;",
            giSpatialSource,
            "Spatial outR must scale weightSum by clamped/sampleCount before Finalize when M exceeds the cap.");
    }

    [Test]
    public void RestirGI_SelfTest_Has_Editor_Menu_Entry()
    {
        // The batchmode SelfTest.Run entrypoint requires Unity Hub + command-line
        // launch, which the user does not run. The editor menu entry lets the
        // same upper-bound assertions execute against the latest editor-Play
        // jsonl logs without batchmode. If this string disappears the editor
        // user has no way to exercise the SelfTest assertions at all.
        string selfTestSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
        Assert.That(File.Exists(selfTestSourcePath), Is.True, $"SelfTest source not found: {selfTestSourcePath}");

        string selfTestSource = File.ReadAllText(selfTestSourcePath);

        StringAssert.Contains("[MenuItem(\"Tools/Verify GI Logs\")]",
            selfTestSource,
            "SelfTest must expose a Tools/Verify GI Logs editor menu so play-mode logs can be verified without batchmode.");
        StringAssert.Contains("VerifyGILogsFromEditor",
            selfTestSource,
            "SelfTest editor menu must dispatch to VerifyGILogsFromEditor.");
        StringAssert.Contains("FindLatestDiagnosticOutputDir",
            selfTestSource,
            "Editor entrypoint must look up the latest Tools/Output/<timestamp>/ subdir produced by Tracing.GetDiagnosticOutputPath, not the legacy non-timestamped paths.");
        StringAssert.Contains("FailMode.LogOnFail",
            selfTestSource,
            "Editor entrypoint must use LogOnFail so a failed assertion does not call EditorApplication.Exit(1).");
    }

    [Test]
    public void RestirGI_SelfTest_Exposes_NonInteractive_Latest_Log_Verification_EntryPoint()
    {
        string selfTestSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
        Assert.That(File.Exists(selfTestSourcePath), Is.True, $"SelfTest source not found: {selfTestSourcePath}");

        string selfTestSource = File.ReadAllText(selfTestSourcePath);

        StringAssert.Contains("public static bool VerifyLatestGILogs(out string report)",
            selfTestSource,
            "SelfTest must expose a non-interactive latest-log verification entrypoint so the agent can run log checks after the user finishes Play Mode.");
        StringAssert.Contains("TryUseLatestReSTIRGILogPaths(out string latestDir)",
            selfTestSource,
            "Non-interactive verification must resolve the latest Tools/Output/<timestamp>/ directory.");
        StringAssert.Contains("Debug.Log($\"[SelfTest] Validating GI logs in {latestDir}\");",
            selfTestSource,
            "Batchmode self-test must validate the timestamped GI logs generated by Tracing.GetDiagnosticOutputPath.");
        StringAssert.Contains("report = s_exitCode == 0 ? $\"PASS\\n\\nLogs verified: {latestDir}\" : $\"FAIL: {s_lastFailReason}\\n\\nLogs: {latestDir}\";",
            selfTestSource,
            "Non-interactive verification must return a machine-readable pass/fail report without requiring dialog inspection.");
        string methodBody = ExtractMethodBody(selfTestSource, "VerifyLatestGILogsNonInteractive");
        StringAssert.DoesNotContain("EditorUtility.DisplayDialog", methodBody,
            "Automation entrypoint must not depend on modal dialogs.");
    }

    [Test]
    public void RestirGI_Final_Summary_Logs_Shader_Raw_Schema_V2()
    {
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string finalShade = RestirShader("gi_shade.hlsl");

        StringAssert.Contains("SchemaVersion = 1u", session);
        StringAssert.Contains("RecordPayloadWordCount = 24", session);
        StringAssert.Contains("RESTIR_STAGE_GI_FINAL", finalShade);
        StringAssert.Contains("finalReflectedRadiance", finalShade);
        StringAssert.Contains("finalWeightedReflectedRadiance", finalShade);
    }

    [Test]
    public void RestirGI_SelfTest_Skips_ZeroEnergy_Probe_Class_Mismatch()
    {
        // 2026-06-01 Sponza regression: a single pixel with weightSum=0 (selected
        // a prev candidate but reevaluation collapsed it) tripped the temporal
        // assertion that "selected probe must be reusable+active". Such pixels
        // never contribute to GlobalColors, so flagging them produces a false
        // alarm. Relaxation: skip rows where the kernel produced zero energy.
        string selfTestSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
        Assert.That(File.Exists(selfTestSourcePath), Is.True, $"SelfTest source not found: {selfTestSourcePath}");

        string selfTestSource = File.ReadAllText(selfTestSourcePath);

        // Temporal block: must guard the "selected probe must be reusable+active"
        // check on a positive weightSum.
        Assert.That(
            Regex.IsMatch(
                selfTestSource,
                @"float\s+weightSum\s*=\s*ExtractFloat\(line,\s*""weightSum""\)\s*;\s*if\s*\(\s*!IsFinitePositive\(\s*weightSum\s*\)\s*\)\s*return\s+false\s*;",
                RegexOptions.Singleline),
            Is.True,
            "Temporal probe-class check must skip rows where weightSum is non-positive (kernel collapsed to empty reservoir).");

        // Final block: must guard the same check on finalContributionPositive.
        StringAssert.Contains("if (!ExtractBool(line, \"finalContributionPositive\"))",
            selfTestSource,
            "Final probe-class check must skip rows where the final pipeline produced zero contribution.");

        // Spatial block: must guard on spatialSelectedTargetPdf > 0 (NOT spatialPi,
        // which falls back to currentTargetPdf when nothing was selected).
        StringAssert.Contains("ExtractFloat(line, \"spatialSelectedTargetPdf\")",
            selfTestSource,
            "Spatial probe-class check must guard on spatialSelectedTargetPdf, not spatialPi.");
    }

    [Test]
    public void Workflow_Document_Uses_Editor_Play_And_Agent_Log_Review_Instead_Of_Batchmode()
    {
        string workflowPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "memory", "unity-path-tracing", "AI_WORKFLOW.md"));
        Assert.That(File.Exists(workflowPath), Is.True, $"Workflow doc not found: {workflowPath}");

        string workflow = File.ReadAllText(workflowPath);

        StringAssert.Contains("你点 Play", workflow, "Workflow must describe the user's Editor Play step explicitly.");
        StringAssert.Contains("你告诉 AI“跑完了”", workflow, "Workflow must describe the handoff where the user tells the agent the play run is complete.");
        StringAssert.Contains("AI 读取最新的 `Tools/Output/<timestamp>/` 日志", workflow, "Workflow must describe the agent reading the latest timestamped diagnostic logs.");
        StringAssert.DoesNotContain("-batchmode", workflow, "Pure Editor workflow must not depend on batchmode commands.");
        StringAssert.Contains("不再依赖 `summary.txt`", workflow, "Workflow must explicitly reject the removed summary.txt convention.");
        StringAssert.DoesNotContain("manage_editor action=play", workflow, "Workflow must not depend on MCP-driven Play control.");
    }

    [Test]
    public void LatestLogVerification_Script_Exists_For_Terminal_PostPlay_Checks()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("restir_gi_probe.jsonl", script, "Terminal verification script must inspect the probe jsonl.");
        StringAssert.Contains("restir_gi_temporal_stats.jsonl", script, "Terminal verification script must inspect temporal stats.");
        StringAssert.Contains("restir_gi_spatial_stats.jsonl", script, "Terminal verification script must inspect spatial stats.");
        StringAssert.Contains("restir_gi_final_stats.jsonl", script, "Terminal verification script must inspect final stats.");
        StringAssert.Contains("PASS", script, "Terminal verification script must emit a pass result.");
        StringAssert.Contains("FAIL", script, "Terminal verification script must emit a fail result.");
    }

    [Test]
    public void LatestLogVerification_Script_Has_Fresh_Run_Gates_For_PostPlay_Runtime_Proof()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("[switch]$RequireFreshReSTIRGI", script,
            "Latest-log verification must expose a strict post-Play mode that distinguishes fresh logs from legacy compatible logs.");
        StringAssert.Contains("[switch]$RequireReuseFormulaCoverage", script,
            "Latest-log verification must expose an explicit full-formula coverage gate for temporal-history and spatial-neighbor reuse.");
        StringAssert.Contains("Assert-TemporalDebugStateIsClean", script,
            "Fresh verification must reject stale temporal debug readback where selected-history slots contain non-state values.");
        StringAssert.Contains("selectedTemporalOffsetIndex", script,
            "Fresh verification must inspect temporal selected offset state that exposed prior stale-data pollution.");
        StringAssert.Contains("selectedPrevJacobian", script,
            "Fresh verification must require selected-history Jacobian data when the temporal shader says history won selection.");
        StringAssert.Contains("Assert-SpatialDebugCountsAreClean", script,
            "Fresh verification must reject stale spatial debug readback where count slots contain non-count values.");
        StringAssert.Contains("spatialShaderCombinedNeighbors", script,
            "Fresh verification must inspect spatial shader count fields that exposed prior stale-data pollution.");
        StringAssert.Contains("RequireFreshReSTIRGI expected finalDiagnosticSchemaVersion>=2", script,
            "Fresh verification must require final schema v2 rows for shader-raw reflected radiance proof.");
        StringAssert.Contains("RequireFreshReSTIRGI expected at least one shader-raw targetPdf branch", script,
            "Fresh verification must require a nonzero shader-raw targetPdf proof branch after a new Play run.");
        StringAssert.Contains("RequireReuseFormulaCoverage expected at least one temporal selected-history Jacobian row", script,
            "Full formula verification must fail when temporal selected-history reuse never appears in the checked logs.");
        StringAssert.Contains("RequireReuseFormulaCoverage expected at least one spatial selected-neighbor Jacobian row", script,
            "Full formula verification must fail when spatial selected-neighbor reuse never appears in the checked logs.");
    }

    [Test]
    public void LatestLogVerification_Strict_Fixture_Covers_Runtime_Formula_Gates()
    {
        string testsRoot = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests"));
        string runnerPath = Path.Combine(testsRoot, "Verify-ReSTIRGIStrictFixture.ps1");
        string fixtureDir = Path.Combine(testsRoot, "Fixtures", "ReSTIRGI_StrictPass", "2026-06-17_000000_strict");
        Assert.That(File.Exists(runnerPath), Is.True, $"Strict fixture runner not found: {runnerPath}");
        Assert.That(Directory.Exists(fixtureDir), Is.True, $"Strict fixture directory not found: {fixtureDir}");

        string runner = File.ReadAllText(runnerPath);
        string probe = File.ReadAllText(Path.Combine(fixtureDir, "restir_gi_probe.jsonl"));
        string temporal = File.ReadAllText(Path.Combine(fixtureDir, "restir_gi_temporal_stats.jsonl"));
        string spatial = File.ReadAllText(Path.Combine(fixtureDir, "restir_gi_spatial_stats.jsonl"));
        string final = File.ReadAllText(Path.Combine(fixtureDir, "restir_gi_final_stats.jsonl"));

        StringAssert.Contains("RequireFreshReSTIRGI", runner,
            "Strict fixture runner must exercise the same strict gates used after a new Editor Play run.");
        StringAssert.Contains("RequireReuseFormulaCoverage", runner,
            "Strict fixture runner must require temporal-history and spatial-neighbor formula coverage.");
        StringAssert.Contains("\"initialWeightSum\":4.0", probe,
            "Strict fixture must cover fresh reservoir 1/proposalPdf initialization.");
        StringAssert.Contains("\"selectedPrevious\":true", temporal,
            "Strict fixture must include a selected-history temporal row.");
        StringAssert.Contains("\"selectedPrevJacobian\":2.0", temporal,
            "Strict fixture must cover temporal proposalPdf/jacobian verification.");
        StringAssert.Contains("\"temporalNormalizationDenominator\":4.56", temporal,
            "Strict fixture must cover temporal normalization denominator verification.");
        StringAssert.Contains("\"combinedPrevious\":true", temporal,
            "Strict fixture must cover temporal rows where a history candidate entered the stream.");
        StringAssert.Contains("\"combinedPrevPi\":0.5", temporal,
            "Strict fixture must cover current-selected temporal normalization with history in the stream.");
        StringAssert.Contains("\"temporalNormalizationDenominator\":1.44", temporal,
            "Strict fixture must cover current-selected temporal denominator with combined history piSum.");
        StringAssert.Contains("\"spatialSelectedNeighborReuseProposalPdf\":0.8", spatial,
            "Strict fixture must cover spatial selected-neighbor proposalPdf/jacobian verification.");
        StringAssert.Contains("\"spatialShaderCombinedNeighbors\":1", spatial,
            "Strict fixture must cover clean spatial debug count gates.");
        StringAssert.Contains("\"finalDiagnosticSchemaVersion\":2", final,
            "Strict fixture must cover final schema v2 shader-raw gates.");
        StringAssert.Contains("\"finalReflectedRadianceSource\":\"shaderRaw\"", final,
            "Strict fixture must mark final reflected radiance as shader raw.");
        StringAssert.Contains("\"finalWeightedReflectedRadiance\":[0.8,3.2,1.6]", final,
            "Strict fixture must cover weighted reflected radiance formula verification.");
        StringAssert.Contains("\"finalWeightedReflectedRadiance\":[12.8,128.0,25.6]", final,
            "Strict fixture must cover low-sample firefly fence clamping after reflectedRadiance * weightSum.");
    }

    [Test]
    public void Workflow_Document_Mentions_Terminal_Side_Latest_Log_Verification_Script()
    {
        string workflowPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "memory", "unity-path-tracing", "AI_WORKFLOW.md"));
        Assert.That(File.Exists(workflowPath), Is.True, $"Workflow doc not found: {workflowPath}");

        string workflow = File.ReadAllText(workflowPath);

        StringAssert.Contains("Tests/Verify-LatestReSTIRGILogs.ps1", workflow, "Workflow must document the terminal-side latest-log verification script.");
    }

    [Test]
    public void RestirGI_Final_Diagnostics_Expose_Runtime_WeightSum_And_Weighted_Radiance()
    {
        string tracingSource = File.ReadAllText(ProjectFile("Assets", "Scripts", "Tracing.cs"));
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string giShadeSource = RestirShader("gi_shade.hlsl");
        string giDispatchBody = ExtractMethodBody(tracingSource, "DispatchReSTIRGI");

        StringAssert.Contains("restir_gi_final_stats.jsonl", session);
        StringAssert.Contains("float4(finalRes.radiance, finalRes.weightSum)", giShadeSource);
        StringAssert.Contains("float4(finalRes.contribution, finalRes.selectedWeight)", giShadeSource);
        StringAssert.Contains("float4(gi, rawLum)", giShadeSource);
        StringAssert.Contains("tracingShader.SetInt(\"_RestirInitialReservoirOffset\", initialIdx * pixelCount);",
            giDispatchBody,
            "Final shading must bind the initial reservoir slot that the shader uses for fallback evaluation.");
        StringAssert.Contains("out float3 reflectedRadiance", giShadeSource,
            "GI final shader must expose raw reflected radiance before multiplying by UCW.");
    }

    [Test]
    public void RestirGI_Secondary_Hit_Radiance_Uses_Direct_Light_Raw_Contribution_As_Incident_Radiance()
    {
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string giInitialSource = File.ReadAllText(giInitialSourcePath);
        string secondaryHitBody = ExtractMethodBody(giInitialSource, "EvaluateSecondaryHitRadiance");

        StringAssert.Contains("radiance += sunSample.contribution;", secondaryHitBody,
            "GI secondary hit radiance must stay in incident-radiance units so later primary-surface reevaluation applies the BRDF/proposal normalization exactly once.");
        StringAssert.Contains("radiance += pointSample.contribution;", secondaryHitBody,
            "GI secondary hit radiance must accumulate raw direct-light contribution as the secondary incident radiance quantity.");
        StringAssert.DoesNotContain("radiance += sunSample.illumination;", secondaryHitBody,
            "Using illumination here bakes 1/proposalPdf into sample.radiance and makes the later GI reservoir/final shading chain divide by proposal density twice.");
        StringAssert.DoesNotContain("radiance += pointSample.illumination;", secondaryHitBody,
            "Using illumination here bakes 1/proposalPdf into sample.radiance and makes the later GI reservoir/final shading chain divide by proposal density twice.");
    }

    [Test]
    public void RestirGI_Secondary_Hit_Radiance_Filters_Direct_Light_By_Visibility()
    {
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string giInitialSource = File.ReadAllText(giInitialSourcePath);
        StringAssert.DoesNotContain("IsDirectLightSampleVisible", giInitialSource,
            "Telemetry installation must not add a new secondary-light visibility estimator branch.");
    }

    [Test]
    public void RestirDI_Diagnostics_Expose_Shading_Source_And_Reprojection_Stability_Summary()
    {
        string temporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "temporal_resampling.hlsl"));
        Assert.That(File.Exists(temporalSourcePath), Is.True, $"Temporal resampling source not found: {temporalSourcePath}");

        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));
        string temporalSource = File.ReadAllText(temporalSourcePath);

        StringAssert.Contains("diTemporalHistoryCombined", session);
        StringAssert.Contains("diTemporalHistorySelected", session);
        StringAssert.Contains("diTemporalReprojectionOutOfBounds", session);
        StringAssert.Contains("diTemporalIncompatibleSurface", session);
        StringAssert.Contains("bool selectedPrevious = false;", temporalSource,
            "DI temporal shader must track whether the previous reservoir won selection.");
        StringAssert.Contains("RESTIR_COUNTER_DI_TEMPORAL_HISTORY_SELECTED", temporalSource);
    }

    [Test]
    public void RestirDI_Temporal_Reprojection_Reweights_History_Using_Current_Surface_Target()
    {
        string temporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "ComputeShader", "main", "restir", "temporal_resampling.hlsl"));
        Assert.That(File.Exists(temporalSourcePath), Is.True, $"DI temporal source not found: {temporalSourcePath}");

        string temporalSource = File.ReadAllText(temporalSourcePath);

        StringAssert.Contains("float prevW = prev.selectedWeight * prevSample.targetLum * max((float)prev.sampleCount, 1.0);", temporalSource,
            "Temporal DI must rebuild the history candidate weight from the re-evaluated current-surface target, otherwise stale history dominates selection and freezes noise.");
        StringAssert.DoesNotContain("float prevW = prev.weightSum;", temporalSource,
            "Temporal DI must not reuse the previous surface's raw weightSum after re-evaluating the history sample on the current surface.");
    }

    [Test]
    public void LatestLogVerification_Script_Summarizes_GI_Brightness_And_DI_Reuse_Anomalies()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        string summaryScriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Summarize-ReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");
        Assert.That(File.Exists(summaryScriptPath), Is.True, $"Summary script not found: {summaryScriptPath}");

        string script = File.ReadAllText(scriptPath);
        string summaryScript = File.ReadAllText(summaryScriptPath);

        StringAssert.Contains("[double]$MaxFinalContributionLum", script,
            "Terminal verification must expose an optional brightness threshold so white-out regressions can fail automatically.");
        StringAssert.Contains("Max finalContributionLum exceeded threshold", script,
            "Terminal verification must fail when the final GI contribution exceeds the configured luminance budget.");
        StringAssert.Contains("Max finalContributionLum", script, "Terminal verification script must summarize GI brightness spikes.");
        StringAssert.Contains("Most reused DI sample", script, "Terminal verification script must summarize DI reuse hotspots.");
        StringAssert.Contains("Unique DI light samples", script, "Terminal verification script must summarize DI sample diversity.");
        StringAssert.Contains("Test-CompleteReSTIRGILogDir", script,
            "Terminal verification must skip partial latest log directories that are missing DI/temporal/spatial/final JSONL files.");
        StringAssert.Contains("if ($RequireReuseFormulaCoverage -and $combinedSpatial.Count -eq 0)",
            script,
            "Basic latest-log verification must allow short diagnostic runs that have final GI rows but no spatial-neighbor reuse coverage.");
        StringAssert.Contains("[string]$OutputDir", summaryScript,
            "Summary script must allow direct inspection of a chosen Tools/Output/<timestamp>/ directory.");
        StringAssert.Contains("Top final GI rows", summaryScript,
            "Summary script must print the brightest final GI rows for visual white-out diagnosis.");
        StringAssert.Contains("MaxFinalContributionLum", summaryScript,
            "Summary script must share the same configurable brightness threshold as the verifier.");
        StringAssert.Contains("restir_gi_frame_stats.jsonl", summaryScript,
            "Summary script must consume frame-wide GI delta logs when they are present.");
        StringAssert.Contains("MaxFrameGIDeltaLum", summaryScript,
            "Summary script must expose a threshold for whole-frame GI white-out checks.");
        StringAssert.Contains("maxGIDeltaLum", summaryScript,
            "Summary script must report the brightest frame-wide GI delta pixel.");
        StringAssert.Contains("maxGIDeltaFinalWeightSum", summaryScript,
            "Summary script must print the max-delta pixel's final reservoir weight when detailed frame logs are present.");
        StringAssert.Contains("Test-CompleteReSTIRGILogDir", summaryScript,
            "Summary script must auto-select only complete ReSTIR log directories.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_Final_GI_Estimator_Formula()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("finalWeightedReflectedRadiance", script,
            "Terminal verification must read the final weighted reflected radiance from final diagnostics.");
        StringAssert.Contains("initialWeightedReflectedRadiance", script,
            "Terminal verification must read the initial weighted reflected radiance from final diagnostics.");
        StringAssert.Contains("$expectedContribution =",
            script,
            "Terminal verification must compute the expected final estimator from weighted reflected radiance and MIS weights.");
        StringAssert.Contains("$finalWeighted * [double]$row.misFinalWeight",
            script,
            "Expected contribution must include the final reservoir MIS branch.");
        StringAssert.Contains("$initialWeighted * [double]$row.misInitialWeight",
            script,
            "Expected contribution must include the initial reservoir fallback branch.");
        StringAssert.Contains("Assert-Near $actualContribution $expectedContribution",
            script,
            "Terminal verification must compare logged finalContribution against the shader estimator formula.");
        StringAssert.Contains("Assert-Near $actualDelta $actualContribution",
            script,
            "Terminal verification must ensure GlobalColors delta matches the logged final GI contribution.");
        StringAssert.Contains("Assert-WeightedReflectedRadiance",
            script,
            "Terminal verification must validate reflectedRadiance * weightSum, including the contribution fence.");
        StringAssert.Contains("RestirGIFireflyLumLimit = 128.0",
            script,
            "Terminal verification must match the shader GI firefly luminance fence.");
        StringAssert.Contains("Formula-checked final rows",
            script,
            "Terminal verification output must report how many final rows passed the formula check.");
        StringAssert.Contains("Weighted-reflected formula branches",
            script,
            "Terminal verification output must report weighted reflected radiance formula coverage.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_TargetPdf_From_Reflected_Radiance()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("Get-VectorMaxComponent", script,
            "Terminal verification must compute max-channel target values from reflected radiance.");
        StringAssert.Contains("finalReflectedRadiance", script,
            "Terminal verification must inspect final reflected radiance for p_hat/targetLum consistency.");
        StringAssert.Contains("initialReflectedRadiance", script,
            "Terminal verification must inspect initial reflected radiance for p_hat/targetLum consistency.");
        StringAssert.Contains("Test-FinalRawShaderSchema", script,
            "Terminal verification must detect final diagnostic schema rows where reflected radiance is known to be shader raw.");
        StringAssert.Contains("finalReflectedRadianceSource=shaderRaw", script,
            "Terminal verification must enforce shaderRaw source labels on finalDiagnosticSchemaVersion>=2 rows.");
        StringAssert.Contains("Assert-Near ([double]$row.finalTargetLum) $finalTargetFromRadiance",
            script,
            "Terminal verification must check finalTargetLum equals max(finalReflectedRadiance).");
        StringAssert.Contains("Assert-Near ([double]$row.initialTargetLum) $initialTargetFromRadiance",
            script,
            "Terminal verification must check initialTargetLum equals max(initialReflectedRadiance).");
        StringAssert.Contains("TargetPdf-checked final branches",
            script,
            "Terminal verification output must report targetPdf formula coverage.");
        StringAssert.Contains("Shader-raw targetPdf branches",
            script,
            "Terminal verification output must report how many targetPdf checks came from schema-marked shader raw reflected radiance.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_Fresh_GI_Reservoir_ProposalPdf_Inverse()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("$RestirGIMinProposalPdf = 1e-3", script,
            "Terminal verification must mirror RESTIR_GI_MIN_PROPOSAL_PDF for fresh reservoir inverse-PDF checks.");
        StringAssert.Contains("$freshInitialRows", script,
            "Terminal verification must collect fresh initial reservoir rows from restir_gi_probe.jsonl.");
        StringAssert.Contains("$expectedInitialWeight = 1.0 / [Math]::Max([double]$row.initialProposalPdf, $RestirGIMinProposalPdf)", script,
            "Fresh initial reservoir weightSum must be checked against 1 / max(proposalPdf, minPdf).");
        StringAssert.Contains("Assert-Near ([double]$row.initialWeightSum) $expectedInitialWeight",
            script,
            "Terminal verification must compare initialWeightSum against the inverse proposal PDF.");
        StringAssert.Contains("Assert-Near ([double]$row.initialSelectedWeight) ([double]$row.initialWeightSum)",
            script,
            "Terminal verification must check selectedWeight mirrors weightSum for fresh GI reservoirs.");
        StringAssert.Contains("Assert-Near ([double]$row.initialProposalPdf) ([double]$row.secondaryProposalPdf)",
            script,
            "Terminal verification must check the staged secondary proposalPdf propagates into the initial reservoir.");
        StringAssert.Contains("ProposalPdf-checked fresh reservoirs",
            script,
            "Terminal verification output must report proposalPdf inverse-weight coverage.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_Temporal_Current_Only_Normalization()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("$temporalCurrentOnlyRows", script,
            "Terminal verification must isolate temporal rows where the current reservoir, not history, won selection.");
        StringAssert.Contains("-not (Test-TrueField $_ 'combinedPrevious')",
            script,
            "Current-only temporal checks must exclude rows where history entered the RIS stream but did not win selection.");
        StringAssert.Contains("Assert-Near ([double]$row.currentTargetPdf) $targetLum",
            script,
            "Current-only temporal rows must keep currentTargetPdf equal to targetLum.");
        StringAssert.Contains("Assert-Near ([double]$row.selectedTargetPdf) $targetLum",
            script,
            "Current-only temporal rows must keep selectedTargetPdf equal to targetLum.");
        StringAssert.Contains("Assert-Near ([double]$row.temporalPi) $targetLum",
            script,
            "Current-only temporal rows must set pi to the selected target.");
        StringAssert.Contains("Assert-Near ([double]$row.temporalPiSum) $expectedPiSum",
            script,
            "Current-only temporal rows must compute piSum from targetLum * sampleCountM.");
        StringAssert.Contains("Assert-Near ([double]$row.temporalNormalizationDenominator) $expectedDenominator",
            script,
            "Current-only temporal rows must compute normalizationDenominator as selectedTargetPdf * temporalPiSum.");
        StringAssert.Contains("Assert-Near ([double]$row.weightSum) $expectedWeight",
            script,
            "Current-only temporal rows must finalize weightSum back to the inverse proposal PDF.");
        StringAssert.Contains("Temporal current-only formula rows",
            script,
            "Terminal verification output must report temporal current-only formula coverage.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_Temporal_Selected_History_Jacobian_When_Present()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("$temporalSelectedHistoryRows", script,
            "Terminal verification must isolate temporal rows where selected history carries valid Jacobian data.");
        StringAssert.Contains("selectedPrevOriginalProposalPdf / [double]$row.selectedPrevJacobian",
            script,
            "Selected-history temporal rows must verify proposalPdf is transformed by the reuse Jacobian.");
        StringAssert.Contains("selectedPrevReuseProposalPdf",
            script,
            "Selected-history temporal rows must consume the reused proposalPdf diagnostic field.");
        StringAssert.Contains("$expectedPiSum = [double]$row.currentTargetPdf * $currentM + [double]$row.combinedPrevPi * [double]$row.combinedPrevSampleCountM",
            script,
            "Selected-history temporal rows must expand temporalPiSum from current and combined-history domains.");
        StringAssert.Contains("Assert-Near ([double]$row.temporalPi) ([double]$row.combinedPrevPi)",
            script,
            "Selected-history temporal rows must use the selected sample target in the combined history domain as pi.");
        StringAssert.Contains("Assert-Near ([double]$row.temporalPiSum) $expectedPiSum",
            script,
            "Selected-history temporal rows must verify the full piSum formula, not only the denominator shape.");
        StringAssert.Contains("Assert-Near ([double]$row.temporalNormalizationDenominator) $expectedDenominator",
            script,
            "Selected-history temporal rows must still compute normalizationDenominator as selectedTargetPdf * temporalPiSum.");
        StringAssert.Contains("Temporal selected-history Jacobian rows",
            script,
            "Terminal verification output must report optional selected-history Jacobian coverage.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_Temporal_Current_Selected_With_History_When_Present()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("$temporalCurrentSelectedHistoryRows", script,
            "Terminal verification must isolate temporal rows where history entered the stream but current won selection.");
        StringAssert.Contains("combinedPrevOriginalProposalPdf / [double]$row.combinedPrevJacobian",
            script,
            "Current-selected temporal history rows must verify the combined history proposalPdf is transformed by the Jacobian.");
        StringAssert.Contains("$currentM = [Math]::Max([double]$row.sampleCountM - [double]$row.combinedPrevSampleCountM, 0.0)",
            script,
            "Current-selected temporal history rows must split current M from combined history M.");
        StringAssert.Contains("$expectedPiSum = [double]$row.currentTargetPdf * $currentM + [double]$row.combinedPrevPi * [double]$row.combinedPrevSampleCountM",
            script,
            "Current-selected temporal history rows must include combinedPrevPi in temporalPiSum.");
        StringAssert.Contains("Temporal current-selected history rows",
            script,
            "Terminal verification output must report current-selected temporal history coverage.");
    }

    [Test]
    public void LatestLogVerification_Script_Checks_Spatial_Selected_Neighbor_Jacobian_When_Present()
    {
        string scriptPath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Tests", "Verify-LatestReSTIRGILogs.ps1"));
        Assert.That(File.Exists(scriptPath), Is.True, $"Verification script not found: {scriptPath}");

        string script = File.ReadAllText(scriptPath);

        StringAssert.Contains("$spatialSelectedNeighborRows", script,
            "Terminal verification must isolate spatial rows where a selected neighbor carries valid Jacobian data.");
        StringAssert.Contains("spatialSelectedNeighborOriginalProposalPdf / [double]$row.spatialSelectedNeighborJacobian",
            script,
            "Selected-neighbor spatial rows must verify proposalPdf is transformed by the reuse Jacobian.");
        StringAssert.Contains("spatialSelectedNeighborReuseProposalPdf",
            script,
            "Selected-neighbor spatial rows must consume the reused proposalPdf diagnostic field.");
        StringAssert.Contains("Assert-Near ([double]$row.spatialSelectedTargetPdf) ([double]$row.spatialSelectedNeighborTargetPdf)",
            script,
            "Selected-neighbor spatial rows must keep the selected target equal to the selected-neighbor target.");
        StringAssert.Contains("Assert-Near ([double]$row.spatialNormalizationDenominator) $expectedDenominator",
            script,
            "Selected-neighbor spatial rows must compute normalizationDenominator as selectedTargetPdf * spatialPiSum.");
        StringAssert.Contains("Spatial selected-neighbor Jacobian rows",
            script,
            "Terminal verification output must report optional spatial selected-neighbor Jacobian coverage.");
    }

    [Test]
    public void Runtime_State_Change_Log_Is_Persisted_For_Mode_Toggle_Analysis()
    {
        string tracingSource = File.ReadAllText(ProjectFile("Assets", "Scripts", "Tracing.cs"));
        string session = File.ReadAllText(ProjectFile("Assets", "Scripts", "ReSTIRDiagnosticsSession.cs"));

        StringAssert.Contains("restir_events.jsonl", session);
        StringAssert.Contains("RecordStateChange", session);
        StringAssert.Contains("reason", session);
        StringAssert.Contains("generation", session);
        StringAssert.Contains("ScheduleResetCapture(reason, sampleCount)", tracingSource);
        StringAssert.Contains("modeFlags", session,
            "The next correlated packet must record current DI/GI/history mode flags after a reset.");
    }

    [Test]
    public void ResetPaths_ClearAccumulationRenderTargets_When_Runtime_Mode_Changes()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string resetSampleCountBody = ExtractMethodBody(tracingSource, "ResetSampleCount");
        string resetAccumulationOnlyBody = ExtractMethodBody(tracingSource, "ResetAccumulationOnly");
        string clearAccumulationBody = ExtractMethodBody(tracingSource, "ClearAccumulationRenderTargets");
        string clearRenderTextureBody = ExtractMethodBody(tracingSource, "ClearRenderTexture");

        StringAssert.Contains("frameId = 0;", resetSampleCountBody,
            "Full resets must restart frameId so temporal/random sequences do not bridge across runtime-mode toggles.");
        StringAssert.Contains("ClearAccumulationRenderTargets();", resetSampleCountBody,
            "Full resets must clear target/frameConverged so stale accumulated lighting is not blended after toggling runtime modes.");
        StringAssert.Contains("frameId = 0;", resetAccumulationOnlyBody,
            "Accumulation-only resets must restart frameId so denoise/ReSTIR toggles do not keep advancing old temporal sequences.");
        StringAssert.Contains("ClearAccumulationRenderTargets();", resetAccumulationOnlyBody,
            "Accumulation-only resets must clear target/frameConverged so old accumulation is not reused after camera/runtime setting changes.");
        StringAssert.Contains("ClearRenderTexture(target);", clearAccumulationBody,
            "Accumulation reset helper must clear raw path-tracing target.");
        StringAssert.Contains("ClearRenderTexture(frameConverged);", clearAccumulationBody,
            "Accumulation reset helper must clear denoised accumulation target.");
        StringAssert.Contains("GL.Clear(false, true, Color.clear);", clearRenderTextureBody,
            "Render-texture clear helper must zero color contents instead of only resetting counters.");
    }

    private static Type TracingType()
    {
        Type type = Type.GetType("Tracing, Assembly-CSharp");
        Assert.That(type, Is.Not.Null, "Tracing type not found in Assembly-CSharp");
        return type;
    }

    private static string ExtractMethodBody(string source, string methodName)
    {
        Match signature = Regex.Match(
            source,
            $@"(?:private\s+|public\s+)?(?:static\s+)?[\w<>\[\],]+\s+{Regex.Escape(methodName)}\s*\([^)]*\)\s*\{{",
            RegexOptions.Multiline);
        Assert.That(signature.Success, Is.True, $"Method not found: {methodName}");

        int bodyStart = signature.Index + signature.Length;
        int depth = 1;
        for (int i = bodyStart; i < source.Length; i++)
        {
            if (source[i] == '{')
                depth++;
            else if (source[i] == '}')
                depth--;

            if (depth == 0)
                return source.Substring(bodyStart, i - bodyStart);
        }

        Assert.Fail($"Method body not terminated: {methodName}");
        return string.Empty;
    }

    private static string ProjectFile(params string[] parts)
    {
        return Path.GetFullPath(Path.Combine(
            TestContext.CurrentContext.TestDirectory,
            "..",
            "..",
            Path.Combine(parts)));
    }

    private static string RestirShader(string fileName)
    {
        return File.ReadAllText(ProjectFile("Assets", "ComputeShader", "main", "restir", fileName));
    }
}
