using System.Reflection;
using System.Text.RegularExpressions;
using System.IO;
using NUnit.Framework;

public class TracingContractsTests
{
    [Test]
    public void Tracing_Exposes_ReSTIR_GI_Toggle()
    {
        FieldInfo field = typeof(Tracing).GetField("UseReSTIRGI", BindingFlags.NonPublic | BindingFlags.Instance);
        Assert.That(field, Is.Not.Null);
        Assert.That(field.FieldType, Is.EqualTo(typeof(bool)));
    }

    [Test]
    public void Tracing_Defines_IndirectReservoirStride_As_80_Bytes()
    {
        FieldInfo field = typeof(Tracing).GetField("IndirectReservoirStride", BindingFlags.NonPublic | BindingFlags.Static);
        Assert.That(field, Is.Not.Null);
        Assert.That((int)field.GetValue(null), Is.EqualTo(80));
    }

    [Test]
    public void Tracing_Exposes_ReSTIR_GI_Diagnostic_Detail_Toggle()
    {
        FieldInfo field = typeof(Tracing).GetField("WriteReSTIRGIDiagnosticDetails", BindingFlags.NonPublic | BindingFlags.Instance);
        Assert.That(field, Is.Not.Null);
        Assert.That(field.FieldType, Is.EqualTo(typeof(bool)));
    }

    [Test]
    public void Tracing_Uses_Separate_Direct_And_Indirect_History_State()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
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
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string methodBody = ExtractMethodBody(source, "AppendReSTIRGIProbeJson");

        StringAssert.Contains("restir_gi_probe.jsonl", methodBody);
        StringAssert.Contains("\"probeId\"", methodBody);
        StringAssert.Contains("\"primaryHit\"", methodBody);
        StringAssert.Contains("\"primaryDistance\"", methodBody);
        StringAssert.Contains("\"secondaryProposalPdf\"", methodBody);
        StringAssert.Contains("\"primaryAlbedo\"", methodBody);
        StringAssert.Contains("\"primaryEmission\"", methodBody);
        StringAssert.Contains("\"primaryRoughness\"", methodBody);
        StringAssert.Contains("\"primaryMetallic\"", methodBody);
        StringAssert.Contains("\"primaryAlpha\"", methodBody);
        StringAssert.Contains("\"primaryIor\"", methodBody);
        StringAssert.Contains("\"secondaryThroughput\"", methodBody);
        StringAssert.Contains("\"secondaryEmissionRadiance\"", methodBody);
        StringAssert.Contains("\"initialReservoirIndex\"", methodBody);
        StringAssert.Contains("\"activeReservoirIndex\"", methodBody);
        StringAssert.Contains("\"initialValid\"", methodBody);
        StringAssert.Contains("\"initialProposalPdf\"", methodBody);
        StringAssert.Contains("\"initialTargetLum\"", methodBody);
        StringAssert.Contains("\"initialWeightSum\"", methodBody);
        StringAssert.Contains("\"initialSelectedWeight\"", methodBody);
        StringAssert.Contains("\"initialSampleCountM\"", methodBody);
        StringAssert.Contains("\"activeValid\"", methodBody);
        StringAssert.Contains("\"activeProposalPdf\"", methodBody);
        StringAssert.Contains("\"activeTargetLum\"", methodBody);
        StringAssert.Contains("\"activeWeightSum\"", methodBody);
        StringAssert.Contains("\"activeSelectedWeight\"", methodBody);
        StringAssert.Contains("\"activeSampleCountM\"", methodBody);
        StringAssert.Contains("\"probeClass\"", methodBody);
        StringAssert.Contains("\"secondaryFlagsHex\"", methodBody);
        StringAssert.Contains("\"stage1InvalidReason\"", methodBody);
        StringAssert.Contains("\"secondaryReserved\"", methodBody);
    }

    [Test]
    public void Tracing_Classifies_ReSTIR_GI_Probe_Sample_Types_For_Runtime_Diagnostics()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string methodBody = ExtractMethodBody(source, "AppendReSTIRGIProbeJson");

        StringAssert.Contains("ClassifyReSTIRGIProbe", source);
        StringAssert.Contains("string probeClass = ClassifyReSTIRGIProbe", methodBody);
        StringAssert.Contains("secondarySurface.flags >= 8.0f", source);
        StringAssert.Contains("secondarySurface.flags < 0.0f", source);
        StringAssert.Contains("IsReservoirNumericallyValid(activeReservoir)", source);
        StringAssert.Contains("IsReservoirNumericallyValid(initialReservoir)", source);
        StringAssert.Contains("\"probeClass\":\"", methodBody);
        StringAssert.Contains("DescribeReSTIRGIStage1InvalidReason", source);
        StringAssert.Contains("secondarySurface.flags <= -11.0f", source);
        StringAssert.Contains("secondarySurface.flags <= -10.0f", source);
        StringAssert.Contains("secondarySurface.flags <= -3.0f", source);
        StringAssert.Contains("secondarySurface.flags <= -2.0f", source);
        StringAssert.Contains("\"stage1InvalidReason\":\"", methodBody);
        StringAssert.Contains("secondarySurface.reserved >= 1.5f", source);
        StringAssert.Contains("secondarySurface.reserved >= 0.5f", source);
        StringAssert.Contains("delta_specular_backfacing", source);
        StringAssert.Contains("ggx_specular_backfacing", source);
    }

    [Test]
    public void RestirGI_BXDF_Diagnostics_Differentiate_Delta_And_GGX_Zero_Throughput_Cases()
    {
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(bxdfSourcePath), Is.True, $"BXDF source not found: {bxdfSourcePath}");
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string bxdfSource = File.ReadAllText(bxdfSourcePath);
        string giInitialSource = File.ReadAllText(giInitialSourcePath);

        StringAssert.Contains("out float zeroReasonCode", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 0.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 1.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 2.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 3.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 4.0;", bxdfSource);
        StringAssert.Contains("zeroReasonCode = 5.0;", bxdfSource);
        StringAssert.Contains("EvaluateBXDFWithDotAndPDFDetailed(hit, ray, f_brdf, sampledSpecular, zeroReasonCode);", bxdfSource);
        StringAssert.Contains("float throughputZeroReason;", giInitialSource);
        StringAssert.Contains("data.reserved = throughputZeroReason;", giInitialSource);
    }

    [Test]
    public void RestirGI_RoughGGX_Specular_Backfacing_Remains_A_Diagnostic_Invalid_In_Stage1()
    {
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
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
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
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
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
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
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string diBody = ExtractMethodBody(source, "DispatchReSTIRDI");
        string giBody = ExtractMethodBody(source, "DispatchReSTIRGI");
        string probeBody = ExtractMethodBody(source, "WriteReSTIRGIProbeIfNeeded");
        string appendBody = ExtractMethodBody(source, "AppendReSTIRGIProbeJson");

        StringAssert.Contains("_RestirGbuffer\", _globalHits", diBody);
        StringAssert.DoesNotContain("_RestirGbuffer\", _primarySurfaceHistory", diBody);

        StringAssert.Contains("_RestirGbuffer\", _globalHits", giBody);
        StringAssert.DoesNotContain("_RestirGbuffer\", _primarySurfaceHistory", giBody);

        StringAssert.Contains("_globalHits.GetData", probeBody);
        StringAssert.Contains("_secondarySurfaces.GetData", probeBody);
        StringAssert.DoesNotContain("_primarySurfaceHistory.GetData", probeBody);
        StringAssert.Contains("IsPrimaryHitValid(primaryHit)", appendBody);
        StringAssert.Contains("_initialGIProbeReadback", probeBody);
        StringAssert.Contains("_activeGIProbeReadback", probeBody);
        StringAssert.Contains("_secondarySurfaceProbeReadback", probeBody);
    }

    [Test]
    public void RestirGI_Stages_Secondary_Surface_Payload_For_Shading()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        string globalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "global.hlsl"));
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
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string computeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "Tracing.compute"));
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
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
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
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);

        StringAssert.Contains("AppendReSTIRGITemporalSummaryJson(", tracingSource);
        StringAssert.Contains("restir_gi_temporal_stats.jsonl", tracingSource);
        StringAssert.Contains("\"selectedProbeId\":", tracingSource);
        StringAssert.Contains("\"proposalPdf\":", tracingSource);
        StringAssert.Contains("\"targetLum\":", tracingSource);
        StringAssert.Contains("\"weightSum\":", tracingSource);
        StringAssert.Contains("\"selectedWeight\":", tracingSource);
        StringAssert.Contains("\"sampleCountM\":", tracingSource);
    }

    [Test]
    public void RestirGI_Final_Diagnostics_Are_Persisted_To_Independent_Jsonl()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);

        StringAssert.Contains("AppendReSTIRGIFinalSummaryJson(", tracingSource);
        StringAssert.Contains("restir_gi_final_stats.jsonl", tracingSource);
        StringAssert.Contains("\"selectedProbeId\":", tracingSource);
        StringAssert.Contains("\"finalContribution\":", tracingSource);
        StringAssert.Contains("\"finalContributionPositive\":", tracingSource);
        StringAssert.Contains("\"globalLightDelta\":", tracingSource);
        StringAssert.Contains("\"misFinalWeight\":", tracingSource);
        StringAssert.Contains("\"misInitialWeight\":", tracingSource);
    }

    [Test]
    public void RestirGI_Temporal_Uses_Same_RW_Buffer_Read_Pattern_As_DI_Temporal()
    {
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        string diTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "temporal_resampling.hlsl"));
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
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string computeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "Tracing.compute"));
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
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
        StringAssert.Contains("IndirectReservoirs[_RestirSpatialReservoirOffset + id.x] = outR;", giSpatialSource);
    }

    [Test]
    public void RestirGI_Reuses_Reservoirs_With_Combine_And_Finalize_Semantics()
    {
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
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
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
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
        string giTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_temporal.hlsl"));
        Assert.That(File.Exists(giTemporalSourcePath), Is.True, $"GI temporal source not found: {giTemporalSourcePath}");

        string giTemporalSource = File.ReadAllText(giTemporalSourcePath);

        Assert.That(
            Regex.IsMatch(
                giTemporalSource,
                @"selectedPrevious\s*=\s*true\s*;\s*selectedTargetPdf\s*=\s*prevTargetLumCur\s*;\s*selectedPrevSurface\s*=\s*hdPrev\s*;\s*selectedPrevCandidate\s*=\s*prevCandidate\s*;\s*selectedPrevTargetPdf\s*=\s*prevTargetLumCur\s*;\s*break\s*;",
                RegexOptions.Singleline),
            Is.True,
            "GI temporal reuse should stop after selecting the first compatible reprojected candidate.");
    }

    [Test]
    public void RestirGI_Spatial_Uses_Two_Pass_Bias_Correction_For_MultiNeighborReuse()
    {
        string giSpatialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_spatial.hlsl"));
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
        StringAssert.DoesNotContain("break;", giSpatialSource, "GI spatial reuse should not early-break after the first selected neighbor when using multi-neighbor bias correction.");
        Assert.That(
            Regex.IsMatch(
                giSpatialSource,
                @"if\s*\(candidateSelected\)\s*\{\s*selected\s*=\s*neighborSampleIdx\s*;\s*selectedTargetPdf\s*=\s*neighborTargetLumCur\s*;\s*\}",
                RegexOptions.Singleline),
            Is.True,
            "GI spatial reuse should keep track of the selected neighbor index for the second normalization pass.");
    }

    [Test]
    public void RestirGI_RuntimeDiagnostics_Report_SelectedProbeSpatialNeighborStats()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(tracingSourcePath), Is.True, $"Tracing source not found: {tracingSourcePath}");

        string tracingSource = File.ReadAllText(tracingSourcePath);
        string giBody = ExtractMethodBody(tracingSource, "DispatchReSTIRGI");

        StringAssert.Contains("TrySelectActiveGIProbePixel", tracingSource);
        StringAssert.Contains("Temporal reservoir probe sampleCount=", giBody);
        StringAssert.Contains("probeId={selectedProbeId}", giBody);
        StringAssert.Contains("Initial reservoir probe sampleCount=", giBody);
        StringAssert.Contains("Initial shading debug sampleCount=", giBody);
        StringAssert.DoesNotContain("Stage2 reservoir probe sampleCount=", giBody);
        StringAssert.DoesNotContain("Stage2 debug sampleCount=", giBody);
        StringAssert.Contains("ComputeCenterSpatialNeighborStats", giBody);
        StringAssert.Contains("_restirGISpatialDebugReadback", tracingSource);
        StringAssert.Contains("_RestirDebugPixelIndex", giBody);
        StringAssert.Contains("selectedProbeX", giBody);
        StringAssert.Contains("selectedProbeY", giBody);
        StringAssert.Contains("selectedProbeId", giBody);
        StringAssert.Contains("temporalReservoirIndex = temporalIdx * pixelCount + selectedProbePixelIndex", giBody);
        StringAssert.Contains("spatialShaderCompatibleNeighbors=", giBody);
        StringAssert.Contains("spatialShaderReevaluateNeighbors=", giBody);
        StringAssert.Contains("spatialShaderJacobianNeighbors=", giBody);
        StringAssert.Contains("spatialShaderCombinedNeighbors=", giBody);
        StringAssert.Contains("spatialShaderReevaluateFailInvalidSample=", giBody);
        StringAssert.Contains("spatialShaderReevaluateFailInvalidSurface=", giBody);
        StringAssert.Contains("spatialShaderReevaluateFailDistance=", giBody);
        StringAssert.Contains("spatialShaderReevaluateFailBrdf=", giBody);
        StringAssert.Contains("spatialShaderReevaluateFailBackfacing=", giBody);
        StringAssert.Contains("spatialShaderReevaluateFailZeroTarget=", giBody);
        StringAssert.Contains("WriteReSTIRGIDiagnosticDetails", giBody);
        StringAssert.Contains("Spatial selected-probe detail", giBody);
        StringAssert.Contains("selectedProbeId=", giBody);
        StringAssert.Contains("dotCurrentNormalToSecondary=", giBody);
        StringAssert.Contains("compatibleNeighbors=", giBody);
        StringAssert.Contains("activeReservoirNeighbors=", giBody);
        StringAssert.Contains("validSurfaceNeighbors=", giBody);
        StringAssert.Contains("Spatial selected-probe stats", giBody);
        StringAssert.Contains("AppendReSTIRGISpatialSummaryJson(", tracingSource);
        StringAssert.Contains("AppendReSTIRGIFinalSummaryJson(", tracingSource);
        StringAssert.Contains("restir_gi_spatial_stats.jsonl", tracingSource);
        StringAssert.DoesNotContain("Spatial center-neighbor stats", giBody);
        StringAssert.DoesNotContain("Spatial center-neighbor detail", giBody);
    }

    [Test]
    public void RestirGI_FinalShading_Uses_CurrentSurfaceAndVisibility()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string giShadeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_shade.hlsl"));
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
        StringAssert.Contains("ComputeFinalGIMISWeight", giShadeSource);
    }

    [Test]
    public void RestirGI_Reevaluation_Uses_TargetContribution_Not_BRDF_Proposal_Pdf_Gating()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
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
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        string giShadeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_shade.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        Assert.That(File.Exists(giShadeSourcePath), Is.True, $"GI shade source not found: {giShadeSourcePath}");

        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);
        string giShadeSource = File.ReadAllText(giShadeSourcePath);

        // Helper signature: thin wrapper that returns reflectedRadiance only.
        StringAssert.Contains("bool EvaluateIndirectSampleAtSurface(", giReservoirSource);
        StringAssert.Contains("EvaluateIndirectSampleAtSurface(hd, res, reflectedRadiance)", giShadeSource);
        // Final shading multiplies reflectedRadiance by reservoir.weightSum (RTXDI FinalShading.hlsl:66 parity).
        StringAssert.Contains("weightedReflectedRadiance = reflectedRadiance * res.weightSum;", giShadeSource);
        // Old dead out-param patterns must be gone.
        StringAssert.DoesNotContain("trueBrdf, weightedRadiance", giShadeSource);
        StringAssert.DoesNotContain("weightedRadiance = sample.radiance * sample.selectedWeight", giReservoirSource);
        // Pre-fix shading-by-selectedWeight pattern must be gone.
        StringAssert.DoesNotContain("weightedReflectedRadiance = reflectedRadiance * res.selectedWeight", giShadeSource);
    }

    [Test]
    public void RestirGI_ReservoirValidity_Uses_Explicit_Finite_Guards()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
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
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        Assert.That(File.Exists(giInitialSourcePath), Is.True, $"GI initial source not found: {giInitialSourcePath}");

        string giInitialSource = File.ReadAllText(giInitialSourcePath);

        StringAssert.Contains("float3 throughputSample;", giInitialSource);
        StringAssert.Contains("EvaluateBXDFWithDotAndPDF(primaryHit, bounceRay, throughputSample);", giInitialSource);
        StringAssert.Contains("if (!IsGISecondaryBypass(sampleFlags))", giInitialSource);
        StringAssert.Contains("data.throughput = 1.0;", giInitialSource);
        StringAssert.Contains("EvaluateIndirectRadianceAtSurface(_RestirGbuffer[id.x], secondary.position, secondaryRadiance, brdfAtPrimary, contribution)", giInitialSource);
    }

    [Test]
    public void RestirGI_DeltaSpecular_Samples_Bypass_Reservoir_And_Shade_Directly()
    {
        string tracingSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        string giInitialSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_initial.hlsl"));
        string bxdfSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "bxdf.hlsl"));
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
        StringAssert.Contains("GlobalColors[id.x].L += max(contribution, 0.0);", giInitialSource);
        StringAssert.Contains("SetBuffer(kernelShadeGISecondarySurfaces, \"GlobalColors\", _globalColors)", tracingSource);
        StringAssert.Contains("EvaluateBXDFWithDotAndPDFDetailed", giInitialSource);
        StringAssert.Contains("void EvaluateBXDFWithDotAndPDFDetailed(", bxdfSource);
        StringAssert.Contains("out bool sampledSpecular", bxdfSource);
    }

    [Test]
    public void SelfTest_Disables_Verbose_ReSTIR_GI_Detail_Logs()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Editor", "SelfTest.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"SelfTest source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string methodBody = ExtractMethodBody(source, "PrepareReSTIRGIValidation");

        StringAssert.Contains("SetPrivateField(tracing, \"WriteReSTIRGIDiagnosticDetails\", false);", methodBody);
    }

    [Test]
    public void Tracing_Warns_When_ReSTIR_GI_Is_Enabled_Without_ReSTIR_DI()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);

        StringAssert.Contains("WarnIfReSTIRDirectLightingConfigurationIsRisky", source);
        StringAssert.Contains("UseReSTIRGI && !UseReSTIRDI", source);
        StringAssert.Contains("ReSTIR GI is enabled while ReSTIR DI is disabled", source);
    }

    [Test]
    public void Tracing_Defines_ReSTIR_DI_Diagnostic_Output_Contract()
    {
        string sourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "Scripts", "Tracing.cs"));
        Assert.That(File.Exists(sourcePath), Is.True, $"Tracing source not found: {sourcePath}");

        string source = File.ReadAllText(sourcePath);
        string methodBody = ExtractMethodBody(source, "AppendReSTIRDISummaryJson");

        StringAssert.Contains("restir_di_stats.jsonl", methodBody);
        StringAssert.Contains("\"reservoirValid\"", methodBody);
        StringAssert.Contains("\"lightType\"", methodBody);
        StringAssert.Contains("\"lightIndex\"", methodBody);
        StringAssert.Contains("\"proposalPdf\"", methodBody);
        StringAssert.Contains("\"targetLum\"", methodBody);
        StringAssert.Contains("\"selectedWeight\"", methodBody);
        StringAssert.Contains("\"sampleCountM\"", methodBody);
        StringAssert.Contains("\"diContribution\"", methodBody);
        StringAssert.Contains("\"globalLightBefore\"", methodBody);
        StringAssert.Contains("\"globalLightAfter\"", methodBody);
        StringAssert.Contains("\"globalLightDelta\"", methodBody);
        StringAssert.Contains("\"globalLightDeltaPositive\"", methodBody);
    }

    [Test]
    public void RestirDI_Temporal_Reuse_Does_Not_Multiply_WeightSum_By_SampleCount_Twice()
    {
        string diTemporalSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "temporal_resampling.hlsl"));
        Assert.That(File.Exists(diTemporalSourcePath), Is.True, $"DI temporal source not found: {diTemporalSourcePath}");

        string diTemporalSource = File.ReadAllText(diTemporalSourcePath);

        StringAssert.Contains("float curW = cur.weightSum;", diTemporalSource);
        StringAssert.Contains("float prevW = prev.weightSum;", diTemporalSource);
        StringAssert.DoesNotContain("cur.weightSum * (float)max(cur.sampleCount, 1u)", diTemporalSource);
        StringAssert.DoesNotContain("prev.weightSum * (float)max(prev.sampleCount, 1u)", diTemporalSource);
    }

    [Test]
    public void RestirGI_SelectedWeight_Mirrors_WeightSum_For_Stride_Compat()
    {
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
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
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
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
    public void RestirGI_RIS_Stream_Weight_Has_No_Extra_M_Multiplier()
    {
        // GetIndirectReservoirRISWeight must NOT multiply by candidate.sampleCount;
        // CombineIndirectReservoirs already adds candidate.M into reservoir.M.
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
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
    public void RestirGI_FinalShading_Uses_WeightSum_Not_SelectedWeight()
    {
        string giShadeSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_shade.hlsl"));
        Assert.That(File.Exists(giShadeSourcePath), Is.True, $"GI shade source not found: {giShadeSourcePath}");

        string giShadeSource = File.ReadAllText(giShadeSourcePath);
        StringAssert.Contains("reflectedRadiance * res.weightSum",
            giShadeSource,
            "EvaluateVisibleGISample must use radiance * weightSum (RTXDI FinalShading parity).");
    }

    [Test]
    public void RestirGI_Finite_Limit_Tightened_For_Firefly_Containment()
    {
        // Old 1e30 allowed 1e+27 reservoirs to spread across frames. 1e6 keeps a
        // healthy headroom over typical 1/p_hat (1..1e3) but kills the runaway tail.
        string giReservoirSourcePath = Path.GetFullPath(Path.Combine(TestContext.CurrentContext.TestDirectory, "..", "..", "..", "Assets", "ComputeShader", "main", "restir", "gi_reservoir.hlsl"));
        Assert.That(File.Exists(giReservoirSourcePath), Is.True, $"GI reservoir source not found: {giReservoirSourcePath}");
        string giReservoirSource = File.ReadAllText(giReservoirSourcePath);

        StringAssert.Contains("static const float RESTIR_GI_FINITE_LIMIT = 1e6;",
            giReservoirSource,
            "RESTIR_GI_FINITE_LIMIT must be 1e6 to contain firefly bubble propagation.");
    }

    private static string ExtractMethodBody(string source, string methodName)
    {
        Match signature = Regex.Match(
            source,
            $@"private\s+(?:static\s+)?void\s+{methodName}\s*\([^)]*\)\s*\{{",
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
}
