using NUnit.Framework;
using UnityEditor;
using UnityEngine;

public sealed class ReSTIRAlgorithmGpuTests
{
    [Test]
    public void ProductionHlsl_Computes_Expected_Reservoir_Weights_And_Material_Compatibility()
    {
        Assume.That(SystemInfo.supportsComputeShaders, Is.True, "Compute shaders are unavailable on this test device.");

        ComputeShader shader = AssetDatabase.LoadAssetAtPath<ComputeShader>(
            "Assets/Tests/Editor/ReSTIRAlgorithmTests.compute");
        Assert.That(shader, Is.Not.Null);

        using var output = new ComputeBuffer(1, sizeof(float) * 4);
        int kernel = shader.FindKernel("kernel_test_restir_algorithm");
        shader.SetBuffer(kernel, "ReSTIRAlgorithmTestOutput", output);
        shader.Dispatch(kernel, 1, 1, 1);

        var values = new Vector4[1];
        output.GetData(values);
        Assert.That(values[0].x, Is.EqualTo(2.0f).Within(1e-5f), "Initial RIS normalization is incorrect.");
        Assert.That(values[0].y, Is.EqualTo(1.0f).Within(1e-5f), "Temporal bias correction is incorrect.");
        Assert.That(values[0].z, Is.EqualTo(1.0f), "Similar materials should permit reuse.");
        Assert.That(values[0].w, Is.EqualTo(0.0f), "Different materials must reject reuse.");
    }
}
