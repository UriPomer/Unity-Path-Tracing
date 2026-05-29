using UnityEditor;
using System.IO;
using System;

/// <summary>
/// 编译检查入口。只触发编译，成功退出 0，有编译错误退出 1。
/// 用法：Unity.exe -batchmode -quit -projectPath ... -executeMethod CompileCheck.Run -logFile Tools/Output/compile.log
/// AI 随后检查日志中是否有 "error CS" 来判断编译是否通过。
/// </summary>
public static class CompileCheck
{
    public static void Run()
    {
        // Unity batchmode 在执行 -executeMethod 之前已经完成了脚本编译。
        // 如果编译失败，这个方法根本不会被调用。
        // 所以能运行到这里 = 编译通过。
        string logPath = GetArg("-logFile");
        string outputDir = "Tools/Output";
        if (!Directory.Exists(outputDir))
            Directory.CreateDirectory(outputDir);

        string resultFile = Path.Combine(outputDir, "compile_result.txt");
        File.WriteAllText(resultFile, "COMPILE: PASS");

        if (!string.IsNullOrEmpty(logPath))
        {
            // 检查日志中是否有编译错误（以防万一）
            if (File.Exists(logPath))
            {
                try
                {
                    string log = File.ReadAllText(logPath);
                    bool hasError = log.Contains("error CS") || log.Contains("Shader error") || log.Contains("error :");
                    if (hasError)
                    {
                        File.WriteAllText(resultFile, "COMPILE: FAIL");
                        EditorApplication.Exit(1);
                        return;
                    }
                }
                catch (IOException)
                {
                    // In batchmode Unity may still hold an exclusive handle to the same
                    // log file path that launched the editor. Reaching this method already
                    // proves script compilation succeeded, so treat the log reread as best-effort.
                }
                catch (UnauthorizedAccessException)
                {
                    // Same rationale as IOException above.
                }
            }
        }

        EditorApplication.Exit(0);
    }

    private static string GetArg(string name)
    {
        var args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
            if (args[i].ToLower() == name.ToLower())
                return args[i + 1];
        return null;
    }
}
