using System;
using System.IO;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
internal static class ReSTIRDiagnosticsEditorBridge
{
    private const string BaselineSessionKey = "ReSTIR.Diagnostics.BaselineSession";

    static ReSTIRDiagnosticsEditorBridge()
    {
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.ExitingEditMode)
        {
            SessionState.SetString(BaselineSessionKey, FindLatestCompletedSessionDirectory() ?? string.Empty);
            return;
        }

        if (state != PlayModeStateChange.EnteredEditMode)
            return;

        EditorApplication.delayCall -= ValidateCompletedPlaySession;
        EditorApplication.delayCall += ValidateCompletedPlaySession;
    }

    private static void ValidateCompletedPlaySession()
    {
        string latest = FindLatestCompletedSessionDirectory();
        string baseline = SessionState.GetString(BaselineSessionKey, string.Empty);
        SessionState.EraseString(BaselineSessionKey);
        if (string.IsNullOrEmpty(latest) || PathsEqual(latest, baseline))
            return;

        bool passed = SelfTest.VerifyLatestGILogsNonInteractive(out string report);
        string status = passed ? "PASS" : "FAIL";
        if (passed)
            Debug.Log($"[ReSTIR][AutoVerify] {status} output={latest}");
        else
            Debug.LogError($"[ReSTIR][AutoVerify] {status} output={latest} report={report}");
    }

    private static string FindLatestCompletedSessionDirectory()
    {
        string root = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Output"));
        if (!Directory.Exists(root))
            return null;

        DirectoryInfo[] directories;
        try
        {
            directories = new DirectoryInfo(root).GetDirectories();
            Array.Sort(directories, (left, right) => right.LastWriteTimeUtc.CompareTo(left.LastWriteTimeUtc));
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[ReSTIR][AutoVerify] cannot enumerate output sessions: {ex.Message}");
            return null;
        }

        for (int i = 0; i < directories.Length; i++)
        {
            string sessionPath = Path.Combine(directories[i].FullName, "restir_session.jsonl");
            try
            {
                if (File.Exists(sessionPath) && File.ReadAllText(sessionPath).Contains("\"event\":\"session_end\""))
                    return directories[i].FullName;
            }
            catch (IOException)
            {
                // A session still being flushed is not complete yet.
            }
        }

        return null;
    }

    private static bool PathsEqual(string left, string right)
    {
        if (string.IsNullOrEmpty(left) || string.IsNullOrEmpty(right))
            return false;

        return string.Equals(
            Path.GetFullPath(left).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar),
            Path.GetFullPath(right).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar),
            StringComparison.OrdinalIgnoreCase);
    }
}
