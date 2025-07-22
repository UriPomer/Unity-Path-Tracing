using UnityEngine;

public class LightCullingDebugger : MonoBehaviour
{
    [Header("Debug Controls")]
    [SerializeField] private bool enableDebugMode = true;
    [SerializeField] private KeyCode toggleKey = KeyCode.F1;
    
    private LightCullingManager lightCullingManager;
    private bool lastCullingState;
    
    private void Start()
    {
        lightCullingManager = GetComponent<LightCullingManager>();
        if (lightCullingManager != null)
        {
            lastCullingState = lightCullingManager.enabled;
        }
    }
    
    private void Update()
    {
        if (Input.GetKeyDown(toggleKey))
        {
            ToggleLightCulling();
        }
    }
    
    private void ToggleLightCulling()
    {
        if (lightCullingManager != null)
        {
            lightCullingManager.enabled = !lightCullingManager.enabled;
            Debug.Log($"Light Culling: {(lightCullingManager.enabled ? "ENABLED" : "DISABLED")}");
        }
    }
    
    private void OnGUI()
    {
        if (!enableDebugMode) return;
        
        GUILayout.BeginArea(new Rect(Screen.width - 300, 10, 290, 150));
        GUILayout.Box("Light Culling Debugger");
        
        GUILayout.Label($"Press {toggleKey} to toggle light culling");
        GUILayout.Label($"Current State: {(lightCullingManager?.enabled == true ? "ENABLED" : "DISABLED")}");
        
        if (LightManager.Instance != null)
        {
            GUILayout.Label($"Point Lights: {LightManager.Instance.GetPointLightsCount()}");
        }
        
        if (GUILayout.Button("Force Disable Culling"))
        {
            if (lightCullingManager != null)
            {
                lightCullingManager.enabled = false;
                Debug.Log("Light Culling FORCE DISABLED");
            }
        }
        
        if (GUILayout.Button("Force Enable Culling"))
        {
            if (lightCullingManager != null)
            {
                lightCullingManager.enabled = true;
                Debug.Log("Light Culling FORCE ENABLED");
            }
        }
        
        GUILayout.EndArea();
    }
}
