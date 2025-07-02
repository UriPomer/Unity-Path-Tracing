using System;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public class TextureArrayPreviewWindow : EditorWindow
{
    public Texture2DArray array;
    private int slice = 0;
    private Texture2D temp;
    
    static TextureArrayPreviewWindow()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.ExitingPlayMode || state == PlayModeStateChange.EnteredEditMode)
        {
            var win = GetWindow<TextureArrayPreviewWindow>("TextureArray 预览");
            if (win != null)
                win.Close();
        }
    }

    void OnGUI()
    {
        array = (Texture2DArray)EditorGUILayout.ObjectField("Array", array, typeof(Texture2DArray), false);
        slice = EditorGUILayout.IntSlider("Layer", slice, 0, array != null ? array.depth - 1 : 0);

        if (array != null)
        {
            if (temp == null || temp.width != array.width || temp.height != array.height)
                temp = new Texture2D(array.width, array.height, array.format, false);

            Graphics.CopyTexture(array, slice, 0, temp, 0, 0);
            GUILayout.Label(temp, GUILayout.Width(256), GUILayout.Height(256));
        }
    }
}

[InitializeOnLoad]
public class TexturePreviewWindow : EditorWindow
{
    public Texture2D texture2D;
    private Vector2 scrollPos;
    
    static TexturePreviewWindow()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.ExitingPlayMode || state == PlayModeStateChange.EnteredEditMode)
        {
            var win = GetWindow<TexturePreviewWindow>("Texture 预览");
            if (win != null)
                win.Close();
        }
    }
    
    private void OnGUI()
    {
        GUILayout.Label("Texture2D 预览器", EditorStyles.boldLabel);

        texture2D = (Texture2D)EditorGUILayout.ObjectField("Texture", texture2D, typeof(Texture2D), false);

        if (texture2D == null)
        {
            EditorGUILayout.HelpBox("请先从上方下拉框选择一个 Texture2D", MessageType.Info);
            return;
        }

        EditorGUILayout.LabelField($"尺寸：{texture2D.width} × {texture2D.height}");
        EditorGUILayout.LabelField($"格式：{texture2D.format}");

        scrollPos = EditorGUILayout.BeginScrollView(scrollPos, GUILayout.Height(300));
        {
            Rect texRect = GUILayoutUtility.GetRect(
                texture2D.width,
                texture2D.height,
                GUILayout.ExpandWidth(false),
                GUILayout.ExpandHeight(false)
            );

            EditorGUI.DrawPreviewTexture(texRect, texture2D);
        }
        EditorGUILayout.EndScrollView();
    }
}