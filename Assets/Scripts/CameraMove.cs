using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMove : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 5f;       // 平移速度
    [SerializeField] private float zoomSpeed = 2f;       // 缩放速度
    [SerializeField] private float rotationSpeed = 100f; // 旋转速度

    private void Update()
    {
        // 鼠标滚轮控制相机的远近
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        transform.Translate(0, 0, scroll * zoomSpeed, Space.Self);

        // 按下右键时锁定鼠标光标
        if (Input.GetMouseButtonDown(1))
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
        // 松开右键时恢复鼠标光标
        if (Input.GetMouseButtonUp(1))
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        // 按住右键时处理旋转和平移
        if (Input.GetMouseButton(1))
        {
            // --- 相机旋转 ---
            float h = Input.GetAxis("Mouse X");
            float v = Input.GetAxis("Mouse Y");
            // 绕全局 Y 轴水平旋转
            transform.Rotate(Vector3.up, h * rotationSpeed * Time.deltaTime, Space.World);
            // 绕本地 X 轴垂直旋转（负值为符合鼠标上下惯性）
            transform.Rotate(Vector3.right, -v * rotationSpeed * Time.deltaTime, Space.Self);

            // --- WSAD 平移 ---
            float moveHorizontal = Input.GetAxis("Horizontal");
            float moveVertical   = Input.GetAxis("Vertical");
            Vector3 movement = new Vector3(moveHorizontal, 0, moveVertical);
            transform.Translate(movement * moveSpeed * Time.deltaTime, Space.Self);

            // --- Q/E 上下 ---
            if (Input.GetKey(KeyCode.Q))
            {
                transform.Translate(Vector3.down * moveSpeed * Time.deltaTime, Space.World);
            }
            if (Input.GetKey(KeyCode.E))
            {
                transform.Translate(Vector3.up * moveSpeed * Time.deltaTime, Space.World);
            }
        }
    }
}