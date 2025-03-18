using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMove : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float zoomSpeed = 2f;
    [SerializeField] private float rotationSpeed = 100f;

    private void Update()
    {
        // 鼠标滚轮控制相机的远近
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        transform.Translate(0, 0, scroll * zoomSpeed, Space.Self);

        // 鼠标右键按住控制相机的旋转和平移
        if (Input.GetMouseButton(1))
        {
            // 相机旋转
            float h = Input.GetAxis("Mouse X");
            float v = Input.GetAxis("Mouse Y");

            transform.Rotate(Vector3.up, h * rotationSpeed * Time.deltaTime, Space.World);
            transform.Rotate(Vector3.right, -v * rotationSpeed * Time.deltaTime, Space.Self);

            // WSAD 键控制相机平移
            float moveHorizontal = Input.GetAxis("Horizontal");
            float moveVertical = Input.GetAxis("Vertical");

            Vector3 movement = new Vector3(moveHorizontal, 0, moveVertical);
            transform.Translate(movement * moveSpeed * Time.deltaTime, Space.Self);

            // Q 和 E 键控制相机上升和下降
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