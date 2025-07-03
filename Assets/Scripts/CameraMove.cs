using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMove : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 5f;       // 平移速度
    [SerializeField] private float zoomSpeed = 2f;       // 缩放速度
    [SerializeField] private float rotationSpeed = 1f; // 旋转速度

    private void Update()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        transform.Translate(0, 0, scroll * zoomSpeed, Space.Self);

        if (Input.GetMouseButtonDown(1))
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
        if (Input.GetMouseButtonUp(1))
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        if (Input.GetMouseButton(1))
        {
            float h = Input.GetAxis("Mouse X");
            float v = Input.GetAxis("Mouse Y");
            transform.Rotate(Vector3.up, h * rotationSpeed, Space.World);
            transform.Rotate(Vector3.right, -v * rotationSpeed, Space.Self);

            float moveHorizontal = Input.GetAxis("Horizontal");
            float moveVertical   = Input.GetAxis("Vertical");
            Vector3 movement = new Vector3(moveHorizontal, 0, moveVertical);
            transform.Translate(movement * moveSpeed * Time.deltaTime, Space.Self);

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