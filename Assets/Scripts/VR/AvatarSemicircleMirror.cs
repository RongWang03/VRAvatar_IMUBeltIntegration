using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public sealed class AvatarSemicircleMirror : MonoBehaviour
{
    [SerializeField] Transform avatar;
    [SerializeField] Camera viewer;
    [SerializeField, Min(1.5f)] float distance = 3.2f;
    [SerializeField, Min(1.5f)] float mirrorSize = 2.6f;
    [SerializeField] float centerHeight = 1.3f;
    [SerializeField, Range(20f, 60f)] float reflectionFieldOfView = 32f;
    [SerializeField, Range(0f, 20f)] float upwardFramingDegrees = 8f;
    [SerializeField, Min(128)] int textureHeight = 1024;

    readonly List<Camera> reflectionCameras = new();
    readonly List<Renderer> panels = new();
    Vector3 lastForward = Vector3.forward;

    public void Configure(Transform target, Camera eyeCamera)
    {
        avatar = target;
        viewer = eyeCamera;
        Rebuild();
    }

    void OnEnable() { Rebuild(); }

    void OnDisable()
    {
        foreach (var cam in reflectionCameras)
            if (cam && cam.targetTexture) cam.targetTexture.Release();
    }

    [ContextMenu("Rebuild Mirror")]
    public void Rebuild()
    {
        if (!avatar) return;
        ClearChildren();

        reflectionCameras.Clear();
        panels.Clear();
        Vector3[] mirrorPositions =
        {
            new Vector3(0f, centerHeight, distance),
            new Vector3(-distance, centerHeight, 0f),
            new Vector3(distance, centerHeight, 0f)
        };
        string[] mirrorNames = { "Front Mirror", "Left Mirror", "Right Mirror" };

        for (int i = 0; i < mirrorPositions.Length; i++)
        {
            Vector3 localPosition = mirrorPositions[i];

            var panel = GameObject.CreatePrimitive(PrimitiveType.Quad);
            panel.name = mirrorNames[i];
            panel.layer = 2;
            panel.transform.SetParent(transform, false);
            panel.transform.localPosition = localPosition;
            panel.transform.localRotation = Quaternion.LookRotation(new Vector3(localPosition.x, 0f, localPosition.z).normalized, Vector3.up);
            panel.transform.localScale = new Vector3(mirrorSize, mirrorSize, 1f);
            var collider = panel.GetComponent<Collider>();
            if (collider) DestroyImmediate(collider);

            var rt = new RenderTexture(textureHeight, textureHeight, 24)
            {
                name = $"AvatarMirror_{i + 1:00}",
                antiAliasing = 2
            };

            var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Texture");
            var material = new Material(shader) { name = $"Avatar Mirror Material {i + 1:00}" };
            material.mainTexture = rt;
            panel.GetComponent<Renderer>().sharedMaterial = material;
            panels.Add(panel.GetComponent<Renderer>());

            var cameraObject = new GameObject($"Reflection Camera {i + 1:00}");
            cameraObject.transform.SetParent(panel.transform, false);
            var reflectionCamera = cameraObject.AddComponent<Camera>();
            reflectionCamera.enabled = false;
            reflectionCamera.targetTexture = rt;
            reflectionCamera.cullingMask &= ~(1 << 2);
            reflectionCamera.allowHDR = true;
            reflectionCamera.allowMSAA = true;
            reflectionCameras.Add(reflectionCamera);
        }
    }

    void ClearChildren()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
            DestroyImmediate(transform.GetChild(i).gameObject);
    }

    void LateUpdate()
    {
        if (!avatar) return;

        Vector3 forward = Vector3.ProjectOnPlane(avatar.forward, Vector3.up);
        if (forward.sqrMagnitude > 0.001f) lastForward = forward.normalized;
        transform.SetPositionAndRotation(avatar.position, Quaternion.LookRotation(lastForward, Vector3.up));

        Camera source = viewer ? viewer : Camera.main;
        if (!source || reflectionCameras.Count != panels.Count) return;

        for (int i = 0; i < panels.Count; i++)
        {
            var panel = panels[i];
            var cam = reflectionCameras[i];
            if (!panel || !cam) continue;

            Vector3 normal = panel.transform.forward;
            Vector3 point = panel.transform.position;
            cam.transform.position = ReflectPoint(source.transform.position, point, normal);
            cam.transform.rotation = Quaternion.LookRotation(
                Vector3.Reflect(source.transform.forward, normal),
                Vector3.Reflect(source.transform.up, normal));
            cam.transform.rotation *= Quaternion.Euler(-upwardFramingDegrees, 0f, 0f);
            cam.fieldOfView = reflectionFieldOfView;
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = source.farClipPlane;
            cam.Render();
        }
    }

    static Vector3 ReflectPoint(Vector3 position, Vector3 planePoint, Vector3 planeNormal)
    {
        return position - 2f * Vector3.Dot(position - planePoint, planeNormal) * planeNormal;
    }
}
