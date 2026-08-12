using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Creates a short expanding ring at each predicted foot landing position.
/// Attach this component to any active GameObject in the scene.
/// </summary>
public sealed class PeakLandingRippleEffect : MonoBehaviour
{
    [Header("Source")]
    [SerializeField]
    private PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
        trajectoryController;

    [Header("Ripple Shape")]
    [Min(12)]
    [SerializeField] private int segments = 64;

    [Min(0.01f)]
    [SerializeField] private float startRadius = 0.06f;

    [Min(0.02f)]
    [SerializeField] private float endRadius = 0.45f;

    [Min(0.001f)]
    [SerializeField] private float startWidth = 0.05f;

    [Tooltip("Raises the ring above the floor to avoid z-fighting.")]
    [Min(0f)]
    [SerializeField] private float groundOffset = 0.03f;

    [Header("Timing")]
    [Min(0.05f)]
    [SerializeField] private float duration = 1.20f;

    [SerializeField]
    private AnimationCurve expansionCurve =
        AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [Header("Appearance")]
    [SerializeField]
    private Color leftFootColor =
        new Color(0.25f, 0.65f, 1f, 1f);

    [SerializeField]
    private Color rightFootColor =
        new Color(1f, 0.45f, 0.30f, 1f);

    [Header("Diagnostics")]
    [SerializeField] private bool logRippleCreation = true;
    [SerializeField] private int ripplesCreated;
    [SerializeField] private bool subscribedToLandingEvents;

    private Material rippleMaterial;

    private void Awake()
    {
        FindControllerIfNeeded();
        CreateMaterial();
    }

    private void OnEnable()
    {
        FindControllerIfNeeded();
        Subscribe();
    }

    private void Start()
    {
        Subscribe();
    }

    private void OnDisable()
    {
        Unsubscribe();
    }

    private void OnDestroy()
    {
        Unsubscribe();

        if (rippleMaterial != null)
        {
            Destroy(rippleMaterial);
        }
    }

    private void FindControllerIfNeeded()
    {
        if (trajectoryController == null)
        {
            trajectoryController =
                FindFirstObjectByType<
                    PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth>();
        }
    }

    private void Subscribe()
    {
        if (trajectoryController == null)
        {
            subscribedToLandingEvents = false;

            Debug.LogWarning(
                "[PeakLandingRippleEffect] No Smooth trajectory controller found.",
                this);

            return;
        }

        trajectoryController.FootLanded -= HandleFootLanded;
        trajectoryController.FootLanded += HandleFootLanded;
        subscribedToLandingEvents = true;
    }

    private void Unsubscribe()
    {
        if (trajectoryController != null)
        {
            trajectoryController.FootLanded -= HandleFootLanded;
        }

        subscribedToLandingEvents = false;
    }

    private void CreateMaterial()
    {
        Shader shader =
            Shader.Find("Sprites/Default");

        if (shader == null)
        {
            shader =
                Shader.Find("Universal Render Pipeline/Unlit");
        }

        if (shader == null)
        {
            Debug.LogError(
                "[PeakLandingRippleEffect] No suitable unlit shader found.",
                this);

            return;
        }

        rippleMaterial = new Material(shader)
        {
            name = "Runtime Landing Ripple Material",
            hideFlags = HideFlags.DontSave
        };

        rippleMaterial.renderQueue = 4000;
    }

    private void HandleFootLanded(
        PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
            .FootLandingEvent landingEvent)
    {
        if (rippleMaterial == null)
        {
            Debug.LogWarning(
                "[PeakLandingRippleEffect] Landing received, but material is missing.",
                this);

            return;
        }

        if (logRippleCreation)
        {
            Debug.Log(
                $"[PeakLandingRippleEffect] Creating {landingEvent.Leg} ripple " +
                $"at {landingEvent.WorldPosition}.",
                this);
        }

        Color color =
            landingEvent.Leg == PeakLegSide.Left
                ? leftFootColor
                : rightFootColor;

        StartCoroutine(
            AnimateRipple(
                landingEvent.WorldPosition,
                color,
                landingEvent.Leg));
    }

    private IEnumerator AnimateRipple(
        Vector3 landingPosition,
        Color baseColor,
        PeakLegSide leg)
    {
        GameObject rippleObject =
            new GameObject($"LandingRipple_{leg}_{ripplesCreated + 1}");

        LineRenderer line =
            rippleObject.AddComponent<LineRenderer>();

        line.useWorldSpace = true;
        line.loop = true;
        line.positionCount = Mathf.Max(12, segments);
        line.alignment = LineAlignment.View;
        line.textureMode = LineTextureMode.Stretch;
        line.numCornerVertices = 6;
        line.numCapVertices = 6;
        line.material = rippleMaterial;
        line.shadowCastingMode = ShadowCastingMode.Off;
        line.receiveShadows = false;
        line.sortingOrder = 100;
        line.startWidth = startWidth;
        line.endWidth = startWidth;

        ripplesCreated++;

        float elapsed = 0f;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;

            float t =
                Mathf.Clamp01(elapsed / Mathf.Max(0.05f, duration));

            float curvedT =
                expansionCurve == null
                    ? t
                    : expansionCurve.Evaluate(t);

            float radius =
                Mathf.Lerp(startRadius, endRadius, curvedT);

            float alpha =
                1f - t;

            Color currentColor =
                new Color(
                    baseColor.r,
                    baseColor.g,
                    baseColor.b,
                    alpha);

            line.startColor = currentColor;
            line.endColor = currentColor;

            float width =
                Mathf.Lerp(startWidth, 0.005f, t);

            line.startWidth = width;
            line.endWidth = width;

            UpdateRingPositions(
                line,
                landingPosition,
                radius);

            yield return null;
        }

        Destroy(rippleObject);
    }

    private void UpdateRingPositions(
        LineRenderer line,
        Vector3 center,
        float radius)
    {
        int count = line.positionCount;

        for (int index = 0; index < count; index++)
        {
            float angle =
                index * Mathf.PI * 2f / count;

            Vector3 position =
                new Vector3(
                    center.x + Mathf.Cos(angle) * radius,
                    center.y + groundOffset,
                    center.z + Mathf.Sin(angle) * radius);

            line.SetPosition(index, position);
        }
    }
}