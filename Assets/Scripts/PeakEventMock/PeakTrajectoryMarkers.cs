using UnityEngine;

/// <summary>
/// Read-only world-space visualisation of the predicted Current Peak,
/// Landing and Next Peak positions for both legs.
/// This component does not modify the gait controller.
/// </summary>
public sealed class PeakTrajectoryMarkers : MonoBehaviour
{
    [Header("Data Source")]
    [SerializeField]
    private PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth controller;

    [Header("Visibility")]
    [SerializeField] private bool showTrajectoryMarkers = true;
    [SerializeField] private bool showLeftLeg = true;
    [SerializeField] private bool showRightLeg = true;
    [SerializeField] private bool hideWhenLegInactive = true;

    [Header("Marker Sizes")]
    [Min(0.005f)]
    [SerializeField] private float currentPeakSize = 0.040f;

    [Min(0.005f)]
    [SerializeField] private float landingSize = 0.050f;

    [Min(0.005f)]
    [SerializeField] private float nextPeakSize = 0.035f;

    [Header("Marker Colours")]
    [SerializeField] private Color leftCurrentPeakColour = new Color(0.1f, 0.8f, 1f, 1f);
    [SerializeField] private Color leftLandingColour = new Color(0.1f, 1f, 0.3f, 1f);
    [SerializeField] private Color leftNextPeakColour = new Color(0.2f, 0.4f, 1f, 1f);

    [SerializeField] private Color rightCurrentPeakColour = new Color(1f, 0.3f, 0.8f, 1f);
    [SerializeField] private Color rightLandingColour = new Color(1f, 0.8f, 0.1f, 1f);
    [SerializeField] private Color rightNextPeakColour = new Color(1f, 0.2f, 0.2f, 1f);

    private GameObject leftCurrentPeakMarker;
    private GameObject leftLandingMarker;
    private GameObject leftNextPeakMarker;

    private GameObject rightCurrentPeakMarker;
    private GameObject rightLandingMarker;
    private GameObject rightNextPeakMarker;

    private void Awake()
    {
        CreateMarkers();
        RefreshVisibility();
    }

    private void LateUpdate()
    {
        if (controller == null)
        {
            SetAllMarkersActive(false);
            return;
        }

        UpdateLegMarkers(
            PeakLegSide.Left,
            showLeftLeg,
            leftCurrentPeakMarker,
            leftLandingMarker,
            leftNextPeakMarker);

        UpdateLegMarkers(
            PeakLegSide.Right,
            showRightLeg,
            rightCurrentPeakMarker,
            rightLandingMarker,
            rightNextPeakMarker);
    }

    private void OnValidate()
    {
        if (Application.isPlaying)
        {
            ApplyMarkerScales();
            ApplyMarkerColours();
            RefreshVisibility();
        }
    }

    [ContextMenu("Create / Recreate Markers")]
    public void CreateMarkers()
    {
        DestroyExistingMarker(ref leftCurrentPeakMarker);
        DestroyExistingMarker(ref leftLandingMarker);
        DestroyExistingMarker(ref leftNextPeakMarker);
        DestroyExistingMarker(ref rightCurrentPeakMarker);
        DestroyExistingMarker(ref rightLandingMarker);
        DestroyExistingMarker(ref rightNextPeakMarker);

        leftCurrentPeakMarker =
            CreateMarker("Left_CurrentPeak", PrimitiveType.Sphere);
        leftLandingMarker =
            CreateMarker("Left_Landing", PrimitiveType.Cylinder);
        leftNextPeakMarker =
            CreateMarker("Left_NextPeak", PrimitiveType.Sphere);

        rightCurrentPeakMarker =
            CreateMarker("Right_CurrentPeak", PrimitiveType.Sphere);
        rightLandingMarker =
            CreateMarker("Right_Landing", PrimitiveType.Cylinder);
        rightNextPeakMarker =
            CreateMarker("Right_NextPeak", PrimitiveType.Sphere);

        ApplyMarkerScales();
        ApplyMarkerColours();
        RefreshVisibility();
    }

    private GameObject CreateMarker(
        string markerName,
        PrimitiveType primitiveType)
    {
        GameObject marker =
            GameObject.CreatePrimitive(primitiveType);

        marker.name = markerName;
        marker.transform.SetParent(transform, true);

        Collider markerCollider =
            marker.GetComponent<Collider>();

        if (markerCollider != null)
        {
            Destroy(markerCollider);
        }

        Renderer markerRenderer =
            marker.GetComponent<Renderer>();

        if (markerRenderer != null)
        {
            markerRenderer.shadowCastingMode =
                UnityEngine.Rendering.ShadowCastingMode.Off;

            markerRenderer.receiveShadows = false;
        }

        return marker;
    }

    private void UpdateLegMarkers(
        PeakLegSide leg,
        bool legVisible,
        GameObject currentPeakMarker,
        GameObject landingMarker,
        GameObject nextPeakMarker)
    {
        bool hasTrajectory =
            controller.TryGetTrajectoryPoints(
                leg,
                out Vector3 currentPeak,
                out Vector3 landing,
                out Vector3 nextPeak);

        bool visible =
            showTrajectoryMarkers &&
            legVisible &&
            (!hideWhenLegInactive || hasTrajectory);

        SetMarkerActive(currentPeakMarker, visible);
        SetMarkerActive(landingMarker, visible);
        SetMarkerActive(nextPeakMarker, visible);

        if (!visible)
        {
            return;
        }

        currentPeakMarker.transform.position = currentPeak;
        landingMarker.transform.position = landing;
        nextPeakMarker.transform.position = nextPeak;
    }

    private void ApplyMarkerScales()
    {
        if (leftCurrentPeakMarker != null)
        {
            leftCurrentPeakMarker.transform.localScale =
                Vector3.one * currentPeakSize;
        }

        if (rightCurrentPeakMarker != null)
        {
            rightCurrentPeakMarker.transform.localScale =
                Vector3.one * currentPeakSize;
        }

        Vector3 landingScale =
            new Vector3(
                landingSize,
                Mathf.Max(0.002f, landingSize * 0.08f),
                landingSize);

        if (leftLandingMarker != null)
        {
            leftLandingMarker.transform.localScale = landingScale;
        }

        if (rightLandingMarker != null)
        {
            rightLandingMarker.transform.localScale = landingScale;
        }

        if (leftNextPeakMarker != null)
        {
            leftNextPeakMarker.transform.localScale =
                Vector3.one * nextPeakSize;
        }

        if (rightNextPeakMarker != null)
        {
            rightNextPeakMarker.transform.localScale =
                Vector3.one * nextPeakSize;
        }
    }

    private void ApplyMarkerColours()
    {
        SetMarkerColour(
            leftCurrentPeakMarker,
            leftCurrentPeakColour);

        SetMarkerColour(
            leftLandingMarker,
            leftLandingColour);

        SetMarkerColour(
            leftNextPeakMarker,
            leftNextPeakColour);

        SetMarkerColour(
            rightCurrentPeakMarker,
            rightCurrentPeakColour);

        SetMarkerColour(
            rightLandingMarker,
            rightLandingColour);

        SetMarkerColour(
            rightNextPeakMarker,
            rightNextPeakColour);
    }

    private void SetMarkerColour(
        GameObject marker,
        Color colour)
    {
        if (marker == null)
        {
            return;
        }

        Renderer markerRenderer =
            marker.GetComponent<Renderer>();

        if (markerRenderer == null)
        {
            return;
        }

        Shader shader =
            Shader.Find("Universal Render Pipeline/Unlit");

        if (shader == null)
        {
            shader = Shader.Find("Unlit/Color");
        }

        if (shader == null)
        {
            shader = Shader.Find("Sprites/Default");
        }

        Material material =
            shader != null
                ? new Material(shader)
                : new Material(markerRenderer.sharedMaterial);

        material.color = colour;
        markerRenderer.material = material;
    }

    private void RefreshVisibility()
    {
        if (!showTrajectoryMarkers)
        {
            SetAllMarkersActive(false);
        }
    }

    private void SetAllMarkersActive(bool active)
    {
        SetMarkerActive(leftCurrentPeakMarker, active);
        SetMarkerActive(leftLandingMarker, active);
        SetMarkerActive(leftNextPeakMarker, active);
        SetMarkerActive(rightCurrentPeakMarker, active);
        SetMarkerActive(rightLandingMarker, active);
        SetMarkerActive(rightNextPeakMarker, active);
    }

    private void SetMarkerActive(
        GameObject marker,
        bool active)
    {
        if (marker != null && marker.activeSelf != active)
        {
            marker.SetActive(active);
        }
    }

    private void DestroyExistingMarker(
        ref GameObject marker)
    {
        if (marker == null)
        {
            return;
        }

        if (Application.isPlaying)
        {
            Destroy(marker);
        }
        else
        {
            DestroyImmediate(marker);
        }

        marker = null;
    }
}
