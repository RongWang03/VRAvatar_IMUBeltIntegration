# VR Avatar IMU Belt Integration

## 1. Project Overview

This Unity project implements a peak-driven lower-body walking system for a first-person VR avatar. Sparse left/right gait events provide timing, while horizontal HMD movement provides the spatial direction and displacement used to estimate step distance. The controller generates world-space foot-target trajectories, and Final IK `LimbIK` components solve the avatar leg bones toward those targets.

The submitted repository contains:

- a lower-body gait controller with timing estimation, event filtering, safe landing, and recovery;
- keyboard-loopback, Quest controller, and UDP peak-event inputs;
- a separate UDP adapter for diagnostic integration with the Yasmine/Devika IMU data source;
- HMD-to-avatar eye alignment;
- Final IK leg solving;
- runtime diagnostics, mirrors, landing feedback, and CSV recording.

The HMD does **not** automatically trigger a step. Peak events trigger and sequence gait motion; HMD samples only provide alignment, direction, and displacement information.

## 2. Final Project Status

**This repository represents the final submitted version of the lower-body VR avatar project.**

The primary completed runtime path is the sparse peak-event interface on UDP port `5055`, including local keyboard loopback and Quest controller inputs. The separate `5056` adapter can receive and parse a narrow provisional external-event grammar, but it is disabled in the final Scene and is not mapped into the gait controller by default. Raw continuous IMU measurements are not converted into a complete gait signal in the submitted version.

This document describes the submitted system as it exists. It is handover documentation, not a development roadmap.

## 3. Unity Version and Dependencies

Open the project with:

```text
Unity 6000.0.77f1
Revision 88f89d0d8b31
```

Important dependencies present in the project include:

- Final IK / RootMotion, used by the two leg `LimbIK` solvers;
- Meta XR SDK `203.0.0` and the Meta Movement package sample dependency;
- Unity Input System `1.19.0`;
- OpenXR `1.16.1` and XR Plugin Management configuration;
- Universal Render Pipeline `17.0.4`;
- TextMeshPro / UGUI.

The exact package declarations are stored in `Packages/manifest.json`. Package and ProjectSettings files should remain associated with this Unity version.

## 4. Final Scene

The final Scene is:

```text
Assets/Scenes/PeakEventComparison/IK_LegWalkingDemo_LowerBodyOnly_EyeAligned.unity
```

It is the only Scene listed and enabled in `ProjectSettings/EditorBuildSettings.asset`.

The avatar is instantiated from:

```text
Assets/Characters/StylizedCharacter.prefab
```

and renamed `Avatar_LegIK_Test` in the Scene.

## 5. Quick Start

1. Clone or copy the repository.
2. Open it with Unity `6000.0.77f1`.
3. Open `Assets/Scenes/PeakEventComparison/IK_LegWalkingDemo_LowerBodyOnly_EyeAligned.unity`.
4. Wait for asset import and script compilation to finish; confirm that the Console has no compilation errors.
5. Enter Play Mode.
6. For keyboard testing, temporarily activate the inactive `UDPTestSender` GameObject. Press `Q` for a left Peak and `E` for a right Peak; alternate Q/E to exercise walking.
7. With Quest Touch controllers, press left-controller `Y` for a left Peak and right-controller `B` for a right Peak.
8. An external Peak sender can transmit a supported UTF-8 message to the Unity PC on UDP `5055`.
9. For an optional `5056` diagnostic test, enable `enableJasmineInput` on `JasmineInputAdapter`. Keep `mapTopToPeak` disabled if the intention is only to inspect parsed/raw external messages.

The checked-in Scene intentionally has:

```text
UDPTestSender GameObject     inactive
LandingRippleManager        inactive
enableJasmineInput          false
mapTopToPeak                false
```

Therefore Q/E loopback, Ripple display, and the `5056` socket are not active in the untouched Scene state.

## 6. Runtime Architecture

### Keyboard Peak Test

```text
Q / E
  -> UdpPeakEventLoopbackSender (UDPTestSender must be active)
  -> 127.0.0.1:5055
  -> UdpPeakEventReceiver
  -> LegPeakEventReceiver
  -> PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
  -> LeftFootTarget / RightFootTarget
  -> Final IK LimbIK
  -> Avatar leg bones
```

### Quest Controller

```text
Left Touch Y / Right Touch B
  -> QuestControllerPeakInput
  -> LegPeakEventReceiver
  -> Smooth gait controller
  -> Foot targets
  -> Final IK LimbIK
  -> Avatar legs
```

### External Peak Input

```text
External sender
  -> UDP 5055
  -> UdpPeakEventReceiver
  -> LegPeakEventReceiver
  -> Smooth gait controller
  -> Foot targets
  -> Final IK LimbIK
```

### External IMU Interface

```text
External Yasmin/Devika system
  -> UDP 5056
  -> JasmineInputAdapter
  -> raw-message logging and provisional semantic parsing
```

In the submitted Scene, this path stops at the adapter because `enableJasmineInput = false`. If enabled temporarily, recognized `Top` events still stop at the adapter while `mapTopToPeak = false`. `HitBottom` and `StartGoingUp` are observation-only in the current code. The diagram deliberately does not show a completed raw-IMU-to-gait pipeline.

## 7. Scene Components

Only objects that materially contribute to the final Scene are described here; ordinary Meta XR anchor children and avatar bone/render children are not expanded.

### `[BuildingBlock] Camera Rig`

**Purpose:** Meta XR camera and headset tracking root.

**Important Components:** `OVRCameraRig`, `OVRManager`, `OVRHeadsetEmulator`, and Meta `BuildingBlock`.

**References:** Owns `TrackingSpace` and its eye/controller anchors.

**Runtime Role:** Updates the tracked HMD and controller poses used by alignment, gait-distance sampling, Quest input, and first-person rendering.

### `TrackingSpace` and `CenterEyeAnchor`

**Purpose:** `TrackingSpace` contains the tracked XR anchors. `CenterEyeAnchor` represents the centre HMD pose and carries the main Camera and AudioListener.

**Important Components:** Transform on `TrackingSpace`; Transform, Camera, AudioListener, and URP camera data on `CenterEyeAnchor`.

**References:** `CenterEyeAnchor` is referenced by `LegPeakEventReceiver`, the gait controller, eye aligner, mirror rig, and CSV recorders.

**Runtime Role:** Supplies avatar horizontal alignment, movement direction, and the position snapshot used for same-leg HMD displacement and step-distance estimation. It does not generate Peak events.

### `Avatar_LegIK_Test`

**Purpose:** Visible avatar Prefab instance used by the final lower-body demonstration.

**Important Components:** Two RootMotion Final IK `LimbIK` components and `FirstPersonAvatarEyeAligner` on the Prefab root. Avatar render, Animator, and bone hierarchy remain inside the Prefab.

**References:** The eye aligner references `CenterEyeAnchor` and the avatar left/right eye transforms. The two LimbIK solvers reference the foot targets and knee bend goals described below.

**Runtime Role:** Its root follows horizontal eye-position error while retaining a fixed Y position. Final IK applies the generated target positions to the leg chains.

### `LegIKTargets`

**Purpose:** Container for the two world-space foot targets generated by the smooth gait controller.

**Children:**

- `LeftFootTarget`
- `RightFootTarget`

**References:** The smooth controller writes both targets. The left and right `LimbIK` components use the corresponding targets.

**Runtime Role:** Provides the boundary between gait trajectory generation and inverse-kinematics bone solving.

### `KneeGoals`

**Purpose:** Container for the bend goals that stabilise each leg's knee direction.

**Children:**

- `LeftKneeGoal`
- `RightKneeGoal`

**References:** The left LimbIK uses `LeftKneeGoal`; the right LimbIK uses `RightKneeGoal`.

**Runtime Role:** Guides LimbIK knee bending without generating gait motion.

### `LegIKController`

**Purpose:** Hosts the final lower-body trajectory controller.

**Important Components:** `PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth` (enabled).

**References:** `PeakEventInput/LegPeakEventReceiver`, both foot targets, `Avatar_LegIK_Test` as the peak anchor, and `CenterEyeAnchor` as the direction transform.

**Runtime Role:** Converts applied Peaks plus sampled HMD displacement into sequenced left/right target trajectories, landing events, fallback landings, and recovery.

### `PeakEventInput`

**Purpose:** Shared entry point for the final Peak-only gait interface.

**Important Components:** `LegPeakEventReceiver` and `UdpPeakEventReceiver`, both enabled.

**References:** The receiver uses `CenterEyeAnchor` as its reference transform. The UDP component forwards accepted messages to the receiver.

**Runtime Role:** Listens on UDP `5055`, queues decoded events, applies them on Unity's main thread, samples HMD position, and broadcasts `PeakApplied`.

### `QuestControllerPeakInput`

**Purpose:** Manual Peak input using Quest Touch face buttons.

**Important Components:** `QuestControllerPeakInput` (enabled).

**References:** `PeakEventInput/LegPeakEventReceiver`.

**Runtime Role:** Debounces Y/B presses with a `0.12 s` per-leg cooldown and submits left/right Peaks.

### `UDPTestSender`

**Purpose:** Local keyboard-to-UDP loopback test tool.

**Important Components:** `UdpPeakEventLoopbackSender`; the component is enabled but the GameObject is **inactive** in the final Scene.

**References:** Destination `127.0.0.1:5055`.

**Runtime Role:** When the GameObject is activated, Q/E send `left is up` / `right is up`; Space toggles a `0.35 s` alternating automatic stream after its configured initial delay.

### `JasmineInput`

**Purpose:** Isolated external IMU/event integration endpoint.

**Important Components:** `JasmineInputAdapter` (GameObject and component enabled).

**References:** `PeakEventInput/LegPeakEventReceiver` is assigned, but forwarding is disabled.

**Runtime Role:** The final Inspector values are `listenPort = 5056`, `enableJasmineInput = false`, `mapTopToPeak = false`, `logRawMessages = true`, `logParsedEvents = true`, and `acceptUnknownMessages = true`. Because the enable flag is false, it does not bind the UDP socket in the untouched final Scene.

### `PeakEventCsvLogger`

**Purpose:** Hosts the four retained runtime CSV recorders.

**Important Components:** `PeakEventCsvRecorder`, `PeakPoseFrameCsvRecorder`, `PeakStanceSummaryCsvRecorder`, and `PeakLandingCsvRecorder`; all components and the GameObject are enabled.

**References:** Receiver, smooth controller, HMD, avatar root, both foot targets, and both foot bones as required by each recorder.

**Runtime Role:** Starts recording automatically and writes under `PeakEventLogs/Proposed` using the configured prefixes.

### `PeakDiagnosticsCanvas/DiagnosticsPanel`

**Purpose:** First-person runtime status display under `CenterEyeAnchor`.

**Important Components:** World/UI Canvas on `PeakDiagnosticsCanvas`; `PeakDiagnosticsPanel` and assigned TextMeshPro fields on `DiagnosticsPanel`.

**References:** Smooth controller and `LegPeakEventReceiver`.

**Runtime Role:** Displays last Peak/time, estimated intervals, raw/filtered step distances, and prediction-blend state.

### `LandingRippleManager`

**Purpose:** Optional visual feedback for foot landings.

**Important Components:** `PeakLandingRippleEffect`; component enabled, GameObject **inactive**.

**References:** Smooth controller's `FootLanded` C# event.

**Runtime Role:** When its GameObject is activated, it creates expanding left/right coloured line-renderer ripples at landing positions. It does not run in the default final Scene state.

### `AvatarMirrorRig`

**Purpose:** Three-panel semicircular view of the avatar.

**Important Components:** `AvatarSemicircleMirror`, three mirror panels, three disabled reflection Cameras, RenderTextures, and generated mirror materials.

**References:** `Avatar_LegIK_Test` and the HMD Camera.

**Runtime Role:** Follows the avatar root and explicitly renders reflected views of the avatar into the front, left, and right panels.

### Supporting Scene Objects

- `Floor`: visible floor and BoxCollider used as the visual/physical ground.
- `Directional Light` and `Global Volume`: URP lighting and post-processing environment.
- `EventSystem`: Input System UI event handling for the diagnostic canvas.
- `LegTestCamera`: inactive desktop/test Camera; not part of the default VR view.
- `Label_MetaTracking`, `Legend_Text`: inactive legacy visual labels; they do not drive the final lower-body system.

## 8. Script Reference

The table covers every custom C# file remaining under `Assets/Scripts`.

| Script | Attached To | Purpose | Input | Output | Runtime Status |
|---|---|---|---|---|---|
| `FirstPersonAvatarEyeAligner.cs` | `Avatar_LegIK_Test` | Aligns avatar eye centre horizontally with the HMD while fixing root Y. | `CenterEyeAnchor`, avatar left/right eyes | Avatar root X/Z | Attached and active; final runtime |
| `AvatarSemicircleMirror.cs` | `AvatarMirrorRig` | Maintains and renders three mirror panels around the avatar. | Avatar root, viewer Camera | Reflection Camera renders / RenderTextures | Attached and active; final runtime visualisation |
| `LegPeakEventReceiver.cs` | `PeakEventInput` | Thread-safe shared Peak queue; samples reference position and publishes applied events on the main thread. | Left/right Peak submissions, `CenterEyeAnchor` | `PeakApplied` event and diagnostics | Attached and active; core runtime |
| `UdpPeakEventReceiver.cs` | `PeakEventInput` | Receives and parses Peak-only UDP datagrams. | UDP `5055` UTF-8 messages | Calls `LegPeakEventReceiver.SubmitPeak` | Attached and active; core runtime |
| `JasmineInputAdapter.cs` | `JasmineInput` | Receives/logs/parses a provisional external event grammar independently from `5055`. | Optional UDP `5056` | Parsed diagnostics; optional Top-to-Peak forwarding | Attached, but socket and forwarding disabled by Inspector flags |
| `UdpPeakEventLoopbackSender.cs` | `UDPTestSender` | Sends local keyboard/automatic test Peaks through the real UDP receiver path. | Q, E, Space | UTF-8 UDP to `127.0.0.1:5055` | Attached, but GameObject inactive by default |
| `QuestControllerPeakInput.cs` | `QuestControllerPeakInput` | Maps Quest Touch Y/B to the common Peak receiver. | Left Y, right B | Left/right `SubmitPeak` calls | Attached and active; runtime test input |
| `PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth.cs` | `LegIKController` | Final gait state machine, timing/distance estimator, foot-target trajectory generator, landing and recovery source. | `PeakApplied`, HMD/reference displacement | Foot-target positions, diagnostics, `FootLanded` | Attached and active; core runtime |
| `PeakDiagnosticsPanel.cs` | `CenterEyeAnchor/PeakDiagnosticsCanvas/DiagnosticsPanel` | Displays controller/receiver state in TextMeshPro fields. | Public controller and receiver diagnostics | Runtime UI text | Attached and active; read-only diagnostics |
| `PeakLandingRippleEffect.cs` | `LandingRippleManager` | Creates expanding coloured ripples for landing events. | Smooth controller `FootLanded` | Runtime LineRenderer objects | Attached, but GameObject inactive by default |
| `PeakEventCsvRecorder.cs` | `PeakEventCsvLogger` | Records one row per applied Peak, including reference/avatar/target and controller prediction state. | `PeakApplied`, HMD/avatar/targets | `PeakEventUDP_*.csv` | Attached, active, `recordOnEnable = true` |
| `PeakPoseFrameCsvRecorder.cs` | `PeakEventCsvLogger` | Records per-frame target/bone positions, tracking error, near-ground state, movement, and stance drift. | Receiver, HMD/avatar, targets, foot bones | `PeakPoseFrames_*.csv` | Attached, active, `recordOnEnable = true` |
| `PeakStanceSummaryCsvRecorder.cs` | `PeakEventCsvLogger` | Summarises each completed left/right stance period and target/bone drift/error. | Receiver, targets, foot bones | `PeakStanceSummary_*.csv` | Attached, active, `recordOnEnable = true` |
| `PeakLandingCsvRecorder.cs` | `PeakEventCsvLogger` | Records one row per `FootLanded` event, including landing reason and prediction data. | Smooth controller `FootLanded` | timestamped landing CSV | Attached, active, `recordOnStart = true` |
| `FirstPersonEyeVerticalDiagnostics.cs` | Not attached in final Scene | Measures HMD eye Y versus avatar eye-centre Y after a stable delay. | HMD and avatar eyes if manually assigned | Inspector diagnostics and one log sample | Source retained; non-runtime in final Scene |
| `StartupAvatarHeightCalibrationCoordinator.cs` | Not attached in final Scene | Contains one-time vertical eye calibration and static leg-reach diagnostics. | Avatar/HMD/eyes/leg references if manually configured | Root-Y offset, component activation, logs | Source retained; non-runtime in final Scene |
| `AvatarRootFollowHMD.cs` | Not attached in final Scene | Alternative general HMD root-follow component with optional yaw. | HMD Transform | Root position and optional yaw | Source retained; non-runtime; final Scene uses eye aligner instead |
| `PeakTrajectoryMarkers.cs` | Not attached in final Scene | Read-only marker visualisation for current Peak, landing, and next Peak predictions. | Smooth controller trajectory query | Runtime primitive markers | Source retained; non-runtime in final Scene |

`PeakEventCsvRecorder` explicitly uses reflection (`BindingFlags.Instance | BindingFlags.NonPublic`) to read selected private diagnostic fields from the smooth controller, including estimated intervals/distances, phases, anchor blending, and fallback state. This coupling is diagnostic only; it does not control gait motion.

## 9. Lower-body Gait Controller

`Assets/Scripts/PeakEventMock/PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth.cs` is the final trajectory controller.

### Inputs and outputs

It subscribes to `LegPeakEventReceiver.PeakApplied`. Each applied event provides the leg, Unity/source time, sequence, and the HMD/reference position sampled when the event is applied. The controller writes `LeftFootTarget` and `RightFootTarget`; it does not directly rotate avatar bones. It also publishes `FootLanded` for CSV and optional Ripple observers.

### Final Scene parameters relevant to timing and distance

```text
defaultSameLegInterval          1.0 s
minimumSameLegInterval          0.7 s
maximumSameLegInterval          1.3 s
minimumReferenceDisplacement    0.03 m
maximumPredictedDistance        0.30 m
displacementScale               1.0
stepLengthSmoothingAlpha        0.7
peakHeight                      0.15 m
groundStartPhase                0.35
groundEndPhase                  0.65
anchorCorrectionDuration        0.05 s
predictionBlendDuration         0.05 s
eventTimeoutGrace               0.35 s
fallbackLandingDuration         0.20 s
```

### Implemented mechanisms

- **Startup Idle:** before the first accepted Peak, both feet follow their stored avatar-relative standing offsets so the avatar can remain aligned under the HMD.
- **First Peak Initialisation:** the first accepted Peak for a leg creates its initial anchored Peak, landing, and next-Peak prediction without blending from a prior prediction.
- **Same-leg Interval Estimation:** for a leg with a prior event, `T` is the elapsed Unity time since that leg's previous Peak, clamped to `[0.7, 1.3] s`; otherwise `1.0 s` is used.
- **Global Cycle Estimation:** valid alternating left/right Peaks update a cross-leg interval. The estimated global cycle is `2 × cross-leg interval`, clamped to `[0.7, 1.3] s`.
- **T/4 Timing:** the current quarter duration is one quarter of the latest valid global cycle; if unavailable, the current leg's same-leg cycle is used, then the `1.0 s` default. The selected cycle is clamped before division by four.
- **HMD Step-distance Estimation:** the controller compares current and previous same-leg HMD/reference positions, removes Y, applies a `0.03 m` dead zone, scales and clamps the result to `0.30 m`, then applies cross-event exponential smoothing.
- **Lowering:** after a confirmed Peak, the current Peak leg moves from its Peak position toward its predicted landing while the opposite foot is held as support.
- **Opposite-leg Rising:** the opposite target rises toward its predicted Peak after the lowering quarter completes.
- **Peak Hold:** the rising foot remains at the Peak until the expected event arrives or the timeout/recovery path takes over.
- **Early Pending:** an expected event arriving during the permitted rising stage can be cached and consumed when Peak Hold begins.
- **Wrong-event Filtering:** same-leg repeats and events inconsistent with the current global phase are counted and ignored/rejected rather than blindly restarting motion.
- **Fallback:** missing events cause a controlled target descent to its stored ground height instead of leaving a leg suspended.
- **Recovery:** clears pending/blend state, makes active legs safe, and returns the state machine from invalid or timed-out motion.
- **Prediction Blend:** blends from the previous predicted trajectory to a newly calculated prediction to reduce discontinuity.
- **Anchor Correction:** when no prediction blend is available, corrects target-to-anchor position error over a short configured duration; very small corrections are direct.
- **Landing:** finalises the target at ground height, updates counters/diagnostics, and raises `FootLanded` with landing reason and position.
- **Post-cycle Avatar Follow:** once both legs are quiescent, standing targets again follow their stored horizontal offsets beneath the avatar rather than remaining stranded in world space.

Walking frequency changes do update the active timing estimate: later valid measured intervals replace earlier estimates, subject to the configured clamp. The values above are bounds/defaults, not a claim that every step is fixed at `1.0 s`.

## 10. Final IK

The runtime chain is:

```text
Smooth gait controller
  -> LeftFootTarget / RightFootTarget
  -> left/right Final IK LimbIK
  -> UpperLeg -> LowerLeg -> Foot bones
```

The smooth controller changes target transforms only; it does **not** directly rotate `UpperLeg`, `LowerLeg`, or `Foot` bones. Final IK solves each three-bone leg chain toward its target.

Verified Scene references:

```text
Left LimbIK target      LeftFootTarget
Left LimbIK bend goal   LeftKneeGoal
Right LimbIK target     RightFootTarget
Right LimbIK bend goal  RightKneeGoal
```

Both LimbIK target and bend-goal references are assigned in the final Scene.

## 11. HMD Alignment

`CenterEyeAnchor` has three distinct uses:

1. `FirstPersonAvatarEyeAligner` compares the HMD X/Z position with the midpoint of the avatar eyes and moves the avatar root horizontally. The Scene sets `positionSmoothing = 0`, so correction is direct; root Y is fixed after startup.
2. `LegPeakEventReceiver` samples the HMD world position when applying each Peak.
3. The smooth controller uses horizontal HMD displacement for movement direction and predicted step distance.

Peak events and HMD movement are intentionally separate:

```text
HMD  -> alignment, direction, displacement / step distance
Peak -> left/right gait timing and state transitions
```

Moving the HMD without a Peak does not by itself initiate the Peak-driven walking cycle.

## 12. Input Methods

### Input Activation Guide

The input paths are independent and can be enabled separately. `PeakEventInput` must remain active because it contains the shared `LegPeakEventReceiver`; disabling it breaks every input path, including the Quest controller and optional `5056` forwarding.

| Input path | Default final-Scene state | How to activate | Gait-producing test |
|---|---|---|---|
| Keyboard loopback | Off (`UDPTestSender` inactive) | In the Hierarchy, select `UDPTestSender` and enable its GameObject. Leave `destinationAddress = 127.0.0.1` and `destinationPort = 5055` for a same-PC test. | Enter Play Mode and alternate Q/E. Space starts or pauses the configured automatic alternating stream. |
| Quest Touch | On | Leave the `QuestControllerPeakInput` GameObject and component enabled and keep its `peakEventReceiver` reference assigned. No UDP setup is required. | In Play Mode with tracked Touch controllers, alternate left Y/right B. |
| External Peak UDP `5055` | On | Leave `PeakEventInput`, `LegPeakEventReceiver`, and `UdpPeakEventReceiver` enabled. Allow inbound UDP `5055` on the Unity PC and send to that PC's LAN IPv4 address. | Send alternating supported left/right messages and confirm `receivedDatagramCount`, the shared Peak counts, and avatar leg motion. |
| External `5056` diagnostics | Off (`enableJasmineInput = false`) | Select `JasmineInput`, enable `enableJasmineInput`, leave `listenPort = 5056`, and keep `mapTopToPeak = false` for observation-only testing. Prefer setting this before Play Mode. | Send `left top` or another supported diagnostic message and confirm raw/parsed Console logs. No gait motion is expected. |
| External `5056` Top-to-Peak forwarding | Off (`enableJasmineInput = false`, `mapTopToPeak = false`) | Enable both fields on `JasmineInput` only when deliberate gait control from the provisional grammar is required. Keep its `peakEventReceiver` reference assigned. | Send alternating `left top` / `right top` messages and confirm `forwardedTopCount`, shared Peak counts, and leg motion. |

Recommended activation procedure:

1. Open the final Scene and choose exactly the input path being tested from the table above.
2. Make the required Inspector changes before entering Play Mode so each component's `OnEnable` startup runs with the intended values.
3. Enter Play Mode and confirm the relevant receiver is ready before sending events: `UdpPeakEventReceiver.isListening` for `5055`, or `JasmineInputAdapter.isListening` and `boundEndpoint` for `5056`.
4. Send alternating left/right events. Repeated same-leg or phase-inconsistent events may be rejected by the gait controller and are not a reliable walking test.
5. Check the input component's counters first, then `LegPeakEventReceiver` counters, then avatar motion. This separates transport/parsing failures from controller filtering or IK problems.
6. After a diagnostic test, stop Play Mode and restore the submitted defaults if they should not be committed: `UDPTestSender` inactive, `enableJasmineInput = false`, and `mapTopToPeak = false`.

Changing `enableJasmineInput` from false to true while its component is already enabled in Play Mode does not by itself call `StartListening()`. After changing it, either disable and re-enable the `JasmineInputAdapter` component or use its **Start Jasmine Listening** component context-menu command. If `5056` fails to bind, check `lastError` and verify that no other process owns the port. Never set the adapter to `5055`; the code deliberately rejects the working Peak receiver's port.

Do not enable multiple gait-producing sources unless duplicate events are intentional. For example, Quest button input, keyboard loopback, external `5055`, and mapped `5056` Top events all converge on the same `LegPeakEventReceiver` and are not automatically de-duplicated across sources.

### Keyboard

The keyboard path belongs to `UdpPeakEventLoopbackSender`:

```text
Q      sends "left is up"
E      sends "right is up"
Space  toggles the alternating automatic stream
```

It sends to `127.0.0.1:5055`. `automaticStream` is false in the final Scene. The `UDPTestSender` GameObject is also inactive, so it must be activated for Q/E/Space to work.

### Quest Controller

`QuestControllerPeakInput` uses Meta `OVRInput`:

```text
Left Touch controller Y   Left Peak
Right Touch controller B  Right Peak
```

Each leg has a `0.12 s` cooldown. This component calls `LegPeakEventReceiver` directly and does not use UDP.

### UDP Peak Input

Send a UTF-8 datagram to `<Unity-PC IPv4>:5055`. The receiver accepts:

```text
1             Left Peak
0             Right Peak
left is up    Left Peak
right is up   Right Peak
left          Left Peak
left up       Left Peak
left_up       Left Peak
right         Right Peak
right up      Right Peak
right_up      Right Peak
```

Aliases are active because `acceptAliases = true` in the final Scene. Parsing trims whitespace, is case-insensitive, and normalises hyphens/tabs.

### External IMU UDP Input

`JasmineInputAdapter` is configured for UDP `5056` but disabled by `enableJasmineInput = false`. If enabled for diagnostic use, its current provisional two-token grammar recognises:

```text
left top              right top
left hitbottom        right hitbottom
left startgoingup     right startgoingup
```

The adapter does not parse arbitrary accelerometer/gyroscope packets. `Top` is only forwarded to `LegPeakEventReceiver` when `mapTopToPeak = true`; `HitBottom` and `StartGoingUp` are logged/observed without gait output.

## 13. UDP 5055 Peak Input

`5055` is the implemented Peak-only gait interface.

```text
Component       UdpPeakEventReceiver
GameObject      PeakEventInput
Bind address    0.0.0.0:5055
Start on enable true
Aliases         true
Message logging true
```

The receiver thread decodes UTF-8 datagrams and queues accepted leg events. Unity's main thread drains the queue and calls `LegPeakEventReceiver.SubmitPeak`, which later publishes `PeakApplied`. The local Q/E test sender uses this same network path rather than bypassing it.

## 14. UDP 5056 External IMU Input

`5056` is a separate interface prepared for Yasmin/Devika external IMU/event integration.

Final Scene values:

```text
listenPort            5056
enableJasmineInput    false
mapTopToPeak          false
logRawMessages        true
logParsedEvents       true
acceptUnknownMessages true
```

With these submitted values, no `5056` socket is opened. The source code can log raw datagrams and parse the narrow event grammar listed above when enabled. Unknown messages can be retained as diagnostics, but they do not generate gait events. The submitted repository does not contain a confirmed general raw-IMU protocol mapping.

### Why there are two UDP ports

- `5055` preserves the working Peak-only gait and test interface.
- `5056` isolates external IMU integration and its different message semantics.

The separation prevents two receivers from competing for the same socket and keeps external protocol experiments from changing the established Peak test path. `JasmineInputAdapter` also checks that its listen port is not the existing `5055` port.

## 15. Diagnostics and Logging

All four CSV components are enabled on `PeakEventCsvLogger` and write to `PeakEventLogs/Proposed` in the project root.

- **PeakDiagnosticsPanel:** reads public receiver/controller diagnostics and updates the headset-mounted TextMeshPro panel with last-event time, per-leg interval, raw/filtered distance, and blend status.
- **PeakLandingRippleEffect:** subscribes to `FootLanded` and creates a temporary expanding LineRenderer. Its GameObject is inactive in the final Scene, so the visual is off by default.
- **PeakEventCsvRecorder:** subscribes to `PeakApplied` and writes event sequence/leg/timestamps, HMD/reference position, avatar/target positions, intervals, distances, phases, correction/blend, and fallback data. It uses reflection to read selected private controller diagnostic fields.
- **PeakPoseFrameCsvRecorder:** records frame/time, receiver counts, avatar/HMD/target/foot-bone positions, target errors, near-ground state, per-frame motion, and stance drift.
- **PeakStanceSummaryCsvRecorder:** writes one summary row per completed stance, including duration/frame count, target and bone displacement/path metrics, and mean/max error.
- **PeakLandingCsvRecorder:** subscribes to `FootLanded` and writes one row per predicted/forced/fallback landing with time, leg, reason, target position, and prediction values.

These systems observe and record gait state; they do not generate trajectories.

## 16. Non-runtime / Experimental Scripts

The following files remain in the repository but are not attached anywhere in the final Scene:

- `Assets/Scripts/FirstPersonEyeVerticalDiagnostics.cs`: delayed vertical eye-alignment measurement and log output.
- `Assets/Scripts/StartupAvatarHeightCalibrationCoordinator.cs`: one-time root-Y calibration and static leg-reach diagnostics; not part of the submitted Scene's startup chain.
- `Assets/Scripts/PeakEventMock/AvatarRootFollowHMD.cs`: alternative root follow with optional yaw; the final Scene instead uses `FirstPersonAvatarEyeAligner`.
- `Assets/Scripts/PeakEventMock/PeakTrajectoryMarkers.cs`: optional read-only primitive markers for current Peak, landing, and next Peak.

Their presence does not mean they execute at runtime. They have no Scene or Prefab attachment in the final Scene dependency graph.

## 17. Network and Firewall

The development PC has inbound Windows Firewall allowances for UDP `5055` and `5056`.

**Firewall rules are machine-specific and are not stored in GitHub.**

On another machine, allow the required inbound UDP ports in Windows Firewall or the active network security product. An external device must send to the Unity PC's LAN IPv4 address, not `127.0.0.1`. Loopback is only for a sender running on the same PC.

## 18. Recommended Code Reading Order

1. `Assets/Scripts/PeakEventMock/LegPeakEventReceiver.cs` — establishes the shared event model and main-thread boundary.
2. `Assets/Scripts/PeakEventMock/UdpPeakEventReceiver.cs` — shows the completed `5055` protocol and how network messages enter that model.
3. `Assets/Scripts/PeakEventMock/PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth.cs` — contains timing, prediction, T/4 motion, filtering, fallback, and recovery.
4. Open the final Scene and inspect `LegIKTargets`, `KneeGoals`, and the two LimbIK components — shows how generated targets become leg-bone motion.
5. `Assets/Scripts/FirstPersonAvatarEyeAligner.cs` — explains first-person avatar/HMD horizontal alignment.
6. `Assets/Scripts/PeakEventMock/JasmineInputAdapter.cs` — documents the separate, disabled `5056` diagnostic adapter and its exact limits.
7. Read `PeakDiagnosticsPanel.cs` and the four CSV recorders — explains what runtime evidence is exposed and recorded.

## 19. Manual Regression Test

After cloning or moving the project, perform this manual Unity Editor check:

1. Open the final Scene and confirm there are no Missing Scripts or compile errors.
2. Enter Play Mode and confirm that `Avatar_LegIK_Test` renders and stands normally.
3. Confirm that avatar horizontal eye alignment follows `CenterEyeAnchor` without changing root Y.
4. Before sending a Peak, confirm Startup Idle keeps both feet under the moving avatar.
5. Activate `UDPTestSender`; press Q and E alternately and confirm left/right Peak counts and leg response.
6. Confirm `UdpPeakEventReceiver` binds UDP `5055` and accepts an external supported message.
7. If Quest hardware is available, verify left Y and right B.
8. Observe Lowering, opposite-leg Rising, Peak Hold, Landing, post-cycle follow, Early Pending where reproducible, and Fallback/Recovery after a missing event.
9. Confirm both LimbIK targets and knee goals remain assigned and both legs solve normally.
10. If Ripple feedback is required, activate `LandingRippleManager` and confirm left/right landing ripples.
11. Confirm all four CSV files are created under `PeakEventLogs/Proposed` and contain rows.
12. For an optional external-interface diagnostic, enable `enableJasmineInput`, keep `mapTopToPeak = false`, send a test datagram to `5056`, and confirm raw/parsed logging without avatar gait output.
13. Confirm that no new Console errors appear during the test.

Static inspection can confirm serialized references and code paths, but it does not replace this hardware/Play Mode regression test.

## 20. Current Limitations

- Peak-driven left/right events remain the primary implemented gait interface.
- The external `5056` adapter is disabled in the submitted Scene and only understands a provisional two-token event grammar when enabled.
- Raw accelerometer/gyroscope streams are not transformed into the complete gait controller input in this submitted version.
- `HitBottom` and `StartGoingUp` are observation-only; only `Top` has an optional Peak mapping, which is disabled.
- The system estimates direction and step distance from horizontal HMD displacement but does not infer walking onset from HMD motion alone.
- Complex arbitrary locomotion and turning are not the main target of the final lower-body implementation.
- Meta upper-body transfer/comparison is not part of the final lower-body Scene.
- Q/E loopback and landing Ripple feedback are present but their GameObjects are inactive in the checked-in Scene.
- Runtime behaviour depends on HMD/controller tracking quality and must be validated in Play Mode on the target XR hardware.

## 21. Handover Summary

The submitted system separates sensing/input, gait control, and inverse kinematics:

```text
Peak source
  -> input adapter / UDP receiver
  -> LegPeakEventReceiver
  -> peak-driven lower-body controller
  -> world-space foot targets
  -> Final IK LimbIK
  -> avatar legs
```

The final Scene, its Build Settings entry, all 18 retained custom scripts, their runtime status, the two UDP interfaces, diagnostic outputs, and current limitations are documented above. The implemented reference path is UDP `5055`/Quest Peaks through the smooth controller and Final IK; the separate `5056` adapter is preserved as a disabled diagnostic integration endpoint in the submitted version.
