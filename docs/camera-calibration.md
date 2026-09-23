# Camera-to-robot calibration

Gives a fixed camera its own coordinate frame on the robot: after calibration, a
pixel seen by the camera maps to a robot X/Y on the calibration plane (and a
fixed Z), so vision results can be used directly as move targets.

## Method

1. The user places a **dot calibration sheet** (a regular grid of dark dots on
   white, or white on dark) in the camera's view and enters the **dot pitch** in
   mm (center-to-center distance, same in both directions).
2. The controller grabs a frame, detects the dots, and fits a grid: every dot
   gets integer indices `(i, j)`, so its **sheet coordinate** is
   `(i × pitch, j × pitch)` mm. A homography `H` is fitted from pixel → sheet
   using all dots, which removes lens perspective as long as the sheet is flat.
3. The user jogs the robot tip onto **two or three dots** (three, not collinear,
   is recommended) and teaches each: the controller records the robot's TCP
   position for that dot index.
4. Solving fits a **rigid transform** (rotation + translation, no scale, since
   the pitch already sets the scale) from sheet mm → robot XY using the taught
   dots, and takes the plane **Z** as the mean taught Z. The composed map
   `pixel → sheet → robot` is stored per camera, with the residual error of the
   taught dots and an estimated pitch-scale mismatch as quality indicators.

Coordinates on the wire are **normalized** image coordinates `u, v` in 0–1 (as
the vision results already use), plus the image size so pixel values can be
recovered.

## Persisted model — `cameraCalibrations/<cameraId>.json`

```json
{
  "cameraId": "CAM_0",
  "imageWidth": 1280, "imageHeight": 720,
  "dotPitchMm": 20,
  "pixelToSheet": [[h11,h12,h13],[h21,h22,h23],[h31,h32,h33]],   // pixel (px) -> sheet mm, homography
  "sheetToRobot": { "cos": c, "sin": s, "tx": x, "ty": y, "mirrored": true }, // rigid 2-D: R·(x, ±y) + t
  "pixelToRobot": [[…],[…],[…]],                                   // composed homography px -> robot mm
  "planeZ": 12.5,
  "taughtDots": [ { "dotIndex": 0, "i": 0, "j": 0, "u": 0.31, "v": 0.44, "robot": { "x":…, "y":…, "z":… }, "errorMm": 0.2 }, … ],
  "gridRows": 6, "gridCols": 8, "dotCount": 48,
  "gridRmsPx": 0.4,          // homography reprojection RMS over all dots, px
  "taughtRmsMm": 0.35,       // residual of taught dots after the rigid fit, mm
  "taughtMaxMm": 0.5,
  "pitchScaleEstimate": 1.004, // >1 means the taught distances are longer than pitch implies
  "mirrored": true,          // the sheet→robot fit used a reflection (usual for a camera looking down)
  "activeTool": "None",      // tool that was active while teaching (the TCP that touched the dots)
  "calibratedUnixMs": 1790000000000
}
```

Exposed as `CameraCalibration` with:
- `PixelToRobot(u, v)` → `(x, y, z)` (u, v normalized 0–1)
- `RobotToPixel(x, y)` → `(u, v)`
- `IsCalibrated` on `CameraState` (additive field `calibrated: true|false` in GetCameras).

## Commands (all additive)

| Command | Params | Response |
|---|---|---|
| `GetCameraCalibration` | `cameraId` | `calibration` (object above) or `calibration: null` |
| `DeleteCameraCalibration` | `cameraId` | — |
| `CalibrationStart` | `cameraId`, `dotPitchMm`, optional `minDotAreaPx`, `maxDotAreaPx`, `darkDots` (default true) | `sessionId`, `imageWidth`, `imageHeight`, `dots: [{ index, i, j, u, v, areaPx }]`, `gridRows`, `gridCols`, `gridRmsPx`, `warnings: [string]`, `imageUrl` (`/calibration/{sessionId}/image`, annotated JPEG with dot indices) |
| `CalibrationRedetect` | `sessionId` (+ same optional detector params) | same as `CalibrationStart` (new frame, same session; taught dots whose index still matches by nearest position are kept) |
| `CalibrationTeachDot` | `sessionId`, `dotIndex` | `taught: [{ dotIndex, i, j, robot: {x,y,z} }]` (records the current TCP position; replaces an earlier teach of the same dot) |
| `CalibrationUnteachDot` | `sessionId`, `dotIndex` | `taught: […]` |
| `CalibrationSolve` | `sessionId`, optional `save` (default true) | `calibration` (object above), `taughtRmsMm`, `taughtMaxMm`, `pitchScaleEstimate`, `warnings` |
| `CalibrationPredict` | `cameraId` or `sessionId`, `u`, `v` | `robot: {x, y, z}` (from the saved calibration, or from a solved-but-unsaved session) |
| `CalibrationDiscard` | `sessionId` | — |

Errors: `unknownCamera`, `cameraNotConnected`, `noDotsFound`, `gridNotFound`
(fewer than 4 dots or no consistent lattice), `unknownSession`, `unknownDot`,
`notEnoughTaught` (< 2), `taughtCollinear` (3+ taught but all on one line, so
only a 2-point solution is possible — a warning when exactly 2), `notCalibrated`.

HTTP: `GET /calibration/{sessionId}/image` → the annotated JPEG (404 unknown
session). Sessions expire 30 minutes after their last use.

## Grid fitting rules

- Dot detection: grayscale → Gaussian blur → Otsu threshold (inverted for
  `darkDots`) → contours; keep blobs by area range and circularity ≥ 0.6;
  centroid by image moments. Default area range 30–20 000 px.
- Lattice: take each dot's two nearest neighbours in roughly perpendicular
  directions to seed basis vectors (median over all dots), assign each dot the
  integer `(i, j)` that best explains its position, fit `H` from `(i, j)` →
  pixel, re-assign by nearest reprojected lattice point, iterate ≤ 5 times, then
  drop dots whose reprojection error exceeds 3 × the RMS, but never below
  0.5 px (partial dots at the edge, dirt; the floor keeps a near-perfect grid
  from shedding good dots to quantisation noise). A blob whose lattice
  position is more than 0.35 cells from an integer, or that claims a position
  another dot fits better, is not on the grid either. Indices are shifted so the minimum `i` and `j` are 0; the origin
  and axis directions are arbitrary — the rigid fit against taught dots absorbs
  the choice, including a mirrored assignment (allow a reflection in the
  sheet→robot fit; report `mirrored: true` in the calibration when used).
- `gridNotFound` when fewer than 4 dots survive or the lattice fit RMS exceeds
  2 % of the median dot spacing.

## Using a calibration in programs

`RunVision` gains `outputFrame: "pixel" | "normalized" | "robot"` (default keeps
today's behaviour). With `"robot"` and a calibrated camera, blob centers,
polygon centroids and ArUco centers written to point variables are robot
X/Y with Z = `planeZ`; validation reports `cameraNotCalibrated` (error) when
`"robot"` is requested for a vision program whose camera has no calibration.
Expression properties: `$camera.<id>.calibrated` (1/0).

## App wizard (cameras page → camera card → "Calibrate")

1. **Setup** — camera, dot pitch (mm), dark/light dots, tip note ("use the tool
   that will touch the sheet; select it as active tool first").
2. **Detect** — annotated image with numbered dots; Re-detect; warnings; the
   grid summary (rows × cols, RMS).
3. **Teach** — tap a dot (or pick its number); embedded jog pad + position
   readout; Teach / Unteach; a list of taught dots with robot coordinates;
   guidance: at least 2, 3 recommended and not in a line.
4. **Solve** — residual table (mm per taught dot, RMS, max), pitch-scale
   estimate with a warning above 2 %, Save.
5. **Verify** — tap any dot → predicted robot X/Y/Z; "Move above dot" (MoveL to
   the prediction with Z + 20 mm) behind a confirmation; Done.

Camera cards show a "calibrated" pill with the date; the vision editor shows the
calibration state next to the camera picker.

## Printable sheets

`docs/calibration-sheet-A4-20mm.svg`, `docs/calibration-sheet-Letter-20mm.svg`
and `docs/calibration-sheet-A4-10mm.svg` are drawn in true millimetres. Print
at 100 % ("actual size", no fit-to-page) and confirm the 100 mm scale bar with
a ruler; the pitch printed in the footer is the value to enter in the wizard.
A 20 mm pitch suits cameras seeing roughly 200–400 mm of the workspace; use
10 mm for a closer view. Tape the sheet flat on the surface the robot will work
on: the calibration is only valid for that plane.
