# Vision tools — building-block model

A vision program is a list of small, generic **tools** that read **image layers** and can follow each
other's found positions (**fixtures**). This replaces the old zone + ten-typed-inspection model and the
special-purpose Frame inspection (contour / lines / corners / checker). Saved programs in the old format
are not converted: the controller skips them when loading (the files stay on disk untouched; saving a
program with the same id overwrites one) and they are rebuilt.

The design follows the common pattern of commercial vision software (Cognex In-Sight / VisionPro,
Keyence CV-X, MVTec MERLIC, Matrox Design Assistant, Omron FH, Aurora Vision):
**locate → inspect → output**, any locating tool can act as a fixture, regions live in the fixture's
coordinates, and filters are stacked layers.

## Conventions

- **Regions** are in normalized image coordinates (0..1 of width / height), as drawn on the teach image.
- **Results** (points, lines, poses, items) are in **pixels** of the processed frame. RunVision converts
  them to pixel / normalized / robot coordinates for program variables (existing output-frame logic).
- **Angles** are degrees, `atan2(dy, dx)` in pixel coordinates with **y down** (the OpenCV convention):
  0 = pointing right, **positive turns clockwise on screen**, range (−180, 180]. A rect region's
  `rotation` uses the same sense (as the old zones did). Everything — line/pose/item angles,
  fixture rotation, `angle` tool — follows it.
- A circle region's `radius` is a fraction of min(width, height) of the image.
- Ids are stable strings; tools link by id, never by name.

## Program

```jsonc
{
  "id": "…", "name": "…", "description": "…", "cameraId": "…", "lastUpdatedUnixMs": 0,
  "layers": [ImageLayer],   // shared filtered images
  "tools":  [VisionTool]    // run in list order (a tool may only reference tools/layers before it)
}
```

## Image layers (shared filters)

```jsonc
{
  "id": "soft", "name": "Soft gray",
  "input": "camera",              // "camera" or an earlier layer id
  "filters": [Filter]             // applied in order
}
```

`Filter = { "id", "type", "enabled": true, ...params }` — params sit flat on the filter object:

| type | params | notes |
|---|---|---|
| `channel` | `channel`: `gray` \| `r` \| `g` \| `b` \| `h` \| `s` \| `v` | single-channel output |
| `colorMask` | `hMin,hMax,sMin,sMax,vMin,vMax` (OpenCV HSV ranges, h 0–179) | output 0/255 mask |
| `blur` | `size` (odd px) | Gaussian |
| `median` | `size` (odd px) | |
| `threshold` | `mode`: `fixed` \| `otsu` \| `adaptive`; `low`, `high` (fixed); `blockSize`, `c` (adaptive); `invert` | binary output |
| `morph` | `op`: `erode` \| `dilate` \| `open` \| `close`; `size`; `iterations` | |
| `contrast` | `mode`: `linear` \| `equalize` \| `clahe`; `brightness`, `contrast` (linear); `clipLimit`, `tileSize` (clahe) | |
| `edges` | `mode`: `sobel` \| `canny`; `low`, `high` (canny) | |
| `backgroundRemove` | `size` (blur size, px) | gray − large blur, re-centred to 128 |
| `invert` | — | |

Filters that need a gray input convert colour to gray first (`threshold`, `contrast` equalize/clahe,
`edges`, `backgroundRemove`); `blur`, `median`, `morph`, linear `contrast` and `invert` keep colour.
`colorMask` with `hMin > hMax` wraps round hue 0 (reds). `threshold` `fixed` keeps `low ≤ v ≤ high`
(255) and drops the rest. Disabled filters are skipped. Layer outputs are cached per frame; a layer
may read any other layer that does not lead back to itself (a cycle is a tool error).
A layer can be previewed at any filter (`upTo` index) for tuning:
`GET /vision/{programId}/layer/{layerId}?upTo=N` (JPEG; omit `upTo` for all filters, `-1` for the
layer's input; `layerId` `camera` = the frame).

## Tools

Common header on every tool:

```jsonc
{
  "id": "t1", "name": "Top edge", "type": "findLine", "enabled": true,
  "input": "camera",             // "camera" or a layer id
  "fixtureId": null,             // id of an earlier tool that produces a pose, or null
  "region": Region,              // omitted for tools that need none (makeFixture, distance, angle)
  "regionFrom": null,            // id of an earlier tool whose region (and fixture) this one uses, or null
  "grid": null,                  // { rows, cols, inset (0..0.45) } for cell-capable tools
  "limits": { "percent": { "min": 20, "max": null } },   // judgement on any numeric value key
  "includeInPass": true,
  "reference": null,             // pose-producing tools: Pose captured at teach time
  "params": { … }                // type-specific, below
}
```

`Region` = `{ "shape": "rect" | "circle" | "polygon" | "quad", "x","y","width","height","rotation" | "cx","cy","radius" | "points": [{x,y}] }`
(`quad` = exactly 4 points TL, TR, BR, BL).

A `quad` region with no fixture is a **fixed skewed area**: it stays where it was drawn, and a grid on
it follows the perspective (each cell is the image of a unit-square cell under the quad's homography).
Use it for a flat target seen at an angle whose position does not change, setting its four corners by
hand on the image.

### Shared regions

`regionFrom` names an earlier tool (enabled or not) whose region this tool uses instead of its own —
and whose fixture, since a fixtured region is stored in that fixture's reference space. Chains resolve
to the first tool without `regionFrom`. The tool's own `region` and `fixtureId` are ignored while it is
set (keep a copy in `region` for controllers that predate it). The `grid` stays the tool's own, so
several tools can measure the same area with different cell insets. A `regionFrom` that is not an
earlier tool, or loops, is a tool error. In the app the area is drawn on either tool and moves for all.

### Fixtures

- A tool that outputs a `pose` can be another tool's fixture.
  `Pose = { x, y, angle, scale (1), quad?: [4 points] }` (pixels).
- A fixture tool stores `reference`, the pose it found on the teach image (set by "Set reference" in the
  app, or automatically on the first successful run when null).
- At run time a dependent tool's region is moved by the transform from `reference` to the pose found now:
  a similarity transform (shift + rotate + scale), or a **perspective** transform when both poses carry a
  `quad`. Grids inside a perspective-moved region follow the perspective (cells are quads).
- Fixtures chain (a fixture tool can itself have a fixture). References must point to earlier tools, so
  there are no cycles.
- **If a fixture is not found, every tool that depends on it (directly or through the chain) is `notRun`
  with a message naming the fixture, and counts as failed.** It never runs at the untransformed region.
  The same holds for a tool whose `sources` (makeFixture, distance, angle) did not succeed, and for a
  fixture / source that is disabled or is not an earlier tool.
- `reference` is set by the `SetVisionToolReference {programId, toolId, clear?}` command (from the
  program's latest result; `clear: true` empties it), or captured automatically on the first
  successful run when null — the controller then saves the program (`RunVisionOnImage` with a saved
  `programId` saves it too; an inline `program` is changed only in the reply's run).

### Results

```jsonc
{
  "id": "t1", "type": "findLine",
  "status": "ok" | "fail" | "notRun" | "error",
  "passed": true,                 // status ok and every limit satisfied
  "message": null,
  "timeMs": 1.2,
  "values": { "found": 1, "angle": 2.3, "x": 312, "y": 88, "fitError": 0.7 },   // numbers (bools as 0/1) and strings
  "pose": Pose | null,
  "points": [{x,y}], "lines": [{x1,y1,x2,y2}],
  "items": [ { … } ],             // list records (blobs, matches, codes, markers…)
  "cells": [ { "row", "col", "index", "value", "passed", "quad": [{x,y}×4] } ],   // quad: the measured cell window (after inset), px, for the overlay
  "region": [{x,y}]               // the region outline as actually used (after fixture), in pixels, for the overlay
}
```

`VisionResult = { programId, timestamp (unix ms), imageWidth, imageHeight, passed (AND of enabled includeInPass tools), tools: [ToolResult], layers: [id], timings, errors }`.
`timings` is ms by tool id, plus `"layer:{id}"` for each layer computed. `errors` are `"{toolId}: {message}"`
for tools with status `error`. A disabled tool has no entry in `tools`.

Status: `ok` = the tool ran (and, for a locating or reading tool, found something); `fail` = it ran
but found nothing (findLine/findCorner/findCircle/contourMatch/patternMatch/aruco/makeFixture/caliper/
barcode with no result — a `blob` count of 0 is still `ok`, judged by limits); `notRun` = its fixture or
a source was not found; `error` = it threw (bad params, unknown type, missing input layer). `pose` is
null unless the status is ok.

**Limits.** `passed` = status ok and every limit satisfied. A limit on a key the tool did not produce
fails. On a gridded tool the limit on its **main value** is applied to each cell (`cells[].passed`) and,
for the tool, means **every cell passed** — the whole-region value is still reported. `cellsPassed`
counts passing cells.

### Tool catalogue

**Locate (output a pose)**

| type | region | params | values / outputs |
|---|---|---|---|
| `findLine` | rect (line runs along its width axis; probes search across its height) | `probes` (8), `polarity`: `darkToLight` \| `lightToDark` \| `any`, `contrast` (20), `smoothing` (1), `ignore` (outliers to drop, 0), `pick`: `strongest` \| `first` \| `last` (strongest) | `found, x, y` (line midpoint), `angle, fitError, pointsUsed`; `lines[0]`, `points`; pose = midpoint + angle |
| `findCorner` | rect = side A; `params.regionB` = rect side B | line params as above (shared) | `found, x, y, angle` (side A angle), `cornerAngle`; `lines[2]`; pose = corner + angle A |
| `findCircle` | circle (annulus `params.innerRatio` 0.5) | `probes` (16), `direction`: `inward` \| `outward`, `polarity`, `contrast`, `smoothing`, `ignore`, `pick` | `found, x, y, radius, fitError, pointsUsed`; `points`; pose = centre, angle 0 |
| `blob` | any | `polarity`: `dark` \| `light`, `threshold`: `otsu` \| `fixed`, `level`, `minArea`, `maxArea`, `minCircularity`, `sort`: `area` \| `x` \| `y`, `maxCount` (100) | `count, found, x, y`; `items [{x,y,area,angle,circularity}]`; pose = first item |
| `contourMatch` | any | `template` `{points:[{x,y}] (0–1 of its own bbox), width, height}` (taught), `minScore` (0.7), `maxCount` (10), `minArea` (50), `polarity` / `threshold` / `level` as blob | `count, score, x, y, angle`; `items [{x,y,angle,score}]`; pose = best |
| `patternMatch` | any (search area) | `template` `{png (base64 gray PNG), originX, originY, width, height}` (taught), `minScore` (0.7), `angleRange` (0), `angleStep` (5), `maxCount` (1) | `count, score, x, y, angle`; `items [{x,y,angle,score}]`; pose = best |
| `aruco` | any | `dictionaryId` (OpenCV predefined id, 1 = 4x4_100; −1 = all 17 classic dictionaries), `markerId` (-1 = any), `minArea`, `maxArea` | `count, id, x, y, angle`; `items [{id,x,y,angle}]`; `lines` = marker outlines; pose = first |
| `makeFixture` | none | `mode`: `pointAngle` \| `twoPoints` \| `lineIntersection` \| `fourLines` \| `fourPoints`; `sources`: [toolId…] | `found, x, y, angle`; pose (with `quad` for the four-* modes) |

Probes and calipers: a profile's edge **contrast** is its intensity step (grey levels) across the edge;
`smoothing` is a Gaussian sigma (px) on the profile. findLine probes run from the region's top side to
its bottom side (its +height direction = the width direction turned 90° clockwise on screen), so for a
region drawn clockwise round an object the probes run from outside in. findLine averages each probe
over a band half its spacing wide; `ignore` drops that many worst-fitting points after a first fit.
findCircle probes are radial (inward = from the circle towards the inner radius).

**Teaching** `contourMatch` / `patternMatch`: `TeachVisionTool {programId, toolId, image?}` runs the
earlier tools (so a fixtured region is where its fixture puts it), then cuts the template from the
tool's region on its input layer (patternMatch: the region unrolled upright as a gray patch, origin at
its centre; contourMatch: the largest blob in the region, thresholded as the tool's params say) and
saves it in `params.template`. The image defaults to the program's newest camera frame.

`makeFixture` modes: `pointAngle` (origin = pose of source 0, angle = pose angle of source 1),
`twoPoints` (origin = source 0, angle = direction to source 1), `lineIntersection` (two line tools →
intersection, angle of the first), `fourLines` (top, right, bottom, left lines → the four corner
intersections TL, TR, BR, BL as a perspective quad), `fourPoints` (four point/pose sources TL, TR, BR, BL).
For the four-* modes the pose origin is the corners' centroid and its angle the TL→TR direction; the
quad must be convex.

**Measure**

| type | region | params | values / outputs |
|---|---|---|---|
| `caliper` | rect (searches along its width, projects along its height) | `polarity`, `mode`: `single` \| `pair` \| `all`, `contrast`, `smoothing`, `pick`: `first` \| `last` \| `strongest` \| `center`, `pairWidth` (0 = any), `pairTolerance` (5) | `count, x, y`, `contrast` (single), `width` (pair); `points` (pair: its two edges; all: every edge) |
| `distance` | none | `mode`: `pointPoint` \| `pointLine`; `sources` [a, b] | `distance` (px), `dx, dy` |
| `angle` | none | `sources` [lineA] or [lineA, lineB] | `angle` (vs image x-axis, or between lines) |
| `corners` | any | `method`: `shiTomasi` \| `harris`, `maxCorners`, `quality`, `minDistance`, `blockSize` | `count`; `points` |
| `lines` | any | `canny1, canny2, houghThreshold, minLineLength, maxLineGap, minAngle, maxAngle` (0–180, undirected; min > max wraps) | `count`; `lines`; `items [{x1,y1,x2,y2,angle,length}]` |

Geometry sources: a tool's pose, else `points[0]`, else values `x`/`y` for points; `lines[0]` for lines.
`distance` pointLine measures to the infinite line; `angle` with two lines is B − A in (−180, 180].

**Presence and count** (all support `grid`; each cell is judged with the tool's limits on its main value)

| type | params | main value | values |
|---|---|---|---|
| `pixelCount` | `mode`: `color` \| `gray`; `colors [{r,g,b,tolerance}]` (tolerance 0–100 → ±2.55·tolerance per channel) or `grayMin, grayMax` | `percent` | `percent, cellsPassed` |
| `intensity` | `statistic`: `mean` \| `stdDev` \| `min` \| `max` \| `contrast` (= max − min) | the chosen statistic | `mean, stdDev, min, max, contrast, cellsPassed` |
| `edgePixels` | `cannyLow, cannyHigh` | `density` (% of pixels) | `density, angle` (dominant edge direction 0–180, whole region), `cellsPassed` |
| `polygon` | `sides, minArea, maxArea, epsilon` (expects a binary/thresholded input; > 127 = object) | `count` | `count, x, y, angle` (largest); `items [{x,y,angle,area}]`, `cellsPassed` |

`grid.inset` (0–0.45) trims each cell on every side before measuring; a circle or polygon region's
cells are laid over its bounding box and clipped to it; a rect's cells turn with it and a quad's (or a
perspective-moved region's) follow the perspective.

**Read**

| type | params | values |
|---|---|---|
| `barcode` | `formats` (ZXing names, e.g. `QR_CODE`; empty = all) | `count, value, format, x, y` (first); `items [{value, format, x, y}]`; status fail when none read |

## RunVision step

```jsonc
{
  "type": "RunVision", "visionProgramId": "…", "outputFrame": "pixel" | "normalized" | "robot",
  "passVariable": "visionOk",               // optional: overall pass 0/1
  "toolOutputs": [
    { "toolId": "t1", "value": "angle",  "variable": "edgeAngle" },   // any values key → number/string
    { "toolId": "t1", "value": "passed", "variable": "edgeOk" },
    { "toolId": "t5", "value": "cells",  "variable": "whiteCells" },  // list of {row,col,index,value,passed}
    { "toolId": "t2", "value": "items",  "variable": "blobs" },       // list of records
    { "toolId": "t3", "value": "points", "variable": "pts" },         // Points list
    { "toolId": "t4", "value": "pose",   "variable": "partPose" }     // record {x,y,angle}
  ]
}
```

Coordinates in `points`, `items` (x/y), `lines`, `pose` and `values.x/y` are converted by `outputFrame`
(default `pixel`); other values (width, radius, distance…) stay in pixels. This replaces the eight typed
`*Outputs` fields **and `visionZoneId` / `visionZoneVar`** (there are no zones any more).
`visionProgramName` stays (display name). Written variable kinds:

| `value` | variable |
|---|---|
| a values key | Number, or String for a text value (barcode `value`, `format`) |
| `passed`, `found` | Boolean 1/0 |
| `status` | String |
| `cells` | Record list `{row, col, index, value, passed}` |
| `items` | Record list — numeric fields only (records hold numbers; barcode text is dropped, read it with `value`) |
| `points` | Point list |
| `lines` | Record list `{x1, y1, x2, y2}` |
| `pose` | Record list with **one** record `{x, y, z, angle}` (z = calibration plane in the robot frame, else 0), empty when not found — read `$partPose[0].x` (there is no single-record variable type) |

A tool with no result (disabled / not in the program) or that did not run empties its lists and writes 0
for `passed`, `found` and `count`; other scalars are left unchanged. Validation: `unknownVisionTool`
(warning) for a `toolId` not in the program, `missingField` for an entry without tool, value or variable.

## Controller endpoints and commands

| | |
|---|---|
| `SetVisionToolReference {programId, toolId, clear?}` | reference := the tool's pose in the latest result (or cleared); saves |
| `TeachVisionTool {programId, toolId, image?}` | cut a patternMatch / contourMatch template (see Teaching); saves |
| `RunVisionOnImage {programId \| program, image, includeAnnotated?}` | one run on a base64 image → `result` (+ `annotated`) |
| `GET /vision/{id}/layer/{layerId}?upTo=N` | layer preview JPEG |
| `GET /vision/{id}/tool/{toolId}/debug` | one tool's result drawn over its input layer (JPEG) |

Details in [websocket-api.md](websocket-api.md#vision).
