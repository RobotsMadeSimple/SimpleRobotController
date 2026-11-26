# Cell packages — the 3D model of a robot's work cell

A **cell package** describes everything a 3D viewer needs to draw a robot cell live: the robot,
the tools that can sit on its flange, and the fixtures around it. The controller stores packages,
the app draws the active one from live status, and the Fusion add-in (FusionRobotSim) builds and
uploads them.

## 1. Frames and units

- Millimetres and degrees throughout.
- **All geometry is written in the robot's base frame at its zero pose**: Z up, the J1 axis
  through the origin, ASTRO's reach pointing −Y. This is the frame `robots/astro.json` in
  FusionRobotSim already uses, and the pose the ASTRO CAD is drawn in (controller J1 90°,
  radial 319.8, vertical 311.1).
- A link's geometry moves with that link's transform. Tool geometry moves with the robot's
  flange link. Fixture geometry never moves.
- GLB meshes use the same frame and units as the JSON: raw millimetre positions, Z up, no node
  transforms. Viewers must not apply glTF's usual metres/Y-up convention.

## 2. Package layout

A zip with `cell.json` at its root and any meshes it references:

```
cell.json
meshes/servo-gripper-body.glb
meshes/servo-gripper-jaw-left.glb
meshes/checker-board.glb
```

Paths in `cell.json` are relative to the package root, forward slashes, no `..`.

## 3. cell.json

```jsonc
{
  "format": "rms-cell",
  "version": 1,
  "id": "checkers-cell",              // stable id; the controller stores the package under it
  "name": "Checkers cell",
  "robot": null,                      // null/absent = the controller's built-in robot for its type
  "tools": [ /* §3.2 */ ],
  "fixtures": [ /* §3.3 */ ]
}
```

### 3.1 Robot

`robot` uses the robot-definition format of FusionRobotSim's `robots/astro.json`, unchanged:
`joints[]` (`name`, `type` revolute|prismatic, `parent`, `child`, `origin`, `axis`,
`value {field, scale, zero}` where joint value = (status[field] − zero) × scale) and `links[]`
(`name`, `color` [r,g,b] 0–255, `shapes[]` of `box {min,max}` and `cylinder {center [x,y],
radius, z [z0,z1]}`; a shape may override `color`; `holes` are informational and viewers may
ignore them). `flange` names the link tools mount to.

A package normally leaves `robot` out and gets the controller's **built-in robot** for its
robot type (ASTRO: the blocky ASTRO Sim model, identical to FusionRobotSim's `astro.json`). A
link may also carry `"meshes": [{ "mesh": "meshes/x.glb", "color"?: [r,g,b] }]` in addition to
or instead of `shapes`.

**Adjustable axes.** The robot definition may declare axes whose length is a controller
setting:

```jsonc
"adjustable": [
  { "id": "vertical", "setting": "astroVerticalTravelMm", "nominal": 400, "direction": [0, 0, 1] },
  { "id": "radial",   "setting": "astroRadialTravelMm",   "nominal": 400, "direction": [0, 1, 0] }
]
```

A shape (or link mesh) tagged `"adjust": { "id": "vertical", "role": "stretch" }` is lengthened
by Δ = setting − nominal along `direction`: its far face (the one further along `direction`)
moves out by Δ, the near face stays. A shape tagged `"role": "end"` is translated by Δ along
`direction`. Untagged shapes are unchanged. Kinematics never depend on these settings; they only
change what's drawn (and, in the controller, nothing else yet). For ASTRO:

- **vertical** (J2 column, grows upward): the 2060 column and both rails stretch (Turret shapes
  spanning z 94.7–494.7); the top motor plate, top motor and top cap move up (Turret shapes at
  z ≥ 494.7).
- **radial** (J3 reach, grows at the back, +Y, so the wrist end and J4 stay put): the reach
  extrusion and both rails stretch (Reach shapes spanning y −292.3→187.7 / −252.3→147.7); the
  back end plate moves (Reach shape at y 187.7–199.7).

### 3.2 Tools

```jsonc
{
  "id": "servo-gripper",
  "name": "Servo Gripper",
  "controllerTool": "a1b2…",          // id of the controller tool (GetTools) this model shows for; optional
  "meshes": [ { "mesh": "meshes/servo-gripper-body.glb" } ],
  "shapes": [],                         // primitives, same as robot links (optional)
  "moving": [                           // parts driven by an IO value (optional)
    {
      "meshes": [ { "mesh": "meshes/servo-gripper-jaw-left.glb" } ],
      "drive": { "field": "output1", "on": 1 },          // status field; "on" = the value meaning closed/actuated
      "slide": { "direction": [-1, 0, 0], "travel": 20.14 }, // open = moved by travel along direction; on = home
      // or "rotate": { "axis": [0,0,1], "origin": [x,y,z], "degrees": -181.3 }
      "speed": 40                       // mm/s (or deg/s), for animating between states; optional
    }
  ]
}
```

Tool geometry is in the base frame at the zero pose **as mounted on the flange**, so it rides
the flange link's transform. The viewer shows the tool whose `controllerTool` matches the
controller's active tool id (`activeToolId` in GetStatus); a tool with no `controllerTool` is
shown when no other tool matches. A package with one tool and no ids always shows it.

### 3.3 Fixtures

```jsonc
{ "id": "checker-board", "name": "Checker board", "meshes": [ { "mesh": "meshes/checker-board.glb" } ],
  "shapes": [], "collision": true }
```

Static geometry in the base frame. `collision` is stored for later collision checks.

## 4. Controller

Packages live in `<data>/cells/<id>/`. HTTP (CORS open, like `/dxf`; `OPTIONS` preflight answered):

| Route | |
|---|---|
| `POST /cells?replace=true` | body = package zip (≤ 200 MB); id from `cell.json`. 200 `{ id, cell: <summary> }`; 400 `{ error: "badZip"\|"badPackage", message }`; 409 `{ error: "idExists" }` if it exists and `replace` isn't set; 413 `{ error: "tooLarge" }` |
| `GET /cells` | `[{ id, name, updatedUnixMs, active, tools, fixtures }]` — `tools`/`fixtures` are counts; `name` falls back to `id` |
| `GET /cells/active` | the active cell.json with `robot` resolved (built-in filled in) and `settings` — the current value of every adjustable setting. With no active package: the built-in robot alone, as `{ format, version, id: null, name: null, robot, tools: [], fixtures: [], settings }` |
| `GET /cells/{id}` | that package's cell.json, resolved the same way; 404 `{ error: "unknownCell" }` |
| `GET /cells/{id}/{**path}` | a file from the package (`.glb` → `model/gltf-binary`, `.gltf` → `model/gltf+json`, `.png`, `.jpg`, else octet-stream); 404 if missing or the path escapes the package |
| `DELETE /cells/{id}` | remove it (clears active if it was); 404 if unknown |

Upload validation: `cell.json` at the zip root (not in a sub-folder), `format` `"rms-cell"`,
`version` 1, `id` matching `[a-z0-9][a-z0-9-_]{0,63}` and not `active` (reserved by the route),
every `"mesh"` path anywhere in cell.json present in the zip, and no zip entry that is absolute,
has a drive, or uses `..` (the whole upload is rejected). A replacement is unpacked to a staging
folder and swapped in, so a bad upload never damages the stored package.

`robot` resolves to the built-in when it is absent or `null`; a robot with no built-in (CNC4Axis)
resolves to `null` with empty `settings`. A package that supplies its own robot with an `adjustable`
setting the controller doesn't know gets that axis's `nominal` in `settings` (drawn unchanged). The
built-in ASTRO is FusionRobotSim's `astro.json` without its FusionRobotSim-only `controller`
connection block (ws url), plus `adjustable` and the `adjust` tags.

WebSocket commands: `GetCells` → `{ cells: [<summary>], activeCellId }`, `SetActiveCell { id | null }`
→ `{ activeCellId }`, `DeleteCell { id }`; unknown ids → `ok: false, error: "unknownCell"`. The
active id is the robot setting `activeCellId` (also settable through `SetRobotConfig`). `GetStatus`
carries `lastCellUpdate` (bumped on any package or active change, a change to an adjustable setting,
or a robot-type change, since that swaps the built-in robot) and `activeToolId` (the active tool's
stable GetTools `id`, null when none).
