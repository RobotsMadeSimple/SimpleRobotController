"""
End-to-end checkers-vs-camera test against a controller, with mock camera images.

A "phone" camera on the controller is fed rendered JPEGs over the real ingest socket
(/camera/{id}/ingest), so every frame goes through the same path a phone's would:
camera -> vision program (skewed 8x8 grid aligned to a frame inspection) -> RunVision in the
real "Checkers Game (camera)" program -> the checkers plugin's detectMove -> applyMove/aiMove.

The scene is an 8x8 board with a border, seen at an angle from the side, on a wood table, with
lighting falloff, sensor noise and JPEG compression. Pieces use the colour definitions of the
real "Checkers Board" vision program. A scripted human plays white: each move is shown first
with a hand over the board (piece lifted, then placed), then the hand leaves. Part-way through,
the camera is bumped to a new angle — the frame inspection has to re-align the grid. The robot
arm is simulated (no hardware): when the program has played its move, the test applies it to the
scene, as the arm would have.

Checks: the program waits for the starting position; every human move is read exactly (from/to,
never before the hand has gone, never an illegal one); every robot move the engine made matches
the scene; it keeps working after the camera is bumped; no program error.

    python checkers_camera_e2e.py --ctrl 127.0.0.1:9101 --plugin-zip <checkers.zip> [--plies 12] [--shots DIR]

Needs a controller you can run programs on (a scratch / virtual one - the arm moves!), Python
with opencv-python, numpy and websocket-client, and node (to build the game program).
"""
import argparse
import base64
import json
import os
import random
import subprocess
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import websocket

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mcp"))

W, H = 640, 480
FLAT, BORDER = 480, 40                       # flat board image: 400 px of squares + 40 px border
SQ = (FLAT - 2 * BORDER) // 8

# Colours of the real "Checkers Board" vision program's inspections (RGB, tolerance, min coverage).
BLACK_COLORS = [((37, 64, 119), 9), ((24, 47, 101), 10), ((22, 38, 74), 9)]
WHITE_COLORS = [((231, 233, 219), 6), ((192, 196, 172), 7), ((234, 237, 218), 9)]
BLACK_PIECE, WHITE_PIECE = (37, 64, 119), (231, 233, 219)   # RGB

# Two camera views: where the board's OUTER border corners land (TL, TR, BR, BL), px.
POSE_A = [(205, 95), (455, 92), (600, 400), (45, 405)]
POSE_B = [(170, 110), (430, 85), (585, 380), (60, 425)]   # "bumped" camera

failures = []


def check(cond, what):
    print(("  PASS " if cond else "  FAIL ") + what, flush=True)
    if not cond:
        failures.append(what)


def search_area(margin=35):
    """The frame's search area as a user would draw it: around the board with some slack for a
    knocked camera (here the convex hull of both poses, grown by a margin), normalized."""
    pts = np.float32(POSE_A + POSE_B)
    hull = cv2.convexHull(pts)[:, 0, :]
    cx, cy = hull.mean(axis=0)
    grown = [(x + (x - cx) / np.hypot(x - cx, y - cy) * margin, y + (y - cy) / np.hypot(x - cx, y - cy) * margin) for x, y in hull]
    return [[round(min(max(float(x), 0), W) / W, 5), round(min(max(float(y), 0), H) / H, 5)] for x, y in grown]


def rc(sq):
    r = sq // 4
    return r, 2 * (sq % 4) + (1 if r % 2 == 0 else 0)


# ── Scene ────────────────────────────────────────────────────────────────────────

class Scene:
    def __init__(self, seed=7):
        self.pieces = {}            # square -> +1/+2 white, -1/-2 black
        self.pose = POSE_A
        self.hand = None            # None or (x, y) centre of the hand, px
        self.lock = threading.Lock()
        self.version = 0
        rng = np.random.default_rng(seed)
        # Wood table: warm base, grain streaks, fixed for the run.
        table = np.zeros((H, W, 3), np.float32)
        table[:] = (95, 135, 165)   # BGR
        grain = rng.normal(0, 1, (H, 1)).repeat(W, 1) * 9 + rng.normal(0, 1, (H, W)) * 3
        table += grain[..., None]
        # Clutter outside the board: a navy mug rim (a "black piece" colour, must be ignored).
        cv2.circle(table, (600, 60), 26, (119, 64, 37), -1)
        cv2.rectangle(table, (8, 20), (110, 60), (60, 60, 60), -1)
        self.table = table
        self.noise_rng = np.random.default_rng(seed + 1)

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)
            self.version += 1

    def flat(self, pieces):
        img = np.zeros((FLAT, FLAT, 3), np.uint8)
        img[:] = (30, 45, 70)                                      # dark wood border (BGR)
        for r in range(8):
            for c in range(8):
                col = (45, 55, 185) if (r + c) % 2 == 0 else (32, 30, 30)    # red / black squares (BGR)
                cv2.rectangle(img, (BORDER + c * SQ, BORDER + r * SQ), (BORDER + (c + 1) * SQ - 1, BORDER + (r + 1) * SQ - 1), col, -1)
        for sq, code in pieces.items():
            r, c = rc(sq)
            cx, cy = BORDER + c * SQ + SQ // 2, BORDER + r * SQ + SQ // 2
            rgb = WHITE_PIECE if code > 0 else BLACK_PIECE
            bgr = (rgb[2], rgb[1], rgb[0])
            cv2.circle(img, (cx, cy), int(SQ * 0.4), bgr, -1)
            # A rim a little darker, as a real disc's edge reads.
            cv2.circle(img, (cx, cy), int(SQ * 0.4), tuple(int(v * 0.8) for v in bgr), 2)
            if abs(code) == 2:   # king: a second disc's ring
                cv2.circle(img, (cx, cy), int(SQ * 0.22), tuple(int(v * 0.85) for v in bgr), 2)
        return img

    def render(self):
        with self.lock:
            pieces, pose, hand = dict(self.pieces), list(self.pose), self.hand
        src = np.float32([[0, 0], [FLAT, 0], [FLAT, FLAT], [0, FLAT]])
        hm = cv2.getPerspectiveTransform(src, np.float32(pose))
        board = cv2.warpPerspective(self.flat(pieces), hm, (W, H), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full((FLAT, FLAT), 255, np.uint8), hm, (W, H)) > 0
        img = self.table.copy()
        img[mask] = board[mask]
        if hand is not None:
            hx, hy = hand
            arm = np.array([[hx - 40, hy], [hx + 40, hy], [hx + 140, H + 40], [hx + 30, H + 40]], np.int32)
            cv2.fillPoly(img, [arm], (120, 150, 205))
            cv2.ellipse(img, (int(hx), int(hy)), (55, 42), -20, 0, 360, (125, 158, 212), -1)
        # Light falloff away from the top-left, then sensor noise.
        yy, xx = np.mgrid[0:H, 0:W]
        light = 1.04 - 0.06 * (xx / W + yy / H) / 2
        img = img * light[..., None] + self.noise_rng.normal(0, 1.5, img.shape)
        img = np.clip(img, 0, 255).astype(np.uint8)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpg.tobytes(), img

    def squares_quad(self):
        """Pixel corners of the 8x8 squares area (inside the border) for the current pose."""
        src = np.float32([[0, 0], [FLAT, 0], [FLAT, FLAT], [0, FLAT]])
        hm = cv2.getPerspectiveTransform(src, np.float32(self.pose))
        pts = np.float32([[[BORDER, BORDER]], [[FLAT - BORDER, BORDER]], [[FLAT - BORDER, FLAT - BORDER]], [[BORDER, FLAT - BORDER]]])
        return cv2.perspectiveTransform(pts, hm)[:, 0, :]

    def cell_px(self, sq):
        """Pixel centre of a square in the current pose (for putting the hand over it)."""
        src = np.float32([[0, 0], [FLAT, 0], [FLAT, FLAT], [0, FLAT]])
        hm = cv2.getPerspectiveTransform(src, np.float32(self.pose))
        r, c = rc(sq)
        p = cv2.perspectiveTransform(np.float32([[[BORDER + c * SQ + SQ / 2, BORDER + r * SQ + SQ / 2]]]), hm)
        return float(p[0, 0, 0]), float(p[0, 0, 1])


def feeder(url, scene, stop):
    """Pushes the current scene ~7 times a second, as a phone would."""
    ws = websocket.create_connection(url, timeout=5)
    cache_v, jpg = -1, None
    try:
        while not stop.is_set():
            if scene.version != cache_v:
                cache_v = scene.version
            jpg, _ = scene.render()   # re-rendered each time: fresh noise, like a real sensor
            ws.send_binary(jpg)
            time.sleep(0.14)
    finally:
        ws.close()


# ── Controller helpers ───────────────────────────────────────────────────────────

class Ctrl:
    def __init__(self, host):
        os.environ["RMS_ROBOT_URL"] = f"ws://{host}/control"
        import robot_mcp
        self.r = robot_mcp
        self.host = host

    def cmd(self, name, params=None):
        return self.r.ROBOT.send(name, params or {})

    def step(self, step_id, params):
        res = self.cmd("RunPluginStep", {"pluginId": "checkers", "id": "checkers", "stepId": step_id, "params": params})
        return res.get("outputs", res)

    def board(self):
        out = {}
        for s in range(32):
            v = self.cmd("CallPluginFunction", {"pluginId": "checkers", "id": "checkers", "name": "pieceAt", "function": "pieceAt", "args": [s]})
            code = v.get("value", v.get("result"))
            if code:
                out[s] = int(code)
        return out

    def vars(self, program):
        res = self.cmd("GetProgramVariables", {"name": program})
        out = {}
        for v in res.get("variables") or []:
            out[v["name"]] = v.get("stringValue") if v.get("isString") else v.get("value")
        return out

    def summary(self, program):
        progs = self.cmd("GetPrograms").get("programs") or []
        if isinstance(progs, str):
            progs = json.loads(progs)
        return next((p for p in progs if p.get("name") == program), {})


def occupancy_cells(pieces):
    """64-cell 0/1 lists (row-major) for detectMove, from a piece map."""
    w, b = [0] * 64, [0] * 64
    for s, code in pieces.items():
        r, c = rc(s)
        (w if code > 0 else b)[r * 8 + c] = 1
    return w, b


def candidate_moves(pieces):
    """Every simple white move and single/multi jump, by geometry (legality is the engine's call)."""
    def at(r, c):
        if 0 <= r < 8 and 0 <= c < 8 and (r + c) % 2 == 1:
            return r * 4 + c // 2
        return None
    out = []
    for s, code in pieces.items():
        if code <= 0:
            continue
        dirs = [(-1, -1), (-1, 1)] + ([(1, -1), (1, 1)] if code == 2 else [])
        r0, c0 = rc(s)
        for dr, dc in dirs:
            t = at(r0 + dr, c0 + dc)
            if t is not None and t not in pieces:
                out.append((s, t, []))

        def jumps(r, c, board, taken):
            found = False
            for dr, dc in dirs:
                mid, land = at(r + dr, c + dc), at(r + 2 * dr, c + 2 * dc)
                if mid is not None and land is not None and board.get(mid, 0) < 0 and land not in board and mid not in taken:
                    found = True
                    nb = dict(board); nb.pop(mid); nb[land] = nb.pop(at(r, c))
                    lr, lc = rc(land)
                    if not jumps(lr, lc, nb, taken + [mid]):
                        out.append((s, land, taken + [mid]))
            return found
        jumps(r0, c0, dict(pieces), [])
    return out


def apply_white(pieces, frm, to, captured):
    p = dict(pieces)
    code = p.pop(frm)
    for c in captured:
        p.pop(c, None)
    if rc(to)[0] == 0:
        code = 2
    p[to] = code
    return p


# ── Test ─────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ctrl", default="127.0.0.1:9101")
    ap.add_argument("--plugin-zip", required=True)
    ap.add_argument("--plies", type=int, default=12, help="stop after this many moves (both sides)")
    ap.add_argument("--bump-after", type=int, default=5, help="bump the camera after this many plies")
    ap.add_argument("--shots", default=None, help="folder for annotated vision frames")
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()
    random.seed(args.seed)
    c = Ctrl(args.ctrl)
    scene = Scene()
    if args.shots:
        os.makedirs(args.shots, exist_ok=True)

    print("== setup")
    req = urllib.request.Request(f"http://{args.ctrl}/plugins/install?replace=true", data=open(args.plugin_zip, "rb").read(),
                                 headers={"Content-Type": "application/zip"}, method="POST")
    urllib.request.urlopen(req, timeout=120).read()
    for _ in range(60):
        pl = next((p for p in c.cmd("GetPlugins").get("plugins", []) if p["id"] == "checkers"), None)
        if pl and pl.get("connected") and pl.get("stepCount", 0) >= 5:
            break
        time.sleep(0.5)
    check(pl is not None and pl.get("stepCount", 0) >= 5, "checkers plugin (with detectMove) running")

    cams = c.cmd("GetCameras").get("cameras") or []
    if not any(x.get("name") == "E2E mock" for x in cams):
        c.cmd("AddCamera", {"name": "E2E mock", "sourceType": "phone", "width": W, "height": H})
        cams = c.cmd("GetCameras").get("cameras") or []
    cam = next(x for x in cams if x.get("name") == "E2E mock")

    # Points, grid and grip routines the game program uses (the arm is simulated).
    for name, xyz in (("Checkers Board origin", (250, -90, 120)), ("Checkers Discard", (200, 150, 120))):
        c.cmd("TeachPoint", {"name": name})
        c.cmd("EditPoint", {"name": name, "x": xyz[0], "y": xyz[1], "z": xyz[2], "rx": 0, "ry": 0, "rz": 0})
    c.cmd("SaveGrid", {"id": "checkers-board", "name": "Checkers Board", "basePointName": "Checkers Board origin",
                       "rowOffsetX": 30, "rowOffsetY": 0, "rowOffsetZ": 0, "colOffsetX": 0, "colOffsetY": 30, "colOffsetZ": 0,
                       "rowCount": 8, "colCount": 8, "rotation": 0})

    # The vision program, as a user would set it up in the editor: frame (Edges) found on the
    # live image, its corners taken as the reference, a Skewed 8x8 zone drawn on the squares.
    stop = threading.Event()
    feed = threading.Thread(target=feeder, args=(f"ws://{args.ctrl}/camera/{cam['id']}/ingest", scene, stop), daemon=True)
    start_pieces = {s: -1 for s in range(12)} | {s: 1 for s in range(20, 32)}
    scene.set(pieces=start_pieces)
    feed.start()
    sq = scene.squares_quad()
    norm = lambda pts: [[round(float(x) / W, 5), round(float(y) / H, 5)] for x, y in pts]
    vp = {
        "id": "e2e-board", "name": "E2E Checkers Board", "description": "", "cameraId": cam["id"],
        # The frame searches a rectangle around where the board sits (with room for the camera to
        # be knocked), not the whole image: the table's grain is full of long straight edges.
        "zones": [{"id": "board", "name": "Board", "grid": {"rows": 8, "cols": 8}, "alignTo": "frame",
                   "geometry": {"shape": "Quad", "points": norm(sq)}},
                  {"id": "search", "name": "Board area", "geometry": {"shape": "Polygon", "points": search_area()}}],
        "frameInspections": [{"id": "frame", "name": "Board frame", "enabled": True, "zoneId": "search", "method": "corners",
                              "minLineLengthPct": 25, "referenceCorners": None}],
        "colorInspections": [
            {"id": "black", "name": "Black Pieces", "enabled": True, "zoneId": "board", "minCoverage": 21.2, "maxCoverage": None,
             "colors": [{"id": f"b{i}", "r": c3[0], "g": c3[1], "b": c3[2], "tolerance": t} for i, (c3, t) in enumerate(BLACK_COLORS)]},
            {"id": "white", "name": "White Pieces", "enabled": True, "zoneId": "board", "minCoverage": 7.8, "maxCoverage": None,
             "colors": [{"id": f"w{i}", "r": c3[0], "g": c3[1], "b": c3[2], "tolerance": t} for i, (c3, t) in enumerate(WHITE_COLORS)]},
        ],
        "inspectionOrder": ["frame", "black", "white"],
    }
    c.cmd("SaveVisionProgram", vp)
    jpg, _ = scene.render()
    res = c.cmd("RunVisionOnImage", {"programId": "e2e-board", "image": base64.b64encode(jpg).decode()})
    res = res.get("result", res)
    fr = (res.get("frameResults") or [{}])[0]
    check(fr.get("found") is True, f"frame (Corners) found on the mock image: {fr.get('corners')}")
    def worst(ref):
        return max(abs(a * W - b[0]) + abs(a2 * H - b[1]) for (a, a2), b in zip(fr.get("corners", [[0, 0]] * 4), ref)) if fr.get("found") else 999
    # Either the board's outer border or the edge of the playing area is a good frame.
    err_border, err_play = worst(POSE_A), worst([tuple(p) for p in sq])
    check(min(err_border, err_play) < 10,
          f"frame is the board's {'border' if err_border <= err_play else 'playing area'} (worst corner {min(err_border, err_play):.1f} px)")
    vp["frameInspections"][0]["referenceCorners"] = fr.get("corners")   # "Use current detection as reference"
    c.cmd("SaveVisionProgram", vp)
    res = c.cmd("RunVisionOnImage", {"programId": "e2e-board", "image": base64.b64encode(jpg).decode()})
    res = res.get("result", res)
    seen_w = sorted(int(x["index"]) for x in (next(r for r in res["colorResults"] if r["inspectionId"] == "white")["cells"]) if x["passed"])
    seen_b = sorted(int(x["index"]) for x in (next(r for r in res["colorResults"] if r["inspectionId"] == "black")["cells"]) if x["passed"])
    want_w, want_b = occupancy_cells(start_pieces)
    check(seen_w == [i for i, v in enumerate(want_w) if v] and seen_b == [i for i, v in enumerate(want_b) if v],
          f"aligned grid reads the starting position ({len(seen_w)} white, {len(seen_b)} black)")
    check(not res.get("errors"), f"no vision errors {res.get('errors')}")

    built = json.loads(subprocess.check_output(["node", os.path.join(HERE, "..", "examples", "dotnet", "Checkers", "program", "checkers-game.mjs"), "json", "e2e-board", "normal"], text=True))
    game = built["game"]
    game["name"] = "E2E Checkers Game"
    for v in game["variables"]:
        if v["name"] == "speed": v["value"] = 3000
        if v["name"] == "readMs": v["value"] = 250
    for r in (built["grip"], built["release"]):
        r["steps"][1]["waitMs"] = 20
        c.cmd("SaveBuiltProgram", r)
    c.cmd("SaveBuiltProgram", game)
    val = c.cmd("ValidateBuiltProgram", {"program": game}).get("problems") or []
    check(not [p for p in val if p.get("severity") == "error"], f"game program validates ({[p.get('message') for p in val if p.get('severity') == 'error']})")

    print("== play")
    # Start with one white piece missing: the program must wait for the starting position.
    scene.set(pieces={k: v for k, v in start_pieces.items() if k != 22})
    # StopProgram only pauses (Start would resume it mid-game with the old variables): reset first.
    c.cmd("ResetProgram", {"programName": game["name"]})
    c.cmd("StartProgram", {"programName": game["name"]})
    time.sleep(3)
    v = c.vars(game["name"])
    check(int(v.get("changed") or 0) > 0 and int(v.get("plies") or 0) == 0,
          f"waits for the starting position (changed={v.get('changed')} plies={v.get('plies')} seen={v.get('seenText')!r})")
    scene.set(pieces=start_pieces)

    pieces = dict(start_pieces)
    plies, bumped = 0, False
    deadline = time.time() + 60 * 15
    shot_n = 0

    def shot(tag):
        nonlocal shot_n
        if not args.shots:
            return
        # The vision program run on the image the camera is showing now, with its overlay.
        jpg_now, _ = scene.render()
        r = c.cmd("RunVisionOnImage", {"programId": "e2e-board", "image": base64.b64encode(jpg_now).decode(), "includeAnnotated": True})
        b64 = r.get("annotated") or (r.get("result") or {}).get("annotated")
        if b64:
            shot_n += 1
            open(os.path.join(args.shots, f"{shot_n:02d}-{tag}.jpg"), "wb").write(base64.b64decode(b64))

    while plies < args.plies and time.time() < deadline:
        # Robot's turn: wait for it to play and park.
        t0 = time.time()
        while time.time() - t0 < 90:
            v = c.vars(game["name"])
            s = c.summary(game["name"])
            if int(v.get("plies") or 0) > plies or s.get("status") not in ("Running", "Starting", None):
                break
            time.sleep(0.3)
        if int(v.get("plies") or 0) <= plies:
            print(f"   robot did not move: status={s.get('status')!r} step={s.get('currentStepDescription')!r} vars={ {k: v.get(k) for k in ('plies','changed','fromSq','toSq','gameOver','moveText','seenText')} }")
        if s.get("status") not in ("Running", "Starting"):
            check(int(v.get("gameOver") or 0) == 1, f"program ended: {s.get('status')} {s.get('errorDescription')}")
            break
        frm, to = int(v["fromSq"]), int(v["toSq"])
        engine = c.board()
        expect = dict(pieces)
        code = expect.pop(frm, None)
        for cap in [k for k in expect if k not in engine]:
            expect.pop(cap)
        expect[to] = engine.get(to, code)
        check(code is not None and code < 0 and expect == engine,
              f"robot ply {plies + 1}: black {frm}->{to} matches the engine's board")
        pieces = engine
        scene.set(pieces=pieces)          # the (simulated) arm has moved the piece
        plies += 1
        if int(v.get("gameOver") or 0):
            check(True, f"game over after ply {plies}: {v.get('winner')} wins")
            break
        if plies >= args.plies:
            break

        if not bumped and plies >= args.bump_after:
            scene.set(pose=POSE_B)
            bumped = True
            print("   camera bumped to a new angle")
            time.sleep(1)

        # Human's turn: pick a legal white move (the engine judges legality via detectMove).
        legal = []
        for frm_h, to_h, caps in candidate_moves(pieces):
            after = apply_white(pieces, frm_h, to_h, caps)
            w_cells, b_cells = occupancy_cells(after)
            out = c.step("detectMove", {"whiteCells": w_cells, "blackCells": b_cells})
            if out.get("status") == "move" and int(out["from"]) == frm_h and int(out["to"]) == to_h:
                legal.append((frm_h, to_h, caps))
        check(bool(legal), f"human has legal moves ({len(legal)})")
        if not legal:
            break
        legal.sort(key=lambda m: -len(m[2]))              # take a capture when there is one
        frm_h, to_h, caps = legal[0] if legal[0][2] else random.choice(legal)
        after = apply_white(pieces, frm_h, to_h, caps)

        # Hand comes in, lifts the piece, puts it down; then the hand leaves.
        hx, hy = scene.cell_px(to_h)
        lifted = {k: val for k, val in pieces.items() if k != frm_h}
        scene.set(hand=(hx + 25, hy + 30), pieces=lifted)
        time.sleep(1.2)
        v = c.vars(game["name"])
        check(int(v.get("plies") or 0) == plies, "nothing taken while the piece is in the hand")
        scene.set(pieces=after)
        time.sleep(0.9)
        v = c.vars(game["name"])
        check(int(v.get("plies") or 0) == plies, "nothing taken while the hand is still over the board")
        shot(f"ply{plies + 1}-hand")
        scene.set(hand=None)
        t0 = time.time()
        while time.time() - t0 < 30:
            v = c.vars(game["name"])
            if int(v.get("plies") or 0) > plies:
                break
            time.sleep(0.25)
        took = int(v.get("plies") or 0) > plies
        check(took and int(v.get("hFrom")) == frm_h and int(v.get("hTo")) == to_h,
              f"human ply {plies + 1}: white {frm_h}->{to_h}{' x' + str(caps) if caps else ''} read as {v.get('hFrom')}->{v.get('hTo')} in {time.time() - t0:.1f}s")
        shot(f"ply{plies + 1}-read")
        if not took:
            break
        # The robot replies at once, so the engine may already hold its next move: the scene
        # is the truth here, and the next robot ply is checked against it.
        pieces = after
        plies += 1

    s = c.summary(game["name"])
    check(not s.get("errorDescription"), f"no program error ({s.get('errorDescription')})")
    c.cmd("AbortProgram", {"programName": game["name"]})
    c.cmd("StopBuiltProgram")
    stop.set()
    print(f"\n{plies} plies played; bumped camera: {bumped}")
    print("ALL PASSED" if not failures else f"{len(failures)} FAILED:\n  " + "\n  ".join(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
