# Network (RTSP / HTTP) cameras

Cameras were USB-only (opened by device index through DirectShow or V4L2). A
camera can now also be a **network stream** opened through OpenCV's FFmpeg
backend: `rtsp://` (and `rtsps://`) IP cameras, and `http(s)://` MJPEG or
snapshot endpoints. Everything downstream (live feed, vision programs,
calibration, snapshots) works the same for both kinds.

## Model — `camera_config.json` and `GetCameras`

Additive fields on `CameraConfig` / `CameraState`:

| Field | Type | Meaning |
|---|---|---|
| `sourceType` | `"usb"` \| `"network"` | Absent = `"usb"` (existing configs keep working). |
| `url` | string | Stream URL for `network` cameras, without credentials (`rtsp://192.168.0.50:554/stream1`, `http://cam/mjpeg`). |
| `username`, `password` | string, optional | Credentials, stored separately and injected into the URL at open time (`rtsp://user:pass@host…`). Stored in `camera_config.json` like every other setting and returned by `GetCameras` (the app edits them; it masks the password in the UI). Never logged: log lines print the URL with the password replaced by `***`. |
| `transport` | `"tcp"` \| `"udp"` | RTSP only. Default `tcp` (reliable on Wi-Fi, no smearing). |
| `streamWidth`, `streamHeight` | int | On `CameraState` only: the size the stream actually delivers (0 until connected). For network cameras `width`/`height`/`supportedResolutions` are informational; the stream's own size is used. |
| `latencyMs` | int | On `CameraState` only: measured decode latency estimate (time between consecutive delivered frames' presentation gap vs wall clock) — best-effort, 0 when unknown. |

`deviceIndex` is ignored for network cameras. `targetFps` still throttles how
often a frame is published to consumers; frames are always **decoded as they
arrive** so no latency builds up in the FFmpeg buffer.

## Commands (additive)

| Command | Params | Response |
|---|---|---|
| `AddCamera` / `SetCameraConfig` | existing params plus `sourceType`, `url`, `username`, `password`, `transport` | unchanged |
| `TestCameraSource` | `url`, `username?`, `password?`, `transport?`, optional `timeoutMs` (default 8000) | `ok`, `width`, `height`, `openMs`, `firstFrameMs`, `error?` — opens the stream once on a worker thread, grabs one frame, closes it. Errors: `invalidUrl`, `openFailed`, `noFrame`, `timeout`. |

`GetCameraResolutions` returns `[]` for a network camera.

## Controller behaviour

- Open with `VideoCapture(url, VideoCaptureAPIs.FFMPEG)`; set
  `CAP_PROP_BUFFERSIZE = 1`, `CAP_PROP_OPEN_TIMEOUT_MSEC = 8000`,
  `CAP_PROP_READ_TIMEOUT_MSEC = 5000` (ignored by builds that lack them).
  RTSP transport is selected through the `OPENCV_FFMPEG_CAPTURE_OPTIONS`
  environment variable (`rtsp_transport;tcp`) set for the process before the
  first network open; document that this is process-wide.
- The capture thread structure (single owner of the handle, generation guard,
  release on exit) is unchanged. On a read failure the stream is reopened with
  the existing 3 s backoff; on `openFailed` the log says so once per streak with
  the masked URL.
- The open runs on the capture thread as today; `Stop()` may have to wait for
  the open timeout (8 s) — the generation guard already covers that.
- At startup log `[Camera] Video backends: FFmpeg=yes|no` from
  `Cv2.GetBuildInformation()`, and refuse to open a network camera with a clear
  log line when FFmpeg is missing (the Linux runtime package must ship it; verify).
- Calibration and vision use the stream's actual frame size; a calibration
  stores `imageWidth/imageHeight` already, so a stream that changes size
  invalidates it exactly like a USB resolution change.

## App

- Add/edit camera: a **Source** segmented control (USB camera / Network camera).
  USB shows the device index and resolution fields as today. Network shows URL,
  username, password (masked, with a reveal toggle), transport (TCP/UDP,
  RTSP only), and a **Test connection** button that calls `TestCameraSource`
  and shows size and timings or the error with a hint (`openFailed` → check the
  address, port and credentials; `noFrame`/`timeout` → the camera answered but
  sent nothing, try UDP/TCP or a different stream path).
- Camera cards and the detail page show a small "RTSP" / "HTTP" tag and the host
  (never the password). Resolution probing UI is hidden for network cameras.
- The calibration wizard and vision editor need no changes beyond the state
  fields; they use the same feed endpoints.

## Implementation notes

- Code: `RobotControl/Camera/NetworkCameraSource.cs` (URL building, masking, FFmpeg
  detection and open, `TestCameraSource`), the network branch of
  `CameraDevice.CaptureLoop`, tests in `RobotControl.Tests/NetworkCameraTests.cs`
  (including an in-process MJPEG HTTP server that exercises the FFmpeg path end to end).
- `OPENCV_FFMPEG_CAPTURE_OPTIONS` is written before **every** network open (managed
  environment plus the native one: `setenv` on Linux, `_putenv_s` on Windows, since
  .NET keeps its own copy of the environment on Unix). It is process-wide and the last
  writer wins. On Windows the FFmpeg plugin DLL may snapshot the environment when it
  is first loaded, so changing a camera's transport may only take effect after a
  controller restart there (unverified; TCP is also OpenCV's own default).
- `SetCameraConfig`: an absent `sourceType`/`url`/`username`/`password`/`transport`
  keeps the camera's current value, so an app that predates network cameras cannot
  turn one into a USB camera by saving it.
- `GetCameraResolutions` takes an optional `id`; a network camera's id answers `[]`.
- `TestCameraSource`'s `timeoutMs` is the whole budget: it is used as the native open
  and read timeouts, and the command answers `timeout` after `timeoutMs + 1 s`.
