"""Camera capture: the frame record, the threaded reader, and the sampling layer.

⭐ Created BY ROADMAP §8.2 item 6, deliberately not before it — an empty package waiting for code is scaffolding, and this repo's rule is that structure arrives with the content that needs it.

The division of labour, and why:

- **`frame.Frame`** is the per-sample record, field-aligned with the team's LaRobot `Frame` (ROADMAP §10.6) so the rebuild lifts capture code unchanged.
- **`grabber.FrameGrabber`** reads ONE camera in a background thread, newest frame wins. It moved here verbatim from `apps/camera_view.py`, where it is hardware-confirmed — the app imports it back, so there is exactly one copy (the §52.1 rule: never a tested copy beside the running copy).
- **`capture.CaptureSet`** samples N grabbers into named `Frame`s at the control loop's own moments, with per-camera sequences and honest timestamps.

⛔ What deliberately does NOT live here: device identification and opening. That machinery (`mac_cameras`, `identify_indices`, `resolve_camera`, `open_camera`) is hardware-confirmed in `apps/camera_view.py` and is measurement-heavy, interactive, and macOS-specific; `apps/capture_probe.py` composes it with this package. An agent can never run a camera (FINDINGS §61.3), so anything here must be provable with fakes — and all of it is (`tests/test_capture.py`).
"""
