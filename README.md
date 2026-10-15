# Mcity-data-recording-toolkit
A collection of ROS2 drivers and recorder scripts to make data capture easier.

Our data captures usually include RTK positioning, lidar, and six cameras.

Hardware:

- OXTS AV200 RTK

- RoboSense lidars (either four M1 directional solid-state lidars, or one rotating Ruby+ lidar)

- Six Lucid cameras (either TRI023S-CC, or TRT023S-CC)

# Drivers
Main ones are "autoexposure" and "static". Stick with autoexposure unless you have a good reason; the new target luminance and gamma settings mostly resolve the discoloring issues with stock autoexposure settings. If you need trigger mode, you can set that flag to true in the config area.

There is also an optional, additional rate-limiter driver which re-publishes the camera topics at a reduced framerate.

## Synchronized trigger mode (PTP + action commands)
`lucid-drivers-sync-trigger.sh` launches all six cameras with `trigger_mode=true`, which now means **PTP-synchronized capture via scheduled GigE Vision action commands** (every camera exposes at the same instant in the OXTS RTK GPS timebase). One camera (cam1) is the action master.

- Launch: `./lucid-drivers-sync-trigger.sh`
- Wait for PTP to lock (master logs `PTP locked … 'Slave'`).
- Fire one synchronized shot: `./trigger-all.sh`
- Continuous: `./trigger-all-rate.sh 5` (≈5 Hz shell loop), or set `action_trigger_rate` in the launcher for a hardware-precise rate.

Camera topics are unchanged (`/arenacamN/images`), so the existing recorders work as-is. Timestamps are **RTK GPS time** (not UTC/Unix) — apply the GPS↔UTC offset downstream.

This **supersedes** `lucid-drivers-autoexposure-trigger-mode.sh`: with the updated driver, `trigger_mode=true` performs action-command sync (not software trigger), so that older script would configure the cameras but never fire.

### Static exposure under trigger mode (expose dark, lift in post)
`lucid-drivers-sync-trigger-static-exposure.sh` is the same synchronized capture with **fixed exposure and fixed gain** instead of autoexposure. Every frame in the run is exposed identically, so brightness is comparable frame-to-frame and camera-to-camera, nothing on the camera reacts to the scene, and the whole run is brightened afterwards with one consistent adjustment per camera. It starts deliberately **dark** (60 µs @ 0 dB, gamma 0.5, measured sun-safe on the vehicle 2026-09-18; use 100–120 µs on overcast days): a dark frame lifts cleanly in post, a blown highlight never comes back.

Why "static" used to come out bright: two things on the camera kept adapting even with a fixed `exposure_time`. `GainAuto` stayed on (auto-gain cranked to hit the brightness target, giving bright noisy frames), and if `target_brightness` was also passed the driver switched `ExposureAuto` back on and discarded the fixed exposure. Both are fixed in the driver (see below), and every camera now logs the exposure state it is actually running.

Workflow:
1. Launch: `./lucid-drivers-sync-trigger-static-exposure.sh` (or `… 800` for 800 µs, `… 800 6.0` for 800 µs @ 6 dB). Wait for PTP lock / frames.
2. Confirm each camera's log line `exposure state (STATIC): ExposureAuto=Off … GainAuto=Off …`. Anything else means the run is not static.
3. Tune in the brightest light you expect: `python3 check-exposure.py` prints per-camera mean / clipped% / black% once a second with a verdict. Aim for clipped% ≈ 0 (well under 0.5%) and a mean that is dark but not black (~25–100 on 0–255). Relaunch with a lower number if clipping, higher if black. Raise exposure before gain; gain is noise. Ceilings: exposure must fit the trigger period (< ~10000 µs at 30 Hz) and avoid motion blur (~13 mm of travel per 1000 µs at 30 mph).
4. Record as usual (`record-rtklidarcam-split-cams.sh`); the images in the bag will look dark. `check-exposure.py --bag <run_dir>` summarizes a recording after the fact (a split-recorder run folder is fine, its `camA`/`camB` bags are read together; a single bag dir also works).
5. Lift in post: `python3 ../postprocess/expose-in-post.py <run_dir> -o <outdir>` (run folder or bag dir, as above) debayers, undoes the camera gamma, lifts by an EV and writes 8-bit JPEG/PNG per frame named by PTP capture stamp, plus `exposure_ev.csv` recording the EV applied to every frame. `--every 30 --max-frames 20` gives a quick look; `--camera-gamma 1.0` if you launched with gamma=1.0.

   How the EV is chosen (`--ev-mode`), which is what handles lighting that changes during a long run:
   - `smooth` (default): per-frame EV from each frame's highlight level, then a centered 2 s rolling median + mean (`--ev-window`). Follows clouds, dusk and tunnels with no lag, because post can see the frames ahead, and no flicker. Use this for 10–30 min runs.
   - `frame`: raw per-frame EV, no smoothing. Tracks instantly but can flicker on glints.
   - `run`: one EV per camera for the whole run (the original behavior).
   - `--ev 3`: fixed +3 stops everywhere.
   - `--ev-global`: one shared EV across all cameras at each instant, set by the brightest camera. Recommended when several cameras feed one reconstruction, so the same surface has the same brightness in every view.

   The lift is a pure gain in linear light, so a pipeline that models per-image exposure (e.g. NuRec) can take `ev_applied` from the CSV as exposure metadata, or divide it back out. It cannot recover pixels that were crushed to black in the bag; capture exposure still has to be chosen so the darkest conditions in the run stay off the floor (60 µs sun / 100–120 µs overcast, or 16-bit Bayer for runs spanning both).

Driver behavior this relies on (rebuild `arena_camera_node` after pulling):
- `exposure_time` **wins** over `target_brightness`. If both are passed, the static exposure is kept and `target_brightness` is ignored with a warning (previously the reverse, silently).
- A static `exposure_time` also locks gain: `GainAuto=Off`, and `Gain` = the passed `gain` or **0 dB** if none was given.
- `exposure_time` / `gain` are clamped into the camera's legal range with a warning instead of crashing the node.
- After `StartStream` each camera reads back and logs `ExposureAuto / ExposureTime / GainAuto / Gain / Gamma`, and logs an **error** if a static launch is not actually Off/Off.
- Note the autoexposure launcher (`target_brightness` without `exposure_time`) runs `ExposureAuto=Once`: it converges on the startup scene and then locks, so a run that starts in shade and drives into sun blows out. Static exposure is the answer for runs where that matters.

**Detailed write-ups (root cause → fix → why it works):**
- `SYNC_RECORDING.md` (this repo) — the recording/transport/operational side: QoS, transmission staggering, memory sizing, gotchas, verification.
- `arena_camera_node/SYNC_TRIGGERING.md` (driver repo) — the camera/capture side: how PTP + scheduled action commands produce synchronized exposure.

# Recorders
Main one is `record-rtklidarcam.sh`. If you are using the rate-limiter and only want to record the rate-limited topics, you can use `record-rtklidarcam-rl.sh`
