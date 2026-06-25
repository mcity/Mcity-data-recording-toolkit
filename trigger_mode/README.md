# trigger_mode — manual PTP-synchronized camera triggering (toolkit side)

Launch + trigger scripts for firing all six LUCID cameras as one
**PTP-synchronized** capture, sharing the OXTS RTK GPS timebase. Pairs with the
renamed driver package `arena_camera_node_trigger` in
`arena_camera_ros2/trigger_mode/`.

> Nothing here was run or tested on the authoring machine. Verify on the vehicle.
> The original toolkit `drivers/` and `recorders/` are untouched.

## Files

| File | Purpose |
|---|---|
| `lucid-drivers-action-trigger-mode.sh` | Launch all six cameras in `action_trigger_mode`. cam1 is the action master. |
| `trigger-all.sh` | Fire **one** synchronized shot (`ros2 service call /trigger_all`). |
| `trigger-all-rate.sh [hz]` | Continuous shell-loop variant (approximate spacing; each shot still synchronized). |

## How it works

1. The launcher starts `ros2 run arena_camera_node_trigger start` six times, each
   with `-r __node:=arenacamN` (the stock node name is hardcoded, so without this
   the per-node `/<node>/trigger_image` services would collide).
2. All six get `action_trigger_mode:=true` and the **same** action keys/mask.
   Each camera is set to `TriggerSource=Action0` and waits for a scheduled action
   command.
3. **Exactly one** camera (cam1) gets `action_master:=true`. It owns the absolute
   `/trigger_all` service and broadcasts the GigE Vision action command. The
   broadcast reaches all cameras (including the master itself).
4. `./trigger-all.sh` calls `/trigger_all`; the master latches PTP (RTK) time,
   schedules an execute time (next whole second + lead), and fires. All six expose
   simultaneously and publish one frame each to `/arenacamN/images`.

The master refuses to fire until its `PtpStatus` settles on `Slave`, so allow a
few tens of seconds after launch for PTP to lock.

## Quick start (ON THE VEHICLE)

```bash
# build the renamed driver package first (see driver README), then:
cd Mcity-data-recording-toolkit/trigger_mode

./lucid-drivers-action-trigger-mode.sh   # 6 tabs; cam1 = master
# ...wait for PTP lock (master logs: PTP locked ... 'Slave') ...
./trigger-all.sh                          # one synchronized shot

# optional continuous:
./trigger-all-rate.sh 5                   # ~5 Hz shell loop
#   or, hardware-precise: edit the launcher and set action_trigger_rate:=5.0
```

## Config (top of `lucid-drivers-action-trigger-mode.sh`)

- `pixelformat`, `gamma` — as in the stock launchers.
- `exposure_time` — **fixed microseconds**; `4000` is a placeholder, tune it.
- `action_device_key` / `action_group_key` / `action_group_mask` — shared, `1/1/1`.
- `action_lead_time` — seconds of scheduling margin (`0.05`).
- `ptp_domain` — must match RTK/switch (cameras commonly assume `0`).
- `action_trigger_rate` — `0.0` = one-shot; `>0` = in-driver continuous on master.

Serial → topic/name mapping is copied from the stock trigger-mode launcher:
`254300057→arenacam1, 058→2, 053→3, 056→4, 055→5, 054→6`.

## Recording (NOT modified)

The `recorders/` scripts are intentionally left untouched. The camera topics are
unchanged (`/arenacamN/images`), so the existing recorders work as-is:

- `recorders/record-rtklidarcam.sh` (and the per-cam variants) already record
  `/arenacamN/images` alongside `/ins/imu`, `/ins/nav_sat_fix`, `/tf`, etc.
- To record more cameras, add the additional `/arenacamN/images` topics to the
  recorder's topic list (each stock per-cam recorder lists a single camera topic).
- Frame timestamps are **RTK GPS time** (from PTP), not UTC/Unix — account for the
  GPS↔UTC leap-second offset in downstream fusion.

## Verification checklist

See `arena_camera_ros2/trigger_mode/README.md` (build, `/trigger_all` presence,
PTP `Slave` + grandmaster identity, one-shot → one frame each with near-identical
stamps, `ros2 topic hz` under the rate option) and
`arena_camera_ros2/trigger_mode/ASSUMPTIONS-AND-VERIFY.md` for the `[verify]` list
(serials, exposure default, SDK node names).
