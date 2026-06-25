# Synchronized multi-camera recording — toolkit changes

This document explains the toolkit-side work for recording the six PTP-synchronized
LUCID cameras: the launch/trigger scripts, the QoS and transmission tuning needed
to record without dropping frames, and the operational lessons learned.

It pairs with the driver-side write-up in
`arena_camera_node/SYNC_TRIGGERING.md` (the camera/capture side). This document is
the **recording/transport/operational** side.

> **Validated on the vehicle:** clean 10 Hz lossless recording (361/361 per
> camera), and **30 Hz at ~99.9%** (4 dropped of ~3925 over 2 min, mostly
> start/stop boundary frames), six cameras synchronized to **6.71 µs**, on a 10 GbE
> switch (1 GbE per camera), recording to an ext4 SATA SSD (476 MB/s), ~4.6 GB RAM.

---

## New / changed files

| File | Purpose |
|---|---|
| `drivers/lucid-drivers-sync-trigger.sh` | Launch all six cameras in `trigger_mode` (cam1 = action master), with shared action keys, continuous rate, recording-friendly QoS, and transmission staggering. |
| `drivers/trigger-all.sh` | Fire one synchronized shot (`ros2 service call /trigger_all …`). |
| `drivers/trigger-all-rate.sh [hz]` | Continuous shell-loop trigger variant. |
| `drivers/check-cam-sync.py` | Read a bag and report the cross-camera `header.stamp` spread (the sync metric). |

The `recorders/` scripts are **unchanged**; the camera topics (`/arenacamN/images`)
are the same, so the existing recorders work as-is.

---

## 1. Root cause — why recording dropped frames

Three separate problems surfaced, in order:

### 1a. Node-name collisions
The driver's node name is hardcoded (`arena_camera_node`), so six instances share
the same `/arena_camera_node/trigger_image` service and collide. The launcher
remaps each with `-r __node:=arenacamN`. The `/trigger_all` service is absolute and
created only on the master, so it never collides.

### 1b. The big one: best-effort QoS + synchronized bursts → silent drops
This is why the bag dropped frames even though the cameras published fine.

- Each image is large (~2.3 MB at 1920×1200 bayer8). Six cameras at 10 Hz =
  ~138 MB/s, and because capture is **synchronized**, all six 2.3 MB frames arrive
  at the recorder **at the same instant** — a ~14 MB burst.
- Each image is fragmented into thousands of UDP datagrams. The burst overflows the
  DDS/kernel receive buffers, and the camera publisher's default `SensorDataQoS` is
  **best-effort**, so a single dropped fragment silently discards the whole image.
- `ros2 topic hz` still read 10 Hz (its lightweight subscriber kept up), which
  **masked** the loss — but rosbag2, subscribing to six big topics at once, could
  not, and best-effort never retransmits.

Critically, this was **not** a bandwidth or disk problem: the SSD sustains
476 MB/s and the link is 10 GbE. The loss was purely in the transport layer.

### 1c. 30 Hz: the synchronized burst overwhelms the receiver
At 30 Hz the 6-wide bursts come every 33 ms instead of every 100 ms — 3× more
often with 3× less time to drain. Even with reliable QoS, the instantaneous burst
overflows the receive buffers faster than they clear. Confirmed by the recorder
process sitting at ~10 % CPU during loss: the frames were being dropped **before**
reaching rosbag2, i.e. down in DDS/kernel transport, not in serialization or disk.

---

## 2. What we implemented

### 2a. Reliable QoS (the fix for 1b)
The launcher sets the camera publishers to **reliable**:
`-p qos_reliability:=reliable -p qos_history:=keep_last -p qos_history_depth:=30`.
Reliable QoS retransmits lost fragments instead of dropping them; rosbag2
auto-matches and records reliably. A reliable publisher is compatible with every
subscriber (live viewers still work).

### 2b. Transmission staggering (the fix for 1c)
The launcher gives each camera a different `gev_scftd` (frame-transmission delay):
cam1=0, cam2=Δ, … cam6=5Δ (`scftd_step`, default 4 ms). The six frames then arrive
at the host spread across the window instead of all at once. This shapes
**transmission only** — capture stays synchronized (driver detail in
`SYNC_TRIGGERING.md` §4), so the 6.71 µs sync is unchanged.

### 2c. One action master, continuous rate
`action_master:=true` on cam1 only; it owns `/trigger_all` and (with
`action_trigger_rate=10.0`, the launcher default) self-fires for continuous
recording. The shared action keys/mask are passed identically to all six.

---

## 3. Why it works

- **Reliable retransmits, disk drains.** Reliable QoS guarantees delivery of every
  fragment; because the SSD (476 MB/s) easily keeps up with the data rate, the
  recorder drains continuously and nothing is permanently lost.
- **Staggering flattens the burst.** Spreading transmission across the trigger
  period keeps the instantaneous arrival rate within what the receive buffers
  absorb, without changing the synchronized capture instant.
- **Capture stays locked.** All sync is measured from `header.stamp` (the camera
  PTP capture time), which none of the transport tuning touches — hence 6.71 µs
  before and after staggering.

---

## 4. Quick start

```bash
# one-time: enlarge the kernel UDP buffer for large DDS messages (persist it)
sudo sysctl -w net.core.rmem_max=2147483647
echo 'net.core.rmem_max=2147483647' | sudo tee /etc/sysctl.d/10-ros2.conf

cd drivers
pkill -f arena_camera_node ; ros2 daemon stop      # always start clean
./lucid-drivers-sync-trigger.sh                    # 6 cams, cam1 = master, self-fires at 10 Hz
#   wait for the cam1 tab to log: PTP locked … 'Slave'

# record (cap the cache to bound RAM — see §6):
cd ../recorders
ros2 bag record -s mcap --max-cache-size 4000000000 /arenacam{1,2,3,4,5,6}/images -o <out>

# verify sync afterward:
python3 ../drivers/check-cam-sync.py "<out>"        # expect max spread well under 1 ms
```

Single synchronized shot instead of a continuous rate: set `action_trigger_rate=0.0`
in the launcher and run `./trigger-all.sh`.

---

## 5. Operational gotchas (learned the hard way)

- **`exposure_time` must be a float.** The driver declares it as a double, so
  `exposure_time=4000` is rejected by rclcpp and the node fails to start. Use
  `4000.0`. The launcher does this.
- **`action_trigger_rate=0.0` means no auto-fire.** With rate 0 the master never
  fires on its own, so no frames flow — it looks like a hang. The launcher defaults
  to 10.0; set 0.0 only for manual one-shot via `./trigger-all.sh`.
- **`ros2 topic hz` / `topic bw` are unreliable meters during recording.** Once the
  publisher is reliable, these CLI tools become slow reliable subscribers that back
  up under load and **under-report** (we saw `hz` read 2 Hz and `bw` read 6 MB/s
  while the bag recorded the full rate). Trust `ros2 bag info`. If you want a live
  rate, force best-effort: `ros2 topic hz /arenacam1/images --qos-reliability best_effort`.
- **Don't run extra subscribers during a real capture.** Every extra reliable
  reader competes for the synchronized bursts and can *cause* the few drops you're
  trying to measure.
- **Start clean.** Stale nodes from a previous launch hold the cameras; always
  `pkill -f arena_camera_node` and `ros2 daemon stop` before relaunching.
- **PTP needs ~30 s–2 min to lock** after a fresh launch; the master won't fire
  until its `PtpStatus` is `Slave`.

---

## 6. Memory / cache sizing

`ros2 bag record` buffers messages in RAM (`--max-cache-size`) before flushing to
disk. The stock recorders set **55 GB**, which on a 30 GB-RAM machine is an OOM
hazard: if the disk ever stalls, the cache grows at the full data rate until it
exceeds physical memory.

- **Cap it at ~4 GB** (`--max-cache-size 4000000000`) — ~10 s of slack at 30 Hz
  full-res, only 20 % of available RAM. The cache stays near-empty in normal
  operation (the SSD keeps up), so steady-state RAM was ~**4.6 GB** total.
- **Memory is a live health gauge while recording:** flat = disk keeping up; a
  steady climb = the disk is falling behind and the cache is filling (lower the
  rate/resolution or check the drive).
- Publisher history (`qos_history_depth=30`) costs ~0.4 GB across all six; raise to
  60 only if you still see drops. **Never `keep_all`** (unbounded).

---

## 7. Verification

```bash
ros2 bag info "<out>"                          # per-camera counts ≈ rate × duration, all six equal
python3 drivers/check-cam-sync.py "<out>"      # cross-camera header.stamp spread (sync metric)
```

Interpreting `check-cam-sync.py`: max spread **< ~1 ms** is well synchronized
(we measure ~6.71 µs). A few-frame difference in per-camera counts is normal —
those are start/stop boundary frames (subscriptions connect/disconnect at slightly
different instants), not mid-stream loss, and no amount of buffering recovers them.

---

## 8. If you still need to push higher rates / zero loss

- **Tune the DDS receive buffers.** Check `echo $RMW_IMPLEMENTATION` (blank = Fast
  DDS, likely already shared-memory on-host; CycloneDDS = UDP). For UDP, raise
  `net.core.rmem_max` + the DDS socket buffer; for Fast DDS SHM, enlarge the SHM
  segment / port queue via an XML profile.
- **Shared-memory transport** (publisher and recorder are co-located) is the best
  long-term answer — it avoids UDP fragmentation entirely.
- **Reduce data** (lower resolution / ROI) if the rate × resolution simply exceeds
  what the pipeline sustains.
- The remaining levers (history depth, DDS buffers) cost memory; the rosbag2 cache
  does **not** help here (it only absorbs disk stalls, which we don't have).
