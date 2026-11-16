# TD_Livox_Mid40

Use a **Livox Mid-40 LiDAR in TouchDesigner like a Kinect**: a live, dense, colour-mapped
point cloud as a Script CHOP or a float point-cloud texture (Script TOP), ready for instancing
or GLSL.

No Livox SDK required: the **Livox SDK1 protocol is implemented in plain Python + numpy**
(discovery, handshake, heartbeat, point packets, CRC). Includes a tiny DHCP server, so the
sensor works on a direct Ethernet cable.

> Prototype. Tested on Windows 11 with a Mid-40 (firmware 03.08) and TouchDesigner 2023/2025.
> Not affiliated with Livox or DJI.

## How it works

```
Mid-40 ──UDP──> [LiDAR server (python.exe)] ──shared memory──> [TouchDesigner: Script CHOP / TOP]
                 1000 packets/s, DHCP,           30 Hz            copy only
                 reconnect, gridding, colours  <── parameters + heartbeat
```

Reception and point processing run in a **separate Python process**, not inside TouchDesigner.
Doing the same work in TD's Python made the grid update drop to ~8 Hz because of GIL contention;
as a separate process it stays at 30 Hz even at 512×512 with 10 s integration.

- TouchDesigner **starts the server automatically** and the server **exits by itself** ~10 s
  after TD closes.
- Requirements: a regular Python 3.11+ with numpy (`pip install numpy`) besides TD's own Python.
  It is found automatically; otherwise set `Python` on the Connection page.
- Server log: `%TEMP%\livox_mid40_server.log`
- You can also run it by hand (TD attaches to it):
  `python -m livox_mid40.server --dhcp --client-timeout 0`

### Making a non-repetitive scanner look like a depth camera

The Mid-40 sweeps one laser in a rosette ("flower") pattern, so a short slice of raw points
looks like lines (about 20 % of the field of view in 0.1 s). **Grid mode** turns it into a
Kinect-style organised point cloud:

1. Points are projected onto a `Resolution × Resolution` pinhole "depth camera" grid covering
   the 38.4° circular field of view.
2. Each pixel keeps the **newest** point in its direction and remembers when it was hit;
   pixels older than `Integration` seconds expire. Only new points are processed, so cost does
   not depend on the integration time.
3. Small gaps are filled from neighbours (`Fill` passes); filled pixels are placed on their own
   pixel ray, so the grid stays regular.

## Repository layout

```
livox_mid40/
  protocol.py   # frames, CRC, commands, point packet parsing (Livox SDK1 protocol)
  lidar.py      # Mid40 class: discovery, handshake, heartbeat, point reception
  dhcp.py       # single-lease DHCP server for a directly cabled LiDAR
  engine.py     # coordinate conversion, incremental grid, hole fill, colour map
  server.py     # LiDAR server process (connection + processing -> shared memory)
  shm.py        # shared-memory layout (server <-> TouchDesigner)
  td_bridge.py  # TD side: launch/watch the server, read/write shared memory
  td_ops.py     # TD side: Script CHOP / Script TOP logic
td/
  livox_callbacks.py      # load this into the Callbacks DAT of a Script CHOP / TOP
  pointcloud_vertex.glsl  # GLSL MAT vertex shader (point rendering)
  pointcloud_pixel.glsl   # GLSL MAT pixel shader
  TD_Livox_Mid40.toe      # prototype TouchDesigner project
tools/
  livox_monitor.py  # command-line connection test, point statistics, PLY export
  fake_mid40.py     # Mid-40 emulator on localhost for testing without hardware
scripts/
  setup_network.ps1 # give the wired adapter a static IP (administrator)
```

## 1. Network setup (once)

The Mid-40 ships in **dynamic IP (DHCP) mode**. On a direct cable (or a plain switch) there is no
DHCP server, so the sensor never gets an address and never broadcasts (it does not even answer
ping). This project gives the PC a static IP and hands the LiDAR `192.168.1.100` with the
bundled DHCP server.

In an **administrator** PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_network.ps1
# undo: ... -File scripts\setup_network.ps1 -Revert
```

This sets the `Ethernet` adapter to `192.168.1.50/24` and allows inbound UDP 55000 / 56000-56002.

| Connection | Notes |
|---|---|
| Direct cable | Use `--dhcp` (TD: `Serve DHCP` on). Tested |
| Unmanaged switch | Same as above. Tested |
| Router with DHCP | The LiDAR may get an address on another subnet: turn `Serve DHCP` off and put the PC on that subnet |

The lease lasts until the LiDAR is power-cycled, so DHCP is needed after every power-on.

## 2. Command-line test

```powershell
pip install -r requirements.txt
python tools\livox_monitor.py --dhcp --discover              # find the LiDAR
python tools\livox_monitor.py --dhcp --seconds 10            # connect and print point statistics
python tools\livox_monitor.py --dhcp --seconds 5 --save scan.ply
```

Expect about 100,000 points/s (1000 packets/s).

- Close Livox Viewer: the sensor stops broadcasting while another host is connected.
- Do not run `livox_monitor.py` while TouchDesigner is connected (they use the same ports).
- The Mid-40 cannot measure closer than 1 m.

Without hardware:

```powershell
python tools\fake_mid40.py                    # terminal 1
python tools\livox_monitor.py --ip 127.0.0.1  # terminal 2
```

## 3. TouchDesigner

### 3-1. Get the points (Script CHOP)

1. Save your project **inside this repository's `td/` folder** so the package is found
   automatically (otherwise set `Repo Folder` on the Connection page).
2. Create a **Script CHOP**. TD also creates `script1_callbacks` (Text DAT).
3. On `script1_callbacks`, set **File** to `livox_callbacks.py` and turn **Sync to File** on
   (pasting the file's contents also works).
4. Press **Setup Parameters** on the Script CHOP: the **Livox** and **Connection** pages appear.
5. After a few seconds the CHOP outputs one sample per point.

CHOP channels: `tx ty tz` (position, m), `r` (reflectivity 0-1), `d` (distance, m),
`cr cg cb` (colour 0-1).

| Livox page | |
|---|---|
| Active | Connect / disconnect |
| Mode | `grid`: Kinect-style organised cloud / `raw`: raw points (rosette) |
| Integration (sec) | How long points are kept (grid up to 10 s, raw up to 2 s). Longer = denser but moving objects leave trails |
| Grid Resolution | Grid size (up to 512) |
| Grid Hole Fill | Gap-filling passes (0-4) |
| Grid FOV (deg) | Field of view covered by the grid (Mid-40: 38.4°) |
| Coordinates | `td`: X right, Y up, LiDAR looking down -Z / `livox`: X forward, Y left, Z up |
| Color | `depth` (turbo colour map, near = blue, far = red) / `reflectivity` / `white` |
| Color Near / Far | Distance range for the depth colours (m) |

| Connection page | |
|---|---|
| Repo Folder | Path of this repository, if not detected automatically |
| LiDAR IP | Empty = auto-discover |
| Serve DHCP | Hand the LiDAR an IP (direct cable / switch) |
| PC IP | The PC's wired IP (`192.168.1.50`) |
| Python | `python.exe` for the server (empty = auto, needs numpy) |
| Restart Server | Restart the LiDAR server |

Suggested settings — the Mid-40 delivers only 100,000 points/s, so higher resolutions need
longer integration:

| Use | Mode | Resolution | Integration | Fill | Max points |
|---|---|---|---|---|---|
| Motion (people) | grid | 128 | 0.2-0.3 | 2 | ~13k |
| Balanced | grid | 256 | 0.5-1 | 1 | ~51k |
| Detailed (static) | grid | 384 | 2-5 | 1 | ~116k |
| Maximum (static) | grid | 512 | 5-10 | 1 | ~206k |

Around 384 a pixel is about the sensor's angular accuracy (0.1°); higher resolutions mostly add
empty pixels. Detail and update speed trade off: each pixel is refreshed roughly every
`pixels / 100,000` seconds (512: ~2 s).

Check the status in the Textport (`grid_hz` around 30 is normal):

```python
import livox_mid40.td_bridge as b; b.status()
```

### 3-2. Draw with instancing (CHOP)

1. In a **Geometry COMP**, replace the torus with a small **Box SOP** (or Rectangle SOP for
   200k+ points).
2. Instance page: Instancing **On**, Instance OP = the Script CHOP, Translate X/Y/Z = `tx`/`ty`/`tz`.
3. Colour: Instance 2 page, Color R/G/B = `cr`/`cg`/`cb`, with a **Constant MAT**.
4. Point spacing is about `distance × 0.70 / Resolution` m (256: ~1.4 cm at 5 m), so set the
   box size accordingly.
5. Add a Camera COMP and Render TOP. The LiDAR sits at the origin looking down -Z.

### 3-3. Point-cloud texture (Script TOP)

Load the same `livox_callbacks.py` into a **Script TOP** for a Kinect-TOP-style output:
RGBA 32-bit float, **RGB = XYZ in metres**, A = 1 for a point / 0 for an empty pixel
(or reflectivity / distance, see `Alpha`). In grid mode each pixel is a viewing direction, so
`Alpha = distance` gives a depth image. The pixel format is set to 32-bit float by Setup Parameters.

### 3-4. Draw with GLSL (lightest, experimental)

Use the Script TOP directly on the GPU; colouring, empty-pixel culling and camera-facing quads
happen in the shader. The CPU only copies one texture per frame (~2 ms at 512×512).

1. Script TOP with Mode = grid, Alpha = **valid**.
2. Geometry COMP containing a **Rectangle SOP** (1×1, centred). Instance page: Instancing On,
   Instance OP = the Script TOP, Translate X/Y/Z = `r`/`g`/`b`.
3. **GLSL MAT** with two Text DATs (File = `td/pointcloud_vertex.glsl` and
   `td/pointcloud_pixel.glsl`, Sync to File on) as Vertex / Pixel Shader.
   - Samplers: `sPoints` = the Script TOP
   - Vectors: `uSize` = 0.01 (point size, m), `uRange` = 1, 10 (colour range, m),
     `uMode` = 0 (0 depth / 1 reflectivity / 2 white)
4. Assign the GLSL MAT to the Geometry COMP.

### Notes

- Several Livox operators share one LiDAR connection. Processing settings come from the first
  operator (the owner); if it stops cooking for 2 s, another operator takes over.
- Operators cook every frame (`onGetCookLevel` returns `CookLevel.ALWAYS`).
- After editing `livox_mid40/*.py`, press **Setup Parameters**: the server is restarted and the
  package reloaded without restarting TD (parameters reset to defaults).
- The camera is not controlled by this project; use the Camera COMP parameters (e.g. Look At).

## Python API

```python
from livox_mid40 import Mid40
from livox_mid40.dhcp import MiniDHCPServer

with MiniDHCPServer("192.168.1.50", "192.168.1.100"), Mid40(buffer_points=200_000) as lidar:
    lidar.start()
    pts = lidar.get_points(count=50_000)   # last 0.5 s, (N, 4) float32: x, y, z [m], reflectivity
```

Livox coordinates: +X forward, +Y left, +Z up.

## Protocol notes (Livox SDK1)

| Item | Value |
|---|---|
| Discovery broadcast | LiDAR -> PC, UDP 55000 |
| Commands | PC -> LiDAR, UDP 65000 |
| Data / command ACK | LiDAR -> PC, ports given in the handshake (default 56001 / 56000) |
| Heartbeat | every 0.5 s (the LiDAR disconnects when it stops) |
| Point packet | 18-byte header + 100 points (cartesian: int32 x, y, z [mm] + uint8 reflectivity) |
| CRC | header CRC16 (MCRF4XX, seed 0x4C49) / frame CRC32 (seed 0x564F580A) |

## License

MIT — see [LICENSE](LICENSE). Livox and Mid-40 are trademarks of their respective owners.
