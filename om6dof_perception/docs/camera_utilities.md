# Camera utilities in om6dof_perception

ROS 2 package for RealSense camera utilities and BOXER RGB-D perception.
The former vision and BOXER packages are now integrated here. Existing topic names,
model/runtime directories, calibration checks, and launch arguments are unchanged.

## BOXER perception

```bash
source /home/kublab/ros2_ws/install/setup.bash
# Manual 2D prompts -> oriented 3D boxes
ros2 launch om6dof_perception boxer_only.launch.py labels:=bottle,keyboard
# Automatic YOLOX -> Boxer3D ONNX (alternative pipeline)
ros2 launch om6dof_perception yolox_boxer3d_onnx.launch.py
```

Run only one camera pipeline at a time. These perception launches do not start
robot controllers or command motion. Models and their Python runtimes still live
outside this package in `~/ros2_ws/src/.boxer_runtime/` and
`~/ros2_ws/src/.boxer3d_runtime/`; no model download is performed by the build.
See [BOXER usage, calibration, dependencies, and provenance](boxer.md).

Camera utilities can also be run through ROS 2 after building this package:

```bash
ros2 run om6dof_perception event_camera_simulator.py --help
ros2 run om6dof_perception realsense_rgb_viewer.py --help
```

The direct Python commands below remain supported from the package source root
(`~/ros2_ws/src/om6dof/om6dof_perception`).

## Pseudo-event camera (RealSense / webcam / video)

`event_camera_simulator.py` reads the RealSense RGB sensor by default through
`pyrealsense2`. Webcam and prerecorded input use OpenCV `VideoCapture`.
It converts grayscale
frames to `log(gray / 255 + epsilon)` and generates positive/negative events
when changes cross the contrast threshold. Original and event views appear
side by side; blue means positive/ON (increasing intensity), red means
negative/OFF (decreasing intensity), and black means no event.
The footer shows FPS, events in the current frame, a rolling one-second event
rate, threshold, mode, retained buffer count, and separate positive/negative
counts with colored `+ BLUE` / `- RED` legends. This mapping follows
[E2VID's event preview](https://github.com/uzh-rpg/rpg_e2vid/blob/master/utils/inference_utils.py).

```bash
python3 event_camera_simulator.py --mode reference
python3 event_camera_simulator.py --source realsense --fps 30 --threshold 0.2
python3 event_camera_simulator.py --source webcam --camera 0 --mode reference
python3 event_camera_simulator.py --video /path/to/video.mp4 --mode reference
```

`--video` must name an existing video file; omit it to use live RealSense RGB.
`--serial` selects a specific RealSense. The existing `requirements.txt`
contains all dependencies, including `pyrealsense2`. Stop other viewers that
are using the camera before starting this program.

`basic` compares consecutive frames and produces at most one event per pixel
per frame. `reference` accumulates subthreshold changes and generates
`floor(abs(delta) / C)` events per pixel, then advances that pixel's reference
by `polarity * count * C`.

Controls: **+** or **=** increases the threshold; **-** decreases it;
**R** clears the buffer without resetting the intensity reference;
**S** exports retained events; **Q**, **Esc**, or closing either window exits.
CSV files are saved to `event_recordings/`, with columns
`timestamp,x,y,polarity`. Timestamps are `time.perf_counter()` seconds with an
arbitrary monotonic origin; every event in one frame shares its observation
timestamp. They are not Unix/UTC timestamps or measured crossing times.

Defaults are configurable at the top of the script or through `--help`:
RealSense source, webcam index 0, requested 640x480 at 30 FPS, threshold 0.20,
epsilon 1/255, basic
mode, 200,000 retained events, and maximum event age of five seconds.
The camera may negotiate a different resolution. Threshold keys change C by
0.02, with a minimum of 0.01. The event rate includes events discarded from the
bounded buffer. CSV export pauses capture while writing.

This is a frame-based approximation. Temporal information inside an exposure
is unavailable, and the program does not reproduce a real event camera's
microsecond temporal resolution. Low thresholds can greatly increase event
counts, memory allocations, and processing time. Auto-exposure, flicker,
compression, and sensor noise can create pseudo-events. Prerecorded video is
paced using its nominal FPS; timestamps and rates reflect processing time,
not the video's original capture timestamps.

For ROS 2, publish generated batches from `generate_events()` with an explicit
timestamp mapping to the ROS clock. For continuous recording, use an
asynchronous writer with a bounded queue and an explicit dropped-event policy;
compressed NumPy/HDF5 streams are more efficient than CSV.

Validation:

```bash
python3 -m pytest test_event_camera_simulator.py -q
```

## RealSense RGB frame subtraction

Program Python untuk membaca stream warna RGB dari kamera RealSense dan
menampilkan **frame terbaru dikurangi frame sebelumnya** di dua window OpenCV
berdampingan: hasil RGB di kiri dan versi grayscale dari hasil yang sama di kanan.
Program dapat dijalankan langsung tanpa node ROS 2.

Perhitungan menggunakan `cv2.subtract(frame_baru, frame_sebelumnya)` per kanal
RGB. Hasil negatif dijadikan 0 agar dapat ditampilkan sebagai gambar `uint8`.
Piksel yang bertambah terang akan terlihat; piksel yang tetap atau menjadi lebih
gelap akan hitam. Frame pertama ditampilkan hitam karena belum ada pembanding.

## Menjalankan

Hubungkan kamera RealSense, lalu jalankan dari terminal desktop:

```bash
cd /home/kublab/ros2_ws/src/om6dof/om6dof_perception
python3 realsense_rgb_viewer.py
```

Default: **640 x 480, 30 FPS**. Tekan **q**, **Esc**, tutup salah satu window, atau
**Ctrl+C** untuk keluar. Stream kamera dan kedua window akan dilepas saat program
selesai.

Untuk mengatur resolusi/FPS atau memilih kamera berdasarkan serial:

```bash
python3 realsense_rgb_viewer.py --width 1280 --height 720 --fps 30
python3 realsense_rgb_viewer.py --serial SERIAL_KAMERA
python3 realsense_rgb_viewer.py --help
```

Gunakan kombinasi resolusi/FPS yang didukung kamera. Hentikan aplikasi lain yang
memakai kamera jika stream tidak dapat dibuka.

## Dependensi

Memerlukan Python 3, `pyrealsense2`, `numpy`, dan OpenCV dengan dukungan GUI.
Dependensi tersebut sudah tersedia pada Python sistem workspace ini.
Untuk lingkungan lain, instal dalam virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

Pada perangkat ARM, jika wheel `pyrealsense2` tidak tersedia, gunakan Python
bindings dari instalasi librealsense yang sesuai perangkat.

Window memerlukan sesi desktop atau SSH dengan X11 forwarding. Paket
`opencv-python-headless` tidak dapat digunakan untuk menampilkan window.

## Data RGB

`rgb_image` berisi array NumPy `uint8` dengan bentuk `(height, width, 3)` dan
urutan kanal **R, G, B**. Nilai piksel dapat dibaca melalui `rgb_image[y, x]`.
`previous_rgb` menyimpan salinan frame sebelumnya. `difference_rgb` berisi hasil
pengurangan, lalu dikonversi ke **B, G, R** sebelum `cv2.imshow()` agar warna
tampil benar.
`grayscale_image` berisi versi satu kanal dari hasil pengurangan, dengan bentuk
`(height, width)`, dan ditampilkan di window kedua.

Referensi API:
[contoh resmi RealSense/OpenCV](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py)
dan [OpenCV HighGUI](https://docs.opencv.org/4.x/d7/dfc/group__highgui.html).
