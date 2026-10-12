# Live dynamics and Jacobian viewers

Visualisasi dynamics OM6DOF dengan **feedback motor aktual**. Tampilan mengikuti
`om6dof_description/view_com`: model transparan, Mass Properties aktif, label
massa, grid, frame `world`, dan sudut kamera yang sama. Ditambahkan panah
gravitasi pada CoM tiap link serta label feedback setiap motor.

## Jalankan

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select om6dof_description
source install/setup.bash
ros2 launch om6dof_description view_dynamic.launch.py
```

Stack hardware harus sudah mempublikasikan `/robot_description`, TF, serta
`/dynamic_joint_states`. Viewer memakai model hardware yang sedang berjalan,
termasuk geometri V2 dan payload-nya. Viewer tidak membuka serial, mengubah
torque enable, atau mengirim perintah motor. Tidak ada joint slider atau
publisher joint states tambahan. `joints:=external` tetap diterima; mode
GUI/statis telah diganti dengan sumber motor aktual.

```bash
# Perbesar panah dan sembunyikan label massa.
ros2 launch om6dof_description view_dynamic.launch.py force_scale:=0.1 labels:=false

# Vektor posisi origin link -> CoM pada pose robot aktual.
ros2 launch om6dof_description view_dynamic.launch.py vector_mode:=com_offset

# Publisher visualisasi tanpa jendela RViz.
ros2 launch om6dof_description view_dynamic.launch.py rviz:=false
```

## Sumber data

| Informasi | Sumber | Unit/arti |
| --- | --- | --- |
| Model, massa dan CoM | `/robot_description` dari hardware | kg dan m; parameter inertial URDF |
| Pose link | TF dari robot state publisher hardware | dari `/joint_states` aktual |
| Posisi dan kecepatan joint | `/dynamic_joint_states`: `position`, `velocity` | rad, rad/s untuk arm; m, m/s untuk gripper |
| Arus motor | GPIO `Present Current` | raw counts untuk arm XM430; mA jika override driver menyatakan mA |
| Tegangan | GPIO `Present Input Voltage` | V |
| Torque enable dan error | GPIO `Torque Enable`, `Hardware Error Status` | ON/OFF dan kode error |
| Komunikasi | `/dynamixel_hardware_interface/dxl_state` | kode `comm_state` driver |

Nama joint, GPIO, dan ID dipetakan dari `ros2_control` dalam model hardware
melalui `transmission_to_joint_matrix`, bukan urutan pesan feedback. Data yang
tidak tersedia ditulis `unavailable`. Nilai nol ditampilkan sebagai nol hanya
jika diterima dari motor.

Label cyan menampilkan `q`, `dq`, `I`, `V`, torque enable, dan error. Label menjadi
merah jika feedback lebih lama dari `feedback_timeout` (default 0.5 s) atau ada
error komunikasi. Angka kedaluwarsa diganti `NO FRESH MOTOR DATA` dan panah lama
kedaluwarsa. Jika hardware belum tersedia, viewer menunggu tanpa pose simulasi.

## Arti panah

Default `vector_mode:=gravity`: panah oranye berawal di CoM tiap link dan
menunjuk searah gravitasi, tetap dalam arah `world` saat joint bergerak. Gaya
adalah `F_i = m_i * g` dalam newton. Panjang tampilan adalah
`force_scale * |F_i|` dalam meter; default **0.05 m/N**. Massa 0.1 kg menghasilkan
gaya 0.981 N dan panah sepanjang 0.04905 m.

Dalam medan gravitasi seragam, pusat gravitasi (CoG) sama dengan pusat massa
(CoM). Massa dan CoM berasal dari URDF, bukan pembacaan langsung motor. Panah
adalah hasil model pada pose aktual; angka feedback adalah pembacaan motor.
`Present Current` tidak dianggap sebagai torsi Nm atau gaya Cartesian tanpa
kalibrasi yang sesuai. Package belum menghitung torsi dynamics penuh, Coriolis,
atau inertia.

## Jacobian pada end-effector

```bash
# Fokus kamera pada TCP, tampilkan Jacobian dan ellipsoid.
ros2 launch om6dof_description view_jacobian.launch.py

# Viewer dynamics biasa juga memiliki display TCP Jacobian.
ros2 launch om6dof_description view_dynamic.launch.py

# Contoh mengubah skala panah atau menonaktifkan Jacobian.
ros2 launch om6dof_description view_jacobian.launch.py jacobian_scale:=0.6
ros2 launch om6dof_description view_dynamic.launch.py jacobian:=false
```

Jacobian geometrik dihitung pada origin `end_effector_link`, termasuk offset
TCP 115 mm dari flange yang didefinisikan model V2. Geometri berasal dari
model hardware aktif dan posisi joint dari `/dynamic_joint_states` aktual.
Tidak ada posisi joint yang diisi nol jika feedback hilang. Untuk reference
frame di luar root URDF, transform root dibaca dari TF pada timestamp feedback.

- Titik putih adalah TCP.
- Panah merah/hijau/biru/magenta/kuning/cyan masing-masing menunjukkan kolom
  joint1 sampai joint6 pada bagian translasi `Jv`. Arah adalah arah kecepatan
  TCP jika joint tersebut bergerak positif; panjang sebanding dengan magnitudonya.
- Garis ellipsoid biru muda adalah manipulability translasi dari SVD `Jv`.
  Label `Jv_sigma` menampilkan singular values translasi. Sumbu yang kolaps
  menunjukkan hilangnya satu arah kemampuan gerak translasi pada pose itu.
- Panah putih menunjukkan `v = Jv * dq` dari kecepatan joint yang dibaca motor.
  Jika semua joint diam, panah kecepatan putih tidak digambar.

Panah kolom Jacobian menggambarkan respons terhadap **unit joint speed**,
sehingga tetap ada ketika robot diam. Panjang panah translasi menggunakan
`jacobian_scale`; ellipsoid menggunakan `ellipsoid_scale`. Besarnya visualisasi
bukan batas workspace atau batas kecepatan motor. Interpretasi ellipsoid
mengasumsikan norma kecepatan joint sama dengan satu. Lihat
[Modern Robotics: manipulability](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/).

Untuk melihat bagian rotasi, aktifkan **TCP Jacobian → Namespaces → angular**
di RViz. Panah rotasi memakai skala terpisah `angular_jacobian_scale`; satuannya
berbeda dari translasi. Joint6 dapat memiliki kolom translasi nol karena
sumbunya melewati TCP, sementara kolom rotasinya tetap ada. Panah nol dihapus
agar tidak berubah menjadi panah dengan arah atau panjang buatan.

Topic tambahan:

- `/om6dof_dynamic/jacobian_markers`: semua marker Jacobian.
- `/om6dof_dynamic/jacobian`: `Float64MultiArray`, matriks row-major `6 x 6`.
  Urutan baris `[vx, vy, vz, wx, wy, wz]` dalam reference frame; label dimensi
  kolom menyebut nama joint secara berurutan.
- `/om6dof_dynamic/jacobian_info`: JSON berisi frame, TCP, timestamp feedback,
  urutan joint, singular values/rank translasi, dan kolom translasi nol.
- `/om6dof_dynamic/ee_twist`: `TwistStamped` berisi `J * dq`, dengan timestamp
  feedback motor. Jika satu kecepatan joint tidak tersedia, twist tidak
  dipublikasikan dan panah putih dihapus.

Marker Jacobian dihapus ketika feedback kedaluwarsa atau posisi chain tidak
lengkap. Kamera pada launch khusus mengikuti `end_effector_link`; pilih
`tip_link` lain untuk menghitung Jacobian link lain dan sesuaikan Target Frame
di RViz jika ingin kamera mengikutinya.

Mode `com_offset` menampilkan offset CoM dari origin link tanpa faktor massa
atau `force_scale`. Link dengan offset nol tidak memiliki panah posisi.

## Pengaturan

Topic (`visualization_msgs/msg/MarkerArray`):

- `/om6dof_dynamic/cog_vectors`
- `/om6dof_dynamic/mass_labels`
- `/om6dof_dynamic/motor_feedback`

Display MarkerArray memakai QoS **Volatile**. RobotModel memakai
`/robot_description` dengan QoS **Transient Local**, sehingga viewer yang
dibuka setelah hardware tetap menerima model.

Gravitasi, warna, dimensi panah, frekuensi, timeout, dan ukuran label ada di
`config/cog_vectors.yaml`; file alternatif: `config:=/path/file.yaml`.
Parameter dibaca saat startup. Argumen `vector_mode`, `force_scale`,
`description_topic`, dan `dynamic_state_topic` menggantikan nilai terkait di
file. Jika mengganti `reference_frame`, sesuaikan `gravity` dan Fixed Frame
RViz. Jika mengganti `description_topic`, sesuaikan Description Topic RViz juga.

## Verifikasi

```bash
colcon test --packages-select om6dof_description --event-handlers console_direct+
colcon test-result --test-result-base build/om6dof_description --verbose
```

Integration test menggunakan domain ROS terpisah dengan fixture URDF, TF,
dan feedback. Tidak membuka bus atau menggerakkan motor. Pemeriksaan meliputi
perubahan pose, arah gravitasi, unit arus, pemetaan nama, data hilang/data
kedaluwarsa, serta launch RViz tanpa publisher hardware/joint tambahan.
