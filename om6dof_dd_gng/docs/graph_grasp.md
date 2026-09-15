# Mendekati dan menjepit benda dari graph eksperimen

Launch baru: `ddgng_experiment_grasp.launch.py`. Environment tetap YOLOX dan
DD-GNG; jalur awal tetap memakai konfigurasi **hijau** dari eksperimen V2.
Dari node hijau terdekat, planner dapat menambahkan koreksi Cartesian pendek
agar pose sebelum menjepit sejajar dengan target. Sesudah pose pregrasp ini,
planner menambahkan pendekatan Cartesian sampai pusat
jepitan berada di pusat bounding box cluster 3D yang diamati. Profil grasp
memisahkan **satu komponen target yang dipilih** dari obstacle, sesuai kebijakan
pickup: node dan edge internal target tidak menghalangi gerakan menuju target.

## Menjalankan

Hentikan launch perception/planning sebelumnya dengan Ctrl+C agar kamera dan
topic planner tidak dipakai dua instance. Controller hardware yang sudah aktif
tetap digunakan. Launch ini tidak menjalankan controller/torque.

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_dd_gng ddgng_experiment_grasp.launch.py \
  camera_calibration_file:=/home/kublab/.config/om6dof/d435_hand_eye.yaml \
  execution_enabled:=true
```

Pilih **bottle → Set planning target → Preview EoE path in RViz → Dekati dan
jepit benda**. Periksa preview penuh sampai pusat target. Launch tanpa
`execution_enabled:=true` tetap menyediakan preview dan tidak mengaktifkan tombol
gerak. Tidak ada gerakan hanya karena launch dimulai.

Urutan setelah klik: buka jari, ikuti trajectory graph, pendekatan terakhir,
tutup jari. Tidak ada pengangkatan atau retreat dengan payload pada tahap ini.

Jika berhenti di pra-jepit, lihat penyebab pada status eksekusi. Nomor track YOLO
dapat berubah saat pendeteksian terputus, walaupun benda tetap di tempatnya.
Coordinator menerima nomor pengganti hanya jika ada satu kandidat kelas sama
dalam toleransi posisi 2 cm dari snapshot awal, baik centroid mentah maupun
centroid hasil smoothing. Kandidat harus konsisten pada setidaknya tiga frame
berurutan yang membentang minimal 0.1 detik. Tidak ada perpindahan diam-diam
ke botol lain bila track asli terlihat sudah bergeser atau kandidat ambigu.

Pada batas antar tahap, coordinator menunggu paling lama
`target_reacquisition_timeout_sec=1.0` untuk gangguan deteksi singkat. Selama
menunggu, robot tidak menerima perintah tahap berikutnya dan encoder serta
kesehatan controller tetap diperiksa. Kehilangan target berkepanjangan tetap
menghentikan pickup. Validator 3D tetap wajib menemukan tepat satu komponen
target dalam 1 cm dari pusat jepitan yang dibekukan dan memeriksa collision
untuk approach dan penutupan. Kebijakan ini tidak mengubah target atau jalur
Preview, dan tidak melanjutkan otomatis pekerjaan yang sudah berstatus gagal;
buat Preview baru untuk percobaan berikutnya.

GUI kini menampilkan tahap dan penyebab berhenti meskipun Preview sudah
kedaluwarsa. Status ROS juga memuat `failure_stage` dan
`observed_target_track_id` untuk diagnosis.

## Frame dan collision

- Sumbu depan fisik gripper V2 ialah **+Z lokal `end_effector_link`**. Dokumentasi
  teleop menyebut arah fisik ini gripper-forward X. URDF robot tidak diputar.
- Pusat jepitan nominal memakai TCP yang ada, `grasp_tcp_to_pinch=[0,0,0]`.
  Ujung mesh jari masih memanjang sekitar 20.7 mm di depan TCP. Ini geometri CAD,
  bukan kalibrasi aperture fisik atau rekonstruksi pusat massa benda.
- Konfigurasi sebelum grasp berjarak 7–13 cm. Pendekatan mempertahankan orientasi
  dan memerlukan arah depan mendekati horizontal serta mengarah ke target.
  IK lokal dibatasi joint URDF, langkah Cartesian 5 mm, lompatan joint 0.15 rad,
  dan residual posisi 3 mm. Interpolasi joint diperiksa collision setiap paling
  banyak 0.025 rad; ini pemeriksaan diskret, bukan bukti collision kontinu.
- Langkah 5 mm adalah langkah awal. Jika solusi IK membutuhkan perubahan joint
  lebih dari 0.15 rad, solver membagi interval Cartesian menjadi dua dan
  menyelesaikannya kembali dari joint terakhir yang diterima. Langkah yang
  terlalu besar tidak dimasukkan ke trajectory. Pembagian berhenti pada batas
  interval 0.1 mm, 64 interval, atau waktu solver 0.15 detik; singularitas dan
  collision tetap menolak jalur. Toleransi posisi per langkah diperketat mengikuti
  panjang interval agar toleransi akhir 3 mm tidak menghilangkan langkah kecil.
- `pregrasp_refinement_enabled=true` pada profil grasp memperluas pencarian
  dengan paling banyak 32 anchor hijau terdekat yang dapat disesuaikan lewat
  translasi TCP maksimal 6 cm. Anchor tidak harus sudah tepat sejajar dengan
  pusat benda. Koreksi mempertahankan orientasi anchor, memeriksa limit joint
  dan mesh, lalu memeriksa kembali jarak 7–13 cm dan alignment minimal
  0.95 sebelum pendekatan maju. Node dan edge arsip tidak diubah; koreksi lokal
  ditambahkan sesudah traversal graph. Ini pencarian lokal terbatas, bukan IK
  global atau jaminan bahwa semua target terjangkau akan ditemukan.
- Profil grasp memakai `capsule_collision_veto=false`: keputusan collision
  mengikuti mesh seluruh robot terhadap sphere/cylinder environment DD-GNG.
  Kapsul besar tetap dihitung untuk diagnostik goal/edge, tetapi overlap kapsul
  saja tidak menolak pose atau jalur yang lolos mesh. Ini juga berlaku pada
  koneksi start, koreksi pra-jepit, dan rencana baru dari endpoint grasp.
  Mode ini wajib mengaktifkan pemeriksaan exact dan model environment penuh;
  node menolak konfigurasi yang mematikannya. Radius obstacle mesh tidak
  diperkecil, dan pengecualian target tetap hanya untuk komponen yang dipilih.
  Profil graph-only lama mempertahankan veto kapsul.
- Launch grasp menyediakan `exact_replan_budget=256` dengan batas waktu
  pencarian grasp tetap 1.5 detik. Pada rekaman kegagalan terbaru, 84 calon edge
  perlu ditolak oleh mesh sebelum jalur alternatif ditemukan; budget lama 80
  berhenti sebelum itu.
- `exclude_selected_target_from_collision=true` pada launch grasp: sphere node
  dan cylinder edge internal satu cluster target dikeluarkan dari model obstacle
  kapsul maupun mesh. Ini berlaku pada seluruh robot selama membuka gripper,
  traversal graph, pendekatan, dan penutupan. Geometri target tetap tersedia untuk
  menentukan pusat jepitan dan tampil di RViz.
- Komponen objek lain, termasuk bottle lain dengan label sama, node tak berlabel,
  dan edge lintas batas cluster tetap obstacle. Semua mesh robot diperiksa
  terhadap obstacle tersebut; limit joint dan self-collision tetap berlaku.
  Kebijakan ini tidak menghapus satu kelas objek secara keseluruhan atau
  memperluas pengecualian berdasarkan radius di sekitar target.
- Planner memakai cluster utama (jumlah node terbesar) dari kelas yang dipilih;
  kegagalan tidak memindahkan target diam-diam ke fragmen label kecil. Obstacle
  dan cache edge ditata ulang untuk target tersebut. Jika satu endpoint graph gagal disambung, planner
  mencoba endpoint lain. Kegagalan tidak menghasilkan potongan approach yang
  dapat dieksekusi.
- Mode parameter `false` mempertahankan kebijakan ketat sebelumnya: target
  diperiksa selama approach dan hanya kontak kedua jari saat closure diizinkan.

Planner menyediakan service `/om6dof_topo_gng_v2/validate_grasp_execution` untuk
memeriksa segmen yang telah dipreview terhadap snapshot environment terbaru.
Coordinator wajib memanggilnya sebelum membuka/traversal, sebelum insertion,
dan sebelum menutup. Titik awal diukur ulang; pergeseran target/obstacle,
snapshot kedaluwarsa, service hilang, dan collision menolak tahap berikutnya.
Posisi akhir pusat jepitan terukur harus berada dalam 5 mm dari target sebelum
closure. Pemeriksaan ulang dilakukan antartahap, bukan sensor collision kontinu.

`ReachabilityPlan` sekarang menyertakan kontrak `grasp_approach_valid`, pemisah
waypoint sebelum insertion, pusat target, residual, posisi gripper yang dimodelkan,
dan `selected_target_excluded_from_collision`. Coordinator memeriksa bahwa
kebijakan planner cocok dengan profil launch dan mengikatnya ke preview beku.
`pregrasp_waypoint_count` mencakup start terukur, traversal graph, dan koreksi
lokal bila diperlukan. Titik terakhir prefix itu merupakan pose pra-jepit
aktual yang dipakai untuk validasi arah dan pemisahan trajectory. Daftar
`reachability_node_ids` tetap hanya memuat node arsip yang dilalui.
Jangan campur planner lama dengan coordinator baru; jalankan ulang launch planning
setelah build. Profil lama `ddgng_experiment_reachability.launch.py` tetap
graph-only secara default.

## Membaca hasil

`pregrasp_pose_unavailable_for_target` berarti tidak ada anchor tersampel yang
lolos kriteria pra-jepit, termasuk calon koreksi lokal jika fitur itu aktif.
Pesan ini tidak membuktikan botol berada di luar jangkauan fisik robot. Jika
anchor tersedia tetapi koreksinya gagal, GUI menampilkan alasan
`pregrasp_bridge_*`, seperti IK, collision, jarak, atau batas waktu.

Penolakan `target_intersection_exact_blocked_or_disconnected` berarti pemeriksaan
mesh menolak konfigurasi pada **calon jalur**, atau edge yang tersisa tidak
menghubungkan start dengan kandidat pregrasp. Ini tidak menyatakan bahwa posisi
robot saat ini bertabrakan, atau membuktikan semua gerakan menuju benda mustahil.
Varian `clearance` menyebut penolakan kapsul; varian `mixed` berarti kedua jenis
penolakan ditemukan selama pencarian.

Detail pemeriksaan tersedia pada topic berikut:

```bash
ros2 topic echo /om6dof_topo_gng_v2/reachability_plan/collision_diagnostics --once
```

JSON memisahkan penolakan kapsul saja, mesh saja, dan keduanya. `last_rejection`
berisi tahap, ID node graph, pasangan collision, serta titik kontak dalam frame
`world`. `dd_gng_environment` adalah gabungan geometri environment, termasuk
target; nama ini sendiri belum membedakan bottle dengan obstacle lain. Nilai
`penetration_m` adalah `contact.depth` mentah dari backend dan dapat bertanda
negatif; jangan menafsirkannya sebagai pengukuran penetrasi benda fisik.
GUI menampilkan pasangan dan koordinat kontak dari diagnostik planner terbaru
yang cocok dengan alasan kegagalan. Data yang diterima lebih dari tiga detik lalu
disembunyikan. Tampilan diagnostik ini tidak mengubah syarat tombol eksekusi.
Field `capsule_collision_veto` menyatakan kebijakan aktif.
`capsule_mesh_clear_goals` dan `capsule_mesh_clear_edges` menghitung kandidat
yang overlap kapsulnya tidak dikonfirmasi sebagai collision mesh.
Field `pregrasp_refinement_enabled`, `pregrasp_bridge_used`, dan
`pregrasp_bridge_waypoints` menunjukkan apakah koreksi lokal aktif dan dipakai
pada rencana yang berhasil.
`grasp_candidate_attempts` menyimpan sampai 32 percobaan endpoint terakhir,
termasuk ID node, pusat target, alasan, jumlah pembagian interval, dan perubahan
joint terbesar yang sempat dicoba solver. Nilai `bridge_peak_attempted_joint_step_rad`
pada diagnostik menyatakan percobaan sebelum pembagian, bukan besar langkah
yang dikirim ke motor. GUI tidak menampilkan penolakan kapsul kandidat graph
sebelumnya sebagai penjelasan untuk kegagalan IK atau joint-jump berikutnya.

Replay scene 2026-09-15 **sebelum kebijakan pengecualian target** mereproduksi
79 edge yang ditolak **mesh saja**. Contoh
kontak pertama berasal dari kedua jari dan `link7` terhadap sphere node bottle
bagian atas. Target cluster sudah berada di tengah, Z sekitar 0.139 m. Pemeriksaan
ulang offline dengan clearance kapsul nol menghasilkan penolakan yang sama;
clearance runtime tidak diubah. Model sphere node memakai radius 12 mm, sehingga
permukaan collision dapat meluas melewati pusat node yang terlihat. Rekaman ini
membuktikan penolakan model, bukan tabrakan fisik terukur. Hasil tersedia di
[validation/collision_20260915/installed_planner_replay.json](validation/collision_20260915/installed_planner_replay.json).
Pada snapshot yang sama, filter arah dan standoff hanya menerima satu witness
hijau untuk cluster bottle utama. Jalur terpendek graph sebelum memasukkan
obstacle memakai 27 node dan naik hingga Z 0.275 m. Keterbatasan konektivitas pose
tersampel ini menjelaskan mengapa pusat benda yang dekat tidak selalu mendapat
jalur langsung. Atribusi kontak dan audit kandidat disimpan di
[frozen_scene_evidence.json](validation/collision_20260915/frozen_scene_evidence.json).
Build perbaikan diagnostik lulus; 30 tes Python (diagnostik GUI dan launch) lulus.
Uji ROS sintetis juga memastikan graph, insertion, dan closure yang valid tetap
lolos, serta obstacle tambahan tetap ditolak:
[synthetic_grasp_regression.json](validation/collision_20260915/synthetic_grasp_regression.json).

`grasp_contact_detected` berarti action stall didukung posisi jari aktual yang
stabil sebelum posisi tutup penuh. Ini indikasi kontak, belum bukti benda aman
diangkat. `closed_unconfirmed` berarti jari selesai menutup tetapi keberadaan
benda belum terkonfirmasi. Software tidak mengubah kondisi itu menjadi klaim
"memegang benda".

Perintah buka harus dikonfirmasi dengan aperture aktual. Konfigurasi
`om6dof_bringup/config/controllers.yaml` sekarang memakai goal tolerance 1.5 mm
dan `allow_stalling: true` pada start controller berikutnya. Coordinator juga
menangani hasil stall dari controller lama, dengan verifikasi posisi aktual.
`gripper_max_effort: 0` tidak berarti gaya jepit fisik terkalibrasi.

## Verifikasi pengembangan

Tes mencakup IK dan interpolasi collision, pemisahan target/obstacle, Jacobian
MoveIt terhadap finite difference pada model V2, kontrak preview, urutan action,
pembatalan, revalidasi scene, dan feedback gripper. Pengujian numerik bukan hasil
eksperimen keberhasilan menggenggam benda fisik.

Uji 2026-09-15: **94 tes Python dan 27 tes C++ lulus**. Smoke ROS terisolasi
memakai 89 witness hijau dan tiga node target sintetis: preview penuh 17 waypoint
(dua waypoint graph) mencapai pusat dengan residual numerik sekitar 0.064 mm.
Validasi graph, insertion, dan closure lulus; penambahan obstacle tak berlabel di
depan jari ditolak sebagai `dd_gng_grasp_obstacles__gripper_left_link`.
Hasil tersimpan di [validation/grasp_20260915/result.json](validation/grasp_20260915/result.json).

Reproduksi smoke setelah source workspace:

```bash
python3 /home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/test/grasp_execution_smoke.py
```

Script hanya menjalankan planner dan sensor sintetis pada ROS domain 218,
localhost, tanpa action client motor atau akses kamera. Log dan hasil baru
ditulis ke direktori sementara yang dicetak script.

Uji kebijakan target terpilih (2026-09-15): **125 tes Python dan 7 tes C++ scene
lulus**. Uji ROS memeriksa kedua kebijakan: titik yang menjadi bagian target
terpilih diterima di jari/pergelangan saat pengecualian aktif, sedangkan titik
yang sama sebagai unknown atau komponen bottle lain tetap ditolak. Mode ketat
tetap menolak kontak target tersebut. Hasil tersimpan di
[target_exclusion_20260915/enabled_result.json](validation/target_exclusion_20260915/enabled_result.json)
dan [strict_result.json](validation/target_exclusion_20260915/strict_result.json).
Replay scene asli dengan build akhir mengecualikan cluster utama (anchor 18520):
penolakan edge mesh menjadi nol, sedangkan 25 edge masih ditolak oleh kapsul.
Contoh terakhir berasal dari node **18677, class_id=-1** di
`[0.294584, 0.029719, 0.076773]` m, dekat dasar botol. Jarak ke kapsul payload
kamera 0.091619 m masih di bawah ambang 0.095 m. Node ini tidak termasuk komponen
target berlabel, sehingga tetap obstacle. Hasil ini belum merupakan keberhasilan
trajectory pada scene asli atau keberhasilan grasp fisik; lihat
[frozen_scene_replay.json](validation/target_exclusion_20260915/frozen_scene_replay.json).
Mode ketat pada script pengujian:

```bash
python3 /home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/test/grasp_execution_smoke.py --strict-target-collision
```

Uji koreksi pra-jepit (2026-09-15): **135 tes Python dan 12 tes C++ IK/model
lulus**. Smoke pembanding memakai tiga witness hijau yang sama dan pusat target
`[0.295, 0.040, 0.150]` m. Tanpa refinement, tidak ada anchor yang memenuhi
kriteria pra-jepit dan pesan `pregrasp_pose_unavailable_for_target` muncul.
Dengan refinement, planner menghasilkan 30 waypoint, termasuk 11 titik koreksi
tambahan; alignment pra-jepit 0.999976 dan residual FK akhir 0.088 mm. Validasi
service untuk graph+koreksi, insertion+closure, dan closure di endpoint sintetis
semuanya lulus. Lihat [paired_smoke.json](validation/pregrasp_refinement_20260915/paired_smoke.json).

Replay snapshot environment lengkap sebelumnya juga berubah dari penolakan
kapsul menjadi preview grasp valid, dengan pusat target
`[0.305650, 0.002614, 0.139258]` m dan residual FK 0.064 mm. Planner tetap
menolak 25 edge kapsul yang sama, lalu memakai anchor hijau alternatif dan
koreksi lokal. Snapshot tersebut berasal dari rekaman sebelumnya, bukan scene
live pada screenshot terbaru. Ini hasil perencanaan numerik, bukan hasil
pengukuran gerakan hardware. Hasil pembanding tersimpan di
[historical_disabled.json](validation/pregrasp_refinement_20260915/historical_disabled.json)
dan [historical_enabled.json](validation/pregrasp_refinement_20260915/historical_enabled.json).
Validasi service pada replay yang sama juga lulus untuk graph+koreksi,
insertion+closure, dan closure saja. FK independen mengonfirmasi arah pendekatan
berselisih 0.336 derajat dari pusat target. Bukti dan fixture tersedia di
[historical_validation/validator_evidence.json](validation/pregrasp_refinement_20260915/historical_validation/validator_evidence.json).
Pada build replay tersebut masih ada perbedaan pemeriksaan: query graph baru dari
endpoint grasp sintetis ini ditolak oleh batas kapsul current-state, walaupun
validasi mesh insertion/closure lulus. Perbaikan ini tidak mengubah gate kapsul
tersebut. Profil mesh pada perbaikan berikutnya menyamakan gate ini dengan
validasi mesh tahap eksekusi; pengangkatan, retreat, dan grasp fisik belum diuji.

Smoke eksekusi diulang dengan build yang sama: 16 pemeriksaan pada kebijakan
target dikecualikan dan mode ketat tetap lulus. Obstacle unknown dan komponen
botol lain tetap ditolak. Reproduksi smoke koreksi pra-jepit:

```bash
python3 /home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/test/pregrasp_refinement_smoke.py
```

Script memakai domain ROS 220 dengan sensor sintetis, tanpa kamera, controller,
atau action motor. Perubahan planner telah dibuild, tetapi proses planning yang
sudah berjalan perlu diluncurkan ulang agar memakai binary baru.

Uji kegagalan `pregrasp_bridge_joint_jump` yang direkam langsung pada 2026-09-15
tersimpan di [adaptive_grasp_20260915](validation/adaptive_grasp_20260915/summary.json).
Environment berada 10.6 ms sebelum plan yang gagal; nilai joint arm pada stream
identik dengan start yang disimpan plan. Binary lama mengulang kegagalan yang
sama. Pembagian langkah saja masih ditolak self-collision link3–link5 pada
anchor 1010, sehingga jalur itu tetap ditolak. Ada 14 kandidat lain yang mesh-nya
bebas collision namun sebelumnya terhapus oleh veto kapsul.

Build akhir memilih anchor 2361 melalui graph, setelah menolak 84 edge yang
benar-benar collision pada model mesh. Preview lengkap memiliki 30 waypoint,
residual FK 0.055 mm, dan semua tahap service (graph+koreksi, insertion+closure,
closure saja) lolos. Pencarian baru dari endpoint sintetis juga lolos. Radius
mesh, data environment, joint limits, dan pengecualian komponen target sama
dengan capture; kapsul tidak lagi menjadi veto pada profil grasp. Rotasi yaw
tambahan dan pengurangan radius kapsul yang dicoba untuk diagnosis tidak
dipakai pada implementasi akhir.

**142 tes Python, 16 tes C++, dan 16 pemeriksaan obstacle ROS lulus.** Uji
obstacle memastikan node unknown dan botol lain tetap ditolak, serta mode
kontak target ketat tetap menolak geometri target di jari/pergelangan.
Semua hasil ini berasal dari model/replay; tidak ada motor yang digerakkan.
Regresi scene kegagalan dapat dijalankan dengan:

```bash
python3 /home/kublab/ros2_ws/src/om6dof/om6dof_dd_gng/test/captured_grasp_smoke.py
```

Regresi ini memakai fixture yang tersimpan dan domain ROS 219, lalu memeriksa
bahwa preview penuh, batas langkah joint, dan ketiga tahap validasi lolos.
