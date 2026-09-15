# Rancangan foveasi berbasis arah, jarak fokus, dan permukaan terlihat

**Digantikan oleh [BIO_INSPIRED_FOVEATION.md](BIO_INSPIRED_FOVEATION.md).**
Aturan nearest-surface dan mask geodesik di bawah adalah rancangan rekayasa
terdahulu, bukan aturan foveasi biologis. Gunakan dokumen pengganti untuk arah
implementasi; berkas ini disimpan untuk menjelaskan perubahan desain.

Status: spesifikasi algoritma untuk implementasi berikutnya. Belum diterapkan ke
runtime dan belum dievaluasi; hasil paper yang ada memakai foveasi lama.
Ini model rekayasa yang terinspirasi pengaturan pandangan dan fokus, bukan klaim
model fisiologi mata manusia.

## Masalah kode saat ini

- `native_fgng/src/main.cpp`: eksentrisitas piksel radial dikalikan bobot
  selisih depth. Fokus otomatis adalah median patch 11 x 11; tidak ada seleksi
  permukaan depan, kontrol jarak fokus manual, maupun uji oklusi graf historis.
  Ketika patch kosong, fokus terakhir ditahan tanpa batas waktu.
- `native_fgng_world/include/world_fgng.hpp`: attention berupa kernel isotropik
  terhadap pusat 3D. Permukaan yang berdekatan di balik objek juga bisa mendapat
  bobot tinggi.

## Kontrak perilaku

Pengguna mengatur arah pandang, jarak fokus, rentang ketajaman, dan lebar area
pencarian secara independen. Kandidat fokus berasal dari depth yang terlihat
pada frame sekarang. Wilayah detail merupakan bagian permukaan terpilih;
batasnya mengikuti diskontinuitas depth dan bentuk permukaan.

Mode:

1. `AUTO_NEAREST`: pilih permukaan terdekat yang memiliki dukungan pengamatan
   memadai dalam area pandang. Abaikan satu piksel noise dekat.
2. `MANUAL_RANGE`: batasi kandidat ke rentang jarak yang diatur pengguna, lalu
   pilih permukaan terdekat yang memenuhi rentang itu. Jangan otomatis pindah
   ke foreground di luar rentang. Jika target jauh tertutup, status `NO_TARGET`.
3. `WORLD_LOCK`: simpan titik fiksasi world yang sudah dipilih dan proyeksikan
   kembali selama kamera bergerak. Target tertutup tidak mendapat detail baru.

"Terdekat" berarti jarak sepanjang sumbu pandang, bukan dekat ke pusat world
bola lama. Pemilihan foreground otomatis dan pemilihan fokus jauh manual
harus merupakan mode berbeda agar kedua kontrol tidak saling membatalkan.

## Input dan koordinat

Input per frame: depth terkalibrasi, intrinsics, pose `T_world_camera`, timestamp,
serta estimasi noise depth jika tersedia. Backproject dahulu menjadi `p_camera`.
Untuk titik graf world, transformasikan kembali ke camera hanya untuk penilaian
perhatian; posisi graf tetap berada di world.

`FocusControl` memuat:

- mode; gaze unit vector `g`; basis ortonormal `e_x, e_y` tegak lurus `g`;
- focus distance `d_cmd > 0` meter sepanjang `g`;
- rentang depan/belakang `delta_near, delta_far > 0` meter;
- sudut pencarian horizontal/vertikal `alpha_x, alpha_y` radian;
- skala detail permukaan `L > 0` meter, gain, attention floor;
- ambang dukungan permukaan, ambang kontinuitas, timeout dan hysteresis.

Untuk titik `p` dalam camera:

    s = dot(p, g)
    theta_x = atan2(dot(p, e_x), s)
    theta_y = atan2(dot(p, e_y), s)
    r2 = (theta_x/alpha_x)^2 + (theta_y/alpha_y)^2

Area pencarian: `s > 0 && r2 <= 1`. Elips ini hanya aperture pencarian;
bentuk area detail akhir ditentukan oleh permukaan. Pada `MANUAL_RANGE`,
tambahkan `d_cmd-delta_near <= s <= d_cmd+delta_far`.

## Algoritma per frame

1. Ambil frame dan kontrol sebagai satu snapshot konsisten. Tolak depth invalid
   atau di luar jangkauan sensor. Pertahankan informasi invalid, jangan mengisi
   lubang depth sebagai permukaan yang pasti ada.
2. Bentuk adjacency piksel 4-neighbor pada depth valid. Hubungkan hanya jika
   jarak 3D dan beda depth konsisten dengan jarak piksel serta noise sensor.
   Normal dapat membantu, tetapi normal yang tidak stabil bukan alasan tunggal
   menghapus kandidat. Larang penyeberangan diskontinuitas besar.
3. Dalam aperture/rentang, temukan komponen permukaan dengan dukungan minimum.
   Nilai kedekatan komponen memakai kuantil 10% dari `s`, bukan minimum mentah.
   Ambang dukungan harus diuji pada objek tipis; jangan menganggap semua komponen
   kecil sebagai noise. Pilih skor terkecil; tie-break jarak angular ke gaze.
4. Pilih seed dari titik pengamatan nyata dekat kuantil tersebut dan dekat gaze.
   Fokus aktual `d_obs` ialah median depth aksial pada tetangga seed di komponen
   yang sama. Jangan merata-ratakan foreground dan background menjadi titik
   fokus di ruang kosong. `d_cmd` tetap merupakan kontrol manual, tidak ditimpa
   `d_obs`; pada autofocus, fokus aktif mengikuti `d_obs` secara temporal.
5. Hitung jarak geodesik lokal `ell` dari seed melalui adjacency permukaan,
   dibatasi aperture dan rentang kedalaman aktif. Panjang edge adalah jarak 3D.
   Ini menghasilkan mask yang mengikuti permukaan; dua objek pada depth sama
   tidak otomatis sama-sama menjadi fokus.
6. Evaluasi attention pada pengamatan dan node graf dengan field yang sama.
   Untuk node world, proyeksikan ke frame dan uji depth sebelum mengambil bobot
   mask. Node di belakang pengamatan lebih dari toleransi noise adalah occluded;
   node di depan pengamatan tanpa dukungan juga tidak diberi bonus perhatian.
   Di batas objek gunakan asosiasi tetangga depth yang konsisten, bukan
   interpolasi bilinear yang mencampur dua lapisan.
7. Terapkan kebutuhan detail ke alokasi FGNG. Pembentukan/penghapusan edge
   tetap memerlukan aturan geometri tersendiri; attention tidak membuktikan
   validitas suatu edge.

## Field perhatian

Untuk titik yang diasosiasikan ke permukaan terpilih:

    A_angle = max(0, 1-r2)^2
    A_surface = exp(-ell/L)
    b = delta_near jika s < d_focus, selain itu delta_far
    q = abs(s-d_focus)/b
    A_depth = max(0, 1-q*q)^2
    A = V * C * A_angle * A_surface * A_depth
    weight = floor + gain*A

`V` bernilai 1 hanya untuk asosiasi terlihat dan konsisten dengan depth frame
sekarang. `C` adalah confidence [0,1] dari dukungan dan stabilitas target.
Di luar komponen/rentang, `A=0`; background hanya mendapat floor. Dalam mode
manual, `d_focus=d_cmd`; mode auto menggunakan fokus aktif hasil seleksi.
Tidak perlu bentuk lingkaran untuk visualisasi: tampilkan heatmap dan kontur
mask aktual, seed, jarak perintah, jarak permukaan, dan status target.

Alternatif parameter antarmuka ialah inverse distance/diopter untuk sensitivitas
slider; simpan dan tampilkan meter. Ini tidak mengubah fokus optik sensor depth.

## Stabilitas dan penyimpanan peta

- Pergantian target otomatis membutuhkan kandidat yang lebih dekat dengan
  margin melampaui noise selama beberapa frame; perubahan kontrol manual
  membatalkan pending target sebelumnya.
- Asosiasikan target antarframe melalui posisi world dan overlap permukaan,
  bukan nomor komponen yang berubah setiap frame.
- Saat occluded/invalid: segera hentikan bonus detail, tahan identitas sementara
  untuk reacquisition, lalu kedaluwarsa sesuai timeout. Jangan memancarkan
  perhatian ke permukaan belakang atau menghapus target dari peta karena hilang
  dari pandangan. `WORLD_LOCK` tidak memilih objek pengganti secara otomatis.
- Pisahkan prioritas perhatian dari confidence keberadaan geometri. Attention
  floor sendiri tidak menjamin retensi atau konektivitas pada core batch lama.
- Batas jumlah node tetap berlaku. Foveasi mengalokasikan detail yang tersedia;
  jika split/merge aman tidak mungkin, laporkan keterbatasan resolusi.

## Integrasi yang direncanakan

Buat modul `SurfaceFocusField` terpisah dengan `update(frame, pose, control)`
dan `evaluate(world_point)`, mengembalikan snapshot immutable selama satu batch.
Gunakan callback `setAttention` yang sudah ada. Native camera menggunakan pose
identitas untuk field, sementara world wrapper memakai pose aktual.

Ganti `updateFoveaDepth` dengan seleksi target di atas, dan `levelAt` dengan
sampling dari mask aktual. Pertahankan sampling perifer nonzero serta koreksi
bobot berdasarkan probabilitas/luas sampling supaya penambahan sampling dan
attention tidak menghitung kepentingan dua kali secara tidak sengaja.

Untuk prototipe, adjacency dan pencarian target memakai raster depth resolusi
tetap; geodesik hanya dihitung pada area fokus dengan priority queue terbatas.
Jangan mencari target dari node FGNG yang sudah jarang: objek kecil dapat hilang
sebelum mendapat kesempatan menjadi fokus.

## Uji penerimaan sebelum mengganti default

1. Foreground 1 m, background 3 m, keduanya dalam aperture: autofocus memilih
   foreground; background tidak memperoleh bonus.
2. Manual 3 m dengan rentang 0.2 m: memilih bagian background yang terlihat;
   foreground 1 m tidak dipilih. Jika background sepenuhnya tertutup: NO_TARGET.
3. Satu outlier dekat tidak mengalahkan komponen valid dengan dukungan memadai.
4. Dua objek terpisah pada depth sama: hanya komponen seed yang mendapat bonus.
5. Permukaan miring: mask mengikuti permukaan sampai batas rentang fokus;
   tidak berubah menjadi bola world atau menyeberangi celah depth.
6. Pose kamera bergerak pada target world tetap: lock mengikuti proyeksi target;
   occlusion mematikan bonus tanpa menghapus geometri historis.
7. Depth invalid dan kandidat bergantian: tidak ada lock tanpa batas atau flicker
   melebihi kebijakan hysteresis; timestamp mundur memicu reset tracker.
8. Naik/turunkan jarak fokus, rentang, dan aperture secara independen: ketiganya
   menghasilkan perubahan berbeda dan tidak saling menimpa.

Bandingkan foveasi lama, angular+depth tanpa seleksi permukaan, serta desain
lengkap pada anggaran node dan input yang sama. Ukur kebocoran bonus ke lapisan
belakang, ketepatan target, waktu perpindahan fokus, error permukaan fokus,
konektivitas, dan waktu komputasi. Belum ada klaim hasil atau kebaruan.
