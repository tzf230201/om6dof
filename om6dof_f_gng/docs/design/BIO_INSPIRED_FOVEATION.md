# FGNG terinspirasi penglihatan manusia/primate

Status: rancangan pengganti V2, belum implementasi atau hasil eksperimen.
Acuan dipilih manusia/primate; bukan gabungan aturan semua spesies.

## Dasar biologis dan batas pemodelan

1. Resolusi penglihatan menurun dengan eksentrisitas dan berbeda menurut arah
   medan pandang. Model retina dan korteks tidak identik; pilih satu profil
   alokasi efektif lalu kalibrasi, jangan mengalikan semua densitas biologis.
   Sumber: Kupers et al. (2022),
   https://doi.org/10.1371/journal.pcbi.1009771 .
2. Vergensi dan akomodasi menyesuaikan terhadap jarak target. Karena itu target
   fokus dapat dekat atau jauh, tidak harus permukaan terdekat seluruh aperture.
   Sumber eksperimen: Hoffman et al. (2008),
   https://pmc.ncbi.nlm.nih.gov/articles/PMC2879326/ .
3. Seleksi lokasi perhatian dapat dimodelkan dengan saliency multiskala dan
   kompetisi antar lokasi; ini sudah memiliki model bio-inspired terdahulu.
   Sumber: Itti, Koch, Niebur (1998),
   https://doi.org/10.1109/34.730558 .

Uraian biologis di atas memotivasi komponen. Persamaan berikut adalah model
komputasional yang diusulkan untuk FGNG, bukan persamaan fisiologi yang telah
dibuktikan atau klaim kontribusi baru dengan sendirinya.

## Empat proses terpisah

### A. Seleksi fiksasi dan gerakan pandangan

State: arah pandang g, target world opsional, target focus demand D_target,
akomodasi aktif D, dan fase FIXATE/SHIFT/LOST.

Target dipilih pengguna atau pengendali saliency. Kandidat otomatis dapat
dinilai melalui kombinasi saliency RGB multiskala, perubahan temporal, dan
tujuan tugas. Bobot tugas/perubahan depth adalah perluasan rekayasa, bukan
komponen yang diklaim berasal seluruhnya dari Itti 1998. Penalti kunjungan baru
dapat membantu eksplorasi, tetapi manual fixation dan tracking tidak dipaksa
meninggalkan target. Kandidat tidak diurutkan semata-mata menurut jarak.

Lokasi baru memicu perpindahan pandangan kemudian fiksasi. World lock
memproyeksikan kembali target ketika kamera bergerak. Dalam kamera tetap,
perpindahan ini adalah pandangan virtual dalam FoV yang tersedia, bukan gerak
bola mata fisik. Jangan menjanjikan melihat di luar FoV sensor.

### B. Sampling retinotopik

Untuk p_camera yang terlihat, r=norm(p_camera),
e=acos(clamp(dot(g,p_camera)/r,-1,1)), dan phi sudut polar terhadap basis gaze.
Semua e memakai radian, bukan jarak meter dari pusat bola world.

    R(e,phi) = clamp(H(phi)/(1+e/e2)^p, R_min, 1)

e2>0, p>0 dan H(phi)>0 dikalibrasi terhadap profil efektif yang dipilih.
H=1 adalah baseline radial sederhana; profil arah yang tidak seragam harus
berasal dari data, bukan elips arbitrer yang disebut biologis. Basis arah
melekat pada kamera/kepala. Sampling perifer tetap nonzero.

R menentukan resolusi yang tersedia di setiap arah pandang. Jangan mengganti
R dengan segmentasi satu objek: beberapa objek yang terlihat dapat berada
dalam wilayah foveal. Mask objek opsional adalah modul perhatian objek.

### C. Akomodasi virtual dan blur

Nyatakan fokus dalam diopter D=1/d dengan d dalam meter. Untuk model paraxial
di sekitar gaze, gunakan kedalaman aksial s=dot(p_camera,g)>0:

    D_target = 1/d_target
    D_next = D + (1-exp(-dt/tau))*(D_target-D)
    delta_D = abs(1/s-D)
    b = k*P*delta_D
    F = 1/(1+(b/b0)^2)

tau>0, pupil virtual P>0 dan k,b0>0 merupakan parameter model yang perlu
kalibrasi. b adalah proksi blur berskala konsisten, bukan PSF mata lengkap.
Untuk P tetap, objek dengan selisih diopter lebih besar menerima detail lebih
rendah. P lebih kecil memperlebar rentang ketajaman dalam pendekatan ini;
model belum mencakup difraksi, aberasi, atau respons pupil terhadap cahaya.

Gunakan depth target dari patch yang konsisten pada lokasi fiksasi, bukan
minimum seluruh area atau median yang mencampur dua objek. Manual D_target
boleh independen dari target, sebagai kontrol eksperimen. Bila ada stereo
terkalibrasi, vergensi dapat dihitung menuju titik fiksasi bersama; bila hanya
RGB-D, jarak sensor adalah proksi dan tidak disebut estimasi vergensi biologis.

F adalah bobot akses detail, tidak mengubah koordinat geometri menjadi blur.
Sensor fixed-focus tetap fixed-focus; model ini tidak mengubah lensa kamera.

### D. Visibilitas dan integrasi graf world

    w(p) = w_base + gain * V(p) * C(p) * R(e,phi) * F(delta_D)

V=1 hanya jika node terasosiasi dengan pengamatan depth frame sekarang dalam
toleransi noise. Node tertutup, di luar FoV, atau tidak terdukung: V=0 untuk
bonus detail. C adalah confidence pengamatan. Oklusi adalah kendala observasi
geometris, bukan mekanisme akomodasi atau alasan menghapus sejarah.

Tidak ada hard nearest-surface gate dan tidak ada kewajiban komponen seed saja
yang boleh mendapat bobot tinggi. Dua objek terlihat pada jarak fokus serupa
dapat sama-sama jelas jika dekat arah tatapan. Permukaan belakang yang tertutup
tetap tidak mendapat bonus, walaupun jaraknya cocok dengan D.

Field dievaluasi di camera frame lalu mengatur kebutuhan node FGNG di world
frame. Sampling dapat mengikuti R; bobot kepentingan w dipakai eksplisit saat
alokasi, dengan koreksi probabilitas/luas sampling untuk mencegah foveasi ganda.
Gunakan snapshot field immutable selama satu batch.

## Konsekuensi untuk topologi

Foveasi biologis adalah inspirasi distribusi sumber daya sensorik; ia tidak
sendiri menjamin konektivitas graf. Pertahankan anggaran representasi kasar
dan validasi split/merge serta edge secara terpisah. Prioritas perhatian rendah
tidak sama dengan bukti objek hilang. Core batch lama tetap memerlukan audit
retensi meski attention floor positif.

Ketika fokus berpindah, detail tumbuh bertahap setelah fiksasi/akomodasi;
retensi struktur global dan pemadatan aman merupakan kontribusi rekayasa FGNG
yang harus diuji. Jangan mengklaim node FGNG setara neuron retina satu per satu.

## Perilaku uji dan rencana validasi

- Gaze tetap, fokus bergeser 1 m ke 3 m: profil R tetap, pembobotan F berubah.
- Fokus tetap, gaze berpindah: pusat resolusi angular berpindah.
- Dua objek terlihat pada depth sama: keduanya dapat tajam; tidak dibatasi
  connected component seed. Batas/oklusi tetap dihormati.
- Objek dekat di tepi aperture tidak merebut fokus manual pada objek jauh.
- Latar sepenuhnya tertutup: pengaturan fokus jauh tidak membuka data belakang.
- Selisih diopter sama menghasilkan F sama pada pupil virtual tetap; lebar
  rentang tajam dalam meter tidak dipaksa konstan di semua jarak.
- Perubahan fokus memperlihatkan respons temporal terkontrol; tau belum boleh
  diberi label waktu respons manusia tanpa kalibrasi data.
- Depth hilang: hentikan bonus yang tidak terdukung, timeout target tracker,
  jangan menghapus graf hanya karena occlusion.

Pisahkan evaluasi kesesuaian biologis (profil eksentrisitas, fokus dan waktu
respons dibanding data) dari manfaat robotik (error lokal, konektivitas,
anggaran node, waktu proses). Ablation: R saja, R+F, R+F+V, lalu pengendali
fiksasi otomatis; bekukan trajectory gaze untuk perbandingan field yang adil.
Hasil lama dalam paper belum menguji desain ini dan tidak boleh diberi label
sebagai validasi bio-inspired baru.
