# Draf baru ICRA 2027 — dual graph dan target cluster

**From Semantic Clusters to Pre-Grasp Poses: Coupling Environment Topology with Configuration Witnesses**

Paper berbahasa Inggris, anonim, memakai template IEEE/PaperCept dua kolom.
Versi awal berisi 6 halaman, 3 figur, 2 tabel, dan 11 referensi yang dikutip.
`main.pdf` adalah naskah yang dapat dibaca; `main.tex` dan `references.bib`
adalah sumber yang dapat diedit. Ini draf riset dengan hasil analisis offline,
belum naskah dengan evaluasi grasp fisik lengkap. Tidak ada paper yang disubmit.
Paper lama dan kode robot tidak diubah selama pembuatan dokumen ini.

## Fokus paper

1. YOLOX dan DD-GNG menghasilkan graph lingkungan berlabel.
2. Target memakai titik tengah batas 3D **cluster yang terlihat**, dengan ID
   node dipakai untuk asosiasi; bukan memilih node cap atau permukaan botol.
3. Graph robot menyimpan konfigurasi joint dan pose FK dari data workspace hijau.
4. Kandidat pre-grasp harus memenuhi jarak, arah approach, koneksi dari start,
   dan pengecekan collision pada state interpolasi.
5. Audit frame menjelaskan mengapa approach masih dapat datang dari atas.

Pusat yang dimaksud implementasi adalah midpoint AABB `(min + max)/2` pada tiap
sumbu. Ini berbeda dari rata-rata titik, pusat volume sebenarnya, atau pusat
massa benda. Bentuk tersembunyi tidak dapat disimpulkan dari satu pengamatan.

## Temuan utama yang sudah didukung data

- **33.401** titik grid IK offline; **15.175** memiliki witness posisi.
- **3.198** witness pose hijau: **9,57%** dari grid dan **21,07%** dari witness posisi.
- **334** pose biru dekat singular tidak masuk seleksi hijau.
- Residual solver terbesar pada seleksi hijau sekitar **0,9997 mm** dan
  **0,43961 derajat**. Angka ini terhadap model URDF; bukan pengukuran akurasi
  kamera atau TCP fisik.
- Scanner memakai transformasi frame yang membuat `end_effector_link +X`
  mengarah ke bawah dan `+Z` mengarah radial horizontal pada pose yang diminta.
  FK independen semua 3.198 konfigurasi pada URDF arsip mengonfirmasi `+X`
  paling jauh 0,439608 derajat dari world-down.
  Karena itu syarat approach lokal `+X` dapat memilih pose di atas target.
  Restart atau mengganti marker tidak mengubah konvensi tersebut.

Persamaan frame dan batas alignment terdapat di Bagian V naskah.
`evidence_audit.md`, `method_audit.md`, dan `data/` berisi jejak bukti.
Data benchmark 21.600 query dari paper reachability lama tidak digunakan sebagai
hasil sistem baru. Rekaman smoke/instrumentasi tidak dihitung sebagai pick sukses.

## File yang disertakan

| File | Kegunaan |
|---|---|
| `main.pdf` | Draf paper hasil kompilasi |
| `main.tex`, `references.bib` | Naskah dan daftar pustaka |
| `paper_source.zip` | Paket sumber untuk dipindahkan atau dibuka di Overleaf |
| `figures/` | Diagram skematik serta plot data, PDF vektor dan PNG |
| `scripts/analyze_evidence.py` | Menghitung ulang statistik dari arsip asli |
| `scripts/check_witness_frames.py` | Verifikasi FK offline seluruh konfigurasi hijau |
| `scripts/draw_schematics.py` | Membuat diagram tanpa data buatan yang disamarkan sebagai observasi |
| `scripts/verify_manuscript.py` | Memeriksa halaman, referensi, overflow, font, dan identitas |
| `verification.json` | Hasil pemeriksaan PDF |
| `EVALUATION_PLAN.md` | Studi terkontrol yang dibutuhkan untuk melengkapi paper |
| `SUBMISSION_NOTES.md` | Ketentuan resmi ICRA 2027 dan batas klaim |

## Build

Di folder ini:

```bash
bash build.sh
```

Perintah memakai figur dan hasil audit yang sudah tersimpan. Dibutuhkan Python
dengan NumPy/Matplotlib, pdfLaTeX, BibTeX, `pdfinfo`, `pdftotext`, dan `pdffonts`.
Untuk menghitung ulang dari arsip eksperimen di checkout robot yang sama:

```bash
bash build.sh --refresh
```

Build tidak menjalankan ROS, kamera, controller, atau motor. Untuk Overleaf,
unggah `paper_source.zip`, pilih `main.tex` sebagai main document dan pdfLaTeX
sebagai compiler. Figur sudah ada sehingga Python tidak diperlukan untuk
kompilasi LaTeX di Overleaf. `template/` berasal dari template PaperCept yang
telah tersedia pada proyek; identitas penulis tidak diambil dari nama akun.

## Yang masih perlu dilengkapi

Paper ini belum membuktikan peningkatan performa terhadap baseline atau
keberhasilan pickup. Langkah penting berikutnya adalah memastikan konvensi
gripper fisik, memperbaiki konsistensi target-pose untuk beberapa objek, dan
menjalankan evaluasi terkontrol pada scene baru. Lihat `EVALUATION_PLAN.md`.

Naskah sudah menjelaskan batas tersebut. Penulis manusia perlu meninjau metode,
angka, referensi, kebaruan dibanding paper lama, dan disclosure penggunaan AI
sebelum memutuskan submission. Arsip sumber dan catatan audit memuat nama/path
proyek untuk kerja internal; **PDF anonim** adalah artefak review, bukan seluruh
folder audit sebagai supplementary anonim.
