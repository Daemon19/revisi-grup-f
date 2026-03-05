# 🤖 Revisi Grup F Bayucaraka

Repositori ini berisi hasil pengembangan dan revisi **Final Project Bayucaraka**, yang mencakup tiga bagian utama:

- ⚡ **Electrical**: desain PCB, skematik, dan firmware mikrokontroler.
- 🔧 **Mechanical**: model CAD komponen dan assembly.
- 💻 **Programming**: workspace pemrograman tambahan (MCU/ROS/script).

## 📝 Catatan

- Catatan revisi yang dilakukan dapat ditemukan pada README di masing-masing directory subdivisi
- Folder `Demo` menyimpan versi saat demo.
- Folder `Revisi` adalah hasil perbaikan akhir.

## 🎯 Tujuan Proyek

Proyek ini membangun sistem aktuator berbasis **ESP32** untuk mekanisme pemindahan objek (pick-and-place sederhana), dengan kombinasi:

- 🔀 2 sumbu gerak linear menggunakan motor stepper (X dan Y),
- 🎮 2 servo untuk mekanisme capit dan naik-turun,
- 🏠 limit switch sebagai referensi gerak/home.

## 📁 Struktur Folder Utama

- ⚡ `electrical/`
  - `Demo/`: versi awal desain dan kode.
  - `Revisi/`: versi perbaikan (PCB + firmware final revisi).
    - `CNC V4 Power Jack (PCB)/`: file KiCad (`.kicad_pcb`, `.kicad_sch`, dsb).
    - `FP BAYUCARAKA/`: proyek PlatformIO firmware ESP32.
- 🔧 `mechanical/`
  - `Demo/`: model mekanik sebelum revisi.
  - `Revisi/`: model mekanik setelah revisi.
- 💻 `programming/`
  - `Demo/`: folder pengembangan kode MCU/ROS/script.
  - `Revisi/`: folder lanjutan untuk revisi pemrograman.
