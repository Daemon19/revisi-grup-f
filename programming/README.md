# Revisi Programming

Adalah hal yang sulit untuk menemukan bug pada proses penggerakan menuju _payload_ tanpa melakukan trial, sehingga bug pada saat demo masih tidak ditemukan.

Meskipun demikian, masih ada ruang untuk mengembangkan basis kode, tepatnya pada node ROS2.
Adapun pengembangannya sebagai berikut.

## Node Camera

- Kegagalan membuka sumber video akan memunculkan error
- Mempublish koordinat pixel saat file homography tidak ditemukan

## Node Detector

- Meningkatkan akurasi deteksi ArUco dengan pengaturan parameter dan preproses
