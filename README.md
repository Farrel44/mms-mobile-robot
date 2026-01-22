# robot_mms

Paket ini merupakan bagian dari sistem navigasi robot omni-wheel. Modul ini berfungsi sebagai **Mobile Management System (MMS)** yang menangani komunikasi tingkat rendah (driver) antara Raspberry Pi (ROS 2) dan mikrokontroler ESP32.

Fungsi utama paket ini adalah mengirimkan instruksi kecepatan roda (RPM) dan menerima data umpan balik sensor (odometri dan sensor jarak).

## Struktur Direktori

Berikut adalah struktur file standar untuk paket ROS 2 berbasis Python (`ament_python`) yang digunakan dalam proyek ini:

```text
robot_mms/
├── robot_mms/                  # Source code Python (Module utama)
│   ├── __init__.py
│   ├── mms_bridge.py           # Node utama untuk komunikasi serial & ROS
│   └── kinematics.py           # Pustaka matematika kinematika robot omni
├── resource/                   # File penanda untuk sistem build Ament
│   └── robot_mms
├── setup.py                    # Skrip konfigurasi instalasi paket
├── package.xml                 # Metadata paket dan dependensi
└── README.md                   # Dokumentasi proyek

```

## Penjelasan Komponen Utama

Bagian ini menjelaskan fungsi teknis dari file-file konfigurasi yang terdapat dalam struktur proyek.

### 1. `setup.py`

File ini adalah skrip konfigurasi berbasis `setuptools` yang digunakan oleh sistem build ROS 2 (`colcon`) untuk menginstal paket. File ini memiliki tiga fungsi krusial:

* **Definisi Paket:** Menyatakan metadata paket seperti nama, versi, dan pengelola (maintainer).
* **Instalasi Resource:** Melalui parameter `data_files`, file ini menginstruksikan sistem untuk menyalin file konfigurasi (seperti `package.xml` dan file dalam folder `resource`) ke direktori instalasi ROS 2.
* **Entry Points:** Mendefinisikan pemetaan antara perintah terminal ROS 2 dengan fungsi Python yang akan dieksekusi.
* Contoh: Konfigurasi `console_scripts` memetakan perintah `ros2 run robot_mms mms_node` agar mengeksekusi fungsi `main()` di dalam file `robot_mms/mms_bridge.py`.



### 2. Direktori `resource/`

Direktori ini berisi satu file kosong dengan nama yang sama persis dengan nama paket (`robot_mms`).

* **Fungsi Teknis:** File ini berfungsi sebagai "marker" (penanda) untuk **Ament Index**, yaitu sistem penemuan paket di ROS 2.
* **Mekanisme:** Karena Python tidak memiliki manifest paket bawaan yang efisien untuk pencarian cepat, ROS 2 mencari keberadaan file penanda ini di jalur instalasi untuk memverifikasi bahwa paket `robot_mms` telah terinstal. Tanpa file ini, perintah seperti `ros2 pkg list` atau `ros2 run` tidak akan dapat mendeteksi paket, meskipun kode sumber Python sudah benar.

### 3. `robot_mms/` (Python Module)

Direktori ini berisi kode sumber logika program.

* `mms_bridge.py`: Node ROS 2 yang melakukan *parsing* data biner dari serial port ESP32 dan mempublikasikan topik ROS.
* `kinematics.py`: Berisi perhitungan matriks *Inverse Kinematics* (menerjemahkan kecepatan linear/angular menjadi RPM roda) dan *Forward Kinematics* (menerjemahkan putaran roda menjadi posisi robot).

## Instalasi dan Build

Ikuti langkah berikut untuk membangun paket ini di lingkungan ROS 2:

1. Masuk ke direktori workspace:
```bash
cd ~/robot_ws

```


2. Lakukan build paket menggunakan `colcon`:
```bash
colcon build --packages-select robot_mms

```


3. Source setup file untuk mendaftarkan paket ke lingkungan shell saat ini:
```bash
source install/setup.bash

```



## Penggunaan

Setelah build berhasil, node dapat dijalankan menggunakan perintah berikut:

```bash
ros2 run robot_mms mms_node

```

Pastikan mikrokontroler ESP32 telah terhubung ke port USB sebelum menjalankan node ini. Konfigurasi port serial dapat diubah melalui parameter ROS (lihat file konfigurasi di paket `robot_bringup`).