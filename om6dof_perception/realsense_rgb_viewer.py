#!/usr/bin/env python3
"""Tampilkan selisih frame RealSense dalam window RGB dan grayscale berdampingan."""

import argparse
import os
import sys


WINDOW_NAME = "OM6DOF Vision - RGB Frame Difference"
GRAYSCALE_WINDOW_NAME = "OM6DOF Vision - Grayscale Frame Difference"
WINDOW_NAMES = (WINDOW_NAME, GRAYSCALE_WINDOW_NAME)


def window_is_closed(cv2, window_name):
    """GTK tidak mendukung WND_PROP_VISIBLE; gunakan AUTOSIZE sebagai fallback."""
    try:
        visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible == 0
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        # Beberapa backend melempar error saat window sudah ditutup.
        return True


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("Nilai harus lebih besar dari nol.")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=positive_int, default=640)
    parser.add_argument("--height", type=positive_int, default=480)
    parser.add_argument("--fps", type=positive_int, default=30)
    parser.add_argument("--serial", help="Serial kamera jika ada beberapa RealSense.")
    args = parser.parse_args()

    if sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        print(
            "Tidak ada display. Jalankan dari terminal desktop atau SSH "
            "dengan X11 forwarding agar window OpenCV dapat ditampilkan.",
            file=sys.stderr,
        )
        return 1

    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as error:
        print(f"Dependensi belum tersedia: {error}. Lihat README.md.", file=sys.stderr)
        return 1

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps
    )

    started = False
    window_created = False
    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        window_created = True
        cv2.namedWindow(GRAYSCALE_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow(WINDOW_NAME, 100, 80)
        cv2.moveWindow(GRAYSCALE_WINDOW_NAME, args.width + 140, 80)
        profile = pipeline.start(config)
        started = True
        device = profile.get_device()
        print(f"Kamera: {device.get_info(rs.camera_info.name)}")
        print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
        print(f"Stream RGB: {args.width}x{args.height} @ {args.fps} FPS")
        print("Tampilan: frame terbaru - frame sebelumnya (nilai negatif menjadi 0).")
        print("Window kiri: RGB. Window kanan: grayscale dari hasil pengurangan.")
        print("Tekan q / Esc, tutup window, atau Ctrl+C untuk keluar.")

        previous_rgb = None
        while True:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            if color_frame:
                # Data asli: uint8, shape (height, width, 3), urutan R/G/B.
                rgb_image = np.asanyarray(color_frame.get_data())
                if previous_rgb is None:
                    # Frame pertama belum memiliki pembanding.
                    difference_rgb = np.zeros_like(rgb_image)
                else:
                    # Pengurangan uint8 dengan saturasi, bukan wrap-around.
                    difference_rgb = cv2.subtract(rgb_image, previous_rgb)
                # Salin karena buffer RealSense dapat digunakan ulang.
                previous_rgb = rgb_image.copy()
                # OpenCV memakai B/G/R saat menampilkan gambar.
                bgr_image = cv2.cvtColor(difference_rgb, cv2.COLOR_RGB2BGR)
                grayscale_image = cv2.cvtColor(difference_rgb, cv2.COLOR_RGB2GRAY)
                cv2.imshow(WINDOW_NAME, bgr_image)
                cv2.imshow(GRAYSCALE_WINDOW_NAME, grayscale_image)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if any(window_is_closed(cv2, name) for name in WINDOW_NAMES):
                break

    except KeyboardInterrupt:
        pass
    except (RuntimeError, cv2.error) as error:
        print(f"Gagal menampilkan stream RGB: {error}", file=sys.stderr)
        print(
            "Periksa sambungan kamera, akses USB, resolusi/FPS yang didukung, "
            "dan pastikan kamera tidak sedang digunakan aplikasi lain.",
            file=sys.stderr,
        )
        return 1
    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            if window_created:
                cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
