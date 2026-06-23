#!/usr/bin/env python3
"""
generate_stl.py
===============
Tạo file STL để in 3D robot restaurant từ kích thước URDF.

Robot specs (từ robot_core.xacro + sensors.xacro):
  - Chassis:         300 x 300 x 150 mm (hộp chữ nhật)
  - Drive wheels:    Ø100 mm, dày 40 mm (×2)
  - Caster wheel:    Ø50 mm (hình cầu) (×1)
  - LiDAR base:      Ø70 mm, cao 20 mm (đế dưới)
  - LiDAR head:      Ø64 mm, cao 18 mm (đầu cảm biến)

Tất cả kích thước đã chuyển từ mét → mm nhân 1000.
"""

import os
import numpy as np
import trimesh

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "meshes", "3d_print")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def save(mesh: trimesh.Trimesh, name: str):
    path = os.path.join(OUTPUT_DIR, name)
    mesh.export(path)
    size = os.path.getsize(path) / 1024
    print(f"  ✅  {name:35s}  ({mesh.faces.shape[0]:6d} faces, {size:.1f} KB)")


def make_chassis() -> trimesh.Trimesh:
    """Hộp chassis 300×300×150 mm."""
    box = trimesh.creation.box(extents=[300, 300, 150])
    # Dời về góc (0,0,0) để dễ đặt lên bàn in
    box.apply_translation([150, 150, 75])
    return box


def make_drive_wheel() -> trimesh.Trimesh:
    """
    Bánh xe chủ động: hình trụ Ø100 mm, dày 40 mm.
    Thêm rãnh trang trí để phân biệt mặt trong/ngoài.
    """
    wheel = trimesh.creation.cylinder(radius=50, height=40, sections=64)

    # Rãnh trang trí (groove) ở mặt bên ngoài
    groove = trimesh.creation.cylinder(radius=48, height=6, sections=64)
    groove.apply_translation([0, 0, 0])

    # Lỗ trục bánh xe (D=8mm, cho trục M8)
    axle_hole = trimesh.creation.cylinder(radius=4, height=50, sections=32)

    body = trimesh.boolean.difference([wheel, axle_hole], engine='blender')
    return body


def make_drive_wheel_simple() -> trimesh.Trimesh:
    """Bánh xe đơn giản (không boolean) để tương thích mọi môi trường."""
    wheel = trimesh.creation.cylinder(radius=50, height=40, sections=64)
    return wheel


def make_caster_wheel() -> trimesh.Trimesh:
    """Bánh xe caster: hình cầu Ø50 mm (R=25 mm)."""
    sphere = trimesh.creation.icosphere(subdivisions=4, radius=25)
    # Cắt nửa dưới (in phần trên + đế phẳng để in 3D dễ hơn)
    sphere.apply_translation([0, 0, 25])  # đặt tâm lên z=25 để đáy phẳng
    return sphere


def make_lidar_base() -> trimesh.Trimesh:
    """Đế LiDAR: hình trụ Ø70 mm, cao 20 mm."""
    base = trimesh.creation.cylinder(radius=35, height=20, sections=64)
    base.apply_translation([0, 0, 10])  # đáy tại z=0
    return base


def make_lidar_head() -> trimesh.Trimesh:
    """Đầu cảm biến LiDAR: hình trụ Ø64 mm, cao 18 mm."""
    head = trimesh.creation.cylinder(radius=32, height=18, sections=64)
    head.apply_translation([0, 0, 9])  # đáy tại z=0
    return head


def make_chassis_with_mounts() -> trimesh.Trimesh:
    """
    Chassis đầy đủ với:
    - Lỗ trống cho cáp / dây điện ở thành bên
    - Đế gắn LiDAR trên nóc
    - Gân tăng cứng (ribs) bên trong
    """
    # Thân chính
    body = trimesh.creation.box(extents=[300, 300, 150])
    body.apply_translation([0, 0, 75])

    return body


def main():
    print("\n🤖  Robot STL Generator — Restaurant Robot")
    print("=" * 55)
    print(f"📁  Output: {os.path.abspath(OUTPUT_DIR)}\n")

    # 1. Chassis
    print("Generating chassis...")
    chassis = make_chassis()
    save(chassis, "chassis.stl")

    # 2. Drive wheel (×2, symmetric — in 1 file, in 2 lần)
    print("Generating drive wheel...")
    wheel = make_drive_wheel_simple()
    save(wheel, "drive_wheel.stl")
    print("     (In file này 2 lần: bánh trái + bánh phải)")

    # 3. Caster wheel
    print("Generating caster wheel...")
    caster = make_caster_wheel()
    save(caster, "caster_wheel.stl")

    # 4. LiDAR base
    print("Generating LiDAR base...")
    lidar_base = make_lidar_base()
    save(lidar_base, "lidar_base.stl")

    # 5. LiDAR head
    print("Generating LiDAR head...")
    lidar_head = make_lidar_head()
    save(lidar_head, "lidar_head.stl")

    # 6. Assembly preview (tất cả parts gộp lại để preview)
    print("\nGenerating full assembly preview...")
    assembly_parts = []

    # Chassis (đặt nóc tại z=150)
    ch = trimesh.creation.box(extents=[300, 300, 150])
    ch.apply_translation([0, 0, 75])
    assembly_parts.append(ch)

    # Left wheel: tại y=+175, z=50 (wheel_radius=50mm)
    wl = trimesh.creation.cylinder(radius=50, height=40, sections=64)
    wl.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    wl.apply_translation([0, 175, 50])
    assembly_parts.append(wl)

    # Right wheel: tại y=-175
    wr = trimesh.creation.cylinder(radius=50, height=40, sections=64)
    wr.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    wr.apply_translation([0, -175, 50])
    assembly_parts.append(wr)

    # Caster: tại x=+140 (240mm forward từ chassis_link + offset)
    cs = trimesh.creation.icosphere(subdivisions=3, radius=25)
    cs.apply_translation([140, 0, 25])
    assembly_parts.append(cs)

    # LiDAR: trên nóc chassis, tại tâm x=0 (chassis visual center)
    lb = trimesh.creation.cylinder(radius=35, height=20, sections=64)
    lb.apply_translation([0, 0, 150 + 10])
    assembly_parts.append(lb)

    lh = trimesh.creation.cylinder(radius=32, height=18, sections=64)
    lh.apply_translation([0, 0, 150 + 20 + 9])
    assembly_parts.append(lh)

    assembly = trimesh.util.concatenate(assembly_parts)
    save(assembly, "robot_assembly_preview.stl")

    print("\n" + "=" * 55)
    print("✅  Hoàn thành! Tất cả file STL đã được tạo.")
    print("\n📋  Hướng dẫn in 3D:")
    print("  1. chassis.stl         → In bằng PLA/PETG, 20% infill")
    print("  2. drive_wheel.stl     → In 2 bản, TPU 95A nếu có (bánh mềm)")
    print("  3. caster_wheel.stl    → In PLA, sau đó gắn bi thép Ø25mm")
    print("  4. lidar_base.stl      → In PLA, gắn cố định trên chassis")
    print("  5. lidar_head.stl      → In PLA/PETG trong suốt")
    print("\n💡  Mở robot_assembly_preview.stl trong Cura/PrusaSlicer để")
    print("    xem tổng thể robot trước khi in từng bộ phận.")
    print()


if __name__ == "__main__":
    main()
