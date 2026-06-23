/* ================================================================
   Robot Prototype Chassis v3.4 — Thiết kế gá chuẩn cơ khí (Gầm Lật)
   
   Bao gồm:
   - Base Plate: Vách gá motor và ụ mắt trâu hướng XUỐNG DƯỚI. Mặt trên phẳng.
   - Gá motor GA25: Lắp ở gầm (Z=-18). Tương thích chính xác motor GA25-370.
   - Caster M4: Ụ nâng lồi xuống 17.5mm để cân bằng chính xác với cánh đa hướng 33mm.
   - 4 trụ đồng: Lỗ Ø3.4mm ở 4 góc, có khoét âm (countersink) giấu đầu vít.
   - LiDAR: 4 lỗ M2.5 trên Top Plate (khoảng cách cần xác nhận với datasheet).
   ================================================================ */

$fn = 64;

// ── Kích thước cơ sở ──
PL = 200;      // Chiều dài
PW = 190;      // Chiều rộng
PT = 5;        // Độ dày tấm (Z = 0 đến 5)
CR = 12;       // Bo góc viền (tăng lên 12mm để khung xe mượt mà, "ngầu" hơn)

// ── Thông số động cơ GA25 (Gầm) ──
MOTOR_X = 67;          // Vị trí động cơ (X)
MOTOR_Z = -18;         // Cao độ tâm trục motor so với đáy tấm Base (Z=0)
                       // Trục ở Z = -18 -> Bánh R=32.5 chạm đất tại Z = -50.5
MOTOR_WALL_H = 32;     // Chiều dài vách gắn motor hướng xuống (đến Z = -32)
MOTOR_BOSS_D = 10.5;   // Lỗ cho phần lõi lồi trung tâm của motor GA25
MOTOR_HOLE_PITCH = 17; // Khoảng cách 2 lỗ vít M3 trên mặt motor GA25
MOTOR_BOLT_D = 3.4;    // Lỗ qua vít M3
MOTOR_SHAFT_D = 12;    // Rãnh thả motor từ dưới lên

// ── Bánh Đa Hướng 33mm (Đế 38x32mm, 4 lỗ M4) ──
// Datasheet: Lỗ Ø4mm (M4). Đế 38x32mm.
// Pitch ước tính (chưa datasheet chính thức): 30mm x 24mm
CASTER_X = 160;        // Vị trí cụm bánh theo trục X
CASTER_PEDESTAL_H = 17.5; // Chiều dài ụ nâng bánh hướng xuống
                       // Cân bằng: H = 50.5mm = |MOTOR_Z|(18) + R_bánh(32.5)
                       // PEDESTAL_H = 50.5 - CASTER_HEIGHT(33) = 17.5mm ✔️
CASTER_PITCH_X = 32;   // Khoảng cách 2 lỗ vít theo trục X
CASTER_PITCH_Y = 38;   // Khoảng cách 2 lỗ vít theo trục Y
CASTER_BOLT_D = 4.4;   // Lỗ qua vít M4 (Ø4mm + 0.4mm khe hở lắp)

// ── LiDAR RPLidar A1M8 (Tầng 2) ──
// Datasheet (top view): 4 lỗ hình CHỮ NHẬT
//   - Thân máy rộng 56mm, cao 45.5mm
//   - Khoảng cách lỗ NGANG: 40mm (cả 2 hàng)
//   - Khoảng cách lỗ DỌC:  70mm (= 42 + 28)
//   - Vít: M2.5
LIDAR_X = 155;          // Vị trí tâm LiDAR theo trục X
LIDAR_SPAN_BOT = 40;    // Khoảng cách lỗ ngang hàng dưới (datasheet: 40±0.05mm)
LIDAR_PITCH_X = 70;     // Khoảng cách lỗ dọc (42 + 28 = 70mm)
LIDAR_BOLT_D = 2.7;     // Lỗ qua vít M2.5
LIDAR_SPAN_TOP = 56;    // Khoảng cách lỗ ngang hàng trên (datasheet: 56±0.05mm)

// ── Raspberry Pi 4 (Tầng 2) ──
// Xoay ngang (chiều rộng 56mm theo trục X, chiều dài 85mm theo trục Y)
PI_X = 30;           // Vị trí tâm Pi 4 theo trục X (đặt ở phần đầu xe)
PI_PITCH_X = 49;     // Khoảng cách lỗ vít theo trục X (chiều rộng Pi)
PI_PITCH_Y = 58;     // Khoảng cách lỗ vít theo trục Y (chiều dài Pi)
PI_BOLT_D = 2.7;     // Lỗ qua vít M2.5

// ── Cản trước (Aerodynamic Bumper) ──
BUMPER_MOUNT_X = PL - 8; // Vị trí khoét 3 lỗ bắt vít trên Base Plate (Cách mép 8mm)
BUMPER_MOUNT_PITCH = 100; // Khoảng cách giữa 2 lỗ vít biên
BUMPER_BOLT_D = 3.4;      // Vít M3 bắt cản

// ── Trụ đồng M3x55mm ──
PILLAR_OFF = 15;
PILLARS = [
    [PILLAR_OFF, PILLAR_OFF],
    [PL - PILLAR_OFF, PILLAR_OFF],
    [PILLAR_OFF, PW - PILLAR_OFF],
    [PL - PILLAR_OFF, PW - PILLAR_OFF]
];
PILLAR_D = 3.4;

// ================================================================
// HELPER MODULES (Bổ sung thẩm mỹ)
// ================================================================
module rplate(x, y, t, r) {
    hull()
        for (px=[r, x-r], py=[r, y-r])
            translate([px, py, 0]) cylinder(r=r, h=t);
}

// Rãnh cắt trang trí (capsule)
module capsule(l, w) {
    hull() {
        translate([-l/2, 0, 0]) cylinder(d=w, h=PT+2);
        translate([l/2, 0, 0]) cylinder(d=w, h=PT+2);
    }
}

// Gân tăng cứng hướng XUỐNG
module y_gusset_down(size, thick) {
    hull() {
        translate([0, 0, -size]) cube([thick, size, 0.1]); // Điểm dưới
        translate([0, 0, 0]) cube([thick, 0.1, 0.1]);      // Điểm góc
        translate([0, 0, -0.1]) cube([thick, size, 0.1]);  // Cạnh trên
    }
}
module y_gusset_rev_down(size, thick) {
    hull() {
        translate([0, -size, -size]) cube([thick, size, 0.1]);
        translate([0, -0.1, 0]) cube([thick, 0.1, 0.1]);
        translate([0, -size, -0.1]) cube([thick, size, 0.1]);
    }
}

// Lỗ gắn trụ đồng 55mm
module pillar_hole_base() {
    translate([0, 0, -1]) cylinder(d=PILLAR_D, h=PT+2);
    // Countersink mặt DƯỚI (bây giờ là Z=0) để giấu đầu vít từ dưới lên
    translate([0, 0, -0.1]) cylinder(d=6.5, h=2.5); 
}
module pillar_hole_top() {
    translate([0, 0, -1]) cylinder(d=PILLAR_D, h=PT+2);
    // Countersink mặt TRÊN để giấu đầu vít từ trên xuống
    translate([0, 0, PT-2.4]) cylinder(d=6.5, h=2.5); 
}

// ================================================================
// TẦNG 1: BASE PLATE (Đáy phẳng ở trên, support chỉa xuống)
// ================================================================
module base_plate() {
    difference() {
        union() {
            // Tấm đáy chính (Z = 0 đến Z = 5)
            rplate(PL, PW, PT, CR);
            
            // ── Vách gắn motor TRÁI (Y=0) hướng XUỐNG ──
            translate([MOTOR_X - 25, 0, -MOTOR_WALL_H]) cube([50, 4, MOTOR_WALL_H]);
            // Gân tăng cứng
            translate([MOTOR_X - 25, 4, 0]) y_gusset_down(14, 4);
            translate([MOTOR_X + 21, 4, 0]) y_gusset_down(14, 4);
            
            // ── Vách gắn motor PHẢI (Y=PW) hướng XUỐNG ──
            translate([MOTOR_X - 25, PW - 4, -MOTOR_WALL_H]) cube([50, 4, MOTOR_WALL_H]);
            // Gân tăng cứng
            translate([MOTOR_X - 25, PW - 4, 0]) y_gusset_rev_down(14, 4);
            translate([MOTOR_X + 21, PW - 4, 0]) y_gusset_rev_down(14, 4);

            // ── Ụ nâng Caster hướng XUỐNG ──
            translate([CASTER_X, PW/2, -CASTER_PEDESTAL_H]) cylinder(d=48, h=CASTER_PEDESTAL_H);
        }
        
        // ── Lỗ gá motor TRÁI (Xuyên vách) ──
        translate([MOTOR_X, -1, MOTOR_Z]) {
            rotate([-90,0,0]) cylinder(d=MOTOR_BOSS_D, h=6);
            for (dx=[-MOTOR_HOLE_PITCH/2, MOTOR_HOLE_PITCH/2])
                translate([dx, 0, 0]) rotate([-90,0,0]) cylinder(d=MOTOR_BOLT_D, h=6);
            // Rãnh luồn trục từ dưới lên (Mở miệng rãnh xuống Z=-32)
            translate([-MOTOR_SHAFT_D/2, 0, -32]) cube([MOTOR_SHAFT_D, 6, 32 + MOTOR_Z]);
        }
        
        // ── Lỗ gá motor PHẢI (Xuyên vách) ──
        translate([MOTOR_X, PW+1, MOTOR_Z]) {
            rotate([90,0,0]) cylinder(d=MOTOR_BOSS_D, h=6);
            for (dx=[-MOTOR_HOLE_PITCH/2, MOTOR_HOLE_PITCH/2])
                translate([dx, 0, 0]) rotate([90,0,0]) cylinder(d=MOTOR_BOLT_D, h=6);
            translate([-MOTOR_SHAFT_D/2, -6, -32]) cube([MOTOR_SHAFT_D, 6, 32 + MOTOR_Z]);
        }
        
        // ── Lỗ bắt vít Caster 4 góc (Xuyên qua ụ nâng) ──
        translate([CASTER_X, PW/2, -CASTER_PEDESTAL_H - 1]) {
            for (dx=[-CASTER_PITCH_X/2, CASTER_PITCH_X/2]) {
                for (dy=[-CASTER_PITCH_Y/2, CASTER_PITCH_Y/2]) {
                    translate([dx, dy, 0]) cylinder(d=CASTER_BOLT_D, h=CASTER_PEDESTAL_H + PT + 2);
                }
            }
            // Lỗ xuyên ở giữa để rỗng ụ nâng cho đỡ tốn nhựa
            cylinder(d=22, h=CASTER_PEDESTAL_H + PT + 2);
        }
        
        // ── 4 lỗ góc cho trụ đồng ──
        for (p = PILLARS) translate([p[0], p[1], 0]) pillar_hole_base();

        // ── Khoét rỗng luồn cáp / trang trí tản nhiệt (Giao diện Sci-fi) ──
        // 1. Lỗ luồn cáp trung tâm (Vẫn giữ diện tích 40x30 nhưng bo tròn mượt mà)
        translate([MOTOR_X, PW/2, -1]) hull() {
            translate([-10, -5, 0]) cylinder(d=20, h=PT+2);
            translate([ 10, -5, 0]) cylinder(d=20, h=PT+2);
            translate([-10,  5, 0]) cylinder(d=20, h=PT+2);
            translate([ 10,  5, 0]) cylinder(d=20, h=PT+2);
        }
        
        // 2. Rãnh chéo tản nhiệt (Thay cho lỗ tròn nhàm chán)
        translate([120, 35, -1]) rotate([0, 0, 30]) capsule(45, 12);
        translate([120, PW-35, -1]) rotate([0, 0, -30]) capsule(45, 12);
        
        // 3. Đường rãnh tốc độ (Speed lines) ở phần đuôi xe
        translate([30, PW/2, -1]) capsule(40, 8);
        translate([25, PW/2 - 25, -1]) capsule(30, 6);
        translate([25, PW/2 + 25, -1]) capsule(30, 6);
        
        // ── 3 Lỗ bắt vít cho cản trước (Bumper) ──
        for (dy = [-BUMPER_MOUNT_PITCH/2, 0, BUMPER_MOUNT_PITCH/2]) {
            translate([BUMPER_MOUNT_X, PW/2 + dy, -1]) cylinder(d=BUMPER_BOLT_D, h=PT+2);
        }
    }
}

// ================================================================
// TẦNG 2: TOP PLATE (Nắp)
// ================================================================
module top_plate() {
    difference() {
        // Tấm nắp
        rplate(PL, PW, PT, CR);
        
        // Lỗ góc cho trụ đồng
        for (p = PILLARS) translate([p[0], p[1], 0]) pillar_hole_top();
        
        // Lỗ đi dây chung (từ dưới lên)
        translate([MOTOR_X, PW/2, -1]) cylinder(d=25, h=PT+2);
        
        // Lỗ đi dây cáp cho RPi/LiDAR
        translate([CASTER_X, PW/2, -1]) cylinder(d=18, h=PT+2);
        
        // Lỗ bắt vít cho LiDAR A1M8 — Hình thang (Top: 56mm, Bot: 40mm, Pitch: 70mm)
        //   - Hàng TRÊN: 2 lỗ cách nhau 56mm, tại LIDAR_X
        //   - Hàng DƯỚI: 2 lỗ cách nhau 40mm, tại LIDAR_X - 70mm
        translate([LIDAR_X, PW/2, -1]) {
            // Hàng trên (56mm span)
            for (dy=[-LIDAR_SPAN_TOP/2, LIDAR_SPAN_TOP/2])
                translate([0, dy, 0]) cylinder(d=LIDAR_BOLT_D, h=PT+2);
            // Hàng dưới (40mm span)
            for (dy=[-LIDAR_SPAN_BOT/2, LIDAR_SPAN_BOT/2])
                translate([-LIDAR_PITCH_X, dy, 0]) cylinder(d=LIDAR_BOLT_D, h=PT+2);
        }
        
        // Lỗ bắt vít cho Raspberry Pi 4 (Xoay ngang)
        translate([PI_X, PW/2, -1]) {
            for (dx=[-PI_PITCH_X/2, PI_PITCH_X/2]) {
                for (dy=[-PI_PITCH_Y/2, PI_PITCH_Y/2]) {
                    translate([dx, dy, 0]) cylinder(d=PI_BOLT_D, h=PT+2);
                }
            }
        }
        
        // ── Khoét rỗng trang trí (Tầng 2) ──
        // Hoa văn sọc dọc mang phong cách Hi-Tech ở vùng trống giữa xe
        translate([95, PW/2, -1]) capsule(60, 12);
        translate([110, PW/2, -1]) capsule(45, 8);
        translate([80, PW/2, -1]) capsule(45, 8);
        
        translate([95, PW/2 - 40, -1]) rotate([0,0,90]) capsule(20, 6);
        translate([95, PW/2 + 40, -1]) rotate([0,0,90]) capsule(20, 6);
    }
}

// ================================================================
// TẦNG 1: FRONT BUMPER (Cản trước nguyên bản - Tối ưu In 3D)
// ================================================================
module front_bumper() {
    difference() {
        union() {
            // Ngàm gắn vào đáy Base Plate (dày 5mm, ăn sâu 15mm)
            translate([PL - 15, 0, -PT]) cube([15, PW, PT]);
            
            // Khối mũi xe (đơn giản, liền mạch, tránh tạo ra slivers mỏng)
            hull() {
                translate([PL - 0.1, 0, -PT]) cube([0.2, PW, PT]); // Gốc nối an toàn
                translate([PL + 30, 20, -PT]) cube([5, PW-40, PT]); // Mũi xe
                translate([PL, 10, 20]) cube([5, PW-20, 2]); // Vát lên
            }
            
            // Cánh lướt gió hông (Winglets) mọc từ ngoài mép Base Plate để không cạ
            for (dy=[0, PW-10]) {
                hull() {
                    translate([PL - 0.1, dy, -PT]) cube([5, 10, PT]); // Gốc mọc
                    translate([PL + 25, dy, 5]) cube([10, 10, 5]); // Đẩy ra ngoài
                    translate([PL + 15, dy, 30]) cube([5, 10, 2]); // Vươn cao
                }
            }
        }
        
        // ── KHOÉT LỖ BẮT VÍT M3 ──
        for (dy = [-BUMPER_MOUNT_PITCH/2, 0, BUMPER_MOUNT_PITCH/2]) {
            translate([BUMPER_MOUNT_X, PW/2 + dy, -PT-1]) cylinder(d=BUMPER_BOLT_D, h=PT+2);
        }
        
        // ── HỐC TẢN NHIỆT ĐƠN GIẢN (Giảm số lượng vertex) ──
        // Xuyên thủng thẳng từ trên xuống để đảm bảo cắt đứt khoát (tránh lỗi non-manifold)
        translate([PL + 15, PW/2, -10]) cylinder(d=20, h=40, $fn=40);
        translate([PL + 20, PW/2 - 35, -10]) cylinder(d=10, h=40, $fn=30);
        translate([PL + 20, PW/2 + 35, -10]) cylinder(d=10, h=40, $fn=30);
    }
}

// ================================================================
// RENDER (Dùng lệnh ! để chọn Part muốn render ra STL)
// ================================================================

// Lật Base Plate lại 180 độ theo trục X để in 3D (mặt phẳng nằm dưới mặt bàn in, ụ/vách chổng lên)
rotate([180, 0, 0])
base_plate();

// top_plate();
// front_bumper();
