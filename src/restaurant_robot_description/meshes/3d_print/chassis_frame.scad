/* ================================================================
   Restaurant Robot — Chassis Frame
   Dựa trên URDF robot_core.xacro

   Kích thước từ simulation:
     Chassis:           300 × 300 × 150 mm
     Khoảng cách bánh: 350 mm (±175mm từ tâm Y)
     Caster vị trí:    x = +240mm từ chassis_link origin

   Thiết kế thực tế:
     - Thân rỗng, có nắp tháo rời (6× bu lông M3)
     - Lỗ trục motor Ø8mm (dùng được cho TT motor lớn, 
       37mm gearbox, hoặc motor N20 — chỉnh lại MOTOR_SHAFT_D)
     - 4× lỗ M3 gắn mặt bích motor mỗi bên
     - 4× lỗ M4 gắn caster wheel (pattern 30×30mm)
     - Trụ standoff bên trong:
         RPi 4:     4× M2.5  (58×49mm)
         STM32:     4× M3    (70×70mm)
         Power Brd: 4× M3    (80×50mm)
     - LiDAR mount: 4× M3 trên vòng Ø60mm ở nắp
     - Lỗ thông gió + lỗ đi cáp

   In 3D: PETG 25% infill, layer 0.2mm, wall 3 perimeters
   ================================================================ */

$fn = 64;

// ── Kích thước chính ──────────────────────────────────────────
CL = 300;   // Chassis Length (X, dọc robot)
CW = 300;   // Chassis Width  (Y, ngang)
CH = 150;   // Chassis Height (Z)

WT = 4;     // Wall thickness
FT = 5;     // Floor thickness
CR = 10;    // Corner radius (bo góc ngoài)

// ── Motor shaft ───────────────────────────────────────────────
// Thay MOTOR_SHAFT_D phù hợp motor bạn mua:
//   TT Motor:       Ø6mm  →  6.5
//   37mm Gearbox:   Ø6mm  →  6.5
//   GA25 Gearbox:   Ø6mm  →  6.5
//   BLDC Hub:       đo thực tế
MOTOR_SHAFT_D  = 8.0;   // Lỗ trục (có dung sai 0.5mm)
MOTOR_BOLT_D   = 3.4;   // Lỗ bu lông M3 (có dung sai)
MOTOR_BOLT_R   = 17;    // Bán kính vòng bu lông từ tâm trục
// z tâm trục motor (từ đáy chassis)
// Wheel center trong URDF = z=0 của chassis_link = đáy chassis
// Đặt 35mm để motor gắn được bên trong chassis dễ hơn
MOTOR_Z = 35;

// ── Caster wheel ──────────────────────────────────────────────
// Caster 50mm tiêu chuẩn, gắn từ dưới đáy
// Pattern lỗ phổ biến: 30×30mm, M4
CASTER_BOLT_D  = 4.4;   // M4 + dung sai
CASTER_PAT     = 30;    // Khoảng cách giữa 2 lỗ (mỗi chiều)
// Vị trí X của caster trong tọa độ chassis:
// caster_joint origin = 0.24m từ chassis_link = 240mm từ chassis origin X
CASTER_X = 240;

// ── Electronics standoffs ──────────────────────────────────────
STANDOFF_H  = 14;   // Chiều cao trụ (khoảng cách mạch - đáy)
STANDOFF_OD =  7;   // Đường kính ngoài trụ

// Raspberry Pi 4: 58×49mm, 4 lỗ M2.5
RPI_X = 58; RPI_Y = 49; RPI_D = 2.7;
// Đặt tại góc sau-trái
RPI_OX = 20; RPI_OY = 20;

// STM32 (Nucleo / custom): 70×70mm, M3
STM_S = 70; STM_D = 3.3;
// Đặt giữa chassis
STM_OX = 115; STM_OY = 115;

// Power board: 80×50mm, M3
PWR_X = 80; PWR_Y = 50; PWR_D = 3.3;
// Đặt phía trước
PWR_OX = 190; PWR_OY = 125;

// ── LiDAR mount (trên nắp) ────────────────────────────────────
LIDAR_CENTER_D  = 44;   // Lỗ trung tâm cho cáp LiDAR
LIDAR_BOLT_R    = 30;   // Bán kính vòng gắn
LIDAR_BOLT_D    = 3.4;  // M3

// ── TOP LID screws (gắn nắp) ─────────────────────────────────
LID_SCREW_D = 3.4;  // M3

// ================================================================
// HELPER: Rounded box (hull of 4 cylinders)
// ================================================================
module rbox(x, y, z, r) {
    hull()
        for (px = [r, x-r], py = [r, y-r])
            translate([px, py, 0]) cylinder(r=r, h=z);
}

// ================================================================
// HELPER: Standoff trụ nội thất
// ================================================================
module standoff(h, od, id) {
    difference() {
        cylinder(d=od, h=h);
        translate([0,0,-0.5]) cylinder(d=id, h=h+1);
    }
}

// ================================================================
// MODULE: CHASSIS TRAY (phần thân dưới)
// ================================================================
module chassis_tray() {
    difference() {
        // ── Vỏ ngoài bo góc ──
        rbox(CL, CW, CH, CR);

        // ── Khoét rỗng bên trong (mở trên) ──
        translate([WT, WT, FT])
            rbox(CL-2*WT, CW-2*WT, CH-FT+1, max(CR-WT, 2));

        // ════ LỖ MOTOR TRÁI (tường Y+ = tường ngoài y=CW) ════
        // Tâm trục: (CL/2, CW, MOTOR_Z) — giữa chiều dài chassis
        translate([CL/2, -1, MOTOR_Z])
            rotate([-90, 0, 0])
                cylinder(d=MOTOR_SHAFT_D, h=WT+2);
        // 4 lỗ bu lông mặt bích (0°,90°,180°,270°)
        for (a = [45, 135, 225, 315])
            translate([CL/2 + MOTOR_BOLT_R*sin(a), -1,
                       MOTOR_Z + MOTOR_BOLT_R*cos(a)])
                rotate([-90, 0, 0])
                    cylinder(d=MOTOR_BOLT_D, h=WT+2);

        // ════ LỖ MOTOR PHẢI (tường Y- = y=0) ════
        translate([CL/2, CW+1, MOTOR_Z])
            rotate([90, 0, 0])
                cylinder(d=MOTOR_SHAFT_D, h=WT+2);
        for (a = [45, 135, 225, 315])
            translate([CL/2 + MOTOR_BOLT_R*sin(a), CW+1,
                       MOTOR_Z + MOTOR_BOLT_R*cos(a)])
                rotate([90, 0, 0])
                    cylinder(d=MOTOR_BOLT_D, h=WT+2);

        // ════ LỖ CASTER WHEEL (đáy, phía trước) ════
        for (dx = [-CASTER_PAT/2, CASTER_PAT/2])
        for (dy = [-CASTER_PAT/2, CASTER_PAT/2])
            translate([CASTER_X + dx, CW/2 + dy, -1])
                cylinder(d=CASTER_BOLT_D, h=FT+2);

        // ════ LỖ ĐI CÁP (tường sau, 2 lỗ Ø16) ════
        for (dy = [-35, 35])
            translate([-1, CW/2 + dy, CH*0.55])
                rotate([0, 90, 0])
                    cylinder(d=16, h=WT+2);

        // ════ LỖ THÔNG GIÓ bên hông (mỗi bên 4 lỗ Ø12) ════
        for (xi = [70, 130, 190, 250]) {
            translate([xi, -1, CH*0.65])
                rotate([-90, 0, 0])
                    cylinder(d=12, h=WT+2);
            translate([xi, CW-WT-1, CH*0.65])
                rotate([-90, 0, 0])
                    cylinder(d=12, h=WT+2);
        }

        // ════ 6 LỖ BU LÔNG GẮN NẮP (xung quanh miệng) ════
        for (pos = [
            [CL*0.2,  WT/2],    [CL*0.8,  WT/2],
            [CL*0.2,  CW-WT/2], [CL*0.8,  CW-WT/2],
            [WT/2,    CW*0.5],  [CL-WT/2, CW*0.5]
        ])
            translate([pos[0], pos[1], CH-14])
                cylinder(d=LID_SCREW_D, h=20);
    }

    // ════ STANDOFF: Raspberry Pi 4 ════
    for (dx = [0, RPI_X], dy = [0, RPI_Y])
        translate([RPI_OX + dx, RPI_OY + dy, FT])
            standoff(STANDOFF_H, STANDOFF_OD, RPI_D);

    // ════ STANDOFF: STM32 ════
    for (dx = [0, STM_S], dy = [0, STM_S])
        translate([STM_OX + dx, STM_OY + dy, FT])
            standoff(STANDOFF_H, STANDOFF_OD, STM_D);

    // ════ STANDOFF: Power Board ════
    for (dx = [0, PWR_X], dy = [0, PWR_Y])
        translate([PWR_OX + dx, PWR_OY + dy, FT])
            standoff(STANDOFF_H, STANDOFF_OD, PWR_D);

    // ════ GÂN TĂNG CỨNG ĐÁY (2 gân dọc) ════
    for (yi = [CW*0.33, CW*0.66])
        translate([WT, yi - 2, FT])
            cube([CL - 2*WT, 4, 8]);
}

// ================================================================
// MODULE: TOP LID (nắp tháo lắp)
// ================================================================
module chassis_top() {
    difference() {
        union() {
            // Tấm nắp phẳng
            rbox(CL, CW, 4, CR);
            // Viền lồng vào thân (lip xuống 10mm)
            translate([WT+0.5, WT+0.5, -10])
                rbox(CL-2*WT-1, CW-2*WT-1, 10, max(CR-WT-1,2));
        }

        // Lỗ trung tâm LiDAR
        translate([CL/2, CW/2, -1]) {
            cylinder(d=LIDAR_CENTER_D, h=10);
            // 4 lỗ bu lông LiDAR (xung quanh)
            for (a = [45, 135, 225, 315])
                translate([LIDAR_BOLT_R*cos(a), LIDAR_BOLT_R*sin(a), 0])
                    cylinder(d=LIDAR_BOLT_D, h=10);
        }

        // 6 lỗ bu lông gắn nắp vào thân
        for (pos = [
            [CL*0.2,  WT/2],    [CL*0.8,  WT/2],
            [CL*0.2,  CW-WT/2], [CL*0.8,  CW-WT/2],
            [WT/2,    CW*0.5],  [CL-WT/2, CW*0.5]
        ])
            translate([pos[0], pos[1], -11])
                cylinder(d=LID_SCREW_D, h=16);

        // Lỗ thông gió nắp (dạng khe dài)
        for (xi = [60, 130, 200])
            translate([xi, CW/2, -1])
                hull() {
                    translate([0, -30, 0]) cylinder(d=8, h=10);
                    translate([0,  30, 0]) cylinder(d=8, h=10);
                }
    }
}

// ================================================================
// EXPORT — Bỏ comment từng cái để xuất STL riêng:
// ================================================================
chassis_tray();

// Nắp (tháo lắp):
// translate([0, 320, 0]) chassis_top();
