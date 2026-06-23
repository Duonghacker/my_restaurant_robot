#!/usr/bin/env python3
"""
Render chassis_frame preview từ tham số trong SCAD file.
Dùng matplotlib để vẽ 3D engineering drawing.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patheffects as pe

# ── Tham số (khớp với SCAD) ──────────────────────────────────
CL, CW, CH = 300, 300, 150   # chassis L/W/H
WT, FT = 4, 5                 # wall/floor thickness
MOTOR_Z = 35                  # motor shaft z from bottom
CASTER_X = 240                # caster position X
LID_H = 4                     # lid thickness
CR = 10                       # corner radius

# Electronics positions (OX, OY, SX, SY, label, color)
ELECTRONICS = [
    (20,  20,  58, 49,  "Raspberry Pi 4\n(58×49mm)", "#22c55e"),
    (115, 115, 70, 70,  "STM32\n(70×70mm)",          "#f59e0b"),
    (190, 125, 80, 50,  "Power Board\n(80×50mm)",     "#ef4444"),
]

OUT = "/home/sonduong/dev_ws/src/restaurant_robot_description/meshes/3d_print/"

# ── Figure setup ─────────────────────────────────────────────
fig = plt.figure(figsize=(20, 14), facecolor='#0d0f14')
fig.suptitle("Restaurant Robot — Chassis Frame Engineering Preview",
             color='white', fontsize=15, fontweight='bold', y=0.97)

# ════════════════════════════════════════════════════════
# PLOT 1: 3D Isometric view
# ════════════════════════════════════════════════════════
ax3d = fig.add_subplot(221, projection='3d', facecolor='#0d0f14')

def draw_hollow_box(ax, x, y, z, lx, ly, lz, wt, ft, color, alpha=0.55):
    """Vẽ hộp rỗng: tường + đáy"""
    def face(pts, c, a):
        poly = Poly3DCollection([pts], alpha=a)
        poly.set_facecolor(c)
        poly.set_edgecolor('#4f8ef7')
        poly.set_linewidth(0.3)
        ax.add_collection3d(poly)

    # Đáy
    face([[x,y,z],[x+lx,y,z],[x+lx,y+ly,z],[x,y+ly,z]], color, alpha)
    # Tường trước (X-)
    face([[x,y,z],[x+wt,y,z],[x+wt,y+ly,z],[x,y+ly,z]], color, alpha+0.1)
    face([[x,y,z],[x+wt,y,z],[x+wt,y+ly,z],[x,y+ly,z+lz]], color, alpha*0.7)
    # Tường sau (X+)
    face([[x+lx-wt,y,z],[x+lx,y,z],[x+lx,y+ly,z],[x+lx-wt,y+ly,z]], color, alpha+0.1)
    face([[x+lx-wt,y,z+lz],[x+lx,y,z+lz],[x+lx,y+ly,z+lz],[x+lx-wt,y+ly,z+lz]], color, alpha*0.7)
    # Tường trái (Y-)
    face([[x,y,z],[x+lx,y,z],[x+lx,y+wt,z],[x,y+wt,z]], color, alpha)
    face([[x,y,z],[x+lx,y,z],[x+lx,y+wt,z+lz],[x,y+wt,z+lz]], color, alpha)
    # Tường phải (Y+)
    face([[x,y+ly-wt,z],[x+lx,y+ly-wt,z],[x+lx,y+ly,z],[x,y+ly,z]], color, alpha)
    face([[x,y+ly-wt,z+lz],[x+lx,y+ly-wt,z+lz],[x+lx,y+ly,z+lz],[x,y+ly,z+lz]], color, alpha)

draw_hollow_box(ax3d, 0, 0, 0, CL, CW, CH, WT, FT, '#3a7bdd')

# Nắp (ghost/transparent)
lid_pts = np.array([[0,0,CH],[CL,0,CH],[CL,CW,CH],[0,CW,CH]])
lid = Poly3DCollection([lid_pts], alpha=0.15)
lid.set_facecolor('#a78bfa'); lid.set_edgecolor('#a78bfa'); lid.set_linewidth(0.5)
ax3d.add_collection3d(lid)

# Vẽ motor holes (circles trên tường Y)
theta = np.linspace(0, 2*np.pi, 32)
r_shaft = 4
# Trái (Y=0 face)
zc, yc, xc = MOTOR_Z, 0, CL/2
ax3d.plot(xc + r_shaft*np.cos(theta), [yc]*32, zc + r_shaft*np.sin(theta), 'r-', lw=1.5)
# Phải (Y=CW face)
ax3d.plot(xc + r_shaft*np.cos(theta), [CW]*32, zc + r_shaft*np.sin(theta), 'r-', lw=1.5)

# Vẽ caster holes (trên đáy)
for sign in [-1, 1]:
    for sign2 in [-1, 1]:
        cx = CASTER_X + sign*15; cy = CW/2 + sign2*15
        ax3d.scatter([cx], [cy], [0], c='orange', s=30, zorder=5)

# Standoff dots (bên trong)
for ox, oy, sx, sy, lbl, col in ELECTRONICS:
    for dx in [ox, ox+sx]:
        for dy in [oy, oy+sy]:
            ax3d.scatter([dx], [dy], [FT+14], c=col, s=20, zorder=5)

ax3d.set_xlim(0, CL); ax3d.set_ylim(0, CW); ax3d.set_zlim(0, CH+20)
ax3d.set_xlabel('X mm', color='#64748b', fontsize=8)
ax3d.set_ylabel('Y mm', color='#64748b', fontsize=8)
ax3d.set_zlabel('Z mm', color='#64748b', fontsize=8)
ax3d.tick_params(colors='#334155', labelsize=7)
ax3d.xaxis.pane.fill = ax3d.yaxis.pane.fill = ax3d.zaxis.pane.fill = False
ax3d.grid(True, color='#1e2535', lw=0.4)
ax3d.set_title("3D View (thân dưới + nắp)", color='#94a3b8', fontsize=10)
ax3d.view_init(elev=28, azim=-55)

# ════════════════════════════════════════════════════════
# PLOT 2: Top view (nhìn từ trên) — bố trí bên trong
# ════════════════════════════════════════════════════════
ax_top = fig.add_subplot(222, facecolor='#0d0f14')
ax_top.set_aspect('equal')
ax_top.set_facecolor('#0d0f14')

# Chassis outline
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
chassis_rect = FancyBboxPatch((0, 0), CL, CW,
    boxstyle=f"round,pad={CR}", linewidth=2,
    edgecolor='#4f8ef7', facecolor='#1a2540', zorder=1)
ax_top.add_patch(chassis_rect)

# Wall thickness indication
inner = FancyBboxPatch((WT, WT), CL-2*WT, CW-2*WT,
    boxstyle=f"round,pad={max(CR-WT,2)}", linewidth=1,
    edgecolor='#2a3a60', facecolor='#0d1a35', linestyle='--', zorder=2)
ax_top.add_patch(inner)

# Motor shaft holes
for y_pos, label in [(0, "Motor L"), (CW, "Motor R")]:
    c = Circle((CL/2, y_pos), 4, color='#ef4444', zorder=5)
    ax_top.add_patch(c)
    # 4 bolt holes
    for a in [45, 135, 225, 315]:
        bx = CL/2 + 17*np.sin(np.radians(a))
        by = y_pos + 17*np.cos(np.radians(a))
        c2 = Circle((bx, by), 1.7, color='#f59e0b', zorder=5)
        ax_top.add_patch(c2)
    ax_top.text(CL/2 + 30, y_pos + (8 if y_pos==0 else -8), label,
               color='#ef4444', fontsize=7, ha='left')

# Caster holes
for sign in [-1, 1]:
    for sign2 in [-1, 1]:
        cx = CASTER_X + sign*15; cy = CW/2 + sign2*15
        c = Circle((cx, cy), 2.2, color='#f59e0b', zorder=5)
        ax_top.add_patch(c)
ax_top.text(CASTER_X, CW/2 + 28, "Caster\n(4×M4)", color='#f59e0b',
           fontsize=7, ha='center')

# Electronics
for ox, oy, sx, sy, lbl, col in ELECTRONICS:
    r = FancyBboxPatch((ox, oy), sx, sy,
        boxstyle="round,pad=1", linewidth=1.5,
        edgecolor=col, facecolor=col+'22', zorder=4)
    ax_top.add_patch(r)
    ax_top.text(ox + sx/2, oy + sy/2, lbl,
               color=col, fontsize=6.5, ha='center', va='center', fontweight='bold')
    # standoffs
    for dx in [ox, ox+sx]:
        for dy in [oy, oy+sy]:
            c = Circle((dx, dy), 3.5, color=col, alpha=0.7, zorder=5)
            ax_top.add_patch(c)

# LiDAR mount hole (nắp) — vẽ bằng nét đứt
c_lidar = Circle((CL/2, CW/2), 22, fill=False,
                  linestyle='--', edgecolor='#a78bfa', linewidth=1.5, zorder=3)
ax_top.add_patch(c_lidar)
ax_top.text(CL/2, CW/2, "LiDAR\nMount\n(nắp)", color='#a78bfa',
           fontsize=7, ha='center', va='center')

# Cable holes (tường sau x=0)
for dy in [-35, 35]:
    c = Circle((0, CW/2+dy), 8, color='#22d3ee', alpha=0.7, zorder=5)
    ax_top.add_patch(c)
ax_top.text(-15, CW/2, "Cáp", color='#22d3ee', fontsize=7, ha='right', rotation=90)

# Vent holes
for xi in [70, 130, 190, 250]:
    for yi in [0, CW]:
        c = Circle((xi, yi), 6, color='#475569', alpha=0.7, zorder=5)
        ax_top.add_patch(c)

# Dimension lines
ax_top.annotate('', xy=(CL, -20), xytext=(0, -20),
    arrowprops=dict(arrowstyle='<->', color='white', lw=1))
ax_top.text(CL/2, -30, f"{CL} mm", color='white', fontsize=8, ha='center')

ax_top.annotate('', xy=(CL+20, CW), xytext=(CL+20, 0),
    arrowprops=dict(arrowstyle='<->', color='white', lw=1))
ax_top.text(CL+35, CW/2, f"{CW} mm", color='white', fontsize=8, ha='left', rotation=90)

ax_top.set_xlim(-40, CL+60); ax_top.set_ylim(-50, CW+30)
ax_top.set_title("Nhìn từ trên (Top View) — Bố trí bên trong", color='#94a3b8', fontsize=10)
ax_top.set_xlabel("X (mm)", color='#64748b'); ax_top.set_ylabel("Y (mm)", color='#64748b')
ax_top.tick_params(colors='#475569')
ax_top.grid(True, color='#1e2535', lw=0.4)

# ════════════════════════════════════════════════════════
# PLOT 3: Side view (nhìn từ bên cạnh)
# ════════════════════════════════════════════════════════
ax_side = fig.add_subplot(223, facecolor='#0d0f14')
ax_side.set_aspect('equal')

# Chassis cross section
r = FancyBboxPatch((0, 0), CL, CH,
    boxstyle=f"round,pad={CR}", linewidth=2,
    edgecolor='#4f8ef7', facecolor='#1a2540')
ax_side.add_patch(r)
inner_s = FancyBboxPatch((WT, FT), CL-2*WT, CH-FT,
    boxstyle=f"round,pad={max(CR-WT,2)}", linewidth=1,
    edgecolor='#2a3a60', facecolor='#0d1a35', linestyle='--')
ax_side.add_patch(inner_s)

# Motor position
m = Circle((CL/2, MOTOR_Z), 4, color='#ef4444', zorder=5)
ax_side.add_patch(m)
ax_side.annotate(f"Motor shaft\nz={MOTOR_Z}mm",
    xy=(CL/2, MOTOR_Z), xytext=(CL/2+60, MOTOR_Z+15),
    color='#ef4444', fontsize=7,
    arrowprops=dict(arrowstyle='->', color='#ef4444', lw=1))

# Caster position
c_cs = Circle((CASTER_X, 0), 5, color='#f59e0b', zorder=5)
ax_side.add_patch(c_cs)
ax_side.text(CASTER_X, -15, f"Caster\nX={CASTER_X}mm", color='#f59e0b',
            fontsize=7, ha='center')

# Standoff height
ax_side.axhline(y=FT+14, xmin=0.05, xmax=0.95,
                color='#22d3ee', lw=1, linestyle=':', alpha=0.7)
ax_side.text(5, FT+14+3, f"PCB level\n(+{FT+14}mm)", color='#22d3ee', fontsize=7)

# Lid
lid_r = FancyBboxPatch((0, CH), CL, 4,
    boxstyle=f"round,pad={CR}", linewidth=1.5,
    edgecolor='#a78bfa', facecolor='#2d1b69', alpha=0.6)
ax_side.add_patch(lid_r)
ax_side.text(CL/2, CH+2, "Nắp (tháo rời)", color='#a78bfa',
            fontsize=7, ha='center', va='center')

# Dimensions
ax_side.annotate('', xy=(CL+15, CH), xytext=(CL+15, 0),
    arrowprops=dict(arrowstyle='<->', color='white', lw=1))
ax_side.text(CL+30, CH/2, f"H={CH}mm", color='white', fontsize=8, ha='left', rotation=90)
ax_side.annotate('', xy=(CL, -20), xytext=(0, -20),
    arrowprops=dict(arrowstyle='<->', color='white', lw=1))
ax_side.text(CL/2, -30, f"L={CL}mm", color='white', fontsize=8, ha='center')

ax_side.set_xlim(-30, CL+60); ax_side.set_ylim(-45, CH+25)
ax_side.set_title("Nhìn từ bên cạnh (Side View)", color='#94a3b8', fontsize=10)
ax_side.set_xlabel("X (mm)", color='#64748b'); ax_side.set_ylabel("Z (mm)", color='#64748b')
ax_side.tick_params(colors='#475569')
ax_side.grid(True, color='#1e2535', lw=0.4)

# ════════════════════════════════════════════════════════
# PLOT 4: Legend / BOM
# ════════════════════════════════════════════════════════
ax_bom = fig.add_subplot(224, facecolor='#0d0f14')
ax_bom.axis('off')

bom_title = "📋  Bill of Materials & Thông số"
ax_bom.text(0.05, 0.97, bom_title, transform=ax_bom.transAxes,
           color='white', fontsize=11, fontweight='bold', va='top')

items = [
    ("KÍCH THƯỚC CHASSIS", None, 'header'),
    ("Dài × Rộng × Cao",  "300 × 300 × 150 mm", '#e2e8f0'),
    ("Độ dày tường",       "4 mm",                '#e2e8f0'),
    ("Độ dày đáy",         "5 mm",                '#e2e8f0'),
    ("Bo góc",             "R10 mm",              '#e2e8f0'),
    ("",None,'space'),
    ("LỖ MOTOR (2 bên)",   None, 'header'),
    ("Trục motor",         "Ø8 mm (chỉnh MOTOR_SHAFT_D)", '#ef4444'),
    ("Lỗ mặt bích",       "4× M3 Ø17mm",         '#f59e0b'),
    ("Cao từ đáy",         "35 mm",               '#e2e8f0'),
    ("",None,'space'),
    ("LỖ CASTER (đáy)",    None, 'header'),
    ("Pattern",            "4× M4, 30×30mm",      '#f59e0b'),
    ("Vị trí X",          "240 mm từ đuôi",      '#e2e8f0'),
    ("",None,'space'),
    ("STANDOFF NỘI THẤT",  None, 'header'),
    ("Raspberry Pi 4",     "4× M2.5, 58×49mm",    '#22c55e'),
    ("STM32 board",        "4× M3, 70×70mm",      '#f59e0b'),
    ("Power board",        "4× M3, 80×50mm",      '#ef4444'),
    ("Cao standoff",       "14 mm",               '#e2e8f0'),
    ("",None,'space'),
    ("NẮP THÁO LẮP",       None, 'header'),
    ("Gắn bằng",           "6× M3 bu lông",       '#a78bfa'),
    ("LiDAR mount",        "4× M3, Ø60mm BCD",    '#a78bfa'),
    ("",None,'space'),
    ("GỢI Ý IN 3D",        None, 'header'),
    ("Vật liệu",           "PETG hoặc PLA+",      '#22d3ee'),
    ("Infill",             "25% Gyroid",          '#22d3ee'),
    ("Layer height",       "0.2 mm",              '#22d3ee'),
    ("Walls",              "3 perimeters",        '#22d3ee'),
    ("Support",            "Không cần (thiết kế không có overhang)", '#22d3ee'),
]

y = 0.91
for item in items:
    label, val, typ = item
    if typ == 'space': y -= 0.012; continue
    if typ == 'header':
        ax_bom.text(0.05, y, label, transform=ax_bom.transAxes,
                   color='#64748b', fontsize=7.5, fontweight='bold',
                   va='top', style='normal')
        ax_bom.plot([0.05, 0.95], [y-0.005, y-0.005], color='#1e2535',
                   lw=0.8, transform=ax_bom.transAxes)
        y -= 0.035; continue
    col = val[1] if isinstance(val, tuple) else typ
    ax_bom.text(0.07, y, f"• {label}:", transform=ax_bom.transAxes,
               color='#94a3b8', fontsize=7.5, va='top')
    ax_bom.text(0.48, y, val or '', transform=ax_bom.transAxes,
               color=col, fontsize=7.5, va='top', fontweight='500')
    y -= 0.032

plt.tight_layout(rect=[0, 0, 1, 0.95])
out_path = OUT + "chassis_engineering_drawing.png"
plt.savefig(out_path, dpi=130, bbox_inches='tight', facecolor='#0d0f14')
print(f"Saved: {out_path}")
