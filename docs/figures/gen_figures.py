#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate publication-quality figures for the MorphQuad design docs.

Replaces the ASCII-art diagrams in the markdown design reports with rendered
PNGs (consistent palette, CJK labels). Run from anywhere; outputs into the
same directory as this script (docs/figures/).
"""
import os
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Circle, Arc
from matplotlib.font_manager import FontProperties
from fontTools.ttLib import TTCollection

HERE = os.path.dirname(os.path.abspath(__file__))
TTC = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
SC_OTF = os.path.join(HERE, "NotoSansCJKsc.otf")

# --- Locate / extract a CJK SC font face so matplotlib can render Chinese ---
if not os.path.exists(SC_OTF):
    coll = TTCollection(TTC)
    target = None
    for f in coll.fonts:
        nm = f["name"].getDebugName(1) or ""
        if "Noto Sans CJK SC" in nm:
            target = f
            break
    if target is None:
        target = coll.fonts[0]
    target.save(SC_OTF)

FP = FontProperties(fname=SC_OTF)
FP_B = FontProperties(fname=SC_OTF, weight="bold")

# --- Palette ---
C_HEADER = "#2E5A87"   # blue
C_GREEN = "#3C8C5A"    # green
C_PURPLE = "#6B4E9E"   # purple
C_ORANGE = "#E07A3F"   # orange
C_GRAY = "#5B6573"     # slate
C_LIGHT = "#EAF0F6"    # light panel
C_LINE = "#33414F"


def box(ax, x, y, w, h, title, lines=None, fc=C_HEADER, tc="white",
        fs=10.5, title_fs=11.5, title_color="white"):
    """Draw a rounded box with a bold title and optional body lines."""
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.10",
        fc=fc, ec="#1f2730", lw=1.3, zorder=2))
    ax.text(x + w / 2, y + h - 0.30, title, ha="center", va="center",
            fontproperties=FP_B, color=title_color, fontsize=title_fs, zorder=3)
    if lines:
        ax.text(x + w / 2, y + (h - 0.30) / 2 - 0.05, "\n".join(lines),
                ha="center", va="center", fontproperties=FP, color=tc,
                fontsize=fs, zorder=3)


def arrow(ax, xa, ya, xb, yb, color=C_LINE, rad=0.0, ls="-", lw=1.6,
          label=None, label_dy=0.16):
    arr = FancyArrowPatch(
        (xa, ya), (xb, yb), arrowstyle="-|>", mutation_scale=16,
        connectionstyle=f"arc3,rad={rad}", color=color, lw=lw,
        linestyle=ls, zorder=1)
    ax.add_patch(arr)
    if label:
        mx, my = (xa + xb) / 2, (ya + yb) / 2
        ax.text(mx, my + label_dy, label, ha="center", va="center",
                fontproperties=FP, color=color, fontsize=8.6, zorder=4)


def title(ax, txt, x, y, fs=14):
    ax.text(x, y, txt, ha="center", va="center", fontproperties=FP_B,
            fontsize=fs, color="#1f2730")


# ---------------------------------------------------------------------------
# 1. Algorithm flowchart  (算法报告 §2)
# ---------------------------------------------------------------------------
def fig_algo_flow():
    fig, ax = plt.subplots(figsize=(7.6, 8.2))
    ax.axis("off")
    ax.set_xlim(-3.6, 3.6)
    ax.set_ylim(-1.4, 6.6)
    title(ax, "图 2  推力分配算法流程", 0, 6.25, fs=14.5)

    W, H = 3.5, 1.0
    # (title, lines, color, bottom_y)
    nodes = [
        ("输入", ["w_d = [f_d, τ_d] ∈ R^6", "（来自几何控制器）"], C_GRAY, 5.0),
        ("Step 1  最小范数分解", ["Moore-Penrose 伪逆解",
                                   "w_d → t_min = [t1 … t4]"], C_HEADER, 3.6),
        ("Step 2  零空间修正", ["万向节锁避免：排斥 β → ±π/2",
                                  "下洗干扰避免：打破推力共线性"], C_GREEN, 2.1),
        ("Step 3  舵机角度反解", ["双分支解 + 最小运动选择",
                                   "多周期缠绕支持 (±2kπ)"], C_PURPLE, 0.6),
        ("输出", ["[Ω0…Ω3, α0,β0,α1,β1,", " α2,β2,α3,β3]", "→ 执行器指令"], C_ORANGE, -0.8),
    ]
    prev_bottom = None
    for t, lines, c, yb in nodes:
        box(ax, -W / 2, yb, W, H, t, lines, fc=c)
        if prev_bottom is not None:
            arrow(ax, 0, prev_bottom, 0, yb + H)
        prev_bottom = yb
    plt.savefig(os.path.join(HERE, "algo_flow.png"), dpi=160,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# 2. System architecture  (软件报告 §2.1)
# ---------------------------------------------------------------------------
def fig_sys_arch():
    fig, ax = plt.subplots(figsize=(13.4, 5.4))
    ax.axis("off")
    ax.set_xlim(-8.2, 10.6)
    ax.set_ylim(-2.6, 2.8)
    title(ax, "图 2.1  PX4 中的 MorphQuad 推力分配架构", 1.2, 2.45, fs=14.5)

    W, H = 2.55, 1.5
    cx = [-6.7, -3.6, -0.3, 3.3, 6.7, 9.3]
    specs = [
        ("Flight Mode\nManager", ["flight_mode_manager"], C_GRAY),
        ("EKF2 /\nState Estimator", ["位姿 / 速度估计"], C_GRAY),
        ("geo_control\n(几何控制器)", ["Manual / Offboard", "Position / Auto"], C_HEADER),
        ("ControlAllocator", ["ActuatorEffectiveness", "MorphQuad (本模块)"], C_GREEN),
        ("PWM Out\nDriver", ["PWM 输出"], C_PURPLE),
        ("ESC / Servo\nHardware", ["4 电机 + 8 舵机"], C_ORANGE),
    ]
    for i, (t, lines, c) in enumerate(specs):
        x = cx[i] - W / 2
        box(ax, x, 0.45, W, H, t, lines, fc=c)

    # horizontal pipeline arrows (centre height 1.2)
    yc = 0.45 + H / 2
    labels = ["", "vehicle_local_\nposition", "vehicle_thrust /\ntorque_setpoint",
              "actuator_\ncontrols_0", "PWM", ""]
    for i in range(len(cx) - 1):
        xa = cx[i] + W / 2
        xb = cx[i + 1] - W / 2
        arrow(ax, xa, yc, xb, yc, label=labels[i + 1])

    # saturation feedback loop (alloc -> geo, curved below the pipeline)
    fb_y = 0.1
    arrow(ax, cx[3] - W / 2, fb_y, cx[2] + W / 2, fb_y, color=C_ORANGE,
          rad=0.18, ls=(0, (4, 3)), label_dy=0.38,
          label="control_allocator_status\n(unallocated)")
    plt.savefig(os.path.join(HERE, "sys_arch.png"), dpi=160,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# 3. Data flow  (软件报告 §2.2)
# ---------------------------------------------------------------------------
def fig_data_flow():
    fig, ax = plt.subplots(figsize=(7.8, 8.6))
    ax.axis("off")
    ax.set_xlim(-3.8, 4.2)
    ax.set_ylim(-1.6, 6.8)
    title(ax, "图 2.2  推力分配数据流", 0, 6.45, fs=14.5)

    W, H = 3.0, 0.92
    nodes = [
        ("vehicle_thrust/torque_setpoint", ["(roll,pitch,yaw) + (x,y,z)"], C_GRAY, 5.2),
        ("control_sp[6]", ["构造 6D 控制向量"], C_GRAY, 4.0),
        ("updateSetpoint()", ["核心分配入口"], C_HEADER, 2.8),
        ("computeThrustVectors()", ["Eq. (12) + (15-16)", "最小范数 + 零空间修正"], C_GREEN, 1.5),
        ("recoverServoAngles()  ×4", ["Eq. (14)", "双分支 + 最小运动"], C_PURPLE, 0.2),
        ("actuator_sp[12]", ["[Ω0..Ω3, α0,β0,α1,β1,", " α2,β2,α3,β3]", "归一化执行器指令"], C_ORANGE, -1.1),
    ]
    chain_right = W / 2  # = 1.5
    prev = None
    for t, lines, c, yb in nodes:
        box(ax, -W / 2, yb, W, H, t, lines, fc=c)
        if prev is not None:
            arrow(ax, 0, prev, 0, yb + H)
        prev = yb

    # saturation feedback note panel on the right
    px, py, pw, ph = 1.8, 0.0, 2.2, 3.2
    ax.add_patch(FancyBboxPatch((px, py), pw, ph,
                 boxstyle="round,pad=0.02,rounding_size=0.10",
                 fc="#FFF4EC", ec=C_ORANGE, lw=1.4, ls=(0, (4, 3)), zorder=2))
    ax.text(px + pw / 2, py + ph - 0.3, "饱和反馈回路", ha="center", va="center",
            fontproperties=FP_B, color=C_ORANGE, fontsize=11, zorder=3)
    ax.text(px + pw / 2, py + ph / 2 - 0.1,
            "geo_control\n→ thrust/torque_sp\n→ ControlAllocator\n→ actuator_sp\n\n回路: control_allocator_\n   status (unallocated)\n→ geo_control\n(抗积分饱和)",
            ha="center", va="center", fontproperties=FP, color="#1f2730",
            fontsize=9, zorder=3)
    arrow(ax, chain_right, 1.5, px, 2.6, color=C_ORANGE, ls=(0, (4, 3)), rad=0.2)
    plt.savefig(os.path.join(HERE, "data_flow.png"), dpi=160,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# 4. Class diagram  (软件报告 §3.1)
# ---------------------------------------------------------------------------
def fig_class_diagram():
    fig, ax = plt.subplots(figsize=(12.5, 6.0))
    ax.axis("off")
    ax.set_xlim(-7.2, 7.2)
    ax.set_ylim(-4.6, 1.4)
    title(ax, "图 3.1  ActuatorEffectivenessMorphQuad 类图", 0, 1.1, fs=14.5)

    BW = 6.6
    # compartments: name, attributes, methods heights
    hn, ha, hm = 0.7, 1.9, 2.7
    total = hn + ha + hm
    top = 0.4

    def class_box(x_center, name, attrs, methods, fill=C_LIGHT, edge=C_HEADER):
        x = x_center - BW / 2
        y0 = top - total
        # outer
        ax.add_patch(FancyBboxPatch((x, y0), BW, total,
                     boxstyle="round,pad=0.01,rounding_size=0.06",
                     fc="white", ec=edge, lw=1.8, zorder=2))
        # name
        ax.add_patch(Rectangle((x, top - hn), BW, hn, fc=fill, ec="none", zorder=2))
        ax.text(x_center, top - hn / 2, name, ha="center", va="center",
                fontproperties=FP_B, color="white" if fill != C_LIGHT else "#1f2730",
                fontsize=12, zorder=3)
        ax.plot([x, x + BW], [top - hn, top - hn], color=edge, lw=1.2, zorder=2)
        # attributes
        ax.text(x + 0.25, top - hn - 0.22, "属性 (Attributes)", fontproperties=FP_B,
                color=edge, fontsize=9.5, zorder=3)
        ax.text(x + 0.25, top - hn - 0.55, "\n".join(attrs), fontproperties=FP,
                color="#1f2730", fontsize=9.3, va="top", zorder=3)
        ax.plot([x, x + BW], [top - hn - ha, top - hn - ha], color=edge, lw=1.0, zorder=2)
        # methods
        ax.text(x + 0.25, top - hn - ha - 0.22, "方法 (Methods)", fontproperties=FP_B,
                color=edge, fontsize=9.5, zorder=3)
        ax.text(x + 0.25, top - hn - ha - 0.55, "\n".join(methods), fontproperties=FP,
                color="#1f2730", fontsize=9.3, va="top", zorder=3)

    class_box(-3.5, "ActuatorEffectiveness  (abstract)",
              ["# NUM_ACTUATORS = 16", "# NUM_AXES = 6",
               "# _flight_phase : FlightPhase", "# _stopped_motors_mask : uint32_t"],
              ["+ name() : const char*  {pure}",
               "+ getEffectivenessMatrix() : bool",
               "+ updateSetpoint()  {default}",
               "+ getUnallocatedControl()  {default}",
               "+ getDesiredAllocationMethod()",
               "+ getNormalizeRPY()"],
              fill=C_GRAY, edge=C_GRAY)

    class_box(3.5, "ActuatorEffectivenessMorphQuad",
              ["- _half_length : float = 0.17",
               "- _half_width  : float = 0.17",
               "- _thrust_coeff: float = 1.0",
               "- _alpha_prev[4], _beta_prev[4]",
               "+ NR = 2",
               "- _unallocated_torque[3], _thrust[3]"],
              ["+ name() : \"MorphQuad\"",
               "+ getEffectivenessMatrix()  [override]",
               "+ updateSetpoint()  [override]",
               "+ getUnallocatedControl()  [override]",
               "- computeThrustVectors()",
               "- recoverServoAngles()",
               "- computeUnallocatedWrench()"],
              fill=C_HEADER, edge=C_HEADER)

    # inheritance arrow (derived -> base)
    ax.add_patch(FancyArrowPatch((0.2, top - total / 2), (-0.2, top - total / 2),
                 arrowstyle="-|>", mutation_scale=18, color=C_LINE, lw=1.8, zorder=1))
    ax.text(0.0, top - total / 2 + 0.35, "extends", ha="center",
            fontproperties=FP, color=C_LINE, fontsize=9.5, zorder=3)
    plt.savefig(os.path.join(HERE, "class_diagram.png"), dpi=160,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# 5. Actuator layout (top view)  (软件报告 §6.1)
# ---------------------------------------------------------------------------
def fig_actuator_layout():
    fig, ax = plt.subplots(figsize=(6.8, 6.8))
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-2.6, 2.6)
    ax.set_ylim(-2.8, 2.6)

    # body axes: x forward (up), y right
    ax.annotate("", xy=(0, 2.2), xytext=(0, -2.2),
                arrowprops=dict(arrowstyle="-|>", color=C_LINE, lw=1.6))
    ax.annotate("", xy=(2.2, 0), xytext=(-2.2, 0),
                arrowprops=dict(arrowstyle="-|>", color=C_LINE, lw=1.6))
    ax.text(0, 2.55, "x  (前 forward)", ha="center", fontproperties=FP_B,
            color=C_HEADER, fontsize=10)
    ax.text(2.35, 0, "y  (右 right)", ha="center", fontproperties=FP_B,
            color=C_GREEN, fontsize=10)

    l, b = 1.3, 1.3  # visual half-arm (body x forward, y right)
    # Motor 0 = right-front (RF); numbering is counter-clockwise: RF -> LF -> LB -> RB
    # Body frame: +x forward, +y right.  In the plot the vertical axis is body x
    # (forward) and the horizontal axis is body y (right), so matplotlib (x,y) = (body_y, body_x).
    rotors = {
        0: (l, b, "FR", "CW"),
        1: (l, -b, "FL", "CCW"),
        2: (-l, -b, "BL", "CW"),
        3: (-l, b, "BR", "CCW"),
    }
    for idx, (bx, by, name, dirn) in rotors.items():
        x, y = by, bx  # convert body frame to matplotlib coordinates
        import math  # local import (used below for spin arrowhead)
        ax.add_patch(Circle((x, y), 0.42, fc=C_LIGHT, ec=C_HEADER, lw=2, zorder=2))
        ax.text(x, y, f"{idx}\n{name}", ha="center", va="center",
                fontproperties=FP_B, color=C_HEADER, fontsize=10, zorder=3)
        # rotation hint arc (direction shown by clockwise/ccw sweep)
        arc = Arc((x, y), 0.95, 0.95, theta1=20, theta2=300,
                  color=C_ORANGE, lw=2.0, zorder=2)
        ax.add_patch(arc)
        # arrowhead at arc end to indicate spin direction
        end = 300 if dirn == "CW" else 60
        ar = math.radians(end)
        hx, hy = x + 0.475 * math.cos(ar), y + 0.475 * math.sin(ar)
        if dirn == "CW":
            dx, dy = -math.sin(ar), math.cos(ar)
        else:
            dx, dy = math.sin(ar), -math.cos(ar)
        ax.annotate("", xy=(hx, hy), xytext=(hx - 0.16 * dx, hy - 0.16 * dy),
                    arrowprops=dict(arrowstyle="-|>", color=C_ORANGE, lw=1.6),
                    zorder=3)
        ax.text(x, y - 0.82, dirn, ha="center", fontproperties=FP_B,
                color=C_ORANGE, fontsize=9, zorder=3)
        # servo pair label (outer α_i + inner β_i) above each rotor
        ax.text(x, y + 0.66, f"α{idx}/β{idx}", ha="center",
                fontproperties=FP_B, color=C_PURPLE, fontsize=8.5, zorder=3)

    ax.text(0, -2.55,
            "执行器编号:  电机 0..3 (Omega 0..3)   舵机 [4]=a0 [5]=b0 [6]=a1 [7]=b1 "
            "[8]=a2 [9]=b2 [10]=a3 [11]=b3",
            ha="center", fontproperties=FP, color="#1f2730", fontsize=8.2)
    title(ax, "图 6.1  MorphQuad 执行器布局 (机体顶视, 电机 0=RF 逆时针编号)",
          0, 2.75, fs=12.5)
    plt.savefig(os.path.join(HERE, "actuator_layout.png"), dpi=160,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------------------
# 6. Thrust vector geometry (3D)  (算法报告 §3.1)
# ---------------------------------------------------------------------------
def fig_thrust_geometry():
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    import numpy as np
    import math

    fig = plt.figure(figsize=(7.0, 6.4))
    ax = fig.add_subplot(111, projection="3d")
    # clean background: hide panes & grid, keep only our arrows/labels
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_visible(False)
    ax.grid(False)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.view_init(elev=18, azim=-52)

    L = 1.0
    # body axes (NED): x forward, y right, z down
    ax.quiver(0, 0, 0, L, 0, 0, color=C_HEADER, lw=2.6, arrow_length_ratio=0.12)
    ax.quiver(0, 0, 0, 0, L, 0, color=C_GREEN, lw=2.6, arrow_length_ratio=0.12)
    ax.quiver(0, 0, 0, 0, 0, L, color=C_ORANGE, lw=2.6, arrow_length_ratio=0.12)
    ax.text(L * 1.14, 0, 0, "x (前)", fontproperties=FP_B, color=C_HEADER, fontsize=10)
    ax.text(0, L * 1.14, 0, "y (右)", fontproperties=FP_B, color=C_GREEN, fontsize=10)
    ax.text(0, 0, L * 1.14, "z (下)", fontproperties=FP_B, color=C_ORANGE, fontsize=10)

    # thrust direction for alpha=35deg, beta=22deg (NED, z down)
    a = np.deg2rad(35)
    be = np.deg2rad(22)
    G = np.array([np.sin(a) * np.cos(be), np.sin(be), -np.cos(a) * np.cos(be)])
    ax.quiver(0, 0, 0, G[0], G[1], G[2], color="#C0392B", lw=3.2,
              arrow_length_ratio=0.12)
    ax.text(G[0] * 1.14, G[1] * 1.14, G[2] * 1.14,
            "G_i(α,β)\n推力方向", fontproperties=FP_B, color="#C0392B", fontsize=9.5)

    # neutral (hover) thrust -z (up)
    ax.quiver(0, 0, 0, 0, 0, -L, color="#7f8c8d", lw=1.8, linestyle=(0, (4, 3)),
              arrow_length_ratio=0.12)
    ax.text(0.10, 0.10, -L * 1.12, "中性 -z (向上)", fontproperties=FP,
            color="#7f8c8d", fontsize=8.5)

    # alpha arc in x-z plane (y = 0): angle from -z toward +x
    r = 0.5
    alpha_val = math.atan2(G[0], -G[2])
    th = np.linspace(0, alpha_val, 26)
    ax.plot(r * np.sin(th), np.zeros_like(th), -r * np.cos(th),
            color="#C0392B", lw=1.5, ls="--")
    ax.text(r * np.sin(alpha_val) * 0.85, 0, -r * np.cos(alpha_val) * 0.85 - 0.04,
            "α", fontproperties=FP_B, color="#1f2730", fontsize=11)

    # beta arc in y-z plane (x = 0): angle from -z toward +y
    beta_val = math.asin(G[1])
    th2 = np.linspace(0, beta_val, 26)
    ax.plot(np.zeros_like(th2), r * np.sin(th2), -r * np.cos(th2),
            color="#C0392B", lw=1.5, ls=":")
    ax.text(0, r * np.sin(beta_val) * 0.85, -r * np.cos(beta_val) * 0.85 - 0.04,
            "β", fontproperties=FP_B, color="#1f2730", fontsize=11)

    ax.set_title("图 3.1  双轴万向节推力方向几何 (NED, z 向下)",
                 fontproperties=FP_B, fontsize=12.5, color="#1f2730", pad=8)
    plt.savefig(os.path.join(HERE, "thrust_geometry.png"), dpi=160,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    fig_algo_flow()
    fig_sys_arch()
    fig_data_flow()
    fig_class_diagram()
    fig_actuator_layout()
    fig_thrust_geometry()
    print("generated:", sorted(f for f in os.listdir(HERE) if f.endswith(".png")))
