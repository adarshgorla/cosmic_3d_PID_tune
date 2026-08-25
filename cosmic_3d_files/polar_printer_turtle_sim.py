#!/usr/bin/env python3
"""
=============================================================================
COSMIC POLAR 400 — SLOW-MOTION STEP-BY-STEP AXIS INSPECTOR & SIMULATOR
=============================================================================
This simulator models the exact edge-mounted radial arm mechanism:
  - Bed Pivot Base: Mounted at edge pillar (X = -200 mm).
  - Radial Arm: 200 mm length (radius of bed).
  - Features Slow-Motion Playback and Step-by-Step Axis Movement Telemetry:
      1. R Axis Bed Rotation angle and AS5600 encoder counts.
      2. Theta Axis MG945 Servo position (0 to 180 deg).
      3. Z Axis height and motor counts (100 counts/mm).
"""

import math
import time
import turtle

BED_RADIUS_MM = 200.0
BED_DIAMETER_MM = 401.0
PULLEY_DIAMETER_MM = 19.0
AS5600_COUNTS_PER_REV = 4096.0
Z_COUNTS_PER_MM = 100.0

SERVO_MIN_DEG = 0
SERVO_MAX_DEG = 180

SCALE = 1.35  # Screen scale


class PolarKinematicsEngine:

    def __init__(self):
        self.last_bed_angle_deg = 0.0
        self.accumulated_bed_angle_deg = 0.0

    def bed_angle_to_counts(self, bed_deg):
        motor_revs = (bed_deg / 360.0) * (
            BED_DIAMETER_MM / PULLEY_DIAMETER_MM
        )
        return int(round(motor_revs * AS5600_COUNTS_PER_REV))

    def radial_mm_to_servo_deg(self, radial_mm):
        radial_mm = max(0.0, min(BED_RADIUS_MM, radial_mm))
        scale = (SERVO_MAX_DEG - SERVO_MIN_DEG) / BED_RADIUS_MM
        return int(round(SERVO_MIN_DEG + radial_mm * scale))

    def z_mm_to_counts(self, z_mm):
        return int(round(z_mm * Z_COUNTS_PER_MM))

    def calculate_ik(self, x, y, z):
        radial_r = min(BED_RADIUS_MM, math.sqrt(x * x + y * y))

        # 1. Arm swing angle (0° at center r=0, 60° at bed edge r=200)
        arm_swing_rad = 0.0 if radial_r == 0 else 2.0 * math.asin(radial_r / (2.0 * BED_RADIUS_MM))
        arm_swing_deg = math.degrees(arm_swing_rad)

        # 2. Nozzle world tip position (pivot at -200, 0)
        wx = -BED_RADIUS_MM + BED_RADIUS_MM * math.cos(arm_swing_rad)
        wy = -BED_RADIUS_MM * math.sin(arm_swing_rad)

        # 3. Target bed angle
        th_target_rad = math.atan2(y, x)
        th_nozzle_world_rad = math.atan2(wy, wx)
        target_bed_deg = math.degrees(th_target_rad - th_nozzle_world_rad)

        # 4. Multi-turn angle tracking
        delta_angle = target_bed_deg - self.last_bed_angle_deg
        while delta_angle > 180.0:
            delta_angle -= 360.0
        while delta_angle < -180.0:
            delta_angle += 360.0

        self.accumulated_bed_angle_deg += delta_angle
        self.last_bed_angle_deg = target_bed_deg

        r_counts = self.bed_angle_to_counts(self.accumulated_bed_angle_deg)
        theta_deg = int(round((arm_swing_deg / 60.0) * 180.0))
        z_counts = self.z_mm_to_counts(z)

        return {
            "r_radius_mm": radial_r,
            "bed_angle_deg": target_bed_deg,
            "accum_bed_deg": self.accumulated_bed_angle_deg,
            "r_counts": r_counts,
            "arm_swing_deg": arm_swing_deg,
            "theta_servo_deg": theta_deg,
            "z_counts": z_counts,
        }, None


def circle_arc_coordinates(center, radius, start_point, end_point, is_cw, n=30):
    """Generate (x, y) points along a circular arc for G02 (CW) or G03 (CCW)."""
    cx, cy = center
    x1, y1 = start_point
    x2, y2 = end_point

    start_angle = math.atan2(y1 - cy, x1 - cx)
    end_angle = math.atan2(y2 - cy, x2 - cx)

    if is_cw:
        if end_angle > start_angle:
            end_angle -= 2 * math.pi
    else:
        if end_angle < start_angle:
            end_angle += 2 * math.pi

    points = []
    for i in range(n + 1):
        t = start_angle + (float(i) / float(n)) * (end_angle - start_angle)
        x = cx + radius * math.cos(t)
        y = cy + radius * math.sin(t)
        points.append((x, y))
    return points


def parse_gcode(gcode_input):
    """
    User-specified G-Code Interpreter.
    Supports: G0, G1, G28 (Homing), G02 (CW Arc), G03 (CCW Arc), G92 (E offset reset).
    Accepts file path or raw G-code string.
    Returns 3D toolpath tuples (x, y, z) starting at bed center (0,0).
    """
    if isinstance(gcode_input, str) and (gcode_input.endswith('.txt') or gcode_input.endswith('.gcode')):
        try:
            with open(gcode_input, "r") as f:
                lines = f.readlines()
        except Exception:
            lines = gcode_input.splitlines()
    elif isinstance(gcode_input, str):
        lines = gcode_input.splitlines()
    else:
        lines = gcode_input

    gx, gy, gz = [0.0], [0.0], [5.0]
    e_val = [0.0]
    feed = [0.0]
    current_e_offset = 0.0
    last_raw_e = 0.0

    raw_pts = [(0.0, 0.0, 5.0)]  # Start at bed center

    for line in lines:
        line = line.strip()
        if ';' in line:
            line = line.split(';')[0].strip()
        line = " ".join(line.split())
        if not line:
            continue

        raw_line_str = line
        tokens = line.split(' ')

        if "G28" in line:
            gx.append(0.0)
            gy.append(0.0)
            gz.append(0.0)
            raw_pts.append((0.0, 0.0, 5.0))
            continue

        if line.startswith("G1") or line.startswith("G0"):
            has_move = False
            for tok in tokens[1:]:
                if not tok:
                    continue
                code = tok[0].upper()
                try:
                    val = float(tok[1:])
                except ValueError:
                    continue

                if code == "X":
                    gx.append(val)
                    has_move = True
                elif code == "Y":
                    gy.append(val)
                    has_move = True
                elif code == "Z":
                    gz.append(val)
                    has_move = True
                elif code == "E":
                    e_val.append(val)
                elif code == "F":
                    feed.append(val)

            if has_move:
                raw_pts.append((gx[-1], gy[-1], gz[-1]))

        elif line.startswith("G02") or line.startswith("G03"):
            is_cw = line.startswith("G02")
            target_x, target_y = gx[-1], gy[-1]
            i_val, j_val = 0.0, 0.0

            for tok in tokens[1:]:
                if not tok:
                    continue
                code = tok[0].upper()
                try:
                    val = float(tok[1:])
                except ValueError:
                    continue

                if code == "X":
                    target_x = val
                elif code == "Y":
                    target_y = val
                elif code == "I":
                    i_val = val
                elif code == "J":
                    j_val = val

            start_point = (gx[-1], gy[-1])
            center = (start_point[0] + i_val, start_point[1] + j_val)
            end_point = (target_x, target_y)
            radius = math.hypot(i_val, j_val)

            if radius > 0:
                arc_pts = circle_arc_coordinates(center, radius, start_point, end_point, is_cw, n=30)
                for px, py in arc_pts:
                    gx.append(px)
                    gy.append(py)
                    raw_pts.append((px, py, gz[-1]))

        elif "G92" in line:
            current_e_offset = 0.0
            last_raw_e = 0.0

    if len(raw_pts) <= 1:
        return []

    # Auto-center coordinates around (0,0) bed center
    xs = [p[0] for p in raw_pts]
    ys = [p[1] for p in raw_pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    center_x = (min_x + max_x) / 2.0 if (min_x >= 0 and max_x > 40) else 0.0
    center_y = (min_y + max_y) / 2.0 if (min_y >= 0 and max_y > 40) else 0.0

    return [(p[0] - center_x, p[1] - center_y, p[2]) for p in raw_pts]


def generate_star_toolpath(
    center_x=0, center_y=0, outer_r=140, inner_r=60, points=5, z=4.0
):
    path = [(center_x, center_y, z)]  # Start at bed center (0,0)
    total_steps = points * 2
    for i in range(total_steps + 1):
        angle = i * (2 * math.pi / total_steps) - math.pi / 2
        r = outer_r if i % 2 == 0 else inner_r
        path.append(
            (center_x + r * math.cos(angle), center_y + r * math.sin(angle), z)
        )
    return path


class PolarPrinterSlowMoVisualizer:

    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(width=920, height=820)
        self.screen.title(
            "Cosmic Polar 400 — Slow-Motion Axis Inspector & Visualizer"
        )
        self.screen.bgcolor("#0f172a")
        self.screen.tracer(0)

        # Turtles
        self.bed_turtle = turtle.Turtle()    # static bed outline
        self.pillar_turtle = turtle.Turtle() # pillar dot
        self.arm_turtle = turtle.Turtle()    # fixed arm
        self.nozzle_turtle = turtle.Turtle() # fixed nozzle dot
        self.draw_turtle = turtle.Turtle()   # print trace (world-space)
        self.text_turtle = turtle.Turtle()   # HUD telemetry
        self.bed_rot_turtle = turtle.Turtle()  # bed rotation indicator

        for t in [
            self.bed_turtle,
            self.pillar_turtle,
            self.arm_turtle,
            self.nozzle_turtle,
            self.draw_turtle,
            self.text_turtle,
            self.bed_rot_turtle,
        ]:
            t.hideturtle()
            t.speed(0)

        self.kinematics = PolarKinematicsEngine()
        self.draw_bed_grid()

    def draw_bed_grid(self):
        self.bed_turtle.clear()

        # Outer 401mm circular bed border
        self.bed_turtle.penup()
        self.bed_turtle.goto(0, -BED_RADIUS_MM * SCALE)
        self.bed_turtle.pendown()
        self.bed_turtle.color("#334155")
        self.bed_turtle.width(3)
        self.bed_turtle.circle(BED_RADIUS_MM * SCALE)

        # Concentric distance rings
        for r_mm in [50, 100, 150, 200]:
            self.bed_turtle.penup()
            self.bed_turtle.goto(0, -r_mm * SCALE)
            self.bed_turtle.pendown()
            self.bed_turtle.color("#1e293b")
            self.bed_turtle.width(1)
            self.bed_turtle.circle(r_mm * SCALE)

        # Center reference point
        self.bed_turtle.penup()
        self.bed_turtle.goto(0, 0)
        self.bed_turtle.dot(6, "#f43f5e")

        # Fixed Z Pillar Column at EDGE of bed (X = -200 mm)
        self.pillar_turtle.clear()
        self.pillar_turtle.penup()
        edge_x = -BED_RADIUS_MM * SCALE
        self.pillar_turtle.goto(edge_x, 0)
        self.pillar_turtle.dot(18, "#0284c7")

    def update_telemetry(
        self,
        x,
        y,
        z,
        ik_data,
        step_idx,
        total_steps,
        est_time_sec,
        shape_name,
    ):
        self.text_turtle.clear()
        self.text_turtle.penup()
        self.text_turtle.color("#f8fafc")

        # Top Title Banner
        self.text_turtle.goto(-430, 370)
        self.text_turtle.write(
            "🚀 COSMIC POLAR 400 — SLOW-MOTION AXIS INSPECTOR",
            font=("Arial", 15, "bold"),
        )

        # Telemetry Box Left
        self.text_turtle.goto(-430, 310)
        info_left = (
            f"Shape: {shape_name}\n"
            f"Step: {step_idx}/{total_steps} ({int(step_idx/total_steps*100)}%)\n"
            f"Cartesian Target: X={x:.1f} mm, Y={y:.1f} mm, Z={z:.1f} mm\n"
            f"Radius R: {ik_data['r_radius_mm']:.1f} mm | Bed Angle: {ik_data['bed_angle_deg']:.1f}°"
        )
        self.text_turtle.write(info_left, font=("Courier", 10, "normal"))

        # Hardware Actuator Telemetry Right
        self.text_turtle.goto(80, 310)
        info_right = (
            f"⚡ ACTUATOR MOVEMENTS:\n"
            f"  • R Bed Motor:  {ik_data['r_counts']} counts\n"
            f"  • Theta MG945: {ik_data['theta_servo_deg']}° Servo Angle\n"
            f"  • Z Lift Motor: {ik_data['z_counts']} counts ({z:.1f}mm)\n"
            f"⏱ Duration:    {est_time_sec:.2f} seconds"
        )
        self.text_turtle.write(info_right, font=("Courier", 10, "bold"))

    def render_arm_and_nozzle(self, arm_angle_deg, bed_angle_deg):
        """
        Draw the rigid swinging arm with nozzle FIXED at the arm tip.
        Polar printer mechanics (Swinging Arm):
          - Pivot: Edge pillar at (-200, 0).
          - Arm: Rigid boom of length 200 mm (radius of bed).
          - Nozzle: Fixed permanently at the tip of the swinging arm.
          - Arm Motion: Pivots in an arc around pillar (-200, 0).
          - Bed Motion: Rotates around center (0,0).
        """
        edge_x = -BED_RADIUS_MM * SCALE   # pillar screen X
        edge_y = 0.0

        # Calculate tip position of swinging arm (arm_length = 200mm)
        arm_rad = math.radians(arm_angle_deg)
        arm_len = BED_RADIUS_MM * SCALE
        tip_x = edge_x + arm_len * math.cos(arm_rad)
        tip_y = edge_y + arm_len * math.sin(arm_rad)

        # Draw swinging arm boom from edge pillar to tip
        self.arm_turtle.clear()
        self.arm_turtle.penup()
        self.arm_turtle.goto(edge_x, edge_y)
        self.arm_turtle.pendown()
        self.arm_turtle.color("#38bdf8")
        self.arm_turtle.width(4)
        self.arm_turtle.goto(tip_x, tip_y)

        # Draw nozzle FIXED at arm tip (never slides along arm)
        self.nozzle_turtle.clear()
        self.nozzle_turtle.penup()
        self.nozzle_turtle.goto(tip_x, tip_y + 2)
        self.nozzle_turtle.dot(14, "#f59e0b")   # orange hotend
        self.nozzle_turtle.goto(tip_x, tip_y - 8)
        self.nozzle_turtle.dot(6, "#ff3300")    # red nozzle tip

        # Bed rotation indicator: rotating spoke from center shows bed angle
        self.bed_rot_turtle.clear()
        spoke_rad = math.radians(bed_angle_deg)
        spoke_len = BED_RADIUS_MM * SCALE
        spoke_ex = spoke_len * math.cos(spoke_rad)
        spoke_ey = spoke_len * math.sin(spoke_rad)
        self.bed_rot_turtle.penup()
        self.bed_rot_turtle.goto(0, 0)
        self.bed_rot_turtle.pendown()
        self.bed_rot_turtle.color("#f43f5e")
        self.bed_rot_turtle.width(2)
        self.bed_rot_turtle.goto(spoke_ex, spoke_ey)

        return tip_x, tip_y

    def simulate_toolpath_slow_mo(
        self,
        toolpath,
        shape_name="Custom Toolpath",
        step_delay_sec=0.3,
        feedrate_mm_s=30.0,
    ):
        if not toolpath:
            return

        self.draw_turtle.clear()
        self.draw_turtle.penup()
        self.draw_turtle.color("#f59e0b")
        self.draw_turtle.width(3)

        total_steps = len(toolpath)
        total_distance = sum(
            math.sqrt(
                (toolpath[i][0] - toolpath[i - 1][0]) ** 2
                + (toolpath[i][1] - toolpath[i - 1][1]) ** 2
                + (toolpath[i][2] - toolpath[i - 1][2]) ** 2
            )
            for i in range(1, total_steps)
        )
        est_time_sec = total_distance / feedrate_mm_s

        print("\n" + "═" * 78)
        print(f" 🖨 COSMIC POLAR 400 — KINEMATICS TELEMETRY & AXIS LOGS: {shape_name}")
        print(f" Machine Config: Bed Radius = {BED_RADIUS_MM}mm | Pivot = (-200, 0) | AS5600 = {AS5600_COUNTS_PER_REV} cnts/rev")
        print(f" Kinematics:     Fixed Nozzle Tip | Swinging Boom (0°-60°) | Turntable Bed (0°-360°)")
        print("═" * 78)
        print(f" {'STEP':<6} | {'TARGET (X,Y,Z)':<18} | {'RADIAL':<8} | {'ARM SWING (SERVO)':<19} | {'BED ANGLE (COUNTS)':<21} | {'NOZZLE TIP (X,Y)':<16}")
        print("─" * 100)

        first = True
        for idx, (x, y, z) in enumerate(toolpath, start=1):
            ik_data, err = self.kinematics.calculate_ik(x, y, z)
            if err:
                print(f" ⚠️ Step {idx:02d}: SKIPPED ({err})")
                continue

            arm_swing_deg = ik_data["arm_swing_deg"]
            arm_servo_deg = ik_data["theta_servo_deg"]
            bed_angle = ik_data["bed_angle_deg"]
            r_mm = ik_data["r_radius_mm"]

            # Tip position of swinging arm (swing angle 0° to 60°)
            arm_rad = math.radians(arm_swing_deg)
            edge_x_mm = -BED_RADIUS_MM
            screen_x = (edge_x_mm + BED_RADIUS_MM * math.cos(arm_rad)) * SCALE
            screen_y = (BED_RADIUS_MM * math.sin(arm_rad)) * SCALE

            if first:
                self.draw_turtle.penup()
                self.draw_turtle.goto(screen_x, screen_y)
                self.draw_turtle.pendown()
                first = False
            else:
                self.draw_turtle.goto(screen_x, screen_y)

            self.render_arm_and_nozzle(arm_swing_deg, bed_angle)
            self.update_telemetry(
                x, y, z, ik_data, idx, total_steps, est_time_sec, shape_name
            )

            tip_world_x = edge_x_mm + BED_RADIUS_MM * math.cos(arm_rad)
            tip_world_y = BED_RADIUS_MM * math.sin(arm_rad)

            print(
                f" #{idx:03d}/{total_steps:03d} | ({x:6.1f}, {y:6.1f}, {z:4.1f}) | {r_mm:6.1f}mm | "
                f"{arm_swing_deg:5.1f}° ({arm_servo_deg:3d}° servo) | "
                f"{bed_angle:7.1f}° ({ik_data['r_counts']:7d} cts) | "
                f"({tip_world_x:6.1f}, {tip_world_y:6.1f})mm"
            )

            self.screen.update()
            time.sleep(step_delay_sec)

        print("─" * 100)
        print(f" ✅ Execution Complete: {shape_name} ({total_steps} steps executed successfully)\n")
        time.sleep(1)


if __name__ == "__main__":
    print("\n🚀 Initializing Cosmic Polar 400 Simulation & Kinematic Logs...")
    sim = PolarPrinterSlowMoVisualizer()

    # 1. Star Toolpath in Slow Motion
    star_path = generate_star_toolpath(0, 0, 140, 60, 5, 4.0)
    sim.simulate_toolpath_slow_mo(
        star_path, shape_name="5-Point Star Toolpath", step_delay_sec=0.20
    )

    # 2. Custom G-Code Sample parsing log demonstration
    sample_gcode = """
    ; Sample G-Code Demo
    G28 ; Home axes
    G0 X0 Y0 Z5 ; Center start
    G1 X50 Y0 Z5 E1.0
    G02 X0 Y50 I-25 J25 ; Clockwise Arc
    G1 X-50 Y0 Z5 E2.0
    G03 X0 Y-50 I25 J-25 ; Counter-clockwise Arc
    """
    print("\n📄 Parsing Sample G-Code with Arcs (G02/G03) & Homing (G28)...")
    gcode_path = parse_gcode(sample_gcode)
    print(f"   Parsed {len(gcode_path)} waypoints from G-Code.\n")

    sim.simulate_toolpath_slow_mo(
        gcode_path, shape_name="Custom G-Code Arc Print", step_delay_sec=0.20
    )

    sim.screen.exitonclick()
