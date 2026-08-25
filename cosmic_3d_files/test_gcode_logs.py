#!/usr/bin/env python3
"""
=============================================================================
COSMIC POLAR 400 — KINEMATICS & G-CODE TERMINAL TELEMETRY INSPECTOR
=============================================================================
"""
import math
from polar_printer_turtle_sim import PolarKinematicsEngine, parse_gcode, generate_star_toolpath

def run_terminal_telemetry_demo():
    engine = PolarKinematicsEngine()
    
    print("\n" + "═" * 84)
    print("  🖨 COSMIC POLAR 400 — SWING-ARM KINEMATICS TELEMETRY LOGS")
    print("  Machine Config: Bed Radius = 200.0mm | Pivot = (-200, 0) | AS5600 = 4096 cnts/rev")
    print("  Kinematics:     Fixed Nozzle Tip | Swinging Boom (0°-60°) | Turntable Bed (0°-360°)")
    print("═" * 84)

    # Demo G-Code
    gcode_sample = """
    ; Cosmic Polar 400 Test G-Code
    G28 ; Home axes to Bed Center (0,0)
    G0 X0 Y0 Z5 ; Center home
    G1 X50 Y0 Z5 E1.0 ; Linear outward move
    G02 X0 Y50 I-25 J25 E2.0 ; Clockwise Circular Arc (G02)
    G1 X-50 Y0 Z5 E3.0 ; Linear move
    G03 X0 Y-50 I25 J-25 E4.0 ; Counter-Clockwise Arc (G03)
    """

    print("📄 Parsing G-Code input (G0, G1, G28, G02, G03, G92)...")
    waypoints = parse_gcode(gcode_sample)
    print(f"✅ Parsed {len(waypoints)} interpolated waypoints.\n")

    print("┌" + "─" * 82 + "┐")
    print(f"│ {'STEP':<6} │ {'TARGET (X,Y,Z)':<16} │ {'RADIAL':<7} │ {'ARM SWING (SERVO)':<18} │ {'BED ANGLE (COUNTS)':<20} │")
    print("├" + "─" * 82 + "┤")

    for idx, (x, y, z) in enumerate(waypoints, start=1):
        ik_data, err = engine.calculate_ik(x, y, z)
        if err:
            print(f"│ #{idx:03d}   │ SKIPPED ({err})")
            continue

        r_mm = ik_data["r_radius_mm"]
        arm_swing = ik_data["arm_swing_deg"]
        servo = ik_data["theta_servo_deg"]
        bed_deg = ik_data["bed_angle_deg"]
        r_counts = ik_data["r_counts"]

        print(
            f"│ #{idx:03d}   │ ({x:6.1f}, {y:6.1f}, {z:3.1f}) │ {r_mm:5.1f}mm │ "
            f"{arm_swing:5.1f}° ({servo:3d}° servo)  │ "
            f"{bed_deg:7.1f}° ({r_counts:7d} cts) │"
        )

    print("└" + "─" * 82 + "┘\n")
    print("✨ Kinematic calculation completed successfully.\n")

if __name__ == "__main__":
    run_terminal_telemetry_demo()
