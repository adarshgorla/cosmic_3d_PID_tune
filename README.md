# cosmic_3d_PID_tune

# Cosmic3D MK9 - PID Tuning Mathematics & Software Architecture Explanation

This document explains the architecture of the two newly created files ([`pid_config.h`](mk9-pid-tuning-test/pid_config.h) and [`mk9-pid-tuning-test.ino`](mk9-pid-tuning-test/mk9-pid-tuning-test.ino)) and provides a rigorous, step-by-step mathematical derivation of the PID parameters.

---

## 1. Overview of the 2 Code Files

### A. [`pid_config.h`](mk9-pid-tuning-test/pid_config.h) — Central Configuration Header
`pid_config.h` acts as a single source of truth for all physical, hardware, and algorithmic constants.

* **Motor & Driver Hardware Parameters**: Defines supply voltage ($12\text{V}$), rated RPM ($60\text{ RPM}$), ESP32-S3 GPIO pin assignments for all 4 motors, and the L293D stiction threshold (`PWM_MIN_FLOOR = 60`).
* **Mathematically Derived PID Defaults**: Stores default arrays `DEFAULT_KP`, `DEFAULT_KI`, and `DEFAULT_KD` for all axes.
* **AS5600 & TCA9548A Multiplexer Config**: Sets I2C pins (SDA 8, SCL 9), addresses (`0x36` for AS5600, `0x70` for TCA9548A), and $4096$ counts/revolution encoder resolution.
* **Kinematics Geometry**: Defines Delta printer tower coordinates ($A, B, C$), pulley radius ($9.5\text{ mm}$), and counts per millimeter ($\approx 68.61\text{ counts/mm}$).
* **SD Logging Specs**: Defines the SD Card SPI pins (SCK 12, MISO 13, MOSI 11, CS 10), log path (`/pid_tune_log.csv`), and sampling rate ($20\text{ ms}$).

---

### B. [`mk9-pid-tuning-test.ino`](mk9-pid-tuning-test/mk9-pid-tuning-test.ino) — Step-by-Step Test Engine
This file implements the closed-loop control system, step-by-step test routines, and diagnostic telemetry.

* **Closed-Loop Engine**:
  - Reads magnetic rotary encoders over I2C through the TCA9548A multiplexer.
  - Converts target Cartesian coordinates $(X, Y, Z)$ to cable lengths using **Inverse Kinematics (IK)**.
  - Computes discrete PID control outputs: $\text{Output} = P + I + D$.
  - Drives motors via PWM on the L293D H-Bridge.
* **Step-by-Step Test Engine**:
  - Implements state machine tests for single axes ($X, Y, Z$), dual axes ($XY, YZ, XZ$), and full 3D moves ($XYZ$).
  - Moves $+20\text{mm}$ out, holds position for $1.5\text{ seconds}$, and returns to origin $(0,0,0)$.
* **Interactive Serial Console**:
  - Accepts user commands over Serial (`1` to `7`, `stop`).
  - Allows **live PID tuning** (e.g. typing `p0=0.85` or `d1=0.04`) without re-flashing.
* **Automatic SD Log Writer**:
  - Telemetry is recorded every $20\text{ ms}$ to `/pid_tune_log.csv` on the SD Card.
  - Logs timestamps, setpoints, actual encoder counts, errors, and fault alerts (e.g. encoder jumps or magnet missing).

---

## 2. Mathematical Derivation of PID Values

### A. Given Original Inputs (Hardware Specifications)

| Parameter | Notation | Given Value | Description |
| :--- | :--- | :--- | :--- |
| **Supply Voltage** | $V_{in}$ | $12.0\text{ V}$ | Power supply voltage to L293D VCC2 |
| **No-Load Motor Speed** | $N_{max}$ | $60\text{ RPM}$ | Max rated rotational speed at 12V |
| **H-Bridge Driver** | — | **L293D** | Dual Bipolar Darlington Driver IC |
| **Encoder Resolution** | $C$ | $4096\text{ counts/rev}$ | 12-bit absolute resolution of AS5600 |
| **PWM Resolution** | $u_{max}$ | $255$ | 8-bit PWM timer ($0 \text{ to } 255$) |
| **Pulley Radius** | $r$ | $9.5\text{ mm}$ | Radius of cable spool |

---

### B. Explicit Assumptions Made

1. **L293D Saturation Voltage Drop ($V_{drop}$)**:
   The L293D uses Darlington transistor output pairs. Each channel drops $\approx 1.4\text{V}$ per transistor stage, creating a total H-Bridge voltage loss of:
   $$V_{drop} \approx 2.8\text{ V}$$

2. **Mechanical Time Constant ($\tau_m$)**:
   The mechanical time constant is the time required for the motor and cable load to reach $63.2\%$ of top speed from rest. For a $60\text{ RPM}$ small geared DC motor under light cable load:
   $$\tau_m = 0.08\text{ seconds } (80\text{ ms})$$

3. **Target Closed-Loop Response (Critical Damping)**:
   For 3D printing accuracy, the system must be **Critically Damped ($\zeta = 1.0$)** to guarantee zero position overshoot.
   $$\zeta = 1.0$$

4. **Target Settling Time ($T_s$) & Natural Frequency ($\omega_n$)**:
   We target a position settling time of $T_s = 0.4\text{ seconds}$ to within $2\%$ of target:
   $$\omega_n \approx \frac{4}{T_s} = \frac{4}{0.4} = 10\text{ rad/sec}$$

5. **Pole-Zero Cancellation**:
   The derivative term $K_d$ is chosen to cancel the slow mechanical motor pole at $s = -1/\tau_m$:
   $$T_d = \frac{K_d}{K_p} = \tau_m = 0.08\text{ s}$$

6. **Integral Reset Time ($T_i$)**:
   The integral term $K_i$ is set to eliminate steady-state error without causing low-frequency hunting:
   $$T_i = \frac{K_p}{K_i} = T_s = 0.4\text{ s}$$

---

### C. Step-by-Step Mathematical Equations & Derived Values

#### Step 1: Effective Motor Terminal Voltage
$$V_{motor} = V_{in} - V_{drop} = 12.0\text{V} - 2.8\text{V} = 9.2\text{ V}$$

#### Step 2: Adjusted Maximum Speed ($N_{adj}$)
$$N_{adj} = N_{max} \times \frac{V_{motor}}{V_{in}} = 60\text{ RPM} \times \frac{9.2\text{V}}{12.0\text{V}} = 46.0\text{ RPM}$$

Converting to revolutions per second:
$$\dot{\theta}_{max} = \frac{46.0\text{ RPM}}{60\text{ sec/min}} = 0.7667\text{ rev/sec}$$

#### Step 3: Maximum Velocity in Encoder Counts ($\omega_{max}$)
$$\omega_{max} = \dot{\theta}_{max} \times C = 0.7667\text{ rev/sec} \times 4096\text{ counts/rev} = 3140.3\text{ counts/sec}$$

#### Step 4: Plant Open-Loop Velocity DC Gain ($K_{plant}$)
The plant gain relates output velocity (in counts/sec) per unit of PWM command $u \in [0, 255]$:
$$K_{plant} = \frac{\omega_{max}}{u_{max}} = \frac{3140.3\text{ counts/sec}}{255\text{ PWM}} = 12.3137\frac{\text{counts/sec}}{\text{PWM unit}}$$

#### Step 5: Laplace Transfer Function of Motor Plant $G(s)$
Relating output position $\Theta(s)$ to input PWM command $U(s)$:
$$G(s) = \frac{\Theta(s)}{U(s)} = \frac{K_{plant}}{s(\tau_m s + 1)} = \frac{12.3137}{s(0.08 s + 1)}$$

#### Step 6: Calculation of Proportional Gain ($K_p$)
Applying pole placement for a critically damped closed-loop system:
$$K_p = \frac{\omega_n^2 \cdot \tau_m}{K_{plant}} = \frac{(10)^2 \cdot 0.08}{12.3137} = \frac{8.0}{12.3137} \approx \mathbf{0.65}$$

#### Step 7: Calculation of Derivative Gain ($K_d$)
Using the pole cancellation relationship $K_d = K_p \times \tau_m$:
$$K_d = 0.65 \times 0.08 = \mathbf{0.052}$$

#### Step 8: Calculation of Integral Gain ($K_i$)
Using integral time $T_i = 0.4\text{s}$:
$$K_i = \frac{K_p}{T_i} = \frac{0.65}{0.4} = \mathbf{1.625}$$

#### Step 9: Stiction PWM Threshold Calculation
$$PWM_{floor} = \frac{V_{drop}}{V_{in}} \times u_{max} = \frac{2.8\text{V}}{12.0\text{V}} \times 255 = 59.5 \approx \mathbf{60}$$

---

### D. Summary Table of Values

| Variable | Type | Value | Formula / Source |
| :--- | :--- | :--- | :--- |
| **$V_{in}$** | Original Given | $12.0\text{ V}$ | Hardware spec |
| **$N_{max}$** | Original Given | $60\text{ RPM}$ | Hardware spec |
| **Driver** | Original Given | **L293D** | Hardware spec |
| **$C$** | Original Given | $4096\text{ counts/rev}$ | AS5600 Datasheet |
| **$r$** | Original Given | $9.5\text{ mm}$ | Mechanics spec |
| **$V_{drop}$** | Assumption | $2.8\text{ V}$ | L293D Darlington Saturation |
| **$\tau_m$** | Assumption | $0.08\text{ s}$ | Mechanical Time Constant |
| **$\zeta$** | Target | $1.0$ | Critical Damping |
| **$\omega_n$** | Target | $10\text{ rad/s}$ | Natural Frequency |
| **$K_{plant}$** | Derived | $12.3137\text{ counts/(s}\cdot\text{PWM)}$ | $K_{plant} = \omega_{max} / 255$ |
| **$K_p$** | Derived | **$0.65$** | $K_p = \omega_n^2 \tau_m / K_{plant}$ |
| **$K_i$** | Derived | **$1.625$** | $K_i = K_p / 0.4$ |
| **$K_d$** | Derived | **$0.052$** | $K_d = K_p \times \tau_m$ |
| **`PWM_MIN_FLOOR`**| Derived | **$60$** | $PWM_{floor} = (V_{drop}/V_{in}) \times 255$ |
