# JR Personal Type Simulator — Interactive 3D Cab Operation and Precision Stop Training Framework

This project introduces a lightweight web-based simulation framework for **JR Series Personal Type Simulators (PTS)** — compact, function-focused train driving systems that replicate the essential control environment of JR East and JR West Electric Multiple Units (EMUs).  

Unlike full-cab or desk-type simulators, this **PTS platform is specifically designed for training essential driving skills**, allowing operators to repeatedly practice braking control and stop-point accuracy under a variety of simulated conditions. The system supports both **manual stop control** and **TASC (Train Automatic Stop Control)**-assisted automatic stopping, enabling side-by-side comparison between human and automated performance.

Powered by **Three.js-based 3D visualization**, the simulator provides a minimalistic yet immersive representation of the driver’s point of view and external track environment. Real-time rendering of velocity–distance trajectories and braking curves offers immediate, intuitive feedback for both trainee drivers and research users.  

By emulating the authentic physical behavior of EMUs, the platform enables comprehensive **repetitive stop training**, **scenario-based evaluation**, and **quantitative performance scoring/feedback** across varying track conditions.

---

## 🚆 Application Scenarios

- **Operator Training:** Repetitive personal-level stopping practice using simplified simulator hardware  
- **Educational Demonstrations:** Visualization of train dynamics and signal-response logic  
- **Research & Development:** Testing human–automation interaction (TASC) and control adaptation strategies  

---

## 🔭 Future Development

Planned upgrades include integration with advanced railway control frameworks and signals such as:

- **ATC (Automatic Train Control)**  
- **ATP (Automatic Train Protection)**  
- **ATS (Automatic Train Stop)**  

---

## ⚙️ Tech Stack
- **Backend**: Python (FastAPI, WebSocket)
- **Frontend**: HTML, CSS, JavaScript (Three.JS for BVE-style graphic)
- **Deployment**: AWS EC2 (Amazon Linux), Nginx, Route53, 내도메인.한국

---

## 🧠 Why TASC?
- **Reduces driver workload & fatigue**: automatically executes initial, stair-step braking and relaxing → reduces cognitive load and control fatigue.
- **Higher stopping accuracy & consistent ride comfort**: deadband and hold time prevent unnecessary notch hunting; always produces a **similar braking profile**.
- **Training efficiency**:
  - **Brake navigation canvas** helps learners understand "where curves meet the red line."
  - Quantitative feedback and scoring allows self-assessment of braking habits.
- **Educational/verification use**: various scenarios (distance/slope/speed/friction), simultaneous evaluation of ride comfort (jerk) and stopping precision.
- **Low-cost and low-risk**: cheaper than ATO systems; TASC allows manual override if equipment malfunctions.

---

## 🧪 How TASC Works (Algorithm Overview)

- **TASC Activation**:  
  - Activating TASC before starting sets it to **armed**.  
  - In armed state, using manual brakes (B1/B2) reduces speed to ≤ 60 km/h before 150m mark.  
  - At the **150m P3 trackside signal**, TASC switches to **active**.  
  - TASC then **automatically controls the brake lever** to assist precision stop (±35cm),  
    but any manual input immediately switches it **OFF** (in reality, TASC applies the higher notch of calculated or manual lever).

- **Initial Brake (B1/B2)**:
  - Optional; in manual operation contributes to scoring.

- **N-Step Build (Stair-Step Braking)**:
  - If current notch stop distance `s_cur` > `(rem - deadband)`, increase notch (↑).  
  - Switch to relax phase when sufficient.

- **Stair-Step Relax**:
  - If `s_dn` ≤ `(rem + deadband)`, decrease notch (↓).  
  - Repeat until finishing at **B1**.

- **Stabilization**:
  - Deadband (±m) and minimum hold time applied → prevent hunting and jerkiness.  
  - Soft linear B1 stop below 5 km/h → improved ride comfort.

---

## 🔧 Project Structure
├── scenario.json # Scenario (distance L, initial speed v0, slope, friction, etc.)
├── vehicle.json # Vehicle specs (mass, notch_accels, time constants, etc.)
├── server.py # FastAPI + WebSocket server, including TASC logic
└── static/
      ├── index.html # UI (HUD/Overlay/TASC switch/animations)
      └── xxxxx.json # Vehicle name display data


---

## 🚀 Run Locally
1. Open in browser  
[Try it live](https://jrtasc.kro.kr/) 

---

## ⚙️ Configuration (Example)
- **e233_1000.json**
  - `notch_accels`: `[EB, B8, B7, ..., B1, N]` order
  - `tau_cmd_ms`, `tau_brk_ms`: control/brake delay constants
  - `mass_t`: single car mass (total mass = formation × passenger load)
- **scenario.json**
  - `L` (target stop distance), `v0` (initial speed), `grade_percent`, `mu` (friction)

---

## 📄 License
MIT License © 2025 Hyungsuk Choi, University of Maryland

---

## 📌 Training Tips
- On the **brake navigation canvas**, watch where the red line intersects curves → read **stair-step braking & relaxing timing**.  
- Observe TASC ON autopilot profiles, then try to **replicate manually** for skill improvement.  
- Aim for **0cm stop bonus** and **on-time arrival bonus**, while monitoring jerk score for **both accuracy and ride comfort**.


