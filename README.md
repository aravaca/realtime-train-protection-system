# JR TASC Simulator — Advanced Precision Stop Simulation Framework for JR Series EMUs

##Abstract

This project presents an advanced simulation framework for **Train Automatic Stop Control (TASC) systems**, designed to replicate and evaluate the braking and precision stop performance of JR East and JR West Electric Multiple Units (EMUs). The simulator integrates a real-time control and visualization environment, combining a FastAPI-based backend with HTML/CSS/JavaScript front-end modules for dynamic rendering of distance–velocity trajectories and automatic brake navigation.

By emulating authentic TASC logic, the system enables precise stop-point training, adaptive feedback analysis, and quantitative scoring for operator performance assessment. The platform serves as both a training and research tool, bridging theoretical control algorithms with real-world railway operations.

Future development aims to extend the simulation scope to encompass modern train protection and supervision systems, including Automatic Train Control (ATC), Automatic Train Protection (ATP), and Automatic Train Stop (ATS). This expansion will establish a comprehensive framework for comparative studies on safety-critical railway automation and human-in-the-loop control dynamics.

---

## ✨ What’s New (Key Updates 2025.08.31)
- Expanded train series: JR East (E233, E235) commuter trains as well as special express, sleeper, and retired trains with different braking profiles.
- Auto-restriction of options by series/composition: train selection automatically limits series/formation.
- Weather & load simulation: friction coefficient (sunny/rainy/snowy) + passenger load (formation × mass) reflected in real-time.
- **TASC Autopilot**:
  - Activates automatically at 150m when passing P3 trackside signal → N-step braking (build) and M-step relaxing → B1 soft stop for ride comfort.
  - **Stair-step build**: compares red "remaining distance" line with brake-distance curves; increases brake step if needed.
  - **Stair-step relax**: decreases brake step gradually if stopping is possible with a lower notch, finishing at **B1**.
  - **Deadband & minimum hold time** prevent hunting; improve ride comfort.
  - Manual intervention (keyboard/touch) immediately switches to **Manual**.
- **Brake Navigation Canvas**:
  - Visualizes brake-distance curves and remaining distance in real-time → intuitive guidance for when to brake/relax.
- **Scoring Improvements**:
  - Extra points for initial braking (B1/B2 1s) during manual operation.
  - Bonus for last B1 stop.
  - **0cm stop (+400 points)** bonus added.
  - **00:00s on-time arrival (+100 points)** bonus.
  - Jerk-based ride comfort scoring.
  - TASC ON allows initial braking omission.
- **Environment Simulation**: snow/rain animation, slope/friction reflection, total mass adjusted by passenger load.

---

## ⚙️ Tech Stack
- **Backend**: Python 3.12, FastAPI, WebSocket
- **Frontend**: HTML, CSS, JavaScript (Canvas-based HUD)
- **Deployment**: AWS EC2 (Amazon Linux), Nginx, Route53

---

## 🧠 Why TASC? (Value of TASC)
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

## 🖥 UI & Controls (Operation & Visualization)

### 1. Displayed Metrics
- **Remaining distance**: distance to target stop; displayed on HUD & brake navigation canvas in real-time.  
- **Current speed**: km/h, brake curves adjust according to speed.  
- **Elapsed time**: simulation time since start; color indicates status:
  - ±2s: **green highlight**
  - Time exceeded (positive): **red warning**
- **Current notch**: driver brake/traction lever state.
- **Gradient**: slope percentage of the track.

---

### 2. Brake Curve Canvas
- **X-axis**: distance, **Y-axis**: speed  
- Intersection of notch brake curves and **red vertical line (remaining distance)** shows **brake/relax timing** intuitively.  
- Curve colors highlight current notch:
  - Selected notch: `#ffae00`  
  - Unselected notch: `#3fa9ff`  
- EB (Emergency Brake) is shown in red.

---

### 3. TASC Indicator (HUD Top)
- **Automation status**:
  - Idle/inactive: gray (`#444`)  
  - Active: yellow (`#fec670`)  
  - Armed blinking: 0.25s intervals to indicate readiness  
- **Text label**: `TASC\nAUTO`  
- **Text color**:
  - Active: black (`#000`)  
  - Inactive/idle: white (`#fff`)  

---

### 4. Precision Stop Button (HUD Top)
- **Position**: left of TASC indicator  
- **State color**:
  - Default: gray (`#444`)  
  - Precision stop achieved: green (`#9be071`)  
- **Text**: `TASC\nSTOP`  
  - Text turns black when precision achieved (`#000`)  
- **Design**: rounded rectangle with gradient border for 3D effect.

---

### 5. Brake Mini Indicator
- **Function**: displays current notch as a mini bar graph.  
- **Composition**:
  - Total blocks = number of notches (including EB)  
  - Block colors:  
    - Active notch: yellow (`#ffd34d`)  
    - EB: red (`#ff5757`)  
    - Inactive: gray (`rgba(60,80,100,0.35)`)

---

### 6. Keyboard / Mobile Controls
- **Keyboard**:
  - `Space`: Start/Restart  
  - `W`: Increase brake notch  
  - `S`: Decrease brake notch  
  - `N`: Release brakes  
  - `E`: Emergency brake  
  - `A`: Maximum service brake  
  - `D`: Minimum service brake  
- **Mobile Touch**:
  - Top touch → Increase brake notch  
  - Bottom touch → Decrease brake notch  
- **TASC Toggle**:
  - HUD top-right ON/OFF switch  
  - **ON** → manual initial braking followed by automatic control (see algorithm)  
  - **OFF** → fully manual operation  

---

### 7. HUD & Brake Navigation Feedback
- Real-time visualization of remaining distance, notch, and speed curve intersections → intuitive brake/relax timing.  
- TASC indicator/button color changes upon precision stop completion.

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

## ⚙️ Configuration
- **vehicle.json**
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


