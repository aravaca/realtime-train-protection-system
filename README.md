# JR Personal Type Simulator (PTS) — Lightweight Precision Stop Training Framework

 
[Try PTS🚆](https://jrtasc.kro.kr/) 
---

A lightweight **web-based simulation framework** replicating key control functions of **JR East/West EMUs**.  
Unlike heavy simulations, PTS is designed for **essential driving skill training**, as it enables repetitive **braking** and **stop-point accuracy** practice.

Supports:
- **Manual stop control**  
- **TASC-assisted automatic stopping**  
→ allowing direct **human vs. automation** performance comparison.

Powered by **Three.js 3D visualization**, the simulator delivers an immersive driver’s POV and realistic track environment.  
Real-time **velocity–distance** and **braking curve** rendering provides intuitive, instant feedback.

By replicating authentic **EMU physics**, it enables:
- **Repetitive stop training**  
- **Scenario-based evaluation**  
- **Quantitative performance scoring** under diverse track conditions
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

## 🧠 Why TASC (Train Automatic Stopping Control)?
- **Reduces driver workload & fatigue**: automatically executes initial, stair-step braking and relaxing → reduces cognitive load and control fatigue.
- **Higher stopping accuracy & consistent ride comfort**: deadband and hold time prevent unnecessary notch hunting; always produces a **similar braking profile**.
- **Training efficiency**:
  - **Brake navigation canvas** helps learners understand "where curves meet the red line."
  - Quantitative feedback and scoring allows self-assessment of braking habits.
- **Educational/verification use**: various scenarios (distance/slope/speed/friction), simultaneous evaluation of ride comfort (jerk) and stopping precision.
- **Low-cost and low-risk**: cheaper than ATO systems; TASC allows manual override if equipment malfunctions.


---

## 🔧 Project Structure
├── scenario.json # Scenario (distance L, initial speed v0, slope, friction, etc.)
├── vehicle.json # Vehicle specs (mass, notch_accels, time constants, etc.)
├── server.py # FastAPI + WebSocket server, including TASC logic
└── static/
      ├── index.html # UI (3D graphics, HUD/Overlay/TASC switch/animations)
      └── xxxxx.json # Vehicle name display data



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



