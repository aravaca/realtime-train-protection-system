# JR Personal Type Simulator (PTS) — Lightweight Precision Stop Training Framework

 
[Try PTS!⛅️ → Play full screen on PC](https://jrtasc.kro.kr/) 
---
<img width="1073" height="915" alt="제목 없음" src="https://github.com/user-attachments/assets/ce7b51dd-c7e5-4300-a2b2-4222b1b92d28" />
<img width="980" height="265" alt="feedback" src="https://github.com/user-attachments/assets/b523694b-a208-4161-b031-6abb18f66f38" />

A lightweight **web-based railway simulation framework** replicating key control functions of **JR East/West EMUs**.  
Unlike heavy simulations, PTS is designed for **essential driving skill training** on low-end PCs, as it enables repetitive **braking** and **stop-point accuracy** practice.

Supports:
- **Manual stop control**  
- **TASC-assisted automatic stopping**  
→ allowing direct **human vs. automation** performance comparison.

Powered by **Three.js 3D visualization**, the simulator delivers an immersive driver’s POV and realistic track environment.  
Real-time **velocity–distance** and **braking curve** rendering provides intuitive, instant feedback.

By replicating authentic **EMU physics**, it enables:
- **Repetitive stop training** using the random function
- **Scenario-based evaluation**  
- **Quantitative performance scoring** under diverse track conditions
---

## ⚙️ Tech Stack
- **Backend**: Python (FastAPI, WebSocket)
- **Frontend**: HTML, CSS, JavaScript (Three.js for BVE1-style graphics)
- **Deployment**: AWS EC2 (Amazon Linux), Nginx, Route53, 내도메인.한국

---

## 🚆 Application Scenarios

- **Operator Training:** Repetitive personal-level stopping practice using simplified simulator hardware  
- **Educational Demonstrations:** Visualization of efficient brake curves 
- **Research & Development:** Testing human–automation interaction (TASC) algorithms    

---

## 🔭 Future Development

Planned upgrades include integration with advanced railway control frameworks and signals such as:

- **ATC (Automatic Train Control)**  
- **ATP (Automatic Train Protection)**  
- **ATS (Automatic Train Stop)**  


---

## 🧠 Why TASC?
- **Reduces driver workload & fatigue**: automatically executes initial, stair-step braking and relaxing → reduces cognitive load and control fatigue.
- **Higher stopping accuracy & consistent ride comfort**: useful when stopping at stations with PSDs (platform doors); always produces a **similar braking profile**.
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

## ⚙️ Config
- **e233_1000.json**
  - `notch_accels`: `[EB, B8, B7, ..., B1, N]` order
  - `tau_cmd_ms`, `tau_brk_ms`: control/brake delay constants
  - `mass_t`: single car mass (total mass = formation × passenger load)
- **scenario.json**
  - `L` (target stop distance), `v0` (initial speed), `grade_percent`, `mu` (friction)

---

## 📄 License

Copyright © 2025 Hyungsuk Choi, University of Maryland

Permission is hereby granted to use and modify this software for personal or internal purposes only.  
Redistribution, reproduction, resale, public posting, or sharing of this software or any modified versions  
is strictly prohibited without the express written permission of the author.





