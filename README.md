# JR TASC Simulator — Advanced TASC-based Precision Stop Simulator for JR Series EMUs
**JR East/West 시리즈 전차의 실제 혹은 가상의 TASC 시스템 기반 정차 훈련 시뮬레이터**

이 풀스택 프로젝트는 **JR 시리즈 전동차**를 모델로 한 정차 훈련용 시뮬레이터입니다.  
프론트엔드(HTML/CSS/JS) + 백엔드(FastAPI WebSocket)를 통합하여, **브레이크 내비게이션 캔버스**(거리-속도 곡선)와 **TASC(자동 정차 보조)**를 활용한 **실시간 제동 연습**과 **피드백/점수 평가**를 제공합니다.

---


## ✨ What’s New (주요 업데이트 2025.08.31)
- 차량군 확대: JR East (E233, E235) 등 통근열차 뿐만 아니라 제동감이 상이한 특급열차, 침대열차, 퇴역열차 등 추가

- 노선/시리즈별 옵션 고정: 전동차 선택에 따라 시리즈/편성(량수) 옵션이 자동 제한

- 날씨/하중 반영: 마찰계수(맑음/비/눈) + 탑승률(편성×질량) 실시간 반영

- **TASC 오토파일럿**:
  - 150m 지점 P3 지상자 신호 수신 시 자동 활성화 → N단 제동(build) 및 M단 완해(relax) → B1로 승차감 유지하며 마무리”
  - **계단 제동(build)**: 빨간 “남은 거리” 세로선과 노치별 제동거리 곡선을 비교해 필요 시 한 단계씩 강화
  - **계단 완해(relax)**: 더 낮은 노치로도 정지 가능하면 **한 단계씩** 완해하면서 B1로 마무리
  - **데드밴드 & 최소 유지시간**으로 헌팅 방지, 승차감 향상
  - 수동 개입(키/터치) 시 즉시 **Manual**로 전환
- **브레이크 내비게이션 캔버스**:
  - 노치별 제동 곡선과 남은 거리의 교차점을 실시간 시각화 → **언제 브레이크/완해할지 직관적으로 학습**
- **스코어 개선**:
  - 수동 운전 시 초제동(B1/B2 1초) 수행 가점
  - 마지막 **B1 정차 가점**
  - **0cm 정차(+400점)** 보너스 추가
  - **00:00초 (+100점) 정시 도착** 보너스 추가
  - **저크(Jerk)** 기반 승차감 점수 반영
  - TASC ON 시 초제동 미실시 허용
- **환경 요소**: 눈/비 애니메이션, 구배/마찰계수(날씨) 반영, 탑승률에 따른 총중량 반영

---
## ⚙️ Tech Stack
- **Backend**: Python 3.12, FastAPI, WebSocket
- **Frontend**: HTML, CSS, JavaScript (Canvas 기반 HUD)
- **Deploy**: AWS EC2 (Amazon Linux), Nginx, Route53

---

## 🧠 Why TASC? (TASC의 가치)
- **운전자의 부담·피로 감소**: 반복 구간에서 자동으로 초제동·계단제동·완해를 수행 → 인지 부하와 조작 피로 ↓
- **정차 정확도↑ / 승차감 일관성↑**: 데드밴드와 홀드 타임으로 불필요한 노치 헌팅을 방지, 항상 **비슷한 프로파일**로 제동
- **훈련 효율↑**:
  - **브레이크 내비 캔버스**로 “곡선과 빨간선이 만날 때”의 타이밍 감각을 체득
  - **피드백/점수**로 본인의 습관을 정량적으로 점검
- **교육/검증용**: 시나리오(거리/구배/속도/마찰) 다양화, **승차감(저크)**과 **정차 정밀도**를 같이 평가
- **도입비용/리스크 분산**: ATO(열차자동운전장치) 대비 도입 비용 저렴, TASC는 기계 오작동시 기관사 개입으로 리스크 최소화 가능
---



## 🧪 How TASC Works (알고리즘 요약)

- **TASC 활성화 조건**:  
  - 시작 전 TASC를 활성화하면 **TASC가 armed** 상태가 됨  
  - armed 상태에서 수동 제동(B1/B2)을 사용하여 **150m 지점 이전까지 속도를 60 km/h 이하로 감소**  
  - 150m 지점 **P3 지상자 위치**에서 신호를 수신하면 **TASC가 active** 상태로 전환  
  - 이후 TASC는 **자동으로 주간 제어기를 작동**하여 정위치 정차(±35cm)를 보조하며,  
    수동 개입 시 즉시 **OFF**로 전환 (현실에서는 OFF 되지는 않고 TASC가 계산한 단과 기관사가 취급한 단 중 높은 단이 적용됨)


- **초제동 (Initial Brake)**:
  - 미실시. 단, 수동운전에서는 평가항목 중 일부.

- **N단 제동 (N-Step Build)**:
  - 현재 노치 정지거리 `s_cur` > `(rem - 데드밴드)` → 한 단계 브레이크 강화(↑)  
  - 충분해지면 **완해 단계로 전환**  

- **계단 완해 (Stair-Step Relax)**:
  - 한 단계 아래 노치 정지거리 `s_dn` ≤ `(rem + 데드밴드)` → 한 단계 완해(↓)  
  - 반복 적용 후 **마지막 B1**로 마무리  

- **안정화 장치 (Stabilization)**:
  - **데드밴드(±m)**와 **최소 유지시간(s)** 적용 → 헌팅/덜컹 방지  
  - 5km/h 이내 구간에서 **B1 선형 소프트 스탑** 적용 → 승차감 개선  

---

## 🖥 UI & Controls (조작 및 시각화)

### 1. 표시 항목
- **남은 거리**: 목표 정차 지점까지 남은 거리, HUD 및 브레이크 내비 캔버스에서 실시간 표시  
- **현재 속도**: km/h 단위로 표시, 속도에 따라 제동 곡선 자동 조정  
- **경과 시간**: 시뮬레이션 시작 후 경과 시간, 색상으로 경과 상태 표시  
  - ±2초 이내: **초록색** 강조  
  - 시간 초과(양수): **빨강**으로 경고  
- **현재 노치**: 운전자의 브레이크/가속 조작 상태 표시  
- **경사도**: 현재 노선 구배를 퍼센트밀로 표시  

---

### 2. 브레이크 내비게이션 캔버스
- **X축**: 거리, **Y축**: 속도  
- 노치별 제동 곡선과 **빨간 세로선(남은 거리)**의 교차점을 통해 **제동/완해 타이밍** 직관적 확인 가능  
- 곡선 색상으로 현재 노치 강조:  
  - 선택 노치: `#ffae00`  
  - 미선택 노치: `#3fa9ff`  
- EB(비상제동)는 빨강으로 표시  

---

### 3. TASC 경고등 (HUD 상단)
- **자동화 상태 표시**:
  - 대기/비활성: 회색 (`#444`)  
  - 활성(작동 중): 노란색 (`#fec670`)  
  - 대기 중 깜빡임: 0.25초 단위 깜빡이며 TASC 준비 상태 표시  
- **텍스트 라벨**: `TASC\n자동`  
- **색상 변화**:
  - 활성 시 텍스트 검정 (`#000`)  
  - 비활성/대기 시 텍스트 흰색 (`#fff`)  

---

### 4. 정위치 정차 버튼 (HUD 상단)
- **위치**: TASC 경고등 왼쪽  
- **상태 표시**:
  - 기본: 회색 (`#444`)  
  - 정위치 정차 완료 시: 초록 (`#9be071`)  
- **텍스트**: `TASC\n정위치`  
  - 정위치 완료 시 검정 텍스트 (`#000`)  
- **버튼 디자인**: 둥근 사각형, 그라데이션 테두리로 3D 느낌 강조  

---

### 5. 브레이크 미니 인디케이터
- **기능**: 현재 노치 상태를 미니 막대기로 직관적 표시  
- **구성**:
  - 총 블록 = 노치 개수 (EB 포함)  
  - 각 블록 색상:  
    - 활성 노치: 노란색 (`#ffd34d`)  
    - EB: 빨강 (`#ff5757`)  
    - 비활성: 회색 (`rgba(60,80,100,0.35)`)  

---

### 6. 키보드/모바일 조작
- **키보드**:
  - `Space`: 시작/재시작  
  - `W`: 브레이크 강화 (노치↑)  
  - `S`: 브레이크 완해 (노치↓)  
  - `N`: 제동 해방  
  - `E`: 비상 제동  
  - `A`: 최대 상용 제동  
  - `D`: 최소 상용 제동  
- **모바일 터치**:
  - 상단 터치 → 브레이크 강화  
  - 하단 터치 → 브레이크 완해  
- **TASC 토글**:
  - HUD 우측 상단 스위치 ON/OFF  
  - **ON** → 수동으로 초제동 후 TASC가 자동 개입 (위 알고리즘 상세 서술 참고)
  - **OFF** → 수동 조작 

---

### 7. HUD & 브레이크 내비 피드백
- 남은 거리, 노치, 속도 곡선과 **빨간선 교차** 시각화 → 제동/완해 타이밍 직관 학습  
- 정위치 정차 완료 시, TASC 경고등/버튼 색상 변화로 즉시 피드백  


---

## 🔧 Project Structure
```
├── scenario.json          # 시나리오(거리 L, 초기속도 v0, 경사, 마찰 등)
├── vehicle.json           # 차량 제원(질량, notch_accels, 시간상수 등)
├── server.py              # FastAPI + WebSocket 서버, TASC 로직 포함
└── static/
    ├── index.html         # UI (HUD/오버레이/TASC 스위치/애니메이션)
    └── xxxxx.json       # 차량 이름 표시용
```

---

## 🚀 Run (로컬 실행)
1. 브라우저에서 접속  
[바로 체험해보기](http://18.222.103.182/) → HUD/오버레이 UI 실행 (https가 아니라 http로 입력해야 웹에 연결됩니다.)


---

## ⚙️ Configuration (설정 포인트)
- **vehicle.json**
  - `notch_accels`: `[EB, B8, B7, ..., B1, N]` 순서
  - `tau_cmd_ms`, `tau_brk_ms`: 제어/제동 지연 상수
  - `mass_t`: 1량 기준 질량 (총질량=편성 수 × 탑승률)
- **scenario.json**
  - `L`(정차 목표 거리), `v0`(초기 속도), `grade_percent`, `mu`(마찰)

---


## 📄 License
MIT License © 2025 Hyungsuk Choi, University of Maryland

---

## 📌 교육·연습 활용 팁
- **브레이크 내비 캔버스**에서 “빨간선과 곡선이 만나는 지점”을 노려 **계단 제동·완해 타이밍**을 읽어보세요.
- TASC ON 상태에서 자동 프로파일을 관찰한 뒤, **수동으로 동일한 패턴을 재현**해보면 실력이 빠르게 향상됩니다.
- **0cm 정차 보너스** 및 **0초 정시 보너스** 를 노리며 저크 점수도 신경 쓰면, 정확도+승차감 **둘 다** 잡을 수 있습니다.

---

# JR TASC Simulator — Advanced TASC-based Precision Stop Simulator for JR Series EMUs
**A simulator for JR East/West EMU series trains based on real or virtual TASC systems for precision stop training.**

This full-stack project models **JR series EMUs** for training purposes.  
Frontend (HTML/CSS/JS) + Backend (FastAPI WebSocket) integration provides **brake navigation canvas** (distance-speed curves) and **TASC (Train Automatic Stop Controller)** for **real-time braking practice** and **feedback/score evaluation**.

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

### 2. Brake Navigation Canvas
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

## 🚀 Run (Local)
1. Open in browser  
[Try it live](http://18.222.103.182/) → HUD/overlay UI will run (HTTP required for WebSocket).

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
