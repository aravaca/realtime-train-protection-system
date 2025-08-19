# train_bias.py  —  precision mode (target ±0.1 m)
import os
import random
import time

from server import Vehicle, Scenario, StoppingSim, _mu_to_rr_factor

# ======= 튜닝 파라미터 (±0.1m 목표) =======
RUNS = 400              # 총 학습 회수(초기 테스트 300~600 권장)
BATCH_FIT = 3           # 매 N회마다 추가 피팅 반복
GAIN = 8                # 동일 관측 반복 주입 횟수(5~10 사이 튜닝)
DEADBAND_M = 0.02       # 2 cm 이내는 0으로 간주(노이즈 억제)
RECENT_WINDOW = 100     # 최근 샘플만 유지(민감도 ↑)
SEED = 42               # 재현성

# ======= 샘플링 범위 (정밀 모드: 실사용 구간으로 좁힘) =======
# 1단계(워밍업): 아래 범위 사용
SPEED_KMH_WARM = (60, 80)
DIST_M_WARM    = (450, 650)
GRADE_PCT_WARM = (-0.3, 0.3)   # -3‰ ~ +3‰
MU_WARM        = (0.90, 1.00)

# 2단계(집중 미세조정): 중심값 ±소폭만
CENTER_V0 = 70.0   # km/h  ← 네 사용 패턴에 맞게 바꿔도 됨
CENTER_L  = 550.0  # m
V0_DELTA  = 4.0    # km/h
L_DELTA   = 30.0   # m
GRADE_PCT_FINE = (-0.2, 0.2)
MU_FINE        = (0.92, 1.00)

def _sample_warm():
    v0_kmh = random.uniform(*SPEED_KMH_WARM)
    L = random.uniform(*DIST_M_WARM)
    grade = random.uniform(*GRADE_PCT_WARM)
    mu = random.uniform(*MU_WARM)
    return v0_kmh, L, grade, mu

def _sample_fine():
    v0_kmh = CENTER_V0 + random.uniform(-V0_DELTA, V0_DELTA)
    L      = CENTER_L  + random.uniform(-L_DELTA,  L_DELTA)
    grade  = random.uniform(*GRADE_PCT_FINE)
    mu     = random.uniform(*MU_FINE)
    return v0_kmh, L, grade, mu

def _one_episode(sim: StoppingSim, v0_kmh: float, L: float, grade_pct: float, mu: float):
    # ---------- 초기 조건 (UI setInitial과 동일) ----------
    sim.scn.v0 = v0_kmh / 3.6
    sim.scn.L = L
    sim.scn.grade_percent = grade_pct
    sim.scn.mu = mu
    sim.rr_factor = _mu_to_rr_factor(mu)

    sim.reset()

    # ---------- 실제 TASC 모드와 동일하게 ----------
    sim.tasc_enabled = True
    sim.manual_override = False
    sim.tasc_armed = True
    sim.tasc_active = False

    sim.running = True

    # ---------- 완료 기반 루프 (스텝 상한) ----------
    MAX_STEPS = 6000   # 필요 시 4000~7000 사이 조정
    steps = 0
    while sim.running and not sim.state.finished and steps < MAX_STEPS:
        sim.step()
        steps += 1

    st = sim.state
    raw_err = st.stop_error_m if st.stop_error_m is not None else 0.0
    err = float(raw_err)

    # ---------- 오차 후처리 ----------
    # 2 cm 이내는 0으로(노이즈 무시)
    if abs(err) < DEADBAND_M:
        err = 0.0
    # 이상치 학습 오염 방지: ±0.2 m로 클리핑(목표 0.1m를 노리면 0.2m 권장)
    if abs(err) > 0.2:
        err = 0.2 if err > 0 else -0.2

    # ---------- 관측 저장/학습 ----------
    # ⚠️ 부호는 건드리지 않는다(서버에서 저장 시 -stop_error_m 로 반전됨).
    sim._append_observation_and_update(
        v0_kmh=v0_kmh,
        L_m=L,
        grade_percent=grade_pct,
        mass_tons=sim.veh.mass_kg / 1000.0,
        stop_error_m=err,
        force_fit=True
    )
    # 동일 관측 반복 주입으로 수렴 가속(부호 안전)
    for _ in range(max(0, GAIN - 1)):
        sim._append_observation_and_update(
            v0_kmh=v0_kmh,
            L_m=L,
            grade_percent=grade_pct,
            mass_tons=sim.veh.mass_kg / 1000.0,
            stop_error_m=err,
            force_fit=False
        )

    return raw_err, err  # 원시/후처리 오차

def _trim_recent(sim: StoppingSim, n: int):
    m = sim._bias_model
    data = m.get("data", [])
    if len(data) > n:
        m["data"] = data[-n:]
    sim._save_bias_model(sim._bias_model_path, m)

def main():
    random.seed(SEED)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(base_dir, "vehicle.json")
    scenario_json_path = os.path.join(base_dir, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    # 프론트/서버 notch 방향 정합: 기존 로직과 동일하게 반전
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))
    vehicle.notches = len(vehicle.notch_accels)

    scenario = Scenario.from_json(scenario_json_path)
    sim = StoppingSim(vehicle, scenario)

    print(f"[train] runs={RUNS}, batch_fit={BATCH_FIT}, gain={GAIN}, deadband={DEADBAND_M}m, window={RECENT_WINDOW}")
    t0 = time.time()

    ema_mae = None
    for i in range(1, RUNS + 1):
        # ------- 커리큘럼 샘플링 -------
        if i <= RUNS // 3:   # 1/3 구간: 워밍업(넓게)
            v0_kmh, L, grade_pct, mu = _sample_warm()
        else:                 # 나머지: 중심값 근처 미세조정
            v0_kmh, L, grade_pct, mu = _sample_fine()

        raw_err, filt_err = _one_episode(sim, v0_kmh, L, grade_pct, mu)

        # 최근 윈도우 유지(민감도 ↑)
        if (i % 5) == 0:
            _trim_recent(sim, RECENT_WINDOW)

        # 추가 피팅 반복(수렴 가속)
        if (i % BATCH_FIT) == 0:
            for _ in range(5):   # 3→5로 강화
                sim._fit_bias_coeffs_from_data(force=True)
            sim._save_bias_model(sim._bias_model_path, sim._bias_model)

        # EMA 업데이트/로그
        mae = abs(filt_err)
        ema_mae = mae if ema_mae is None else 0.9 * ema_mae + 0.1 * mae

        if (i % 10) == 0 or i <= 10:
            elapsed = time.time() - t0
            print(f"[train] {i:4d}/{RUNS}  raw_err={raw_err:+.3f} m  |err|={abs(filt_err):.3f} m  EMA|err|={ema_mae:.3f} m  elapsed={elapsed:.1f}s")

        # 조기 종료: 최소 200회 실행 후 EMA 0.10m 달성
        if i >= 200 and ema_mae is not None and ema_mae < 0.10:
            print(f"[train] early stop at {i} runs (EMA|err|={ema_mae:.3f} m)")
            break

    sim._save_bias_model(sim._bias_model_path, sim._bias_model)
    print(f"[train] done. bias_model at: {sim._bias_model_path}")

if __name__ == "__main__":
    main()