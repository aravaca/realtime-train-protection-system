# train_bias.py — final stability (target ≈ ±0.1 m, smooth convergence)
import os
import random
import time

from server import Vehicle, Scenario, StoppingSim, _mu_to_rr_factor

# ======= 튜닝 파라미터 (안정 우선) =======
RUNS = 250                 # 총 반복
BATCH_FIT = 8              # fit 주기
GAIN = 4                   # 동일 관측 주입
DEADBAND_M = 0.005         # 5 mm 이내는 0으로
RECENT_WINDOW = 300        # 최근 샘플 보존 넓게
SEED = 42

# 학습 안정 옵션
MIN_SAMPLES_FOR_FIT = 80   # 최소 표본 수 전에는 fit 금지
COEFF_EMA_BETA = 0.95      # 새 계수 반영 비율 (매우 보수적)
DELTA_W_MAX = 0.02         # 계수 변화 캡 (절대값)
RAMP_START = 80            # 램프-인 시작
RAMP_END = 160             # 램프-인 종료

# 서버의 릿지 정규화 강제 상향
FORCED_RIDGE_LAMBDA = 5e-3

# ======= 샘플링 범위 =======
SPEED_KMH_WARM = (50, 70)
DIST_M_WARM    = (200, 350)
GRADE_PCT_WARM = (-0.3, 0.3)
MU_WARM        = (0.90, 1.00)

CENTER_V0 = 70.0
CENTER_L  = 300
V0_DELTA  = 10
L_DELTA   = 50
GRADE_PCT_FINE = (-0.2, 0.2)
MU_FINE        = (0.92, 1.00)

def _sample_warm():
    return (
        random.uniform(*SPEED_KMH_WARM),
        random.uniform(*DIST_M_WARM),
        random.uniform(*GRADE_PCT_WARM),
        random.uniform(*MU_WARM),
    )

def _sample_fine():
    return (
        CENTER_V0 + random.uniform(-V0_DELTA, V0_DELTA),
        CENTER_L  + random.uniform(-L_DELTA,  L_DELTA),
        random.uniform(*GRADE_PCT_FINE),
        random.uniform(*MU_FINE),
    )

def _one_episode(sim: StoppingSim, v0_kmh: float, L: float, grade_pct: float, mu: float):
    # ---------- 초기 조건 ----------
    sim.scn.v0 = v0_kmh / 3.6
    sim.scn.L = L
    sim.scn.grade_percent = grade_pct
    sim.scn.mu = mu
    sim.rr_factor = _mu_to_rr_factor(mu)
    sim.reset()

    # ---------- TASC ----------
    sim.tasc_enabled = True
    sim.manual_override = False
    sim.tasc_armed = True
    sim.tasc_active = False
    sim.running = True

    # ---------- 시뮬 ----------
    MAX_STEPS = 6000
    steps = 0
    while sim.running and not sim.state.finished and steps < MAX_STEPS:
        sim.step()
        steps += 1

    st = sim.state
    raw_err = st.stop_error_m if st.stop_error_m is not None else 0.0
    err = float(raw_err)

    # ---------- 오차 후처리 ----------
    if abs(err) < DEADBAND_M:
        err = 0.0
    if abs(err) > 0.10:     # 클리핑 ±0.10 m
        err = 0.10 if err > 0 else -0.10

    # ---------- 관측 저장 ----------
    sim._append_observation_and_update(
        v0_kmh=v0_kmh,
        L_m=L,
        grade_percent=grade_pct,
        mass_tons=sim.veh.mass_kg / 1000.0,
        stop_error_m=err,
        force_fit=False
    )
    for _ in range(max(0, GAIN - 1)):
        sim._append_observation_and_update(
            v0_kmh=v0_kmh,
            L_m=L,
            grade_percent=grade_pct,
            mass_tons=sim.veh.mass_kg / 1000.0,
            stop_error_m=err,
            force_fit=False
        )
    return raw_err, err

def _trim_recent(sim: StoppingSim, n: int):
    m = sim._bias_model
    data = m.get("data", [])
    if len(data) > n:
        m["data"] = data[-n:]
    sim._save_bias_model(sim._bias_model_path, m)

def _safe_fit(sim: StoppingSim, episode_idx: int):
    """안정형 배치 학습"""
    # (0) λ 강제 상향
    try:
        sim._bias_model["lambda"] = float(FORCED_RIDGE_LAMBDA)
    except Exception:
        pass

    # (1) 최소 표본 조건
    data_len = len(sim._bias_model.get("data", []))
    if data_len < MIN_SAMPLES_FOR_FIT:
        return

    old_w = list(sim._bias_model.get("coeffs", [0.0]*7))

    # (2) 내부 피팅
    sim._fit_bias_coeffs_from_data(force=True)
    new_w = list(sim._bias_model.get("coeffs", [0.0]*7))

    # (3) 새 계수 EMA 블렌딩 + 변화량 캡
    blended = []
    for ow, nw in zip(old_w, new_w):
        delta = max(-DELTA_W_MAX, min(DELTA_W_MAX, nw - ow))
        capped = ow + delta
        bw = COEFF_EMA_BETA * ow + (1.0 - COEFF_EMA_BETA) * capped
        blended.append(float(bw))
    sim._bias_model["coeffs"] = blended

    # (4) 램프-인
    if episode_idx < RAMP_END:
        if episode_idx <= RAMP_START:
            scale = 0.0
        else:
            scale = (episode_idx - RAMP_START) / max(1, (RAMP_END - RAMP_START))
            scale = max(0.0, min(1.0, scale))
        sim._bias_model["coeffs"] = [w * scale for w in sim._bias_model["coeffs"]]

    sim._save_bias_model(sim._bias_model_path, sim._bias_model)

def main():
    random.seed(SEED)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(base_dir, "vehicle.json")
    scenario_json_path = os.path.join(base_dir, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))
    vehicle.notches = len(vehicle.notch_accels)

    scenario = Scenario.from_json(scenario_json_path)
    sim = StoppingSim(vehicle, scenario)

    try:
        sim._bias_model["lambda"] = float(FORCED_RIDGE_LAMBDA)
        sim._save_bias_model(sim._bias_model_path, sim._bias_model)
    except Exception:
        pass

    print(f"[train] runs={RUNS}, batch_fit={BATCH_FIT}, gain={GAIN}, deadband={DEADBAND_M}m, window={RECENT_WINDOW}")
    t0 = time.time()
    ema_mae = None

    for i in range(1, RUNS + 1):
        if i <= RUNS // 3:
            v0_kmh, L, grade_pct, mu = _sample_warm()
        else:
            v0_kmh, L, grade_pct, mu = _sample_fine()

        raw_err, filt_err = _one_episode(sim, v0_kmh, L, grade_pct, mu)

        if (i % 10) == 0:
            _trim_recent(sim, RECENT_WINDOW)

        if (i % BATCH_FIT) == 0:
            _safe_fit(sim, i)

        mae = abs(filt_err)
        ema_mae = mae if ema_mae is None else 0.9 * ema_mae + 0.1 * mae

        if (i % 5) == 0 or i <= 10:
            elapsed = time.time() - t0
            print(f"[train] {i:4d}/{RUNS}  raw_err={raw_err:+.3f} m  |err|={abs(filt_err):.3f} m  EMA|err|={ema_mae:.3f} m  elapsed={elapsed:.1f}s")

        if i >= max(200, MIN_SAMPLES_FOR_FIT) and ema_mae is not None and ema_mae < 0.10:
            print(f"[train] early stop at {i} runs (EMA|err|={ema_mae:.3f} m)")
            break

    _safe_fit(sim, RUNS)
    print(f"[train] done. bias_model at: {sim._bias_model_path}")

if __name__ == "__main__":
    main()