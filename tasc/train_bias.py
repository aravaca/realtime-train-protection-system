# train_bias.py — grid-biased trainer (bilinear LUT)
import os
import random
import time

from server import Vehicle, Scenario, StoppingSim, _mu_to_rr_factor

# ======= 튜닝 파라미터 =======
RUNS = 240
BATCH_FIT = 12          # 그리드는 fit 불필요하지만, 호환을 위해 주기만 맞춰 호출 (no-op)
GAIN = 4
DEADBAND_M = 0.005      # 5mm 이내는 0
SEED = 42

# 샘플 보존(그리드는 파일 자체에 누적되므로 trim은 보수적으로)
RECENT_WINDOW = 800

# ======= 샘플링 =======
SPEED_KMH_WARM = (50, 90)
DIST_M_WARM    = (250, 400)
GRADE_PCT_WARM = (-0.3, 0.3)
MU_WARM        = (0.90, 1.00)

CENTER_V0 = 70.0
CENTER_L  = 300.0
V0_DELTA  = 6.0
L_DELTA   = 60.0
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
    sim.scn.v0 = v0_kmh / 3.6
    sim.scn.L = L
    sim.scn.grade_percent = grade_pct
    sim.scn.mu = mu
    sim.rr_factor = _mu_to_rr_factor(mu)
    sim.reset()

    sim.tasc_enabled = True
    sim.manual_override = False
    sim.tasc_armed = True
    sim.tasc_active = False
    sim.running = True

    MAX_STEPS = 6000
    steps = 0
    while sim.running and not sim.state.finished and steps < MAX_STEPS:
        sim.step()
        steps += 1

    st = sim.state
    raw_err = st.stop_error_m if st.stop_error_m is not None else 0.0
    err = float(raw_err)

    # 오차 후처리: 소프트 클립(±0.10m), 작은 데드밴드
    if abs(err) < DEADBAND_M:
        err = 0.0
    if abs(err) > 0.10:
        err = 0.10 if err > 0 else -0.10

    # 관측 저장
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
    # 그리드 모델은 data 리스트가 없지만, 하위호환 필드가 있어도 무시됨.
    m = sim._bias_model
    data = m.get("data", [])
    if isinstance(data, list) and len(data) > n:
        m["data"] = data[-n:]
        sim._save_bias_model(sim._bias_model_path, m)

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

    print(f"[train] runs={RUNS}, batch_fit={BATCH_FIT}, gain={GAIN}, deadband={DEADBAND_M}m")
    t0 = time.time()
    ema_mae = None

    for i in range(1, RUNS + 1):
        # 커리큘럼: 1/3 워밍업, 이후 파인
        if i <= RUNS // 3:
            v0_kmh, L, grade_pct, mu = _sample_warm()
        else:
            v0_kmh, L, grade_pct, mu = _sample_fine()

        raw_err, filt_err = _one_episode(sim, v0_kmh, L, grade_pct, mu)

        if (i % 12) == 0:
            _trim_recent(sim, RECENT_WINDOW)

        # 하위호환용 no-op 호출(그리드에선 의미 없음)
        if (i % BATCH_FIT) == 0:
            sim._fit_bias_coeffs_from_data(force=True)
            sim._save_bias_model(sim._bias_model_path, sim._bias_model)

        mae = abs(filt_err)
        ema_mae = mae if ema_mae is None else 0.9 * ema_mae + 0.1 * mae

        if (i % 10) == 0 or i <= 10:
            elapsed = time.time() - t0
            print(f"[train] {i:4d}/{RUNS}  raw_err={raw_err:+.3f} m  |err|={abs(filt_err):.3f} m  EMA|err|={ema_mae:.3f} m  elapsed={elapsed:.1f}s")

        # 조기 종료(EMA 0.10m 달성 시)
        if i >= 120 and ema_mae is not None and ema_mae < 0.10:
            print(f"[train] early stop at {i} runs (EMA|err|={ema_mae:.3f} m)")
            break

    # 종료 시 저장
    sim._save_bias_model(sim._bias_model_path, sim._bias_model)
    print(f"[train] done. bias_model at: {sim._bias_model_path}")

if __name__ == "__main__":
    main()