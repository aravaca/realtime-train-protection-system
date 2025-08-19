# train_bias.py
import os
import random
import math
import time

# ← 여기에 너의 클래스들이 정의된 모듈명을 맞춰줘.
# 예: server.py 안에 Vehicle/Scenario/StoppingSim 이 있으면 아래처럼.
from server import Vehicle, Scenario, StoppingSim, _mu_to_rr_factor

# --------- 튜닝 파라미터 ---------
RUNS = 100000            # 총 학습 회수 (원하면 5만~10만도 OK)
BATCH_FIT = 5          # 매 N회마다 강제 추가 피팅
GAIN = 10            # 오차 가중(수렴 가속)
DEADBAND_M = 0.05       # 5 cm: 이 범위 이하면 0으로 간주
RECENT_WINDOW = 100     # 최근 데이터만 남겨 민감도 ↑
SEED = 42               # 재현성

# 샘플링 범위 (필요하면 조정)
SPEED_KMH = (40, 90)   # 초기 속도
DIST_M = (300, 900)    # 정지 목표 거리
GRADE_PCT = (-1, 1)     # 경사(%)
MU = (0.3, 1.0)         # 마찰계수

def _sample():
    v0_kmh = random.uniform(*SPEED_KMH)
    L = random.uniform(*DIST_M)
    grade = random.uniform(*GRADE_PCT)
    mu = random.uniform(*MU)
    return v0_kmh, L, grade, mu

def _one_episode(sim: StoppingSim, v0_kmh: float, L: float, grade_pct: float, mu: float):
    # 초기화
    sim.scn.v0 = v0_kmh / 3.6
    sim.scn.L = L
    sim.scn.grade_percent = grade_pct
    sim.scn.mu = mu
    sim.rr_factor = _mu_to_rr_factor(mu)

    sim.reset()
    sim.running = True

    # 끝까지 진행
    # 안전: 무한루프 방지(최대 60초 시뮬)
    start_t = time.time()
    while sim.running:
        sim.step()
        if time.time() - start_t > 30:
            # 너무 오래 걸리면 강제 종료
            sim.running = False
            break

    st = sim.state
    err = float(st.stop_error_m or 0.0)

    # 5cm 이내면 0으로 간주(오버피팅 방지)
    if abs(err) < DEADBAND_M:
        err = 0.0

    # 부호 반전 + 가중(수렴 가속)
    # _append_observation_and_update 에 직접 넣지 않고 여기서 보정하여 주입
    rec_err = -err * GAIN

    # 내부 JSON에 직접 주입하기 위해 공개 메서드를 사용
    v0 = v0_kmh
    sim._append_observation_and_update(
        v0_kmh=v0, L_m=L, grade_percent=grade_pct,
        mass_tons=sim.veh.mass_kg/1000.0,
        stop_error_m=rec_err,  # 이미 부호/가중 조정된 값
        force_fit=True
    )

    return err

def _trim_recent(sim: StoppingSim, n: int):
    m = sim._bias_model
    data = m.get("data", [])
    if len(data) > n:
        m["data"] = data[-n:]
    # 저장
    sim._save_bias_model(sim._bias_model_path, m)

def main():
    random.seed(SEED)

    # vehicle/scenario 로딩(없으면 생성)
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

    ema_mae = None  # 지수평활 MAE 추정(모니터링 용)
    for i in range(1, RUNS + 1):
        v0_kmh, L, grade_pct, mu = _sample()
        err = _one_episode(sim, v0_kmh, L, grade_pct, mu)  # 실제 오차(부호반전 전)

        # 최근 윈도우로 잘라 민감도 유지
        if (i % 5) == 0:
            _trim_recent(sim, RECENT_WINDOW)

        # 추가 피팅: 내부 append에서 한 번 하지만, 더 자주 반영하고 싶다면 반복 피팅
        if (i % BATCH_FIT) == 0:
            for _ in range(3):  # 3회 추가 피팅으로 수렴 가속
                sim._fit_bias_coeffs_from_data(force=True)
            sim._save_bias_model(sim._bias_model_path, sim._bias_model)

        # 진행 로그
        mae = abs(err)
        if ema_mae is None:
            ema_mae = mae
        else:
            ema_mae = 0.9 * ema_mae + 0.1 * mae

        if (i % 100) == 0:
            elapsed = time.time() - t0
            print(f"[train] {i}/{RUNS}  EMA|err|={ema_mae:.3f} m  elapsed={elapsed:.1f}s")

        # 조기 종료(옵션): 평균 오차가 2cm 이하로 내려오면 그만
        if ema_mae is not None and ema_mae < 0.02:
            print(f"[train] early stop at {i} runs (EMA|err|={ema_mae:.3f} m)")
            break

    # 최종 저장 보증
    sim._save_bias_model(sim._bias_model_path, sim._bias_model)
    print(f"[train] done. bias_model at: {sim._bias_model_path}")

if __name__ == "__main__":
    main()