# simcore/dynamics.py

import numpy as np


def run_sim(seg, veh, params, controller, input_fn=None):
    """
    seg: Segment (L, grade)
    veh: Vehicle (m, a_max, j_max, tau_cmd, tau_brk, notches)
    params: SimParams (dt, v0, mu)
    controller: controller instance with .step() method
    input_fn: optional function to override controller output (e.g. user input)

    Returns: dict trace with time series of t, s, v, a, notch
    """
    dt = params.dt
    t = 0.0
    s = 0.0
    v = params.v0
    a = 0.0
    notch = 0

    tau_cmd = veh.tau_cmd / 1000  # ms to s
    tau_brk = veh.tau_brk / 1000

    # buffers for delay simulation
    cmd_buffer = [0] * (int(tau_cmd / dt) + 1)
    a_buffer = [0] * (int(tau_brk / dt) + 1)

    trace = {"t": [], "s": [], "v": [], "a": [], "notch": []}

    max_steps = int(seg.L / (v * dt)) * 10  # heuristic max steps
    max_steps = max(max_steps, 10000)

    for step in range(max_steps):
        # 시간 기록
        trace["t"].append(t)
        trace["s"].append(s)
        trace["v"].append(v)
        trace["a"].append(a)
        trace["notch"].append(notch)

        # 정지 조건 (0.3m 이내, 속도 잔류 < 0.2km/h = 0.055m/s)
        if s >= seg.L - 0.3 and v <= 0.055:
            break

        # 사용자 입력이 있으면 override notch
        if input_fn:
            notch = input_fn(t, notch)
        else:
            # PI 컨트롤러로 노치 결정
            v_ref = 0  # 목표 속도 0 (정차)
            notch = controller.step(s, v, v_ref, dt)

        # 입력 명령 지연 적용
        cmd_buffer.append(notch)
        notch_cmd = cmd_buffer.pop(0)

        # 목표 가속도 (a_cmd) = 노치별 감속 (음수)
        a_cmd = veh.notch_accels[notch_cmd]

        # 응답 지연 1차 시스템 근사: a_dot = (mu * a_cmd(t-tau_cmd) - a) / tau_brk
        a_new = a + dt * (params.mu * a_cmd - a) / tau_brk

        # 저크 제한: |(a_new - a) / dt| <= j_max
        jerk = (a_new - a) / dt
        jerk_limit = veh.j_max
        if jerk > jerk_limit:
            a_new = a + jerk_limit * dt
        elif jerk < -jerk_limit:
            a_new = a - jerk_limit * dt

        # 속도 이산 적분
        v_new = max(0, v + a_new * dt)
        s_new = s + v * dt + 0.5 * a_new * dt * dt

        # 상태 업데이트
        a = a_new
        v = v_new
        s = s_new
        t += dt

    return trace
