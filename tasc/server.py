import math
import json
import asyncio
import time
import os

from dataclasses import dataclass
from collections import deque
from typing import Optional, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles


# =========================
# Data classes
# =========================

@dataclass
class Vehicle:
    name: str = "EMU-233-JR-East"
    mass_t: float = 200.0
    a_max: float = 1.0
    j_max: float = 0.8
    notches: int = 9
    notch_accels: list = None
    tau_cmd: float = 0.150
    tau_brk: float = 0.250
    mass_kg: float = 200000.0
    C_rr: float = 0.005
    rho_air: float = 1.225
    Cd: float = 1.8
    A: float = 10.0

    def update_mass(self, length: int):
        """편성 량 수에 맞춰 총 질량(kg)을 업데이트"""
        self.mass_kg = self.mass_t * 1000 * length

    @classmethod
    def from_json(cls, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        mass_t = data.get("mass_t", 200.0)
        return cls(
            name=data.get("name", "EMU-233-JR-East"),
            mass_t=mass_t,
            a_max=data.get("a_max", 1.0),
            j_max=data.get("j_max", 0.8),
            notches=data.get("notches", 8),
            notch_accels=data.get(
                "notch_accels",
                [-1.5, -1.10, -0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.0],
            ),
            tau_cmd=data.get("tau_cmd_ms", 150) / 1000.0,
            tau_brk=data.get("tau_brk_ms", 250) / 1000.0,
            mass_kg=mass_t * 1000,
            C_rr=0.005,
            rho_air=1.225,
            Cd=1.8,
            A=10.0,
        )


@dataclass
class Scenario:
    L: float = 500.0
    v0: float = 25.0
    grade_percent: float = 0.0
    mu: float = 1.0
    dt: float = 0.005

    @classmethod
    def from_json(cls, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        v0_kmph = data.get("v0", 25.0)
        v0_ms = v0_kmph / 3.6
        return cls(
            L=data.get("L", 500.0),
            v0=v0_ms,
            grade_percent=data.get("grade_percent", 0.0),
            mu=data.get("mu", 1.0),
            dt=data.get("dt", 0.005),
        )


@dataclass
class State:
    t: float = 0.0
    s: float = 0.0
    v: float = 0.0
    a: float = 0.0
    lever_notch: int = 0
    finished: bool = False
    stop_error_m: Optional[float] = None
    residual_speed_kmh: Optional[float] = None
    score: Optional[int] = None
    running: bool = False


# =========================
# Helpers
# =========================

def build_vref(L: float, a_ref: float):
    def vref(s: float):
        rem = max(0.0, L - s)
        return math.sqrt(max(0.0, 2.0 * a_ref * rem))
    return vref


# =========================
# Simulator
# =========================

class StoppingSim:
    def __init__(self, veh: Vehicle, scn: Scenario):
        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self.vref = build_vref(scn.L, 0.8 * veh.a_max)
        self._cmd_queue = deque()

        # 첫 제동(B1/B2) 판정용
        self.first_brake_start = None
        self.first_brake_done = False

        # 기록
        self.notch_history: List[int] = []
        self.time_history: List[float] = []

        # EB 사용 여부
        self.eb_used = False

        # 저크 평가
        self.prev_a = 0.0
        self.jerk_history: List[float] = []

        # ---------- TASC (자동 정차) ----------
        self.tasc_enabled = False
        self.manual_override = False
        self.tasc_deadband_m = 2.0        # 목표 대비 여유(m) → 헌팅 방지
        self.tasc_hold_min_s = 0.40       # 기본 노치 최소 유지시간(s)
        self._tasc_last_change_t = 0.0

    # 저노치 구간 홀드시간(마무리 급완해 방지)
    def _tasc_hold_required(self, notch: int) -> float:
        if notch in (2, 3):      # B2, B3
            return 0.90
        if notch == 1:           # B1은 약간 길게
            return 0.60
        return 0.40              # 그외 기본

    def _clamp_notch(self, n: int) -> int:
        return max(0, min(self.veh.notches - 1, n))

    def queue_command(self, name: str, val: int = 0):
        self._cmd_queue.append(
            {"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val}
        )

    def _apply_command(self, cmd: dict):
        st = self.state
        name = cmd["name"]
        val = cmd["val"]

        if name == "stepNotch":
            old_notch = st.lever_notch
            st.lever_notch = self._clamp_notch(st.lever_notch + val)
            print(f"Applied stepNotch: {old_notch} -> {st.lever_notch}")
        elif name == "release":
            st.lever_notch = 0
        elif name == "emergencyBrake":
            st.lever_notch = self.veh.notches - 1
            self.eb_used = True

    def reset(self):
        self.state = State(
            t=0.0, s=0.0, v=self.scn.v0, a=0.0, lever_notch=0, finished=False
        )
        self.running = False
        self._cmd_queue.clear()

        # 초기화 (B1/B2 초제동용)
        self.first_brake_start = None
        self.first_brake_done = False

        # 기록 초기화
        self.notch_history.clear()
        self.time_history.clear()

        # 저크 초기화
        self.prev_a = 0.0
        self.jerk_history = []

        # 수동 개입 플래그 초기화(오토 유지)
        self.manual_override = False
        # self.tasc_enabled 은 유지 (UI 토글 상태 그대로)

        print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True
        print("Simulation started")

    def eb_used_from_history(self) -> bool:
        """기록에서 EB(최대 인덱스) 사용 여부 확인"""
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # --------- TASC Core: 원하는 노치 선택 ----------
    def _tasc_choose_notch(self, v: float, rem: float) -> int:
        """
        현재 속도 v(m/s), 남은거리 rem(m)에서
        s_brake = v^2 / (2*|a|) >= rem - margin
        을 만족하는 '가장 낮은 노치'를 반환.
        0=N, 1=B1 ... 마지막=EB
        """
        margin = max(0.0, rem - self.tasc_deadband_m)
        if rem <= 0.0:
            return 1  # 도착 직전~이하: B1 마무리

        best = 0  # 기본 완해
        for notch, a in enumerate(self.veh.notch_accels):
            if notch == 0:
                continue  # N 스킵
            if a >= 0:
                continue  # 제동 아닌 값 무시
            s_brake = (v * v) / (2.0 * abs(a))
            if s_brake >= margin:
                best = notch
                break
        return best

    def step(self):
        st = self.state
        dt = self.scn.dt

        # 예약된 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 기록 & 초제동(B1/B2) 1초 체크
        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)

        if not self.first_brake_done:
            if st.lever_notch in (1, 2):  # B1 또는 B2
                if self.first_brake_start is None:
                    self.first_brake_start = time.time()
                elif time.time() - self.first_brake_start >= 1.0:
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ---------- TASC 자동 제동 ----------
        if self.tasc_enabled and not self.manual_override and not st.finished:
            # 노치별 최소 유지시간 적용
            hold_need = self._tasc_hold_required(st.lever_notch)
            dwell_ok = (st.t - self._tasc_last_change_t) >= hold_need

            rem_now = self.scn.L - st.s
            speed_kmh = st.v * 3.6

            if not self.first_brake_done:
                # 속도 70km/h 이상이면 초제동은 B2 이상으로 시작
                desired = 2 if speed_kmh >= 70.0 else 1
            else:
                desired = self._tasc_choose_notch(st.v, rem_now)

                # 마무리 바이어스(완화): 너무 일찍 B1 강제하지 않기
                if rem_now < 2.0 and speed_kmh < 3.0:
                    desired = 1

            if dwell_ok and desired != st.lever_notch:
                new_notch = desired

                # *** 완해 방향 제한: 한 번에 1단계 + 85% 소진 gate ***
                if new_notch < st.lever_notch:
                    cur_a = self.veh.notch_accels[st.lever_notch]
                    if cur_a < 0:
                        s_cur = (st.v * st.v) / (2.0 * abs(cur_a))
                    else:
                        s_cur = float('inf')

                    # 현재 노치로 설 수 있는 거리의 85%는 써야 다음 단계로 내려감
                    release_gate_ok = rem_now <= 0.85 * s_cur

                    if not release_gate_ok:
                        new_notch = st.lever_notch  # 아직 일러서 유지
                    else:
                        # 한 단계만 내리기
                        new_notch = max(new_notch, st.lever_notch - 1)

                # *** 강화 방향도 한 단계만(승차감 보호) ***
                if new_notch > st.lever_notch:
                    new_notch = min(new_notch, st.lever_notch + 1)

                if new_notch != st.lever_notch:
                    st.lever_notch = self._clamp_notch(new_notch)
                    self._tasc_last_change_t = st.t

        # 제동 감속 (음수, μ 적용)
        if st.lever_notch < len(self.veh.notch_accels):
            a_brake = self.veh.notch_accels[st.lever_notch] * self.scn.mu
        else:
            a_brake = 0.0

        # 경사 가속도 (보정 포함)
        a_grade = -9.81 * (self.scn.grade_percent / 100.0)
        a_grade /= 10.0

        # 구름 저항
        g = 9.81
        if st.v > 0:
            a_rr = -g * self.veh.C_rr
        elif st.v < 0:
            a_rr = g * self.veh.C_rr
        else:
            a_rr = 0.0

        # 공기 저항
        if st.v > 0:
            F_drag = 0.5 * self.veh.rho_air * self.veh.Cd * self.veh.A * st.v * st.v
            a_drag = -F_drag / self.veh.mass_kg
        else:
            a_drag = 0.0

        # 목표 가속도
        a_target = a_brake + a_grade + a_rr + a_drag

        # 1차 지연 응답
        st.a += (a_target - st.a) * (dt / max(1e-6, self.veh.tau_brk))

        # 저크 제한
        a_desired = a_brake + a_grade + a_rr + a_drag
        max_da = self.veh.j_max * dt
        da = a_desired - st.a
        if da > max_da:
            da = max_da
        elif da < -max_da:
            da = -max_da
        st.a += da

        # 적분
        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        # 종료 판정
        rem = self.scn.L - st.s
        if not st.finished and (rem <= -5.0 or st.v <= 0.0):
            st.finished = True
            st.stop_error_m = self.scn.L - st.s
            st.residual_speed_kmh = st.v * 3.6

            score = 0
            st.issues = {}

            # EB 사용 감점
            if self.eb_used or self.eb_used_from_history():
                score -= 500
                st.issues["unnecessary_eb_usage"] = True

            # 초제동(B1/B2 1초) 보너스/감점
            if not self.first_brake_done:
                score -= 100
            else:
                score += 300

            # 정차시 B1 여부
            last_notch = self.notch_history[-1] if self.notch_history else 0
            if last_notch == 1:
                score += 300
                st.issues["stop_not_b1"] = False
                st.issues["stop_not_b1_msg"] = "정차 시 B1로 정차함 - 승차감 양호"
            elif last_notch == 0:
                score -= 100
                st.issues["stop_not_b1"] = True
                st.issues["stop_not_b1_msg"] = "정차 시 N으로 정차함 - 열차 미끄러짐 주의"
            else:
                score -= 100
                st.issues["stop_not_b1"] = True
                st.issues["stop_not_b1_msg"] = "정차 시 B2 이상으로 정차함 - 승차감 불쾌"

            # 계단 패턴 점수
            if self.is_stair_pattern(self.notch_history):
                score += 500

            # 정지 오차 점수 (0m → 500점, 10m → 0점)
            err_abs = abs(st.stop_error_m or 0.0)
            error_score = max(0, 500 - int(err_abs * 500))
            score += error_score

            # ★ 0 cm 정차 보너스(±0.5cm 이내면 +100)
            if abs(st.stop_error_m or 0.0) <= 0.005:
                score += 100

            # 이슈 플래그들
            st.issues["early_brake_too_short"] = not self.first_brake_done
            st.issues["step_brake_incomplete"] = not self.is_stair_pattern(self.notch_history)
            st.issues["stop_error_m"] = st.stop_error_m

            # 저크 기록
            jerk = abs((st.a - self.prev_a) / dt)
            if jerk > 30:
                print(f"High jerk detected: {jerk:.3f} at t={st.t:.3f}")
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            # 저크 점수 반영
            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False
            print(f"Avg jerk: {avg_jerk:.4f}, jerk_score: {jerk_score:.2f}, final score: {score}")
            print(f"Simulation finished. Score: {score}")

    def is_stair_pattern(self, notches: List[int]) -> bool:
        """
        초제동(B1/B2)에서 시작해 한 번만 올라갔다 내려오고, 마지막이 B1인지 체크.
        - TASC ON & 수동개입 없음: 단계 건너뛰어도(증감폭>1) 허용 (오토파일럿 패턴 인정)
        - 사람이 개입(manual_override=True)하거나 TASC OFF: 기존처럼 한 단계씩만 허용
        """
        if len(notches) < 3:
            return False

        # 최초 유효 노치: B1 또는 B2 허용
        first_brake_notch = None
        for n in notches:
            if n > 0:
                first_brake_notch = n
                break
        if first_brake_notch not in (1, 2):
            return False

        allow_skip = self.tasc_enabled and not self.manual_override  # 오토파일럿은 건너뛰기 허용
        peak_reached = False
        prev = notches[0]

        for i in range(1, len(notches)):
            cur = notches[i]
            diff = cur - prev

            # 수동 상황에서는 한 단계씩
            if not allow_skip and abs(diff) > 1:
                return False

            if not peak_reached:
                # 상승 구간: 감소가 나오면 내리막 시작
                if cur < prev:
                    peak_reached = True
            else:
                # 내리막 구간: 다시 오르면 실패
                if cur > prev:
                    return False

            prev = cur

        # 마지막은 반드시 B1
        if notches[-1] != 1:
            return False

        return True

    def compute_jerk_score(self):
        """최근 1초 평균 저크 기반 점수 (<=25: 500점, 25~50: 선형감점, >50: 0점)"""
        dt = self.scn.dt
        window_time = 1.0
        n = int(window_time / dt)
        recent_jerks = self.jerk_history[-n:] if len(self.jerk_history) >= n else self.jerk_history

        if not recent_jerks:
            print("jerk_history is empty")
            return 0.0, 0

        avg_jerk = sum(recent_jerks) / len(recent_jerks)
        high_jerk_count = sum(1 for j in recent_jerks if j > 30)
        penalty_factor = min(1, high_jerk_count / 10)
        adjusted_jerk = avg_jerk * (1 + penalty_factor)

        print(
            f"avg_jerk={avg_jerk:.2f}, high_jerk_count={high_jerk_count}, "
            f"penalty_factor={penalty_factor:.2f}, adjusted_jerk={adjusted_jerk:.2f}"
        )

        if adjusted_jerk <= 25:
            jerk_score = 500
        elif adjusted_jerk <= 50:
            jerk_score = 500 * (50 - adjusted_jerk) / 25
        else:
            jerk_score = 0

        return adjusted_jerk, jerk_score

    def snapshot(self):
        st = self.state
        return {
            "t": round(st.t, 3),
            "s": st.s,
            "v": st.v,
            "a": st.a,
            "lever_notch": st.lever_notch,
            "remaining_m": self.scn.L - st.s,
            "L": self.scn.L,
            "v_ref": self.vref(st.s),
            "finished": st.finished,
            "stop_error_m": st.stop_error_m,
            "residual_speed_kmh": st.v * 3.6,
            "running": self.running,
            "grade_percent": self.scn.grade_percent,
            "grade": getattr(st, "grade", None),
            "score": getattr(st, "score", 0),
            "issues": getattr(st, "issues", {}),
            "tasc_enabled": getattr(self, "tasc_enabled", False),
        }


# =========================
# FastAPI app
# =========================

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/")
async def root():
    return HTMLResponse(open("static/index.html", "r", encoding="utf-8").read())


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(BASE_DIR, "vehicle.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    # 프론트가 EB→...→N으로 올 때 서버는 N→...→EB로 쓰기 위해 반전
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.start()

    last_sim_time = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - last_sim_time

            try:
                msg = await asyncio.wait_for(ws.receive_text(), timeout=0.01)
                data = json.loads(msg)
                if data.get("type") == "cmd":
                    payload = data["payload"]
                    name = payload.get("name")

                    if name == "setInitial":
                        speed = payload.get("speed")
                        dist = payload.get("dist")
                        grade = payload.get("grade", 0.0)
                        mu = payload.get("mu", 1.0)
                        if speed is not None and dist is not None:
                            sim.scn.v0 = float(speed) / 3.6
                            sim.scn.L = float(dist)
                            sim.scn.grade_percent = float(grade)
                            sim.scn.mu = float(mu)
                            sim.reset()

                    elif name == "start":
                        sim.start()

                    elif name in ("stepNotch", "applyNotch"):
                        delta = int(payload.get("delta", 0))
                        # 수동 개입 → TASC 해제
                        sim.manual_override = True
                        sim.tasc_enabled = False
                        sim.queue_command("stepNotch", delta)

                    elif name == "release":
                        sim.manual_override = True
                        sim.tasc_enabled = False
                        sim.queue_command("release", 0)

                    elif name == "emergencyBrake":
                        sim.manual_override = True
                        sim.tasc_enabled = False
                        sim.queue_command("emergencyBrake", 0)

                    elif name == "setTrainLength":
                        length = int(payload.get("length", 8))
                        vehicle.update_mass(length)
                        print(f"Train length set to {length} cars.")
                        sim.reset()

                    elif name == "setMassTons":
                        mass_tons = float(payload.get("mass_tons", 200.0))
                        # 차량 1량 질량 갱신(정보 목적), 총질량은 mass_kg에 반영
                        vehicle.mass_t = mass_tons / int(payload.get("length", 8))
                        vehicle.mass_kg = mass_tons * 1000.0
                        print(f"차량 전체 중량을 {mass_tons:.2f} 톤으로 업데이트 했습니다.")
                        sim.reset()

                    elif name == "setTASC":
                        enabled = bool(payload.get("enabled", False))
                        sim.tasc_enabled = enabled
                        if enabled:
                            sim.manual_override = False
                        print(f"TASC set to {enabled}")

                    elif name == "reset":
                        sim.reset()

            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                break
            except Exception as e:
                print(f"Error during receive: {e}")

            # 시뮬 step
            if elapsed >= sim.scn.dt:
                if sim.running:
                    sim.step()
                last_sim_time = now

            await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass