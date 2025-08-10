# 

import math
import json
import asyncio
import time

from dataclasses import dataclass
from collections import deque
from typing import Optional, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

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
        # 질량을 량 수에 맞춰 업데이트
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
    score: Optional[int] = None  # 점수로 대체
    running: bool = False


def build_vref(L: float, a_ref: float):
    def vref(s: float):
        rem = max(0.0, L - s)
        return math.sqrt(max(0.0, 2.0 * a_ref * rem))

    return vref


def is_stair_pattern(notches: List[int]) -> bool:
    # 간단히 계단 패턴: 1 2 3 ... N ... 3 2 1 형태인지 체크
    # 혹은 1 2 3 4 5 4 3 2 1 과 같이 한 번만 완만하게 올라갔다 내려오는지 확인
    if len(notches) < 3:
        return False

    # 연속적으로 증가하다가 감소하는 패턴 확인
    increasing = True
    for i in range(1, len(notches)):
        if increasing:
            if notches[i] < notches[i - 1]:
                increasing = False
        else:
            if notches[i] > notches[i - 1]:
                return False

    # 증가 구간과 감소 구간 모두 계단으로 (차이가 1 혹은 0)
    for i in range(1, len(notches)):
        if abs(notches[i] - notches[i - 1]) > 1:
            return False

    return True


import math
from collections import deque


class StoppingSim:
    def __init__(self, veh, scn):
        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self.vref = build_vref(scn.L, 0.8 * veh.a_max)
        self._cmd_queue = deque()

        # 첫 B1 제동 관련 상태
        self.first_brake_b1_start = None
        self.first_brake_done = False

        # 노치 기록 (계단제동 체크용)
        self.notch_history = []
        self.time_history = []

        self.eb_used = False  # EB 사용 여부 기록

        self.prev_a = 0.0  # 이전 가속도 저장용
        self.jerk_history = []

    def _clamp_notch(self, n):
        return max(0, min(self.veh.notches - 1, n))

    def queue_command(self, name, val=0):
        self._cmd_queue.append(
            {"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val}
        )

    def _apply_command(self, cmd):
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
        self.first_brake_b1_start = None
        self.first_brake_done = False
        self.notch_history.clear()
        self.time_history.clear()
        self.prev_a = 0.0  # 초기 가속도
        self.jerk_history = []

        print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True
        print("Simulation started")

    def eb_used_from_history(self):
    # EB 노치 = 최대 노치 번호(예: 8) 사용 여부 체크
        return any(n == self.veh.notches - 1 for n in self.notch_history)


    def step(self):
        st = self.state
        dt = self.scn.dt

        # 예약된 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 노치 기록 & 첫 B1 제동 1초 체크
        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)

        # 예를 들어 self.first_brake_b1_start 초기값 None
        if not self.first_brake_done:
            if st.lever_notch == 1:
                if self.first_brake_b1_start is None:
                    self.first_brake_b1_start = time.time()  # 현재 시간 초 단위(실수)
                elif time.time() - self.first_brake_b1_start >= 1:
                    self.first_brake_done = True
            else:
                self.first_brake_b1_start = None


        # 제동 감속 (음수)
        a_brake = (
            self.veh.notch_accels[st.lever_notch] * self.scn.mu
            if st.lever_notch < len(self.veh.notch_accels)
            else 0.0
        )

        # 경사 가속도
        a_grade = -9.81 * (self.scn.grade_percent / 100.0)
        a_grade /= 10 #보정

        # Rolling resistance
        v = st.v
        g = 9.81
        a_rr = 0.0
        if v > 0:
            a_rr = -g * self.veh.C_rr
        elif v < 0:
            a_rr = g * self.veh.C_rr

        # Aerodynamic drag
        a_drag = 0.0
        if v > 0:
            F_drag = 0.5 * self.veh.rho_air * self.veh.Cd * self.veh.A * v * v
            a_drag = -F_drag / self.veh.mass_kg

        # 총 목표 가속도
        a_target = a_brake + a_grade + a_rr + a_drag

        # 1차 시스템 응답
        st.a += (a_target - st.a) * (dt / max(1e-6, self.veh.tau_brk))
        # 총 목표 가속도
        a_desired = a_brake + a_grade + a_rr + a_drag

        # 현재 가속도 변화량 한계
        max_da = self.veh.j_max * dt  # 최대 가속도 변화량 (저크 제한)

        # 가속도 변화량 계산
        da = a_desired - st.a

        # 가속도 변화량 제한
        if da > max_da:
            da = max_da
        elif da < -max_da:
            da = -max_da

        # 가속도 업데이트
        st.a += da

        # 속도, 위치 적분
        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        rem = self.scn.L - st.s
        if not st.finished and (rem <= -5.0 or st.v <= 0.0):
            st.finished = True
            st.stop_error_m = self.scn.L - st.s
            st.residual_speed_kmh = st.v * 3.6

            # 점수 계산 시작
            score = 0

            st.issues = {}
            # EB 사용 시 감점
            if self.eb_used or self.eb_used_from_history():
                score -= 500
                st.issues["unnecessary_eb_usage"] = True
            else:
                if "unnecessary_eb_usage" in st.issues:
                    del st.issues["unnecessary_eb_usage"]



            # 첫 제동 B1 1초 미만이면 감점
            if not self.first_brake_done:
                score -= 100
            else:
                score += 300

            # 마지막 노치가 B1(1)인지 체크 (정차 시 필수)
            last_notch = self.notch_history[-1] if self.notch_history else 0
            if last_notch == 1:
                score += 300
            else:
                score -= 100

            # 마지막 노치 상태 저장 (정차 시)
            if last_notch == 1:
                st.issues["stop_not_b1"] = False  # 문제 없음
                st.issues["stop_not_b1_msg"] = "정차 시 B1로 정차함 - 승차감 양호"
            elif last_notch == 0:
                st.issues["stop_not_b1"] = True
                st.issues["stop_not_b1_msg"] = "정차 시 N으로 정차함 - 열차 미끄러짐 주의"
            else:
                st.issues["stop_not_b1"] = True
                st.issues["stop_not_b1_msg"] = "정차 시 B2 이상으로 정차함 - 승차감 불쾌"


            # 계단 패턴인지 체크
            if self.is_stair_pattern(self.notch_history):
                score += 500

            # 정차 오차에 따른 점수 (0~500)
            err_abs = abs(st.stop_error_m or 0.0)
            error_score = max(0, 500 - int(err_abs * 500))  # 오차 0m → 500점, 10m → 0점
            score += error_score

            # 초기 제동 체크
            st.issues["early_brake_too_short"] = not self.first_brake_done

            # 마지막 노치가 B1(1)인지 여부
            last_notch = self.notch_history[-1] if self.notch_history else 0
            st.issues["stop_not_b1"] = last_notch != 1

            # 계단 패턴 체크
            st.issues["step_brake_incomplete"] = not self.is_stair_pattern(
                self.notch_history
            )

            st.issues["stop_error_m"] = st.stop_error_m

            if not hasattr(self, "prev_a"):
                self.prev_a = 0.0
            if not hasattr(self, "jerk_history"):
                self.jerk_history = []

            # 가속도 변화량(저크) 계산
            jerk = abs((st.a - self.prev_a) / dt)
            if jerk > 30:
                print(f"High jerk detected: {jerk:.3f} at t={st.t:.3f}")
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            print(self.jerk_history)
            if st.finished:
                avg_jerk, jerk_score = self.compute_jerk_score()
                score += int(jerk_score)
                st.score = score
                print(f"Avg jerk: {avg_jerk:.4f}, jerk_score: {jerk_score:.2f}, final score: {score}")
                self.running = False

                print(f"Simulation finished. Score: {score}")




    def is_stair_pattern(self, notches: List[int]) -> bool:
        if len(notches) < 3:
            return False

        # 최초 제동 노치는 반드시 1이어야 함
        first_brake_notch = None
        for n in notches:
            if n > 0:
                first_brake_notch = n
                break
        if first_brake_notch != 1:
            return False

        # 노치가 1에서 시작해서 올라갔다가 다시 1로 내려오는 산 모양인지 체크
        peak_reached = False
        prev = notches[0]

        for i in range(1, len(notches)):
            cur = notches[i]
            diff = cur - prev

            # 노치 증감이 1단계 이하인지
            if abs(diff) > 1:
                return False

            if not peak_reached:
                # 올라가는 중 (증가 또는 동일)
                if cur < prev:
                    peak_reached = True  # 내리막 시작
            else:
                # 내려가는 중
                if cur > prev:
                    return False  # 내렸다가 다시 올라가면 실패

            prev = cur

        # 마지막 노치는 반드시 1이어야 함 (정차 B1 조건)
        if notches[-1] != 1:
            return False

        return True
        
    def compute_jerk_score(self):
        dt = self.scn.dt
        window_time = 1.0
        n = int(window_time / dt)
        recent_jerks = self.jerk_history[-n:] if len(self.jerk_history) >= n else self.jerk_history

        if not recent_jerks:
            print("jerk_history is empty")
            return 0.0, 0

        avg_jerk = sum(recent_jerks) / len(recent_jerks)
        high_jerk_count = sum(1 for j in recent_jerks if j > 30)  # 임계치 30

        penalty_factor = min(1, high_jerk_count / 10)  # 10회 이상 급격한 저크 발생 시 페널티 최대
        adjusted_jerk = avg_jerk * (1 + penalty_factor)

        print(f"avg_jerk={avg_jerk:.2f}, high_jerk_count={high_jerk_count}, penalty_factor={penalty_factor:.2f}, adjusted_jerk={adjusted_jerk:.2f}")

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
            "residual_speed_kmh": st.residual_speed_kmh,
            "running": self.running,
            "grade_percent": self.scn.grade_percent,
            "grade": getattr(st, "grade", None),
            "score": getattr(st, "score", 0),
            "issues": getattr(st, "issues", {}),
        }


app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")
@app.get("/")
async def root():
    return HTMLResponse(open("static/index.html", "r", encoding="utf-8").read())


import time  # 최상단 임포트 위치에 추가해도 무방

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()

    import os

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(BASE_DIR, "vehicle.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
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
                        grade = payload.get("grade", 0.0)  # 기본값 0.0
                        mu = payload.get("mu", 1.0)
                        if speed is not None and dist is not None:
                            sim.scn.v0 = float(speed) / 3.6  # km/h -> m/s
                            sim.scn.L = float(dist)
                            sim.scn.grade_percent = float(grade)
                            sim.scn.mu = float(mu)
                            sim.reset()
                    elif name == "start":
                        sim.start()
                    elif name == "stepNotch" or name == "applyNotch":
                        delta = int(payload.get("delta", 0))
                        sim.queue_command("stepNotch", delta)
                    elif name == "release":
                        sim.queue_command("release", 0)
                    elif name == "emergencyBrake":
                        sim.queue_command("emergencyBrake", 0)
                    elif name == "setTrainLength":
                        length = int(payload.get("length", 8))  # 기본값 8량
                        vehicle.update_mass(length)  # 차량의 질량 업데이트
                        print(f"Train length set to {length} cars.") 
                        sim.reset()
                    elif name == "reset":
                        sim.reset()
            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                break
            except Exception as e:
                print(f"Error during receive: {e}")

            # 실제 시간 경과에 맞춰 시뮬레이션 step 실행
            if elapsed >= sim.scn.dt:
                if sim.running:
                    sim.step()
                last_sim_time = now

            await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
            await asyncio.sleep(0)  # 너무 빠른 루프 방지용 짧은 대기
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass
