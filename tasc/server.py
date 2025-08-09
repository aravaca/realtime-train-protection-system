import math, json, asyncio
from dataclasses import dataclass
from collections import deque
from typing import Optional, Deque, Dict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

import json

@dataclass
class Vehicle:
    name: str = "EMU-233-JR-East"
    mass_t: float = 200.0
    a_max: float = 1.0
    j_max: float = 0.8
    notches: int = 8
    notch_accels: list = None
    tau_cmd: float = 0.150
    tau_brk: float = 0.250
    mass_kg: float = 200000.0  # ton → kg 변환 (200t → 200,000kg)
    C_rr: float = 0.002  # rolling resistance coefficient
    rho_air: float = 1.225  # 공기 밀도 kg/m³
    Cd: float = 1.8  # 공기저항 계수
    A: float = 10.0  # 전면 투영 면적 m²

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
            notch_accels=data.get("notch_accels", [-1.10, -0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.0]),
            tau_cmd=data.get("tau_cmd_ms", 150) / 1000.0,
            tau_brk=data.get("tau_brk_ms", 250) / 1000.0,
            mass_kg=mass_t * 1000,  # ton → kg
            C_rr=0.002,
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
        v0_kmph = data.get("v0", 25.0)   # JSON 에서 km/h 단위로 입력 받음
        v0_ms = v0_kmph / 3.6            # km/h → m/s 변환
        return cls(
            L=data.get("L", 500.0),
            v0=v0_ms,
            grade_percent=data.get("grade_percent", 0.0),
            mu=data.get("mu", 1.0),
            dt=data.get("dt", 0.005)
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
    grade: Optional[str] = None


def build_vref(L: float, a_ref: float):
    def vref(s: float):
        rem = max(0.0, L - s)
        return math.sqrt(max(0.0, 2.0 * a_ref * rem))
    return vref


class StoppingSim:
    def __init__(self, veh: Vehicle, scn: Scenario):
        self.veh = veh
        self.scn = scn
        self.state = State()
        self.running = False
        # 보수적으로 a_max 기준으로 vref 설정
        self.vref = build_vref(scn.L, 0.8 * veh.a_max)
        self._cmd_queue: Deque[Dict] = deque()

    def _clamp_notch(self, n: int) -> int:
        return max(0, min(self.veh.notches - 1, n))

    def queue_command(self, name: str, val=0):
        if name == "stepNotch" or name == "applyNotch":
            name = "stepNotch"
            val = int(val)
        self._cmd_queue.append({
            "t": self.state.t + self.veh.tau_cmd,
            "name": name,
            "val": val
        })
        print(f"Queued command: {name} {val} at t={self.state.t + self.veh.tau_cmd:.3f}")

    def _apply_command(self, cmd: Dict):
        st = self.state
        name = cmd["name"]
        val = cmd["val"]

        if name == "stepNotch":
            old_notch = st.lever_notch
            st.lever_notch = self._clamp_notch(st.lever_notch + val)
            print(f"Applied stepNotch: {old_notch} -> {st.lever_notch}")
        elif name == "release":
            st.lever_notch = 0
            print("Applied release: lever_notch=0")
        elif name == "emergencyBrake":
            st.lever_notch = self.veh.notches - 1
            print(f"Applied emergencyBrake: lever_notch={st.lever_notch}")

    def reset(self):
        self.state = State(t=0.0, s=0.0, v=self.scn.v0, a=0.0, lever_notch=0, finished=False)
        self._cmd_queue.clear()
        print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True
        print("Simulation started")

    def step(self):
        st = self.state
        dt = self.scn.dt

        # 예약된 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 제동 감속 (음수) — notch_accels * mu
        a_brake = self.veh.notch_accels[st.lever_notch] * self.scn.mu if st.lever_notch < len(self.veh.notch_accels) else 0.0

        # 경사 가속도
        a_grade = -9.81 * (self.scn.grade_percent / 100.0)

        # Rolling resistance 가속도 (항상 감속, 속도 0이면 0)
        # a_rr = -g * C_rr * sign(v)
        v = st.v
        g = 9.81
        a_rr = 0.0
        if v > 0:
            a_rr = -g * self.veh.C_rr
        elif v < 0:
            a_rr = g * self.veh.C_rr  # 역방향(거꾸로) 가는 상황도 대비

        # Aerodynamic drag 가속도: a_drag = F_drag / m = (0.5 * rho * Cd * A * v^2) / m
        a_drag = 0.0
        if v > 0:
            F_drag = 0.5 * self.veh.rho_air * self.veh.Cd * self.veh.A * v * v
            a_drag = -F_drag / self.veh.mass_kg

        # 총 목표 가속도
        a_target = a_brake + a_grade + a_rr + a_drag

        # 1차 시스템 응답 모델 (시간 상수 tau_brk)
        st.a += (a_target - st.a) * (dt / max(1e-6, self.veh.tau_brk))

        # 속도, 위치 적분
        st.v = max(0.0, st.v + st.a * dt)  # 음수 속도 방지
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        rem = self.scn.L - st.s
        if not st.finished and (rem <= -5.0 or st.v <= 0.0):

            st.finished = True
            st.stop_error_m = self.scn.L - st.s  # 부호 유지: 목표 - 실제 위치
            st.residual_speed_kmh = st.v * 3.6

            err_abs = abs(st.stop_error_m or 0.0)

            if st.stop_error_m < 0:  # 오버런(목표지점 지나감) 시 더 엄격 평가
                if err_abs <= 0.2:
                    st.grade = "S"
                elif err_abs <= 0.3:
                    st.grade = "A"
                elif err_abs <= 0.5:
                    st.grade = "B"
                else:
                    st.grade = "F"
            else:  # 정상 범위 내 정지 시 원래 기준
                if err_abs <= 0.30:
                    st.grade = "S"
                elif err_abs <= 0.65:
                    st.grade = "A"
                elif err_abs <= 1.0:
                    st.grade = "B"
                elif err_abs <= 2.0:
                    st.grade = "C"
                elif err_abs <= 5.0:
                    st.grade = "F"
                else:
                    st.grade = "F"

            self.running = False
            print(f"Simulation finished with grade {st.grade}")


    def snapshot(self):
        st = self.state
        return {
            "t": round(st.t, 3),
            "s": st.s,
            "v": st.v,
            "a": st.a,
            "lever_notch": st.lever_notch,
            "remaining_m": self.scn.L - self.state.s,  # 음수 허용, -5까지 표시 가능
            "L": self.scn.L,
            "v_ref": self.vref(st.s),
            "finished": st.finished,
            "stop_error_m": st.stop_error_m,
            "residual_speed_kmh": st.residual_speed_kmh,
            "grade": st.grade,
            "running": self.running,
            "grade_percent": self.scn.grade_percent,   # 여기에 추가
        }



app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/")
async def root():
    return HTMLResponse(open("static/index.html", "r", encoding="utf-8").read())


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()

    import os

    scenario = Scenario()

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(BASE_DIR, "vehicle.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))

    scenario = Scenario.from_json(scenario_json_path)


    sim = StoppingSim(vehicle, scenario)
    sim.start()

    try:
        while True:
            try:
                msg = await asyncio.wait_for(ws.receive_text(), timeout=0.01)
                data = json.loads(msg)
                print(f"recv: {data}")
                if data.get("type") == "cmd":
                    payload = data["payload"]
                    name = payload.get("name")
                    if name == "start":
                        sim.start()
                    elif name == "stepNotch" or name == "applyNotch":
                        delta = int(payload.get("delta", 0))
                        sim.queue_command("stepNotch", delta)
                    elif name == "release":
                        sim.queue_command("release", 0)
                    elif name == "emergencyBrake":
                        sim.queue_command("emergencyBrake", 0)
                        print("Queued emergencyBrake")
                    elif name == "reset":
                        sim.reset()
                        print("Simulation reset")
            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                print("Client disconnected")
                break
            except Exception as e:
                print(f"Error during receive: {e}")

            if sim.running:
                sim.step()
                print(f"step: t={sim.state.t:.3f}, v={sim.state.v:.3f}, notch={sim.state.lever_notch}")

            await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
            await asyncio.sleep(0)
    finally:
        await ws.close()
