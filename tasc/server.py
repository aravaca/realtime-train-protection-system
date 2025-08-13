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
    mass_t: float = 200.0     # 전체 편성 기준 톤(초기값)
    a_max: float = 1.0
    j_max: float = 0.8
    notches: int = 9
    notch_accels: list = None   # 인덱스: 0..(notches-1), 음수 감속, 0은 N, 최댓값은 EB
    tau_cmd: float = 0.150
    tau_brk: float = 0.250
    mass_kg: float = 200000.0
    C_rr: float = 0.005
    rho_air: float = 1.225
    Cd: float = 1.8
    A: float = 10.0

    def update_mass(self, length: int):
        """편성 량 수에 맞춰 총 질량(kg) 업데이트 (mass_t = 1량 톤수 * length 아님 주의)"""
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
            notches=data.get("notches", 9),
            notch_accels=data.get(
                "notch_accels",
                [-1.5, -1.10, -0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.0],
            ),
            tau_cmd=data.get("tau_cmd_ms", 150) / 1000.0,
            tau_brk=data.get("tau_brk_ms", 250) / 1000.0,
            mass_kg=mass_t * 1000,
            C_rr=data.get("C_rr", 0.005),
            rho_air=data.get("rho_air", 1.225),
            Cd=data.get("Cd", 1.8),
            A=data.get("A", 10.0),
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
        v0_kmph = data.get("v0", 90.0)
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


def _mu_to_rr_factor(mu: float) -> float:
    """
    μ가 낮을수록(미끄러울수록) 구름저항 스케일을 낮춰 코스팅에서 더 미끄러지게.
    맑음 μ=1.0 → 1.0, 비 μ=0.6 → ~0.88, 눈 μ=0.3 → ~0.79
    """
    mu_clamped = max(0.0, min(1.0, float(mu)))
    return 0.7 + 0.3 * mu_clamped


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

        # 초제동(B1/B2) 판정용
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

        # ---------- TASC ----------
        self.tasc_enabled = False
        self.manual_override = False
        self.tasc_deadband_m = 0.5
        self.tasc_hold_min_s = 0.25
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1

        # 날씨→코스팅 영향
        self.rr_factor = _mu_to_rr_factor(self.scn.mu)

        # 접착 한계(서비스/EB), μ 효율 보정
        self.k_adh_srv = 0.70
        self.k_adh_eb  = 0.90
        self.alpha_mu_eff = 0.20  # 낮은 μ에서 장비 효율이 다소 떨어지는 폭

    def _clamp_notch(self, n: int) -> int:
        return max(0, min(self.veh.notches - 1, n))

    def queue_command(self, name: str, val: int = 0):
        self._cmd_queue.append({"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val})

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
        self.state = State(t=0.0, s=0.0, v=self.scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self._cmd_queue.clear()

        # 초기화
        self.first_brake_start = None
        self.first_brake_done = False
        self.notch_history.clear()
        self.time_history.clear()
        self.prev_a = 0.0
        self.jerk_history = []

        self.manual_override = False
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1

        print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True
        print("Simulation started")

    def eb_used_from_history(self) -> bool:
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # ------ stopping distance helper ------
    def _stopping_distance(self, notch: int, v: float) -> float:
        """TASC 판단용 정지거리 (장비 테이블 a와 μ 하한 사용)"""
        if notch <= 0 or notch >= len(self.veh.notch_accels):
            return float('inf')
        a = self.veh.notch_accels[notch]
        if a >= 0:
            return float('inf')
        mu = max(0.1, float(self.scn.mu))  # 판단 안정성
        return (v * v) / (2.0 * abs(a) * mu)

    # μ가 장비 효율에 주는 완만한 영향
    def _f_mu_efficiency(self, mu: float) -> float:
        mu_clamped = max(0.0, min(1.0, float(mu)))
        return 1.0 - self.alpha_mu_eff * (1.0 - mu_clamped)

    def step(self):
        st = self.state
        dt = self.scn.dt

        # 예약 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 기록 & 초제동(B1/B2) 체크
        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)
        if not self.first_brake_done:
            if st.lever_notch in (1, 2):
                if self.first_brake_start is None:
                    self.first_brake_start = time.time()
                elif time.time() - self.first_brake_start >= 1.0:
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ---------- TASC ----------
        if self.tasc_enabled and not self.manual_override and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            speed_kmh = st.v * 3.6
            cur = st.lever_notch
            max_normal_notch = self.veh.notches - 2  # EB-1까지

            if not self.first_brake_done:
                desired = 2 if speed_kmh >= 70.0 else 1
                if dwell_ok and cur != desired:
                    step = 1 if desired > cur else -1
                    st.lever_notch = self._clamp_notch(cur + step)
                    self._tasc_last_change_t = st.t
            else:
                s_cur = self._stopping_distance(cur, st.v) if cur > 0 else float('inf')
                s_dn  = self._stopping_distance(cur - 1, st.v) if cur - 1 >= 1 else float('inf')
                changed = False
                if self._tasc_phase == "build":
                    if cur < max_normal_notch and s_cur > (rem_now - self.tasc_deadband_m):
                        if dwell_ok:
                            st.lever_notch = self._clamp_notch(cur + 1)
                            self._tasc_last_change_t = st.t
                            self._tasc_peak_notch = max(self._tasc_peak_notch, st.lever_notch)
                            changed = True
                    else:
                        self._tasc_phase = "relax"
                if self._tasc_phase == "relax" and not changed:
                    if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m):
                        if dwell_ok:
                            st.lever_notch = self._clamp_notch(cur - 1)
                            self._tasc_last_change_t = st.t

        # ====== 동역학 ======
        # 브레이크 감속 (클램프: 장비 vs 접착)
        if st.lever_notch < len(self.veh.notch_accels):
            base = self.veh.notch_accels[st.lever_notch]  # 음수
            eff = self._f_mu_efficiency(self.scn.mu)
            a_equipment = base * eff
            k_adh = self.k_adh_eb if st.lever_notch == (self.veh.notches - 1) else self.k_adh_srv
            a_cap = -k_adh * self.scn.mu * 9.81
            a_brake = max(a_equipment, a_cap)  # (덜 음수) 선택
        else:
            a_brake = 0.0

        # 경사 (보정 포함)
        a_grade = -9.81 * (self.scn.grade_percent / 100.0)
        a_grade /= 10.0

        # 구름 저항 (날씨 반영)
        g = 9.81
        Crr_eff = self.veh.C_rr * self.rr_factor
        if st.v > 0:
            a_rr = -g * Crr_eff
        elif st.v < 0:
            a_rr =  g * Crr_eff
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

        # 1차 지연 + 저크 제한
        st.a += (a_target - st.a) * (dt / max(1e-6, self.veh.tau_brk))
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

            if self.eb_used or self.eb_used_from_history():
                score -= 500
                st.issues["unnecessary_eb_usage"] = True

            if not self.first_brake_done:
                score -= 100
            else:
                score += 300

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

            if self.is_stair_pattern(self.notch_history):
                score += 500
            else:
                if self.tasc_enabled and not self.manual_override:
                    score += 500

            err_abs = abs(st.stop_error_m or 0.0)
            error_score = max(0, 500 - int(err_abs * 500))
            score += error_score

            if abs(st.stop_error_m or 0.0) < 0.01:
                score += 100

            st.issues["early_brake_too_short"] = not self.first_brake_done
            st.issues["step_brake_incomplete"] = not self.is_stair_pattern(self.notch_history)
            st.issues["stop_error_m"] = st.stop_error_m

            jerk = abs((st.a - self.prev_a) / dt)
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False
            print(f"Simulation finished. Score: {score}")

    def is_stair_pattern(self, notches: List[int]) -> bool:
        if len(notches) < 3:
            return False
        first_brake_notch = None
        for n in notches:
            if n > 0:
                first_brake_notch = n
                break
        if first_brake_notch not in (1, 2):
            return False
        peak_reached = False
        prev = notches[0]
        for i in range(1, len(notches)):
            cur = notches[i]
            diff = cur - prev
            if abs(diff) > 1:
                return False
            if not peak_reached:
                if cur < prev:
                    peak_reached = True
            else:
                if cur > prev:
                    return False
            prev = cur
        if notches[-1] != 1:
            return False
        return True

    def compute_jerk_score(self):
        dt = self.scn.dt
        window_time = 1.0
        n = int(window_time / dt)
        recent_jerks = self.jerk_history[-n:] if len(self.jerk_history) >= n else self.jerk_history
        if not recent_jerks:
            return 0.0, 0
        avg_jerk = sum(recent_jerks) / len(recent_jerks)
        high_jerk_count = sum(1 for j in recent_jerks if j > 30)
        penalty_factor = min(1, high_jerk_count / 10)
        adjusted_jerk = avg_jerk * (1 + penalty_factor)
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
            "score": getattr(st, "score", 0),
            "issues": getattr(st, "issues", {}),
            "tasc_enabled": getattr(self, "tasc_enabled", False),
            # HUD 동기화용 파라미터
            "mu": float(self.scn.mu),
            "rr_factor": float(self.rr_factor),
            "veh": {
                "name": self.veh.name,
                "mass_kg": self.veh.mass_kg,
                "C_rr": self.veh.C_rr,
                "rho_air": self.veh.rho_air,
                "Cd": self.veh.Cd,
                "A": self.veh.A,
                "notches": self.veh.notches,
                # 서버 내부 인덱싱(N..EB)에 맞춘 음수 감속 테이블 그대로 전달
                "notch_accels": self.veh.notch_accels,
            },
            "adhesion": {
                "k_srv": self.k_adh_srv,
                "k_eb": self.k_adh_eb,
                "alpha_eff": self.alpha_mu_eff,
            },
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
    # 프론트는 N..EB 기준으로 그릴 수 있게 이 배열을 그대로 사용 (서버 시뮬도 N..EB 사용)
    # 필요시 기존 역순 로직 제거
    # vehicle.notch_accels = list(reversed(vehicle.notch_accels))  # 이제 불필요

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
                        mu = float(payload.get("mu", 1.0))
                        if speed is not None and dist is not None:
                            sim.scn.v0 = float(speed) / 3.6
                            sim.scn.L = float(dist)
                            sim.scn.grade_percent = float(grade)
                            sim.scn.mu = mu
                            sim.rr_factor = _mu_to_rr_factor(mu)
                            print(f"setInitial: v0={speed}km/h, L={dist}m, grade={grade}%, mu={mu}, rr_factor={sim.rr_factor:.3f}")
                            sim.reset()

                    elif name == "start":
                        sim.start()

                    elif name in ("stepNotch", "applyNotch"):
                        delta = int(payload.get("delta", 0))
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
                        print(f"Train length set to {length} cars (mass_kg={vehicle.mass_kg:.0f}).")
                        sim.reset()

                    elif name == "setMassTons":
                        mass_tons = float(payload.get("mass_tons", 200.0))
                        vehicle.mass_t = mass_tons / int(payload.get("length", 8) or 8)
                        vehicle.mass_kg = mass_tons * 1000.0
                        print(f"차량 전체 중량을 {mass_tons:.2f} 톤으로 업데이트 했습니다.")
                        sim.reset()

                    elif name == "setTASC":
                        enabled = bool(payload.get("enabled", False))
                        sim.tasc_enabled = enabled
                        if enabled:
                            sim.manual_override = False
                            sim._tasc_last_change_t = sim.state.t
                            sim._tasc_phase = "build"
                            sim._tasc_peak_notch = 1
                        print(f"TASC set to {enabled}")

                    elif name == "setMu":
                        value = float(payload.get("value", 1.0))
                        sim.scn.mu = value
                        sim.rr_factor = _mu_to_rr_factor(value)
                        print(f"마찰계수(mu)={value} / rr_factor={sim.rr_factor:.3f} 로 설정")
                        sim.reset()

                    elif name == "reset":
                        sim.reset()

            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                break
            except Exception as e:
                print(f"Error during receive: {e}")

            # 시뮬 시간 보정 루프
            dt = sim.scn.dt
            while elapsed >= dt:
                if sim.running:
                    sim.step()
                last_sim_time += dt
                elapsed -= dt

            await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass