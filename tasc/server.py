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

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------
DEBUG = False # 디버그 로그를 보고 싶으면 True

# ------------------------------------------------------------
# Data classes
# ------------------------------------------------------------

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

    # Davis 계수 (열차 전체)
    A0: float = 1200.0
    B1: float = 30.0
    C2: float = 8.0

    # 공기계수 등
    C_rr: float = 0.005
    rho_air: float = 1.225
    Cd: float = 1.8
    A: float = 10.0

    def update_mass(self, length: int):
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
            A0=data.get("davis_A0", 1200.0),
            B1=data.get("davis_B1", 30.0),
            C2=data.get("davis_C2", 8.0),
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


# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------

def build_vref(L: float, a_ref: float):
    def vref(s: float):
        rem = max(0.0, L - s)
        return math.sqrt(max(0.0, 2.0 * a_ref * rem))
    return vref

def _mu_to_rr_factor(mu: float) -> float:
    mu_clamped = max(0.0, min(1.0, float(mu)))
    return 0.7 + 0.3 * mu_clamped


# ------------------------------------------------------------
# Simulator
# ------------------------------------------------------------

class StoppingSim:
    def __init__(self, veh: Vehicle, scn: Scenario):
        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self.vref = build_vref(scn.L, 0.75 * veh.a_max)
        self._cmd_queue = deque()

        self.eb_used = False

        self.notch_history: List[int] = []
        self.time_history: List[float] = []

        self.first_brake_start = None
        self.first_brake_done = False

        self.prev_a = 0.0
        self.jerk_history: List[float] = []

        self.tasc_enabled = False
        self.manual_override = False
        self.tasc_deadband_m = 0.3
        self.tasc_hold_min_s = 0.20
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1
        self.tasc_armed = False
        self.tasc_active = False

        self.rr_factor = _mu_to_rr_factor(self.scn.mu)

        self._tasc_pred_cache = {
            "t": -1.0, "v": -1.0, "notch": -1,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        }
        self._tasc_pred_interval = 0.05
        self._tasc_last_pred_t = -1.0
        self._tasc_speed_eps = 0.3

        self._need_b5_last_t = -1.0
        self._need_b5_last = False
        self._need_b5_interval = 0.05

        self.brk_accel = 0.0
        self.tau_apply = 0.25
        self.tau_release = 0.8
        self.tau_apply_eb = 0.15
        self.tau_release_lowv = 0.8

        self._in_predict = False

        self._b1_air_boost_state = 1.0
        self._b1_i = 0.0

    # ----------------- Physics helpers -----------------
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        if notch >= len(self.veh.notch_accels):
            return 0.0
        base = float(self.veh.notch_accels[notch]) 

        blend_cutoff_speed = 40.0 / 3.6
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))
        speed_kmh = v * 3.6
        air_boost = 0.72 if speed_kmh <= 3.0 else 1.0 

        if notch == 1 and (not self._in_predict) and self.state is not None and (not self.state.finished):
            rem_now = self.scn.L - self.state.s
            s_b1_nominal = self._estimate_stop_distance(1, v, include_margin=False)
            error_m = rem_now - s_b1_nominal

            k_base, k_near = 0.25, 0.08
            scale = min(1.0, max(0.0, abs(error_m) / 0.8))
            k = k_near + (k_base - k_near) * scale

            dt_sim = max(1e-3, self.scn.dt)
            ki, leak = 0.4, 0.985
            self._b1_i = (self._b1_i * leak) + (ki * error_m * dt_sim)
            self._b1_i = max(-0.25, min(0.60, self._b1_i))

            adjust = 1.0 - k * error_m
            target_boost = max(0.25, min(1.35, adjust))
            target_boost *= (1.0 + self._b1_i)

            if rem_now < 1.5 and v < 1.2:
                target_boost = max(0.28, target_boost - 0.04)

            alpha = min(0.65, dt_sim / 0.022) 
            self._b1_air_boost_state += alpha * (target_boost - self._b1_air_boost_state)
            air_boost *= self._b1_air_boost_state

        blended_accel = base * (regen_frac + (1 - regen_frac) * air_boost)

        k_srv, k_eb = 0.85, 0.98
        is_eb = (notch == self.veh.notches - 1)
        k_adh = k_eb if is_eb else k_srv
        a_cap = -k_adh * float(self.scn.mu) * 9.81
        a_eff = max(blended_accel, a_cap)

        if a_eff <= a_cap + 1e-6:
            a_eff = a_cap * (0.90 if v > 8.0 else 0.85)

        return a_eff

    def _grade_accel(self) -> float:
        return -9.81 * (self.scn.grade_percent / 100.0)

    def _davis_accel(self, v: float) -> float:
        A0 = self.veh.A0 * self.rr_factor
        B1 = self.veh.B1 * self.rr_factor
        C2 = self.veh.C2
        F = A0 + B1 * v + C2 * v * v
        return -F / self.veh.mass_kg if v != 0 else 0.0

    def _update_brake_dyn(self, a_cmd: float, v: float, is_eb: bool, dt: float):
        going_stronger = (a_cmd < self.brk_accel)
        if going_stronger:
            tau = self.tau_apply_eb if is_eb else self.tau_apply
        else:
            tau = self.tau_release_lowv if v < 3.0 else self.tau_release
        alpha = dt / max(1e-6, tau)
        self.brk_accel += (a_cmd - self.brk_accel) * alpha

    # ------ stopping distance helpers ------
    def _estimate_stop_distance(self, notch: int, v0: float, include_margin: bool = True) -> float:
        dt = 0.01  # 더 정밀한 예측
        v = max(0.0, v0)
        a = 0.0
        s = 0.0
        rem_now = self.scn.L - self.state.s
        limit = float(rem_now + 5.0)

        self._in_predict = True
        try:
            for _ in range(5000): 
                a_brk = self._effective_brake_accel(notch, v) 
                a_grade = self._grade_accel()
                a_davis = self._davis_accel(v)
                a_target = a_brk + a_grade + a_davis

                # tau 적용
                tau = max(0.15, self.veh.tau_brk)
                a += (a_target - a) * (dt / tau)

                # 저크 제한
                max_da = self.veh.j_max * dt
                da = a_target - a
                if da > max_da:
                    da = max_da
                elif da < -max_da:
                    da = -max_da
                a += da

                v = max(0.0, v + a * dt)
                s += v * dt + 0.5 * a * dt * dt
                if v <= 0.01 or s > limit:
                    break
        finally:
            self._in_predict = False

        return s

    # ----------------- Main step -----------------
    def step(self):
        st = self.state
        dt = self.scn.dt

        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)
        if not self.first_brake_done:
            if st.lever_notch in (1, 2): 
                if self.first_brake_start is None:
                    self.first_brake_start = st.t
                elif (st.t - self.first_brake_start) >= 1.0:
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ====== 동역학 ======
        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v) 
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self._update_brake_dyn(a_cmd_brake, st.v, is_eb, dt)

        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)

        a_target = self.brk_accel + a_grade + a_davis

        max_da = self.veh.j_max * dt
        da = a_target - st.a
        if da > max_da:
            da = max_da
        elif da < -max_da:
            da = -max_da
        st.a += da

        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        if st.lever_notch != 1 or st.finished:
            self._b1_i *= 0.9
            if abs(self._b1_i) < 1e-3:
                self._b1_i = 0.0

        rem = self.scn.L - st.s
        if not st.finished and (rem <= -5.0 or st.v <= 0.0):
            st.finished = True
            st.stop_error_m = self.scn.L - st.s
            st.residual_speed_kmh = st.v * 3.6
            st.score = 0
            self.running = False

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
            st.lever_notch = self._clamp_notch(st.lever_notch + val)
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
        self.tasc_active = False
        self.tasc_armed = bool(self.tasc_enabled)
        self._tasc_pred_cache.update({"t": -1.0, "v": -1.0, "notch": -1,
                                      "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')})
        self._tasc_last_pred_t = -1.0
        self._need_b5_last_t = -1.0
        self._need_b5_last = False
        self.brk_accel = 0.0
        self._b1_air_boost_state = 1.0
        self._b1_i = 0.0

# ------------------------------------------------------------
# FastAPI app
# ------------------------------------------------------------

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/")
async def root():
    return HTMLResponse(open("static/index.html", "r", encoding="utf-8").read())


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(BASE_DIR, "vehicle