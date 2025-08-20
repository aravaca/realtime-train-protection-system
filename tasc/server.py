import math
import json
import asyncio
import time
import os

from dataclasses import dataclass
from collections import deque
from typing import Optional, List, Tuple, Dict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------
DEBUG = False  # 디버그 로그를 보고 싶으면 True

# ====== Grid Bias Params (속도-거리 그리드) ======
V_STEP = 10.0       # km/h bin size
L_STEP = 50.0       # m bin size
V_MIN, V_MAX = 0.0, 140.0
L_MIN, L_MAX = 50.0, 1000.0

CELL_ALPHA = 0.25   # 각 셀 EMA(지수평활) 반영률
CELL_CLIP = 0.20    # 셀 자체 바이어스 클립 (±m)
BIAS_MAX = 0.15     # 보간 결과의 전역 안전 클립 (±m)
CONF_MIN_COUNT = 6  # 4코너 최소 샘플 수가 이보다 작으면 가중 축소
CONF_FULL_COUNT = 24  # 이 이상이면 신뢰도 1.0

# ------------------------------------------------------------
# (Optional) NumPy 체크 (안 씀, 호환을 위해 남김)
# ------------------------------------------------------------
_HAS_NUMPY = True
try:
    import numpy as _np  # type: ignore
except Exception:
    _HAS_NUMPY = False
    _np = None  # type: ignore

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

    # 공기계수 등 (현재는 rr_factor로만 반영)
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
    dt: float = 0.03

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
            dt=data.get("dt", 0.03),
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
# Simulator + Grid Bias
# ------------------------------------------------------------

class StoppingSim:
    def __init__(self, veh: Vehicle, scn: Scenario):
        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self.vref = build_vref(scn.L, 0.75 * veh.a_max)
        self._cmd_queue = deque()

        # EB 사용 여부
        self.eb_used = False

        # 기록
        self.notch_history: List[int] = []
        self.time_history: List[float] = []

        # 초기 제동(B1/B2) 판정
        self.first_brake_start = None
        self.first_brake_done = False

        # 저크 계산
        self.prev_a = 0.0
        self.jerk_history: List[float] = []

        # ---------- TASC ----------
        self.tasc_enabled = False
        self.manual_override = False
        self.tasc_deadband_m = 0.05  # (기존 0.3 → 0.05 권장)
        self.tasc_hold_min_s = 0.01
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1
        self._tasc_peak_duration = 0.0
        self.tasc_armed = False
        self.tasc_active = False

        # 날씨 코스팅 영향
        self.rr_factor = _mu_to_rr_factor(self.scn.mu)

        # 예측 캐시
        self._tasc_pred_cache = {
            "t": -1.0, "v": -1.0, "notch": -1,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        }
        self._tasc_pred_interval = 0.05
        self._tasc_last_pred_t = -1.0
        self._tasc_speed_eps = 0.3

        # B5 필요 여부 캐시
        self._need_b5_last_t = -1.0
        self._need_b5_last = False
        self._need_b5_interval = 0.05

        # ---------- 제동장치 동역학 ----------
        self.brk_accel = 0.0
        self.tau_apply = 0.25
        self.tau_release = 0.8
        self.tau_apply_eb = 0.15
        self.tau_release_lowv = 0.8

        # 예측 중 재귀 방지 플래그
        self._in_predict = False

        # ---------- Grid Bias Model ----------
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self._bias_model_path = os.path.join(base_dir, "bias_model.json")
        self._bias_model = self._load_bias_model(self._bias_model_path)

    # ===== Grid Model Persistence =====
    def _default_bias_model(self):
        # 그리드 기반 바이어스 (v0_kmh, L_m)
        # cells: { "vXX:lYYY": {"bias": float, "count": int} }
        return {
            "version": 2,
            "type": "grid",
            "grid": {
                "v_step": V_STEP,
                "l_step": L_STEP,
                "v_min": V_MIN,
                "v_max": V_MAX,
                "l_min": L_MIN,
                "l_max": L_MAX,
                "cells": {}
            },
            "cell_alpha": CELL_ALPHA,
            "cell_clip": CELL_CLIP
        }

    def _load_bias_model(self, path: str):
        try:
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as f:
                    m = json.load(f)
                # 안전검사
                if isinstance(m, dict) and m.get("type") == "grid":
                    g = m.get("grid", {})
                    if "cells" not in g:
                        g["cells"] = {}
                    m["grid"] = g
                    # 하위호환 기본값
                    m.setdefault("cell_alpha", CELL_ALPHA)
                    m.setdefault("cell_clip", CELL_CLIP)
                    return m
        except Exception as e:
            if DEBUG:
                print(f"[bias] load failed: {e}")
        return self._default_bias_model()

    def _save_bias_model(self, path: str, model: dict):
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(model, f, ensure_ascii=False, indent=2)
        except Exception as e:
            if DEBUG:
                print(f"[bias] save failed: {e}")

    # ===== Grid Helpers =====
    def _grid_key(self, v_bin: float, l_bin: float) -> str:
        return f"v{int(round(v_bin))}:l{int(round(l_bin))}"

    def _grid_bins(self, v0_kmh: float, L_m: float) -> Tuple[float, float, float, float, float, float]:
        # 주변 2×2 코너의 bin 기준값과 보간비(0~1)를 반환
        v0 = max(V_MIN, min(V_MAX, float(v0_kmh)))
        L0 = max(L_MIN, min(L_MAX, float(L_m)))
        v_base = math.floor((v0 - V_MIN) / V_STEP) * V_STEP + V_MIN
        l_base = math.floor((L0 - L_MIN) / L_STEP) * L_STEP + L_MIN
        v1 = min(v_base + V_STEP, V_MAX)
        l1 = min(l_base + L_STEP, L_MAX)
        # 보간비
        tx = 0.0 if V_STEP == 0 else (0.0 if v1 == v_base else (v0 - v_base) / (v1 - v_base))
        ty = 0.0 if L_STEP == 0 else (0.0 if l1 == l_base else (L0 - l_base) / (l1 - l_base))
        return v_base, v1, l_base, l1, tx, ty

    def _grid_get_cell(self, v_bin: float, l_bin: float) -> Dict[str, float]:
        cells = self._bias_model["grid"]["cells"]
        key = self._grid_key(v_bin, l_bin)
        if key not in cells:
            cells[key] = {"bias": 0.0, "count": 0}
        return cells[key]

    def _grid_update_cell(self, v0_kmh: float, L_m: float, y_bias: float):
        # y_bias는 +면 "앞당겨"야 함. (우리는 y = - stop_error_m 규약 사용)
        v_base, v1, l_base, l1, tx, ty = self._grid_bins(v0_kmh, L_m)
        # 단순: 해당 포인트를 가장 가까운 셀 하나로 스냅하여 업데이트
        # (원한다면 4코너에 분할 가중치로 배분해도 됨. 안정성을 위해 단일 셀 EMA만 사용)
        # 가까운 코너 선택
        v_sel = v_base if tx <= 0.5 else v1
        l_sel = l_base if ty <= 0.5 else l1
        cell = self._grid_get_cell(v_sel, l_sel)
        alpha = float(self._bias_model.get("cell_alpha", CELL_ALPHA))
        # EMA 업데이트
        new_bias = (1.0 - alpha) * float(cell["bias"]) + alpha * float(y_bias)
        # 셀 자체 클립
        clipv = float(self._bias_model.get("cell_clip", CELL_CLIP))
        new_bias = max(-clipv, min(clipv, new_bias))
        cell["bias"] = new_bias
        cell["count"] = int(cell.get("count", 0)) + 1

    def _grid_bilinear_bias(self, v0_kmh: float, L_m: float) -> Tuple[float, int]:
        v_base, v1, l_base, l1, tx, ty = self._grid_bins(v0_kmh, L_m)
        c00 = self._grid_get_cell(v_base, l_base)
        c10 = self._grid_get_cell(v1,    l_base)
        c01 = self._grid_get_cell(v_base, l1)
        c11 = self._grid_get_cell(v1,    l1)
        b00, n00 = float(c00["bias"]), int(c00["count"])
        b10, n10 = float(c10["bias"]), int(c10["count"])
        b01, n01 = float(c01["bias"]), int(c01["count"])
        b11, n11 = float(c11["bias"]), int(c11["count"])

        # 쌍선형 보간
        bx0 = (1.0 - tx) * b00 + tx * b10
        bx1 = (1.0 - tx) * b01 + tx * b11
        b = (1.0 - ty) * bx0 + ty * bx1
        min_count = min(n00, n10, n01, n11)
        return b, min_count

    # ===== API (train_bias.py 호환) =====
    def _append_observation_and_update(self, *, v0_kmh: float, L_m: float,
                                       grade_percent: float, mass_tons: float,
                                       stop_error_m: float, force_fit: bool = False):
        # 규약: y = - stop_error_m  (오버런(+)이면 음수 바이어스로 앞당김)
        y = -float(stop_error_m)
        self._grid_update_cell(v0_kmh, L_m, y)
        # 그리드는 배치 피팅이 필요 없음. force_fit 무시.

    def _fit_bias_coeffs_from_data(self, force: bool = False):
        # 전역 선형모델과의 호환 위해 남겨둔 no-op
        return

    # ----------------- 동적 마진 (물리 + 그리드 바이어스) -----------------
    def compute_margin(self, mu: float, grade_permil: float, peak_notch: int, peak_dur_s: float) -> float:
        BASE_1C_T = self.veh.mass_t
        PAX_1C_T = 10.5
        REF_LOAD = 0.70

        mass_tons = self.veh.mass_kg / 1000.0
        L = getattr(self, "train_length", 10)

        baseline_tons = L * (BASE_1C_T + PAX_1C_T * REF_LOAD)
        delta = mass_tons - baseline_tons

        mass_corr = (-2.5e-4) * delta + (1.5e-8) * (delta ** 3)
        mass_corr = max(-0.05, min(0.08, mass_corr))

        margin = -0.675

        # 거리 스케일: 0m → 0.3, 100m 이상 → 1.0
        scale = min(1.0, self.scn.L / 100.0)
        if grade_permil >= 0:
            grade_corr = -0.002 * grade_permil * (1 + abs(grade_permil) / 10.0) * scale
        else:
            grade_corr = -0.010 * grade_permil * scale

        mu_corr = (mu - 1.0) * (0.03 / (0.3 - 1.0))
        return margin + grade_corr + mu_corr + mass_corr

    def _dynamic_margin(self, v0: float, rem_now: float) -> float:
        mu = self.scn.mu
        grade_permil = self.scn.grade_percent * 10.0
        margin = self.compute_margin(mu, grade_permil, self._tasc_peak_notch, self._tasc_peak_duration)

        # ===== 그리드 보간 바이어스 =====
        v0_kmh = max(0.0, float(v0) * 3.6)
        L_m = float(self.scn.L)

        raw_bias, min_count = self._grid_bilinear_bias(v0_kmh, L_m)

        # 신뢰도 스케일: 코너의 최소 샘플수 기반
        if min_count <= CONF_MIN_COUNT:
            conf = 0.0 if min_count <= 1 else (min_count - 1) / max(1, (CONF_MIN_COUNT - 1))
        else:
            # CONF_FULL_COUNT까지 선형 증가, 그 이상은 1.0
            conf = min(1.0, (min_count - CONF_MIN_COUNT) / max(1, (CONF_FULL_COUNT - CONF_MIN_COUNT)))

        bias_term = conf * raw_bias
        bias_term = max(-BIAS_MAX, min(BIAS_MAX, bias_term))

        return margin + bias_term

    # ----------------- Physics helpers -----------------
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        if notch >= len(self.veh.notch_accels):
            return 0.0
        base = float(self.veh.notch_accels[notch])  # 음수

        # 전기/공기 블렌딩 기본
        blend_cutoff_speed = 40.0 / 3.6
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))
        speed_kmh = v * 3.6
        air_boost = 0.72 if speed_kmh <= 3.0 else 1.0  # 저속 기본 약화

        # --- B1: 마지막 구간 약화만 (PI/예측 없음) ---
        if notch == 1 and (not self._in_predict) and self.state is not None and (not self.state.finished):
            rem_now = max(0.0, self.scn.L - self.state.s)
            if rem_now < 0.25:
                air_boost = min(air_boost, 0.35)
            elif rem_now < 0.8:
                air_boost = max(0.52, min(0.58, air_boost))

        # 최종 블렌딩
        blended_accel = base * (regen_frac + (1 - regen_frac) * air_boost)

        # 접착 한계 및 간단 WSP
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
        A0 = self.veh.A0 * (0.7 + 0.3 * self.scn.mu)
        B1 = self.veh.B1 * (0.7 + 0.3 * self.scn.mu)
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

    # ----------------- Controls -----------------
    def _clamp_notch(self, n: int) -> int:
        return max(0, min(self.veh.notches - 1, n))

    def queue_command(self, name: str, val: int = 0):
        self._cmd_queue.append({"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val})

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

    # ----------------- Lifecycle -----------------
    def reset(self):
        self.state = State(t=0.0, s=0.0, v=self.scn.v0, a=0.0, lever_notch=0, finished=False)
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

        # TASC 상태 초기화 (토글 상태는 유지)
        self.manual_override = False
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1
        self._tasc_peak_duration = 0.0
        self.tasc_active = False
        self.tasc_armed = bool(self.tasc_enabled)

        # 예측 캐시 초기화
        self._tasc_pred_cache.update({"t": -1.0, "v": -1.0, "notch": -1,
                                      "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')})
        self._tasc_last_pred_t = -1.0

        # B5 필요 여부 캐시 초기화
        self._need_b5_last_t = -1.0
        self._need_b5_last = False

        # 제동장치 상태 초기화
        self.brk_accel = 0.0

        if DEBUG:
            print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True

    def eb_used_from_history(self) -> bool:
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # ------ stopping distance helpers ------
    def _estimate_stop_distance(self, notch: int, v0: float, include_margin: bool = True) -> float:
        dt = 0.03
        v = max(0.0, v0)
        a = 0.0
        s = 0.0
        tau = max(0.15, self.veh.tau_brk)
        rem_now = self.scn.L - self.state.s
        limit = float(rem_now + 5.0)

        self._in_predict = True
        try:
            for _ in range(1200):
                a_brk = self._effective_brake_accel(notch, v)
                a_grade = self._grade_accel()
                a_davis = self._davis_accel(v)
                a_target = a_brk + a_grade + a_davis
                a += (a_target - a) * (dt / tau)
                v = max(0.0, v + a * dt)
                s += v * dt + 0.5 * a * dt * dt
                if v <= 0.01 or s > limit:
                    break
        finally:
            self._in_predict = False

        if include_margin:
            s += self._dynamic_margin(v0, rem_now)
        return s

    def _stopping_distance(self, notch: int, v: float, include_margin: bool = True) -> float:
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v, include_margin=include_margin)

    def _tasc_predict(self, cur_notch: int, v: float):
        st = self.state
        need = False
        if (st.t - self._tasc_last_pred_t) >= self._tasc_pred_interval:
            need = True
        if abs(v - self._tasc_pred_cache["v"]) >= self._tasc_speed_eps:
            need = True
        if cur_notch != self._tasc_pred_cache["notch"]:
            need = True
        if not need:
            return (self._tasc_pred_cache["s_cur"],
                    self._tasc_pred_cache["s_up"],
                    self._tasc_pred_cache["s_dn"])

        max_normal_notch = self.veh.notches - 2
        s_cur = self._stopping_distance(cur_notch, v) if cur_notch > 0 else float('inf')
        s_up = self._stopping_distance(cur_notch + 1, v) if cur_notch + 1 <= max_normal_notch else 0.0
        s_dn = self._stopping_distance(cur_notch - 1, v) if cur_notch - 1 >= 1 else float('inf')

        self._tasc_pred_cache.update({"t": st.t, "v": v, "notch": cur_notch,
                                      "s_cur": s_cur, "s_up": s_up, "s_dn": s_dn})
        self._tasc_last_pred_t = st.t
        return s_cur, s_up, s_dn

    def _need_B5_now(self, v: float, remaining: float) -> bool:
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last
        s_b4 = self._stopping_distance(2, v)  # 인덱스 환경 맞춰 조정 가능
        need = s_b4 > (remaining + self.tasc_deadband_m)
        self._need_b5_last = need
        self._need_b5_last_t = st.t
        return need

    # ----------------- Main step -----------------
    def step(self):
        st = self.state
        dt = self.scn.dt

        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 기록 & 초제동(B1/B2) 판정 체크
        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)
        if not self.first_brake_done:
            if st.lever_notch in (1, 2):
                if self.first_brake_start is None:
                    self.first_brake_start = st.t
                elif (st.t - self.first_brake_start) >= 2.0:
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ---------- TASC ----------
        if self.tasc_enabled and not self.manual_override and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            speed_kmh = st.v * 3.6
            cur = st.lever_notch
            max_normal_notch = self.veh.notches - 2

            if self.tasc_armed and not self.tasc_active:
                if self._need_B5_now(st.v, rem_now):
                    self.tasc_active = True
                    self.tasc_armed = False
                    self._tasc_last_change_t = st.t

            if self.tasc_active:
                if not self.first_brake_done:
                    desired = 2 if speed_kmh >= 75.0 else 1
                    if dwell_ok and cur != desired:
                        stepv = 1 if desired > cur else -1
                        st.lever_notch = self._clamp_notch(cur + stepv)
                        self._tasc_last_change_t = st.t
                else:
                    s_cur, s_up, s_dn = self._tasc_predict(cur, st.v)
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
                        # 여유 0.02 m (더 촘촘)
                        if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m + 0.02):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur - 1)
                                self._tasc_last_change_t = st.t

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

        # peak notch 지속시간
        if st.lever_notch > self._tasc_peak_notch:
            self._tasc_peak_notch = st.lever_notch
            self._tasc_peak_duration = 0.0
        if st.lever_notch == self._tasc_peak_notch and st.lever_notch > 0:
            self._tasc_peak_duration += dt

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

            # 저크 기록/점수(간단)
            jerk = abs((st.a - self.prev_a) / dt)
            self.prev_a = st.a
            self.jerk_history.append(jerk)
            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False

            if DEBUG:
                print(f"Simulation finished: stop_error={st.stop_error_m:.3f} m, score={score}")

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
        g = self._bias_model.get("grid", {})
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
            "grade": self.scn.grade_percent,
            "score": getattr(st, "score", 0),
            "issues": getattr(st, "issues", {}),
            "tasc_enabled": getattr(self, "tasc_enabled", False),
            # HUD/디버그용
            "mu": float(self.scn.mu),
            "rr_factor": float(0.7 + 0.3 * self.scn.mu),
            "davis_A0": self.veh.A0,
            "davis_B1": self.veh.B1,
            "davis_C2": self.veh.C2,
            # Grid bias 디버그
            "grid_meta": {
                "v_step": g.get("v_step", V_STEP),
                "l_step": g.get("l_step", L_STEP),
                "cells_count": len(g.get("cells", {})),
            },
        }


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
    vehicle_json_path = os.path.join(BASE_DIR, "vehicle.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    # 프론트가 EB→...→N으로 올 때 서버는 N→...→EB로 쓰기 위해 반전
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.start()

    last_sim_time = time.perf_counter()
    last_send = 0.0
    send_interval = 1.0 / 30.0

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
                        grade = payload.get("grade", 0.0) / 10.0
                        mu = float(payload.get("mu", 1.0))
                        if speed is not None and dist is not None:
                            sim.scn.v0 = float(speed) / 3.6
                            sim.scn.L = float(dist)
                            sim.scn.grade_percent = float(grade)
                            sim.scn.mu = mu
                            sim.rr_factor = _mu_to_rr_factor(mu)
                            if DEBUG:
                                print(f"setInitial: v0={speed}km/h, L={dist}m, grade={grade}%, mu={mu}, rr_factor={sim.rr_factor:.3f}")
                            sim.reset()
                            sim.running = True

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
                        sim.train_length = length
                        if DEBUG:
                            print(f"Train length set to {length} cars.")
                        sim.reset()

                    elif name == "setLoadRate":
                        load_rate = float(payload.get("loadRate", 0.0)) / 100.0
                        length = int(payload.get("length", 8))
                        base_1c_t = vehicle.mass_t
                        pax_1c_t = 10.5
                        total_tons = length * (base_1c_t + pax_1c_t * load_rate)
                        vehicle.update_mass(length)
                        vehicle.mass_kg = total_tons * 1000.0
                        sim.train_length = length
                        sim.reset()
                        if DEBUG:
                            print(f"[LoadRate] {length}량, 탑승률={load_rate*100:.1f}%, 총 {total_tons:.1f} t")

                    elif name == "setTASC":
                        enabled = bool(payload.get("enabled", False))
                        sim.tasc_enabled = enabled
                        if enabled:
                            sim.manual_override = False
                            sim._tasc_last_change_t = sim.state.t
                            sim._tasc_phase = "build"
                            sim._tasc_peak_notch = 1
                            sim._tasc_peak_duration = 0.0
                            sim.tasc_armed = True
                            sim.tasc_active = False
                        if DEBUG:
                            print(f"TASC set to {enabled}")

                    elif name == "setMu":
                        value = float(payload.get("value", 1.0))
                        sim.scn.mu = value
                        sim.rr_factor = _mu_to_rr_factor(value)
                        if DEBUG:
                            print(f"마찰계수(mu)={value} / rr_factor={sim.rr_factor:.3f} 로 설정")
                        sim.reset()

                    elif name == "reset":
                        sim.reset()

                    # (옵션) 수동 저장/더미 피팅
                    elif name == "biasSave":
                        sim._save_bias_model(sim._bias_model_path, sim._bias_model)
                    elif name == "biasFit":
                        sim._fit_bias_coeffs_from_data(force=True)
                        sim._save_bias_model(sim._bias_model_path, sim._bias_model)

            except asyncio.TimeoutError:
                pass
            except WebSocketDisconnect:
                break
            except Exception as e:
                if DEBUG:
                    print(f"Error during receive: {e}")

            dt = sim.scn.dt
            while elapsed >= dt:
                if sim.running:
                    sim.step()
                last_sim_time += dt
                elapsed -= dt

            if (now - last_send) >= send_interval:
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                last_send = now

            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass