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
# (Optional) NumPy 사용 가능하면 활용, 아니면 순수 파이썬 대체 사용
# ------------------------------------------------------------
_HAS_NUMPY = True
try:
    import numpy as _np  # type: ignore
except Exception:
    _HAS_NUMPY = False
    _np = None  # type: ignore


def _gauss_jordan_inv(mat):
    """
    순수 파이썬 가우스-조르당 역행렬 (작은 행렬용, 안정성용으로 소규모 람다 가산 권장)
    mat: List[List[float]] (정방행렬)
    return: List[List[float]] inverse
    """
    n = len(mat)
    # 단위행렬
    inv = [[0.0]*n for _ in range(n)]
    for i in range(n):
        inv[i][i] = 1.0
    # 확장 행렬 [A | I]
    A = [row[:] + inv_row[:] for row, inv_row in zip(mat, inv)]

    # 피벗
    for col in range(n):
        # 최대 절댓값 피벗 선택
        pivot_row = max(range(col, n), key=lambda r: abs(A[r][col]))
        if abs(A[pivot_row][col]) < 1e-12:
            # 특이행렬 방지용 작은 값 더하기
            A[pivot_row][col] = 1e-12
        # swap
        if pivot_row != col:
            A[col], A[pivot_row] = A[pivot_row], A[col]
        # 정규화
        pivot = A[col][col]
        inv_pivot = 1.0 / pivot
        for j in range(2*n):
            A[col][j] *= inv_pivot
        # 다른 행 소거
        for r in range(n):
            if r == col:
                continue
            factor = A[r][col]
            if factor != 0.0:
                for j in range(2*n):
                    A[r][j] -= factor * A[col][j]
    # 분리
    inv = [row[n:] for row in A]
    return inv


def _matmul_vec(M, v):
    return [sum(M[i][j]*v[j] for j in range(len(v))) for i in range(len(M))]


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
        self.tasc_deadband_m = 0.3
        self.tasc_hold_min_s = 0.01
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1
        self._tasc_peak_duration = 0.0 # peak notch 유지 누적 시간(s)
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

        # B1 미세조정 상태 (P+I)
        self._b1_air_boost_state = 1.0
        self._b1_i = 0.0

        # ---------- Bias Model (저장/불러오기/학습/예측) ----------
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self._bias_model_path = os.path.join(base_dir, "bias_model.json")
        self._bias_model = self._load_bias_model(self._bias_model_path)

    # ================= Bias 모델: 퍼시스턴스 =================
    def _default_bias_model(self):
        # 선형모델: y = w0 + w1*v0_kmh + w2*L_m + w3*grade_percent + w4*mass_tons + w5*(v0*grade) + w6*(L*grade)
        return {
            "version": 1,
            "coeffs": [0.0]*7,   # 초기 0
            "lambda": 1e-4,      # 릿지용
            "data": []           # 각 항목: {"x":[...7], "y": float}
        }

    def _load_bias_model(self, path: str):
        try:
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as f:
                    m = json.load(f)
                # 안전검사
                if "coeffs" in m and isinstance(m.get("coeffs"), list) and len(m["coeffs"]) == 7:
                    if "data" not in m or not isinstance(m["data"], list):
                        m["data"] = []
                    if "version" not in m:
                        m["version"] = 1
                    if "lambda" not in m:
                        m["lambda"] = 1e-4
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

    # ================= Bias 모델: 특징/예측/학습 =================
    def _bias_features(self, v0_kmh: float, L_m: float, grade_percent: float, mass_tons: float):
        # 단순/안정적 스케일: km/h, m, %, ton, 상호작용 2개
        return [
            1.0,
            float(v0_kmh),
            float(L_m),
            float(grade_percent),
            float(mass_tons),
            float(v0_kmh) * float(grade_percent),
            float(L_m) * float(grade_percent),
        ]

    def _predict_bias(self, v0_kmh: float, L_m: float, grade_percent: float, mass_tons: float) -> float:
        w = self._bias_model.get("coeffs", [0.0]*7)
        x = self._bias_features(v0_kmh, L_m, grade_percent, mass_tons)
        return sum(wi*xi for wi, xi in zip(w, x))

    def _append_observation_and_update(self, *, v0_kmh: float, L_m: float,
                                       grade_percent: float, mass_tons: float,
                                       stop_error_m: float, force_fit: bool = False):
        """
        train_bias.py에서 호출.
        ⚠️ 서버 저장 시 부호 반전: y = - stop_error_m
           (오버런(+))이면 '앞당겨야 하므로' 음의 바이어스를 학습하도록 설계
        """
        y = -float(stop_error_m)
        x = self._bias_features(v0_kmh, L_m, grade_percent, mass_tons)
        self._bias_model.setdefault("data", []).append({"x": x, "y": y})

        # 데이터 과도 팽창 방지 (안전스위치, 매우 큼)
        if len(self._bias_model["data"]) > 50000:
            self._bias_model["data"] = self._bias_model["data"][-25000:]

        if force_fit:
            self._fit_bias_coeffs_from_data(force=True)

    def _fit_bias_coeffs_from_data(self, force: bool = False):
        """
        배치 학습: 최소 표본이 적어도 7~10개는 있을 때 안정.
        릿지(λI)로 X^T X 역행렬 안정화. NumPy 있으면 사용, 없으면 내장 역행렬 사용.
        """
        data = self._bias_model.get("data", [])
        if not data:
            return
        n = len(data)
        p = 7
        if (not force) and n < p + 3:
            return

        # X, y 생성
        if _HAS_NUMPY:
            X = _np.array([d["x"] for d in data], dtype=float)  # (n,7)
            y = _np.array([d["y"] for d in data], dtype=float)  # (n,)
            lam = float(self._bias_model.get("lambda", 1e-4))
            XtX = X.T @ X + lam * _np.eye(p)
            Xty = X.T @ y
            try:
                w = _np.linalg.solve(XtX, Xty)
            except _np.linalg.LinAlgError:
                w = _np.linalg.pinv(XtX) @ Xty
            self._bias_model["coeffs"] = [float(v) for v in w.tolist()]
            return

        # 순수 파이썬 경로
        # XtX, Xty 계산
        XtX = [[0.0]*p for _ in range(p)]
        Xty = [0.0]*p
        lam = float(self._bias_model.get("lambda", 1e-4))
        for d in data:
            x = d["x"]
            yy = d["y"]
            for i in range(p):
                Xty[i] += x[i]*yy
                for j in range(p):
                    XtX[i][j] += x[i]*x[j]
        # 릿지
        for i in range(p):
            XtX[i][i] += lam

        # (XtX)^(-1) * Xty
        try:
            XtX_inv = _gauss_jordan_inv(XtX)
            w = _matmul_vec(XtX_inv, Xty)
            self._bias_model["coeffs"] = [float(val) for val in w]
        except Exception as e:
            if DEBUG:
                print(f"[bias] fit failed: {e}")

    # ----------------- 동적 마진 함수 -----------------
    def compute_margin(self, mu: float, grade_permil: float, peak_notch: int, peak_dur_s: float) -> float:

        BASE_1C_T = self.veh.mass_t # JSON 기반
        PAX_1C_T = 10.5
        REF_LOAD = 0.70

        mass_tons = self.veh.mass_kg / 1000.0
        L = getattr(self, "train_length", 10)

        baseline_tons = L * (BASE_1C_T + PAX_1C_T * REF_LOAD)
        delta = mass_tons - baseline_tons

        mass_corr = (-2.5e-4) * delta + (1.5e-8) * (delta ** 3)

        # 클램프
        if mass_corr > 0.08:
            mass_corr = 0.08
        elif mass_corr < -0.05:
            mass_corr = -0.05

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
        """
        baseline(-0.675) + 선형 보정(grade/mu/mass) + (학습된 bias 추가)
        """
        mu = self.scn.mu
        grade_permil = self.scn.grade_percent * 10.0

        margin = self.compute_margin(mu, grade_permil, self._tasc_peak_notch, self._tasc_peak_duration)

        # ===== 학습된 바이어스 추가 =====
        v0_kmh = max(0.0, float(v0) * 3.6)
        L_m = float(self.scn.L)
        grade_percent = float(self.scn.grade_percent)
        mass_tons = float(self.veh.mass_kg) / 1000.0

        bias_term = self._predict_bias(v0_kmh, L_m, grade_percent, mass_tons)

        return margin + bias_term

    # ----------------- Physics helpers -----------------
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        """
        노치별 장비 목표감속 + 속도 기반 전기/공기 블렌딩
        + B1 롱브레이크 미세조정(P+I) — _in_predict로 재귀 차단
        + 마지막 1.5m/1.0m에서 B1일 때만 에어부스트 완화(0.50~0.60)
        """
        if notch >= len(self.veh.notch_accels):
            return 0.0
        base = float(self.veh.notch_accels[notch]) # 음수

        # 전기/공기 블렌딩 기본
        blend_cutoff_speed = 40.0 / 3.6
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))
        speed_kmh = v * 3.6
        air_boost = 0.72 if speed_kmh <= 3.0 else 1.0 # 저속 기본 약화

        # --- B1 미세조정: 롱 B1 유지하면서 0cm 근접 ---
        if notch == 1 and (not self._in_predict) and self.state is not None and (not self.state.finished):
            rem_now = self.scn.L - self.state.s

            # B1 정지거리 '편향 없는' 예측: 마진 제외
            s_b1_nominal = self._estimate_stop_distance(1, v, include_margin=False)

            # error(+): 언더런(남은거리가 더 김) → 제동 약화(air_boost ↓)
            error_m = rem_now - s_b1_nominal

            # 간결한 P+I
            dt_sim = max(1e-3, self.scn.dt)
            k_p = 0.20
            ki, leak = 0.35, 0.985
            self._b1_i = (self._b1_i * leak) + (ki * error_m * dt_sim)
            self._b1_i = max(-0.25, min(0.60, self._b1_i))

            adjust = 1.0 - k_p * error_m
            target_boost = max(0.25, min(1.35, adjust))
            target_boost *= (1.0 + self._b1_i)

            # 마지막 구간 완화 (B1에서만 동작)
            if rem_now < 0.3 and notch == 1:
                target_boost = 0.3
            if rem_now < 1.0 and notch == 1:
                target_boost = max(0.50, min(0.60, target_boost))

            # 스무딩(LPF)
            alpha = min(0.65, dt_sim / 0.022)
            self._b1_air_boost_state += alpha * (target_boost - self._b1_air_boost_state)
            air_boost *= self._b1_air_boost_state

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
        A0 = self.veh.A0 * (0.7 + 0.3 * self.scn.mu) # rr_factor 반영
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

    # ----------------- Lifecycle -----------------
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

        # B1 미세조정 상태 초기화
        self._b1_air_boost_state = 1.0
        self._b1_i = 0.0

        if DEBUG:
            print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True

    def eb_used_from_history(self) -> bool:
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # ------ stopping distance helpers ------
    def _estimate_stop_distance(self, notch: int, v0: float, include_margin: bool = True) -> float:
        """
        해당 노치 고정 가정한 정지거리 예측.
        include_margin=True면 동적마진을 포함해 편향 보정(제어에서 사용).
        B1 미세조정용에는 include_margin=False로 '편향 없는' 값을 사용.
        """
        dt = 0.03 # 예측은 더 큰 스텝(≈33Hz)로 성능 확보
        v = max(0.0, v0)
        a = 0.0
        s = 0.0
        tau = max(0.15, self.veh.tau_brk)
        rem_now = self.scn.L - self.state.s
        limit = float(rem_now + 5.0)

        # 재귀 방지 플래그 ON
        self._in_predict = True
        try:
            for _ in range(1200): # 최대 ≈36s
                a_brk = self._effective_brake_accel(notch, v) # _in_predict=True → B1 미세조정 비활성
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
        """
        'B5가 필요하냐?'를 상수 시간으로 판정.
        여기서는 'B4로 못 멈추면'으로 판정: s(B4) > 남은거리(+데드밴드)
        Notch index: 0:N, 1:B1, 2:B2, 3:B3, 4:B4, 5:B5 ...
        """
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last
        s_b4 = self._stopping_distance(2, v) # TODO: 환경에 맞게 인덱스 조정 필요시 변경
        need = s_b4 > (remaining + self.tasc_deadband_m)
        self._need_b5_last = need
        self._need_b5_last_t = st.t
        return need

    # ----------------- Main step -----------------
    def step(self):
        st = self.state
        dt = self.scn.dt

        # 예약된 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 기록 & 초제동(B1/B2) 판정 체크 (시뮬 시간 기반) - 판정시간 2초
        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)
        if not self.first_brake_done:
            if st.lever_notch in (1, 2): # B1 또는 B2
                if self.first_brake_start is None:
                    self.first_brake_start = st.t
                elif (st.t - self.first_brake_start) >= 2.0:
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ---------- TASC 자동 제동 ----------
        if self.tasc_enabled and not self.manual_override and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            speed_kmh = st.v * 3.6
            cur = st.lever_notch
            max_normal_notch = self.veh.notches - 2 # EB-1까지

            # (A) B5 필요 시점 감지 → 그때 TASC 활성화(초제동 시퀀스 시작)
            if self.tasc_armed and not self.tasc_active:
                if self._need_B5_now(st.v, rem_now):
                    self.tasc_active = True
                    self.tasc_armed = False
                    self._tasc_last_change_t = st.t

            if self.tasc_active:
                # (B) 초제동 유지: 활성화 이후에만 B1/B2를 2초간 강제
                if not self.first_brake_done:
                    desired = 2 if speed_kmh >= 75.0 else 1
                    if dwell_ok and cur != desired:
                        stepv = 1 if desired > cur else -1
                        st.lever_notch = self._clamp_notch(cur + stepv)
                        self._tasc_last_change_t = st.t
                else:
                    # (C) 빌드/릴렉스 로직
                    s_cur, s_up, s_dn = self._tasc_predict(cur, st.v)
                    changed = False

                    if self._tasc_phase == "build":
                        # 더 강한 제동이 필요하면 한 단계 강화
                        if cur < max_normal_notch and s_cur > (rem_now - self.tasc_deadband_m):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur + 1)
                                self._tasc_last_change_t = st.t
                                self._tasc_peak_notch = max(self._tasc_peak_notch, st.lever_notch)
                                changed = True
                        else:
                            self._tasc_phase = "relax"

                    if self._tasc_phase == "relax" and not changed:
                        # 더 약한 제동으로도 충분하면 한 단계 완해
                        if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m + 0.1):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur - 1)
                                self._tasc_last_change_t = st.t

        # ====== 동역학 ======
        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v) # 음수
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self._update_brake_dyn(a_cmd_brake, st.v, is_eb, dt) # self.brk_accel 갱신

        # 외력
        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)

        # 목표 가속도
        a_target = self.brk_accel + a_grade + a_davis

        # 저크 제한
        max_da = self.veh.j_max * dt
        da = a_target - st.a
        if da > max_da:
            da = max_da
        elif da < -max_da:
            da = -max_da
        st.a += da

        # 적분
        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        # ----- peak notch 지속시간 누적 -----
        # 최고 노치를 갱신하면 지속시간 초기화, 동일 최고노치 유지 시 누적
        if st.lever_notch > self._tasc_peak_notch:
            self._tasc_peak_notch = st.lever_notch
            self._tasc_peak_duration = 0.0
        if st.lever_notch == self._tasc_peak_notch and st.lever_notch > 0:
            self._tasc_peak_duration += dt

        # B1이 아니거나 종료면 I항 서서히 소거(드리프트 방지)
        if st.lever_notch != 1 or st.finished:
            self._b1_i *= 0.9
            if abs(self._b1_i) < 1e-3:
                self._b1_i = 0.0

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

            # 초제동(B1/B2 2초) 보너스/감점
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

            # 계단 패턴 점수 (TASC ON은 점프 허용)
            if self.is_stair_pattern(self.notch_history):
                score += 500
            else:
                if self.tasc_enabled and not self.manual_override:
                    score += 500

            # 정지 오차 점수 (0m → 500점, 10m → 0점)
            err_abs = abs(st.stop_error_m or 0.0)
            error_score = max(0, 500 - int(err_abs * 500))
            score += error_score

            # 0 cm 정차 보너스 (+100)
            if abs(st.stop_error_m or 0.0) < 0.01:
                score += 100

            # 이슈 플래그들
            st.issues["early_brake_too_short"] = not self.first_brake_done
            st.issues["step_brake_incomplete"] = not self.is_stair_pattern(self.notch_history)
            st.issues["stop_error_m"] = st.stop_error_m

            # 저크 기록
            jerk = abs((st.a - self.prev_a) / dt)
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            # 저크 점수 반영
            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False

            # ====== (선택) 자동 데이터 적재용 훅 ======
            # UI/수동 주행만으로도 학습 누적하려면 여기서 관측 추가 가능.
            # 본 요청에서는 train_bias.py가 담당하므로 기본 비활성.
            # v0_kmh = self.scn.v0 * 3.6
            # self._append_observation_and_update(
            #     v0_kmh=v0_kmh, L_m=self.scn.L, grade_percent=self.scn.grade_percent,
            #     mass_tons=self.veh.mass_kg/1000.0, stop_error_m=float(st.stop_error_m), force_fit=False
            # )
            # self._save_bias_model(self._bias_model_path, self._bias_model)

            if DEBUG:
                print(f"Simulation finished: stop_error={st.stop_error_m:.3f} m, score={score}")

    def is_stair_pattern(self, notches: List[int]) -> bool:
        """초기(B1/B2)에서 시작해 한 번만 올라갔다 내려오고, 마지막이 B1인지 체크"""
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
        """최근 1초 평균 저크 기반 점수 (<=25: 500점, 25~50: 선형감점, >50: 0점)"""
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
            # 히스토리 디버그
            "peak_notch": self._tasc_peak_notch,
            "peak_dur_s": self._tasc_peak_duration,
            # Bias 상태(디버그)
            "bias_coeffs": self._bias_model.get("coeffs", []),
            "bias_data_len": len(self._bias_model.get("data", [])),
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
    # 송신 속도 제한(30Hz)
    last_send = 0.0
    send_interval = 1.0 / 30.0

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - last_sim_time

            try:
                # 입력 처리 (최대 100Hz 정도로 충분)
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
                        if DEBUG:
                            print(f"Train length set to {length} cars.")
                        sim.reset()

                    elif name == "setLoadRate":
                        load_rate = float(payload.get("loadRate", 0.0)) / 100.0
                        length = int(payload.get("length", 8))

                        # JSON에서 읽은 차량 기본 질량 (1량)
                        base_1c_t = vehicle.mass_t # 예: 39.9
                        pax_1c_t = 10.5 # 승객 만석 시 1량당 질량 (톤). json에 넣을 수도 있음.

                        # 총 질량 (tons)
                        total_tons = length * (base_1c_t + pax_1c_t * load_rate)

                        # vehicle 객체 갱신
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
                            # 즉시 작동 금지: B5 필요 시점까지 '대기'
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

                    # ==== (추가) Bias 관리용 커맨드 (선택) ====
                    # UI에서 필요시 호출 가능. train_bias.py는 직접 파일을 만지므로 필수는 아님.
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

            # ---- 고정된 시뮬 시간 흐름 (현실 시간과 동기화) ----
            dt = sim.scn.dt
            while elapsed >= dt:
                if sim.running:
                    sim.step()
                last_sim_time += dt
                elapsed -= dt

            # ---- 송신 속도 제한 (30Hz) ----
            if (now - last_send) >= send_interval:
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                last_send = now

            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass