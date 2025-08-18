import math
import json
import asyncio
import time
import os
import random  # <<< 추가: pre-training 샘플링용

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
# RLS 보정 모델 (v0, L 전용) + JSON 저장/로드
# ------------------------------------------------------------

class CorrectionModelRLS:
    """
    온라인 RLS(Recursive Least Squares) 회귀로 correction = f(v0, L)을 학습
    - 입력: v0[km/h], L[m]
    - 특성: [1, v0, L, v0^2, L^2, v0*L] (6차원)
    - 저장: JSON (w, P, lam, n_features)
    - 의존성: 순수 파이썬(리스트/루프) → 외부 라이브러리 불필요
    """
    def __init__(self, n_features: int = 6, lam: float = 0.99, delta: float = 1000.0):
        self.n = n_features
        self.lam = float(lam)
        # 가중치 벡터 w (n x 1)
        self.w = [0.0] * self.n
        # 공분산 행렬 P (n x n) — 초기: (1/delta) * I
        self.P = [[0.0]*self.n for _ in range(self.n)]
        inv_delta = 1.0 / float(delta)
        for i in range(self.n):
            self.P[i][i] = inv_delta

    # ---- 선형대수 유틸 (순수 파이썬) ----
    def _features(self, v0_kmh: float, L_m: float):
        v = float(v0_kmh)
        L = float(L_m)
        return [1.0, v, L, v*v, L*L, v*L]

    @staticmethod
    def _dot(a: List[float], b: List[float]) -> float:
        return sum(x*y for x, y in zip(a, b))

    @staticmethod
    def _matvec(M: List[List[float]], x: List[float]) -> List[float]:
        return [sum(row[j]*x[j] for j in range(len(x))) for row in M]

    @staticmethod
    def _transpose(M: List[List[float]]) -> List[List[float]]:
        n = len(M); m = len(M[0]) if n else 0
        return [[M[i][j] for i in range(n)] for j in range(m)]

    def predict(self, v0_kmh: float, L_m: float) -> float:
        x = self._features(v0_kmh, L_m)
        return self._dot(self.w, x)

    def update(self, v0_kmh: float, L_m: float, target_correction: float) -> float:
        """
        RLS 업데이트: w, P 갱신
        반환: 업데이트 후 잔차(= target - 예측)
        """
        x = self._features(v0_kmh, L_m)               # (n,)
        y = float(target_correction)

        # Px = P @ x
        Px = self._matvec(self.P, x)                  # (n,)
        # den = lam + x^T P x
        den = self.lam + self._dot(x, Px)
        if den == 0.0:
            den = 1e-9

        # k = Px / den
        k = [val/den for val in Px]                   # (n,)
        # 예측 오차 e = y - w^T x
        wx = self._dot(self.w, x)
        e = y - wx

        # w += k * e
        for i in range(self.n):
            self.w[i] += k[i] * e

        # P = (P - k (x^T P)) / lam
        #   x^T P = (P^T x)^T → 먼저 Pt_x 계산
        Pt = self._transpose(self.P)
        Pt_x = self._matvec(Pt, x)                    # (n,)
        # 행렬 업데이트
        for i in range(self.n):
            ki = k[i]
            if ki == 0.0:
                continue
            row = self.P[i]
            for j in range(self.n):
                row[j] -= ki * Pt_x[j]
        inv_lam = 1.0 / self.lam
        for i in range(self.n):
            for j in range(self.n):
                self.P[i][j] *= inv_lam

        return e

    # ---- JSON 저장/로드 ----
    def save(self, path: str):
        data = {
            "w": self.w,
            "P": self.P,
            "lam": self.lam,
            "n_features": self.n
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str):
        if not os.path.exists(path):
            return cls()
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        m = cls(n_features=int(data.get("n_features", 6)), lam=float(data.get("lam", 0.99)))
        m.w = list(map(float, data.get("w", [0.0]*m.n)))
        P = data.get("P", None)
        if not P:
            m.P = [[0.0]*m.n for _ in range(m.n)]
            inv_delta = 1.0/1000.0
            for i in range(m.n):
                m.P[i][i] = inv_delta
        else:
            # 안전하게 float 변환
            m.P = [[float(v) for v in row] for row in P]
        return m


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
        self.tasc_hold_min_s = 0.20
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

        # ---------- RLS 보정 관련 ----------
        self.rls_model: Optional[CorrectionModelRLS] = None
        self.rls_save_path: Optional[str] = None
        self._rls_updates: int = 0  # 저장 주기 제어

    def compute_margin(self, mu: float, grade_permil: float, peak_notch: int, peak_dur_s: float) -> float:

        BASE_1C_T = self.veh.mass_t # JSON 기반
        PAX_1C_T = 10.5
        REF_LOAD = 0.70

        mass_tons = self.veh.mass_kg / 1000.0
        L = getattr(self, "train_length", 10)

        baseline_tons = L * (BASE_1C_T + PAX_1C_T * REF_LOAD)
        delta = mass_tons - baseline_tons

        mass_corr = (-2.5e-4) * delta + (1.5e-8) * (delta ** 3)

        # 클램프 극단적으로 조정
        if mass_corr > 0.08:
            mass_corr = 0.08
        elif mass_corr < -0.05:
            mass_corr = -0.05
        # 중량 이전 opt 값=-0.05 -0.05 -0.05
        margin = -0.675
        # 거리 스케일: 0m → 0.3, 100m 이상 → 1.0
        scale = min(1.0, self.scn.L / 100.0)
        if grade_permil >= 0:
            grade_corr = -0.002 * grade_permil * (1 + abs(grade_permil) / 10.0) * scale
        else:
            grade_corr = -0.010 * grade_permil * scale

        mu_corr = (mu - 1.0) * (0.03 / (0.3 - 1.0))

        #hist_corr = -0.1 * max(0, peak_notch - 2) - 0.05 * max(0.0, peak_dur_s)

        return margin + grade_corr + mu_corr + mass_corr

    # ----------------- 동적 마진 함수 -----------------
    def _dynamic_margin(self, v0: float, rem_now: float) -> float:
        """
        baseline(튜닝값) + (grade/mu/mass) 보정 + [RLS 예측 보정(v0,L)].
        """
        mu = self.scn.mu
        grade_permil = self.scn.grade_percent * 10.0

        margin = self.compute_margin(mu, grade_permil, self._tasc_peak_notch, self._tasc_peak_duration)

        # --- RLS 보정 추가 (v0,L만 사용) ---
        # v0: m/s -> km/h
        if self.rls_model is not None:
            try:
                v0_kmh = float(self.scn.v0) * 3.6
                delta = self.rls_model.predict(v0_kmh, float(self.scn.L))
                # 안전 클리핑(과보정 방지)
                if delta > 0.8:
                    delta = 0.8
                elif delta < -0.8:
                    delta = -0.8
                margin += float(delta)
            except Exception:
                pass

        return margin

    # ----------------- Physics helpers -----------------
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        if notch >= len(self.veh.notch_accels):
            return 0.0
        base = float(self.veh.notch_accels[notch]) # 음수

        # 전기/공기 블렌딩 기본
        blend_cutoff_speed = 40.0 / 3.6
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))
        speed_kmh = v * 3.6
        air_boost = 0.72 if speed_kmh <= 3.0 else 1.0 # 저속 기본 약화

        # --- B1 미세조정 ---
        if notch == 1 and (not self._in_predict) and self.state is not None and (not self.state.finished):
            rem_now = self.scn.L - self.state.s
            s_b1_nominal = self._estimate_stop_distance(1, v, include_margin=False)
            error_m = rem_now - s_b1_nominal
            dt_sim = max(1e-3, self.scn.dt)
            k_p = 0.20
            ki, leak = 0.35, 0.985
            self._b1_i = (self._b1_i * leak) + (ki * error_m * dt_sim)
            self._b1_i = max(-0.25, min(0.60, self._b1_i))

            adjust = 1.0 - k_p * error_m
            target_boost = max(0.25, min(1.35, adjust))
            target_boost *= (1.0 + self._b1_i)

            if rem_now < 0.3 and notch == 1:
                target_boost = 0.3
            if rem_now < 1.0 and notch == 1:
                target_boost = max(0.50, min(0.60, target_boost))

            alpha = min(0.65, dt_sim / 0.022)
            self._b1_air_boost_state += alpha * (target_boost - self._b1_air_boost_state)
            air_boost *= self._b1_air_boost_state

        blended_accel = base * (regen_frac + (1 - regen_frac) * air_boost)

        k_srv, k_eb = 0.85
        is_eb = (notch == self.veh.notches - 1)
        k_adh = 0.98 if is_eb else k_srv
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
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last
        # 당신이 의도적으로 2로 두었다고 했으니 그대로 유지합니다.
        s_b4 = self._stopping_distance(2, v)  # <-- 의도된 값 유지
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
                        if cur < max_normal_notch and s_cur > (rem_now - self.tasc_deadband_m):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur + 1)
                                self._tasc_last_change_t = st.t
                                self._tasc_peak_notch = max(self._tasc_peak_notch, st.lever_notch)
                                changed = True
                        else:
                            self._tasc_phase = "relax"

                    if self._tasc_phase == "relax" and not changed:
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
        if st.lever_notch > self._tasc_peak_notch:
            self._tasc_peak_notch = st.lever_notch
            self._tasc_peak_duration = 0.0
        if st.lever_notch == self._tasc_peak_notch and st.lever_notch > 0:
            self._tasc_peak_duration += dt

        # B1이 아니거나 종료면 I항 서서히 소거
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

            # ---- Online RLS 학습 + 저장(주기적) ----
            if self.rls_model is not None:
                try:
                    v0_kmh = float(self.scn.v0) * 3.6
                    target = -float(st.stop_error_m or 0.0)
                    self.rls_model.update(v0_kmh, float(self.scn.L), target)
                    self._rls_updates += 1
                    # 파일 I/O 부담 줄이기: 8회마다 저장
                    if self.rls_save_path and (self._rls_updates % 8 == 0):
                        self.rls_model.save(self.rls_save_path)
                except Exception:
                    pass

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
        }


# ------------------------------------------------------------
# FastAPI app
# ------------------------------------------------------------

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/")
async def root():
    return HTMLResponse(open("static/index.html", "r", encoding="utf-8").read())


def _pretrain_rls(rls_model: CorrectionModelRLS, vehicle_json_path: str, scenario_json_path: str, n_samples: int = 180):
    """
    시작 전에 빠르게 N회 시뮬로 RLS를 예열(Pre-training).
    - v0: 40~130 km/h
    - L:  150~600 m
    - mu/grade/mass: 중립(1.0/0‰/10량)로 고정 → v0,L 형상만 학습
    - 저장은 호출측에서 한 번만 수행
    """
    if n_samples <= 0:
        return

    vehicle2 = Vehicle.from_json(vehicle_json_path)
    vehicle2.notch_accels = list(reversed(vehicle2.notch_accels))
    scenario2 = Scenario.from_json(scenario_json_path)
    sim2 = StoppingSim(vehicle2, scenario2)

    # 중립 조건: 튜닝된 compute_margin의 상정값에 가깝게
    vehicle2.update_mass(10)
    sim2.train_length = 10
    sim2.scn.mu = 1.0
    sim2.scn.grade_percent = 0.0

    for _ in range(int(n_samples)):
        v0 = random.uniform(40.0, 130.0)         # km/h
        L = random.uniform(150.0, 600.0)         # m
        sim2.scn.v0 = v0 / 3.6
        sim2.scn.L = L
        # vref는 HUD용이지만 형식상 갱신
        sim2.vref = build_vref(sim2.scn.L, 0.75 * sim2.veh.a_max)

        # RLS를 실제 제어에도 반영하도록 동일 모델 연결
        sim2.rls_model = rls_model
        sim2.rls_save_path = None  # pretrain 중 파일 저장은 생략하여 I/O 최소화

        sim2.reset()

        # ✅ 예열은 자동 제동 강제: TASC ON + ACTIVE
        sim2.tasc_enabled = True
        sim2.tasc_armed   = False
        sim2.tasc_active  = True
        sim2._tasc_phase  = "build"
        sim2._tasc_last_change_t = sim2.state.t
        # 초기 한틱이라도 제동이 걸리게 B1으로 시작
        sim2.state.lever_notch = 1

        sim2.running = True

        # ✅ 세이프가드: 최대 스텝(예: 4000스텝 ≈ 120초 시뮬) — 무한 루프 방지
        max_steps = 4000
        steps = 0
        while sim2.running and steps < max_steps:
            sim2.step()
            steps += 1
        # 세이프가드에 걸렸다면 다음 에피소드로 넘어감(무한 루프 방지)

    # 호출측에서 한 번만 저장 (I/O 1회) — rls_model.save(path) 는 호출측에서 수행


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    vehicle_json_path = os.path.join(BASE_DIR, "vehicle.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")
    rls_model_path = os.path.join(BASE_DIR, "rls_model.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    # 프론트가 EB→...→N으로 올 때 서버는 N→...→EB로 쓰기 위해 반전
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))

    scenario = Scenario.from_json(scenario_json_path)

    # ---- RLS 모델 로드 ----
    rls_model = CorrectionModelRLS.load(rls_model_path)

    # ---- Pre-training (빠르게 예열) ----
    try:
        preN = int(os.environ.get("RLS_PRETRAIN_N", "180"))
    except ValueError:
        preN = 180
    # 가벼운 동기 pretrain → 렉 체감 없음
    _pretrain_rls(rls_model, vehicle_json_path, scenario_json_path, n_samples=preN)
    # 1회 저장
    try:
        rls_model.save(rls_model_path)
    except Exception:
        pass

    sim = StoppingSim(vehicle, scenario)
    # RLS 모델을 시뮬에 주입 + 저장 경로 설정
    sim.rls_model = rls_model
    sim.rls_save_path = rls_model_path
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
                            # vref(HUD) 갱신
                            sim.vref = build_vref(sim.scn.L, 0.75 * sim.veh.a_max)
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
                        sim.train_length = length
                        if DEBUG:
                            print(f"Train length set to {length} cars.")
                        sim.reset()

                    elif name == "setLoadRate":
                        load_rate = float(payload.get("loadRate", 0.0)) / 100.0
                        length = int(payload.get("length", 8))

                        # JSON에서 읽은 차량 기본 질량 (1량)
                        base_1c_t = vehicle.mass_t # 예: 39.9
                        pax_1c_t = 10.5 # 승객 만석 시 1량당 질량 (톤)

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
                # 필요 시 RLS 모델 주기적 저장(온라인 업데이트는 step()쪽에서 8회마다 저장)
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                last_send = now

            await asyncio.sleep(0)
    finally:
        try:
            # 세션 종료 시 마지막으로 한 번 저장(안전망)
            try:
                rls_model.save(rls_model_path)
            except Exception:
                pass
            await ws.close()
        except RuntimeError:
            pass