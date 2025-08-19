import math
import json
import asyncio
import time
import os
import sys

from dataclasses import dataclass
from collections import deque
from typing import Optional, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

# =========================
# NumPy: 방어적 로드
# =========================
HAS_NUMPY = True
try:
    import numpy as np  # type: ignore
    print("NumPy loaded:", np.__version__)
except Exception as e:
    HAS_NUMPY = False
    np = None  # type: ignore
    print("[WARN] NumPy unavailable:", e, file=sys.stderr)

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------
DEBUG = False  # 디버그 로그를 보고 싶으면 True

# ------------------------------------------------------------
# 안전: static 폴더/파일 자동 생성 (없으면 FastAPI가 죽을 수 있음)
# ------------------------------------------------------------
if not os.path.isdir("static"):
    os.makedirs("static", exist_ok=True)
index_path = os.path.join("static", "index.html")
if not os.path.isfile(index_path):
    with open(index_path, "w", encoding="utf-8") as f:
        f.write("<!doctype html><meta charset='utf-8'><title>TASC</title><h1>TASC Ready</h1>")

# ------------------------------------------------------------
# 안전 유틸
# ------------------------------------------------------------
def _atomic_write_json(path: str, obj: dict):
    """JSON을 tmp에 쓴 뒤 교체(원자적 갱신). 쓰기 실패해도 원본 보전."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)

def _ensure_json(path: str, default_obj: dict) -> dict:
    """존재하지 않거나 손상되면 기본값으로 생성."""
    try:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        _atomic_write_json(path, default_obj)
        return default_obj
    except Exception:
        _atomic_write_json(path, default_obj)
        return default_obj

def _safe_float(x, default=0.0):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
        return default
    except Exception:
        return default

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
        length = max(1, int(length))
        self.mass_kg = self.mass_t * 1000 * length

    @classmethod
    def from_json(cls, filepath):
        default = {
            "name": "EMU-233-JR-East",
            "mass_t": 200.0,
            "a_max": 1.0,
            "j_max": 0.8,
            # EB(끝) ~ N(0) 순서 기준 가속도(음수=제동), 길이 9
            "notch_accels": [-1.5, -1.10, -0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.0],
            "notches": 9,
            "tau_cmd_ms": 150,
            "tau_brk_ms": 250,
            "davis_A0": 1200.0,
            "davis_B1": 30.0,
            "davis_C2": 8.0,
        }
        data = _ensure_json(filepath, default)

        notch_accels = data.get("notch_accels") or default["notch_accels"]
        if not isinstance(notch_accels, list) or len(notch_accels) < 2:
            notch_accels = default["notch_accels"]

        # notches는 항상 리스트 길이에 맞춤(인덱스 불일치 방지)
        notches = int(data.get("notches", len(notch_accels)))
        notches = max(2, min(len(notch_accels), notches))

        mass_t = _safe_float(data.get("mass_t", default["mass_t"]), default["mass_t"])
        tau_cmd = _safe_float(data.get("tau_cmd_ms", default["tau_cmd_ms"]), 150.0) / 1000.0
        tau_brk = _safe_float(data.get("tau_brk_ms", default["tau_brk_ms"]), 250.0) / 1000.0

        return cls(
            name=str(data.get("name", default["name"])),
            mass_t=mass_t,
            a_max=_safe_float(data.get("a_max", default["a_max"]), default["a_max"]),
            j_max=_safe_float(data.get("j_max", default["j_max"]), default["j_max"]),
            notches=notches,
            notch_accels=notch_accels,
            tau_cmd=tau_cmd,
            tau_brk=tau_brk,
            mass_kg=mass_t * 1000,
            A0=_safe_float(data.get("davis_A0", default["davis_A0"]), default["davis_A0"]),
            B1=_safe_float(data.get("davis_B1", default["davis_B1"]), default["davis_B1"]),
            C2=_safe_float(data.get("davis_C2", default["davis_C2"]), default["davis_C2"]),
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
        default = {
            "L": 500.0,
            "v0": 25.0,            # km/h
            "grade_percent": 0.0,  # %
            "mu": 1.0,
            "dt": 0.03,
        }
        data = _ensure_json(filepath, default)
        v0_kmph = _safe_float(data.get("v0", default["v0"]), default["v0"])
        v0_ms = v0_kmph / 3.6
        return cls(
            L=_safe_float(data.get("L", default["L"]), default["L"]),
            v0=v0_ms,
            grade_percent=_safe_float(data.get("grade_percent", default["grade_percent"]), default["grade_percent"]),
            mu=_safe_float(data.get("mu", default["mu"]), default["mu"]),
            dt=max(0.005, _safe_float(data.get("dt", default["dt"]), default["dt"])),
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
    a_ref = max(1e-6, float(a_ref))
    def vref(s: float):
        rem = max(0.0, L - s)
        v2 = 2.0 * a_ref * rem
        return math.sqrt(v2) if v2 > 0.0 else 0.0
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
        self.vref = build_vref(scn.L, 0.75 * max(0.1, veh.a_max))
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
        self.tasc_hold_min_s = 0.1
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

        # B1 미세조정 상태 (P+I)
        self._b1_air_boost_state = 1.0
        self._b1_i = 0.0

        # ---- 다항 보정 모델 로딩/초기화 ----
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self._bias_model_path = os.path.join(base_dir, "bias_model.json")
        self._bias_model = self._load_poly_bias_model(self._bias_model_path)

    # ---------- POLY-BIAS: JSON 모델 ----------
    def _default_bias_model(self):
        # 2차 다항 15항
        terms = [
            "1",
            "v0","L","grade","mass",
            "v0^2","L^2","grade^2","mass^2",
            "v0*L","v0*grade","v0*mass","L*grade","L*mass","grade*mass"
        ]
        coeffs = [0.0]*len(terms)
        return {
            "degree": 2,
            "terms": terms,
            "coeffs": coeffs,
            "features": ["v0","L","grade","mass"],
            "ridge_lambda": 1e-6,
            "max_points": 2000,
            "min_samples": 5,
            "data": []
        }

    def _load_poly_bias_model(self, path: str):
        try:
            if os.path.exists(path):
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                tmpl = self._default_bias_model()
                for k, v in tmpl.items():
                    if k not in data:
                        data[k] = v
                if len(data.get("terms", [])) != len(data.get("coeffs", [])):
                    data["coeffs"] = tmpl["coeffs"]
                if not isinstance(data.get("data", []), list):
                    data["data"] = []
                return data
            else:
                data = self._default_bias_model()
                self._save_bias_model(path, data)
                if DEBUG:
                    print(f"[POLY-BIAS] Created template at {path}")
                return data
        except Exception as e:
            if DEBUG:
                print(f"[POLY-BIAS] load failed: {e}")
            data = self._default_bias_model()
            self._save_bias_model(path, data)
            return data

    def _save_bias_model(self, path: str, data: dict):
        try:
            _atomic_write_json(path, data)
        except Exception as e:
            if DEBUG:
                print(f"[POLY-BIAS] save failed: {e}")

    # ---------- POLY-BIAS: 항/특성, 예측 ----------
    def _poly_term_value(self, term: str, v0: float, L: float, grade: float, mass: float) -> float:
        if term == "1":
            return 1.0
        if term.endswith("^2"):
            base = term[:-2]
            x = {"v0":v0, "L":L, "grade":grade, "mass":mass}.get(base, 0.0)
            return x * x
        if "*" in term:
            a, b = term.split("*", 1)
            vals = {"v0":v0, "L":L, "grade":grade, "mass":mass}
            return vals.get(a, 0.0) * vals.get(b, 0.0)
        return {"v0":v0, "L":L, "grade":grade, "mass":mass}.get(term, 0.0)

    def _poly_features_row(self, terms: List[str], v0_kmh: float, L_m: float, grade_percent: float, mass_tons: float) -> List[float]:
        return [self._poly_term_value(t, v0_kmh, L_m, grade_percent, mass_tons) for t in terms]

    def _predict_bias_from_json(self, v0_kmh: float, L_m: float, grade_percent: float, mass_tons: float) -> float:
        m = self._bias_model or self._default_bias_model()
        terms = m["terms"]
        coeffs = m["coeffs"]
        feats = self._poly_features_row(terms, v0_kmh, L_m, grade_percent, mass_tons)
        try:
            return float(sum(float(c)*float(f) for c, f in zip(coeffs, feats)))
        except Exception:
            return 0.0

    # ---------- POLY-BIAS: 학습 (오차≠0이면 무조건 시도, NumPy 없으면 패스) ----------
    def _fit_bias_coeffs_from_data(self, force: bool = False):
        """NumPy가 없거나 실패하면 기존 계수 유지."""
        if not HAS_NUMPY:
            if DEBUG:
                print("[POLY-BIAS] NumPy not available → skip fit")
            return

        m = self._bias_model
        data = m.get("data", [])
        if not data:
            return

        min_samples = 1 if force else int(m.get("min_samples", 5))
        if len(data) < max(1, min_samples):
            if DEBUG:
                print(f"[POLY-BIAS] skip fit: N={len(data)} < min={min_samples} (force={force})")
            return

        terms = m["terms"]
        lam = float(m.get("ridge_lambda", 1e-6))

        X_rows, y = [], []
        for rec in data:
            v0 = float(rec["v0"]); L = float(rec["L"])
            g = float(rec["grade"]); mass = float(rec["mass"])
            err = float(rec["err"])
            X_rows.append(self._poly_features_row(terms, v0, L, g, mass))
            y.append(err)

        try:
            X = np.array(X_rows, dtype=float)  # [N,P]
            y = np.array(y, dtype=float)       # [N]
        except Exception:
            return

        # 타깃 이상치 클립 (±10m)
        try:
            y = np.clip(y, -10.0, 10.0)
        except Exception:
            pass

        P = X.shape[1]
        # 열 정규화
        try:
            col_norm = np.linalg.norm(X, axis=0)
            col_norm[col_norm == 0.0] = 1.0
            Xn = X / col_norm

            XtX = Xn.T @ Xn
            Xty = Xn.T @ y
            A = XtX + lam * np.eye(P)

            coeffs = None
            try:
                coeffs = np.linalg.solve(A, Xty)
            except np.linalg.LinAlgError:
                try:
                    coeffs, *_ = np.linalg.lstsq(A, Xty, rcond=None)
                except Exception:
                    try:
                        coeffs = np.linalg.pinv(A) @ Xty
                    except Exception:
                        if DEBUG:
                            print("[POLY-BIAS] all solvers failed; keep old coeffs")
                        return

            # 정규화 해제 및 계수 클램프
            coeffs = coeffs / col_norm
            coeffs = np.clip(coeffs, -5.0, 5.0)

            m["coeffs"] = [float(c) for c in coeffs]
        except Exception as e:
            if DEBUG:
                print(f"[POLY-BIAS] fit failed: {e}")

    def _append_observation_and_update(
        self,
        v0_kmh: float,
        L_m: float,
        grade_percent: float,
        mass_tons: float,
        stop_error_m: float,
        force_fit: bool = False
    ):
        m = self._bias_model
        m.setdefault("data", [])
        m.setdefault("max_points", 2000)

        # 관측 추가
        rec = {"v0": float(v0_kmh), "L": float(L_m), "grade": float(grade_percent),
               "mass": float(mass_tons), "err": float(-stop_error_m)}
        m["data"].append(rec)

        # 상한 유지 (최근 max_points)
        maxp = int(m.get("max_points", 2000))
        if len(m["data"]) > maxp:
            m["data"] = m["data"][-maxp:]

        # 학습 시도 (force_fit=True면 표본 1개여도 수행)
        try:
            self._fit_bias_coeffs_from_data(force=force_fit)
        except Exception as e:
            if DEBUG:
                print(f"[POLY-BIAS] fit exception ignored: {e}")

        # 저장 (실패 무시)
        self._save_bias_model(self._bias_model_path, m)

        if DEBUG:
            n = len(m["data"])
            c0 = m["coeffs"][0] if m.get("coeffs") else 0.0
            pred = self._predict_bias_from_json(v0_kmh, L_m, grade_percent, mass_tons)
            print(f"[POLY-BIAS] N={n}, coeff0={c0:.6f}, last_err={stop_error_m:.3f} m, pred_bias_now={pred:.3f} m")

    # ---------- Margin/Physics ----------
    def compute_margin(self, mu: float, grade_permil: float, peak_notch: int, peak_dur_s: float) -> float:
        BASE_1C_T = self.veh.mass_t
        PAX_1C_T = 10.5
        REF_LOAD = 0.70

        mass_tons = max(1.0, self.veh.mass_kg / 1000.0)
        Lcars = getattr(self, "train_length", 10)

        baseline_tons = Lcars * (BASE_1C_T + PAX_1C_T * REF_LOAD)
        delta = mass_tons - baseline_tons

        mass_corr = (-2.5e-4) * delta + (1.5e-8) * (delta ** 3)
        mass_corr = max(-0.05, min(0.08, mass_corr))

        margin = -0.675
        scale = min(1.0, self.scn.L / 100.0)
        if grade_permil >= 0:
            grade_corr = -0.002 * grade_permil * (1 + abs(grade_permil) / 10.0) * scale
        else:
            grade_corr = -0.010 * grade_permil * scale

        mu_corr = (mu - 1.0) * (0.03 / (0.3 - 1.0))

        base_margin = margin + grade_corr + mu_corr + mass_corr

        # 다항 보정 더하기
        v0_kmh = self.scn.v0 * 3.6
        L_m = self.scn.L
        grade_percent = self.scn.grade_percent
        mass_total_tons = mass_tons
        bias_poly = self._predict_bias_from_json(v0_kmh, L_m, grade_percent, mass_total_tons)

        return base_margin + bias_poly

    def _dynamic_margin(self, v0: float, rem_now: float) -> float:
        mu = self.scn.mu
        grade_permil = self.scn.grade_percent * 10.0
        margin = self.compute_margin(mu, grade_permil, self._tasc_peak_notch, self._tasc_peak_duration)
        return margin

    def _effective_brake_accel(self, notch: int, v: float) -> float:
        if notch < 0:
            notch = 0
        if self.veh.notch_accels is None or len(self.veh.notch_accels) == 0:
            return 0.0
        if notch >= len(self.veh.notch_accels):
            return 0.0

        base = float(self.veh.notch_accels[notch])  # 음수

        blend_cutoff_speed = 40.0 / 3.6
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))
        speed_kmh = v * 3.6
        air_boost = 0.72 if speed_kmh <= 3.0 else 1.0

        # B1 미세조정 (실시간)
        # B1 미세조정 (실시간) — I 제거, 약한 P만 유지 + 엔드게임 승차감 가드 유지
        if notch == 1 and (not self._in_predict) and self.state is not None and (not self.state.finished):
            rem_now = self.scn.L - self.state.s
    # 현재 B1로 끝까지 가면 어디쯤 설지 추정
            s_b1_nominal = self._estimate_stop_distance(1, v, include_margin=False)
            error_m = rem_now - s_b1_nominal
    # 과도한 튐 방지용 에러 클리핑
            if error_m > 5.0: 
                error_m = 5.0
            elif error_m < -5.0:
                error_m = -5.0

            dt_sim = max(1e-3, self.scn.dt)

    # --- PI 중 I 제거: 누적항 완전 비활성화 ---
            self._b1_i = 0.0

    # --- 약한 P만 사용 (너무 약하면 0.04~0.08에서 튜닝) ---
            k_p = 0.06
            adjust = 1.0 - k_p * error_m

    # 기본 상하한 (제동력 과도/과소 방지)
            target_boost = max(0.70, min(1.10, adjust))

    # ⬇️ 네가 요청한 승차감 가드 유지 (마지막 1 m/0.3 m)
            if rem_now < 0.3:
                target_boost = 0.3
            if rem_now < 1.0:
                target_boost = max(0.50, min(0.60, target_boost))

    # 반응 속도 제한(저크 저감): 기존보다 부드럽게
            alpha = min(0.40, dt_sim / 0.030)
            self._b1_air_boost_state += alpha * (target_boost - self._b1_air_boost_state)

    # 최종 반영
            air_boost *= self._b1_air_boost_state

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
        return -F / max(1.0, self.veh.mass_kg) if v != 0 else 0.0

    def _update_brake_dyn(self, a_cmd: float, v: float, is_eb: bool, dt: float):
        going_stronger = (a_cmd < self.brk_accel)
        tau = self.tau_apply_eb if is_eb else self.tau_apply if going_stronger else (self.tau_release_lowv if v < 3.0 else self.tau_release)
        alpha = dt / max(1e-6, tau)
        self.brk_accel += (a_cmd - self.brk_accel) * alpha

    # ----------------- Controls -----------------
    def _clamp_notch(self, n: int) -> int:
        return max(0, min(self.veh.notches - 1, n))

    def queue_command(self, name: str, val: int = 0):
        self._cmd_queue.append({"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val})

    def _apply_command(self, cmd: dict):
        st = self.state
        name = cmd["name"]; val = cmd["val"]
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
        self._tasc_peak_duration = 0.0
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
        rem_now = max(0.0, self.scn.L - self.state.s) if self.state else 0.0
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
        return max(0.0, s)

    def _stopping_distance(self, notch: int, v: float, include_margin: bool = True) -> float:
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v, include_margin=include_margin)

    def _tasc_predict(self, cur_notch: int, v: float):
        st = self.state
        need = False
        if (st.t - self._tasc_last_pred_t) >= self._tasc_pred_interval: need = True
        if abs(v - self._tasc_pred_cache["v"]) >= self._tasc_speed_eps: need = True
        if cur_notch != self._tasc_pred_cache["notch"]: need = True
        if not need:
            return (self._tasc_pred_cache["s_cur"], self._tasc_pred_cache["s_up"], self._tasc_pred_cache["s_dn"])
        max_normal_notch = self.veh.notches - 2
        s_cur = self._stopping_distance(cur_notch, v) if cur_notch > 0 else float('inf')
        s_up = self._stopping_distance(cur_notch + 1, v) if cur_notch + 1 <= max_normal_notch else 0.0
        s_dn = self._stopping_distance(cur_notch - 1, v) if cur_notch - 1 >= 1 else float('inf')
        self._tasc_pred_cache.update({"t": st.t, "v": v, "notch": cur_notch, "s_cur": s_cur, "s_up": s_up, "s_dn": s_dn})
        self._tasc_last_pred_t = st.t
        return s_cur, s_up, s_dn

    def _need_B5_now(self, v: float, remaining: float) -> bool:
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last
        s_b4 = self._stopping_distance(2, v)  # Notch index 주의(환경에 맞게)
        need = s_b4 > (remaining + self.tasc_deadband_m)
        self._need_b5_last = need
        self._need_b5_last_t = st.t
        return need

    # ----------------- Main step -----------------
    def step(self):
        st = self.state
        dt = self.scn.dt

        # 큐 명령 적용
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            try:
                self._apply_command(self._cmd_queue.popleft())
            except Exception:
                pass

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
                        if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m + 0.1):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur - 1)
                                self._tasc_last_change_t = st.t

        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v)
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self._update_brake_dyn(a_cmd_brake, st.v, is_eb, dt)

        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)
        a_target = self.brk_accel + a_grade + a_davis

        max_da = max(0.0, self.veh.j_max) * dt
        da = a_target - st.a
        if da > max_da: da = max_da
        elif da < -max_da: da = -max_da
        st.a += da

        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        if st.lever_notch > self._tasc_peak_notch:
            self._tasc_peak_notch = st.lever_notch
            self._tasc_peak_duration = 0.0
        if st.lever_notch == self._tasc_peak_notch and st.lever_notch > 0:
            self._tasc_peak_duration += dt

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

            # 저크 기록 (간소)
            self.prev_a = st.a
            self.jerk_history.append(self.scn.dt)
            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False

            # === 런 종료 후: 관측 추가 + 즉시 학습/저장 (오차가 0이 아니면 무조건) ===
            try:
                v0_kmh = self.scn.v0 * 3.6
                L_m = self.scn.L
                grade_percent = self.scn.grade_percent
                mass_total_tons = self.veh.mass_kg / 1000.0
                err = float(st.stop_error_m or 0.0)
                if err != 0.0:
                    self._append_observation_and_update(
                        v0_kmh, L_m, grade_percent, mass_total_tons, err,
                        force_fit=True  # 무조건 학습
                    )
            except Exception as e:
                if DEBUG:
                    print(f"[POLY-BIAS] update failed: {e}")

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
        n = max(1, int(window_time / dt))
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
        m = getattr(self, "_bias_model", None) or {}
        def _clean(v):
            try:
                v = float(v)
                return 0.0 if not math.isfinite(v) else v
            except Exception:
                return 0.0
        return {
            "t": round(st.t, 3),
            "s": _clean(st.s),
            "v": _clean(st.v),
            "a": _clean(st.a),
            "lever_notch": int(st.lever_notch),
            "remaining_m": _clean(self.scn.L - st.s),
            "L": _clean(self.scn.L),
            "v_ref": _clean(self.vref(st.s)),
            "finished": bool(st.finished),
            "stop_error_m": _clean(st.stop_error_m if st.stop_error_m is not None else 0.0),
            "residual_speed_kmh": _clean(st.v * 3.6),
            "running": bool(self.running),
            "grade_percent": _clean(self.scn.grade_percent),
            "grade": _clean(self.scn.grade_percent),
            "score": int(getattr(st, "score", 0) or 0),
            "issues": getattr(st, "issues", {}),
            "tasc_enabled": bool(getattr(self, "tasc_enabled", False)),
            # 디버그/모니터링
            "mu": float(self.scn.mu),
            "rr_factor": float(0.7 + 0.3 * self.scn.mu),
            "davis_A0": _clean(self.veh.A0),
            "davis_B1": _clean(self.veh.B1),
            "davis_C2": _clean(self.veh.C2),
            "peak_notch": int(self._tasc_peak_notch),
            "peak_dur_s": _clean(self._tasc_peak_duration),
            "pb_data_n": int(len(m.get("data", []))),
            "pb_coeff0": _clean((m.get("coeffs", [0.0])[0] if m.get("coeffs") else 0.0)),
            "pb_bias_pred": _clean(self._predict_bias_from_json(self.scn.v0*3.6, self.scn.L, self.scn.grade_percent, self.veh.mass_kg/1000.0)),
            "learning_enabled": bool(HAS_NUMPY),
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

    # 파일이 없거나 손상되어도 안전하게 로드/생성
    vehicle = Vehicle.from_json(vehicle_json_path)

    # 프론트가 EB→...→N으로 올 때 서버는 N→...→EB로 쓰기 위해 반전
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))
    # 반전 후 인덱스 오류 방지: notches를 길이에 맞춤
    vehicle.notches = len(vehicle.notch_accels)

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.start()

    last_sim_time = time.perf_counter()
    last_send = 0.0
    send_interval = 1.0 / 30.0  # 30Hz

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - last_sim_time

            # ---- 입력 처리(논블로킹) ----
            try:
                msg = await asyncio.wait_for(ws.receive_text(), timeout=0.01)
                data = json.loads(msg)
                if data.get("type") == "cmd":
                    payload = data.get("payload", {})
                    name = payload.get("name")

                    if name == "setInitial":
                        speed = payload.get("speed")
                        dist = payload.get("dist")
                        grade = _safe_float(payload.get("grade", 0.0), 0.0) / 10.0  # ‰ → %
                        mu = _safe_float(payload.get("mu", 1.0), 1.0)
                        if speed is not None and dist is not None:
                            sim.scn.v0 = _safe_float(speed, 0.0) / 3.6
                            sim.scn.L = max(0.0, _safe_float(dist, 0.0))
                            sim.scn.grade_percent = grade
                            sim.scn.mu = max(0.0, min(1.0, mu))
                            sim.rr_factor = _mu_to_rr_factor(sim.scn.mu)
                            if DEBUG:
                                print(f"setInitial: v0={speed}km/h, L={dist}m, grade={grade}%, mu={mu}")
                            sim.reset()
                            sim.running = True

                    elif name == "start":
                        sim.start()

                    elif name in ("stepNotch", "applyNotch"):
                        delta = int(payload.get("delta", 0) or 0)
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
                        length = int(payload.get("length", 8) or 8)
                        vehicle.update_mass(length)
                        sim.train_length = length
                        if DEBUG:
                            print(f"Train length set to {length} cars.")
                        sim.reset()

                    elif name == "setLoadRate":
                        load_rate = _safe_float(payload.get("loadRate", 0.0), 0.0) / 100.0
                        length = int(payload.get("length", 8) or 8)
                        base_1c_t = vehicle.mass_t
                        pax_1c_t = 10.5
                        total_tons = length * (base_1c_t + pax_1c_t * max(0.0, min(1.0, load_rate)))
                        vehicle.update_mass(length)
                        vehicle.mass_kg = max(1.0, total_tons * 1000.0)
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
                        value = _safe_float(payload.get("value", 1.0), 1.0)
                        value = max(0.0, min(1.0, value))
                        sim.scn.mu = value
                        sim.rr_factor = _mu_to_rr_factor(value)
                        if DEBUG:
                            print(f"마찰계수(mu)={value} / rr_factor={sim.rr_factor:.3f}")
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

            # ---- 고정 시뮬 시간 ----
            dt = sim.scn.dt
            # 너무 밀리지 않도록 한 번에 처리할 최대 스텝 제한
            max_steps = 5
            steps = 0
            while elapsed >= dt and steps < max_steps:
                if sim.running:
                    try:
                        sim.step()
                    except Exception as e:
                        if DEBUG:
                            print(f"[step] exception: {e}")
                        sim.running = False
                last_sim_time += dt
                elapsed -= dt
                steps += 1

            # ---- 송신 제한 (30Hz) ----
            if (now - last_send) >= send_interval:
                try:
                    await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}, ensure_ascii=False))
                except WebSocketDisconnect:
                    break
                except Exception as e:
                    if DEBUG:
                        print(f"send failed: {e}")
                last_send = now

            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except Exception:
            pass

# ------------------------------------------------------------
# 로컬 실행용 엔트리포인트
# ------------------------------------------------------------
if __name__ == "__main__":
    # uvicorn main:app --reload 로 띄워도 됩니다.
    import uvicorn
    port = int(os.environ.get("PORT", "8000"))
    uvicorn.run("main:app", host="127.0.0.1", port=port, reload=False, log_level="info")