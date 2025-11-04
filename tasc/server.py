import math
import json
import asyncio
import time
import os

from dataclasses import dataclass
from collections import deque
from typing import Optional, List, Tuple

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------
DEBUG = False  # 디버그 로그를 보고 싶으면 True
# I realized that the soft stop logic does not make sense and makes the simulation less realistic 
# soft_stop_di = 10.0 
# soft_stop_const = -0.18
air_brake_vi = 12.0  # km/h 이하에서 공기제동 밸브 지연 반영 시작
test_notch = 5  # TASC 릴렉스 시도 시점에 비교할 목표 노치
test_margin = 2.0  # TASC 릴렉스 시도 시점에 비교할 거리 마진 (m)
# ------------------------------------------------------------
# Data classes
# ------------------------------------------------------------

@dataclass
class Vehicle:
    name: str = "EMU-233-JR-East"
    mass_t: float = 39.9
    a_max: float = 1.0
    j_max: float = 0.4
    notches: int = 10
    notch_accels: list = None
    tau_cmd: float = 0.150
    tau_brk: float = 0.250
    mass_kg: float = 39900
    # Vehicle 클래스 내
    maxSpeed_kmh: float = 140.0
    forward_notches: int = 5
    forward_notch_accels: list = None  # [-1.5, -1.1] 등


    # Davis 계수 (열차 전체) : F = A0 + B1 * v + C2 * v^2 [N], v[m/s]
    A0: float = 1200.0
    B1: float = 30.0
    C2: float = 8.0

    # 공력/기본 파라미터
    C_rr: float = 0.005
    rho_air: float = 1.225
    Cd: float = 1.8
    A: float = 10.0

    # --- Davis 자동추정용 튜닝 파라미터(추가) ---
    davis_k0: float = 0.0017        # A0 = k0 * m * g
    davis_m_ref: float = 200000.0   # B1 기준 질량(kg) = 200t
    davis_B1_ref: float = 40.0      # B1 기준값 (N·s/m) @ 200t

    def recompute_davis(self, mass_kg: Optional[float] = None):
        """현재 총질량(kg)에 맞춰 A0, B1, C2를 현실적으로 재계산"""
        m = float(mass_kg) if mass_kg is not None else float(self.mass_kg)
        # C2: 공력 항력(물리식)
        self.C2 = 0.5 * self.rho_air * self.Cd * self.A
        # A0: 구름/기계 상수항
        self.A0 = self.davis_k0 * m * 9.81
        # B1: 속도항(완만한 질량 비례)
        self.B1 = self.davis_B1_ref * (m / max(1.0, self.davis_m_ref))
        if DEBUG:
            print(f"[Davis] mass={m:.0f} kg -> A0={self.A0:.1f}, B1={self.B1:.2f}, C2={self.C2:.2f}")

    def update_mass(self, length: int):
        """편성 량 수에 맞춰 총 질량(kg)을 업데이트"""
        self.mass_kg = self.mass_t * 1000 * length
        # ★ 총질량 반영 후 Davis 재계산
        self.recompute_davis(self.mass_kg)

    @classmethod
    def from_json(cls, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        mass_t = data.get("mass_t", 200.0)
        obj = cls(
            name=data.get("name", "EMU-233-JR-East"),
            a_max=data.get("a_max", 1.0),
            j_max=data.get("j_max", 0.8),
            notches=data.get("notches", 8),
            notch_accels=data.get(
                "notch_accels",
                [-1.5, -1.10, -0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.0],
            ),
            maxSpeed_kmh=data.get("maxSpeed_kmh", 140.0),
            forward_notches=data.get("forward_notches", 5),
            forward_notch_accels=data.get("forward_notch_accels", [ 0.250, 0.287, 0.378, 0.515, 0.694 ]),
            tau_cmd=data.get("tau_cmd_ms", 150) / 1000.0,
            tau_brk=data.get("tau_brk_ms", 250) / 1000.0,
            mass_t=mass_t,
            mass_kg=mass_t * 1000,

            # 초기값(로드 시점 값; 재계산으로 덮어씀)
            A0=data.get("davis_A0", 1200.0),
            B1=data.get("davis_B1", 30.0),
            C2=data.get("davis_C2", 8.0),

            C_rr=0.005,
            rho_air=data.get("rho_air", 1.225),
            Cd=data.get("Cd", 1.8),
            A=data.get("A", 10.0),

            # Davis 추정용 옵션 로드(없으면 기본)
            davis_k0=data.get("davis_k0", 0.0017),
            davis_m_ref=data.get("davis_m_ref", 200000.0),
            davis_B1_ref=data.get("davis_B1_ref", 40.0),
        )
        # ★ 총질량 기준으로 Davis 재계산(최초 1회)
        obj.recompute_davis(obj.mass_kg)
        return obj


@dataclass
class Scenario:
    L: float = 500.0
    v0: float = 25.0
    grade_percent: float = 0.0
    mu: float = 1.0
    dt: float = 0.01

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

    # ▼ 타이머(카운트다운): float 원본 + 정수 표시값
    time_budget_s: float = 0.0            # 스테이지 부여 시간(초)
    time_remaining_s: float = 0.0         # 남은 시간(초) — 0 아래로 내려갈 수 있음
    timer_enabled: bool = False           # 타이머 사용 여부
    time_remaining_int: int = 0           # 정수 표시용(내림)
    time_overrun_s: float = 0.0           # 초과 시간(양수)
    time_overrun_int: int = 0             # 초과 시간 정수 표시
    time_overrun_started: bool = False    # 오버런 진입 여부


# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------

def build_vref(L: float, a_ref: float):
    def vref(s: float):
        rem = max(0.0, L - s)
        return math.sqrt(max(0.0, 2.0 * a_ref * rem))
    return vref


# ------------------------------------------------------------
# Simulator
# ------------------------------------------------------------

class StoppingSim:
    def __init__(self, veh: Vehicle, scn: Scenario):

       
        self._planned_v0 = scn.v0  # 출발 버튼 누를 때 쓸 예정 속도
        # --- 인계 지점(남은거리) & 히스테리시스 설정 ---
        self.tasc_takeover_rem_m = 150.0 # 인계 거리 (m)
        self.tasc_takeover_hyst_m = 1 # add some randomness like real world tasc beacons!

        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self.vref = build_vref(scn.L, 0.8 * veh.a_max)
        self._cmd_queue = deque()

        # 초기 제동(B1/B2) 판정
        self.first_brake_start: Optional[float] = None
        self.first_brake_done: bool = False

        # 기록
        self.notch_history: List[int] = []
        self.time_history: List[float] = []

        # EB 사용 여부
        self.eb_used = False

        # 저크 계산
        self.prev_a = 0.0
        self.jerk_history: List[float] = []

        # ---------- TASC ----------``
        self.tasc_enabled = True
        self.manual_override = False
        self.tasc_deadband_m = 0.05 #0.05
        self.tasc_hold_min_s = 0.05
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"  # "build" → "relax"
        self._tasc_peak_notch = 1
        # 대기/활성 상태
        self.tasc_armed = False
        self.tasc_active = False

        self.tasc_relax_margin_m = test_margin
        # 직전 단계 변경 후 추가로 요구할 최소 홀드시간 (초)
        self.tasc_relax_hold_s = 0

        # μ-저항 분리: rr_factor는 항상 1.0로 고정(μ와 무관)
        self.rr_factor = 1.0

        # ---- 성능 최적화: TASC 예측 캐시/스로틀 ----
        self._tasc_pred_cache = {
            "t": -1.0, "v": -1.0, "notch": -1,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        }
        self._tasc_pred_interval = 0.05  # 50ms
        self._tasc_last_pred_t = -1.0
        self._tasc_speed_eps = 0.3  # m/s

        # ---- B5 필요 여부 캐시/스로틀 ----
        self._need_b5_last_t = -1.0
        self._need_b5_last = False
        self._need_b5_interval = 0.05

        
        # -------------------- 동력/응답/상태 --------------------
        self.pwr_accel = 0.0   # 동력 가속도 (forward_notch_accels 반영)
        
        # -------------------- 제동/응답/상태 --------------------
        self.brk_accel = 0.0
        self.brk_elec = 0.0
        self.brk_air  = 0.0



        self.tau_apply = 0.25
        self.tau_release = 0.8
        self.tau_apply_eb = 0.15
        self.tau_release_lowv = 0.8

        self.wsp_state = "normal"
        self.wsp_timer = 0.0

        self._a_cmd_filt = 0.0  # 명령 가속도 1차 필터

        # -------------- 타이머 정책(카운트다운) --------------
        # 표 기반 / 공식 기반 / 보정 기반 자동 산출
        self.timer_use_table = False
        self.timer_table = {}             # 예: {60:35, 70:30, 80:26}
        self.timer_v_target_kmh = 70.0    # 공식 기반 목표 속도(km/h)
        self.timer_buffer_s = 0.0         # 여유초

        # ---------- 타이머 자동 산출(보정 데이터 기반) ----------
        # 보정 데이터: [{"v":60, "L":200, "t":23}, ...]  (km/h, m, sec)
        # ---------- 타이머 자동 산출(보정 데이터 기반) ----------
# 보정 데이터: [{"v":60, "L":200, "t":23}, ...]  (km/h, m, sec)
        self.timer_calib: List[dict] = [
            {"v": 40, "L": 150, "t": 27},
            {"v": 60, "L": 200, "t": 28},
            {"v": 70, "L": 300, "t": 32},
            {"v": 90, "L": 500, "t": 40},
            {"v": 130, "L": 900, "t": 49}
        ]
        self.timer_idw_power = 2.0         # IDW 거듭제곱
        # 속도/거리 정규화 스케일(거리 계산 공정성 확보)
        self.timer_norm_v = 100.0          # km/h 스케일
        self.timer_norm_L = 300.0          # m 스케일
        # 기준점에서 멀면 공식기반과 블렌딩
        self.timer_blend_threshold = 1.5   # 정규화 거리 기준

        # ▼ 극단값/이상치 처리용 가드레일
        self.timer_min_s = 5.0
        self.timer_max_s = 300.0
        self.timer_min_effective_v_kmh = 12.0   # 공식에 쓰는 최소 유효 속도
        self.timer_max_effective_v_kmh = 110.0  # 공식에 쓰는 최대 유효 속도
        self.timer_far_outlier_scale = 0.35     # 아주 멀면 공식 가중 하한

        # 타임오버 페널티/보너스 정책
        self.timer_overtime_penalty_per_s = 20.0  # 1초당 -20점
        self.timer_overtime_penalty_cap = 400.0   # 최대 페널티
        self.timer_exact_bonus = 100              # 정수 0초 도착 시 +100점

        # 입력 보정 기록(클라이언트에 안내용)
        self.last_input_sanitized = {}

    # ----------------- Timer helpers -----------------

    def set_timer_calibration(self, points: List[dict],
                              norm_v: float = None,
                              norm_L: float = None,
                              idw_power: float = None,
                              blend_threshold: float = None):
        """보정 표를 통째로 교체"""
        self.timer_calib = []
        for p in points:
            v = float(p.get("v") or p.get("v_kmh"))
            L = float(p.get("L") or p.get("dist"))
            t = float(p.get("t") or p.get("time"))
            self.timer_calib.append({"v": v, "L": L, "t": t})
        if norm_v is not None: self.timer_norm_v = float(norm_v)
        if norm_L is not None: self.timer_norm_L = float(norm_L)
        if idw_power is not None: self.timer_idw_power = float(idw_power)
        if blend_threshold is not None: self.timer_blend_threshold = float(blend_threshold)

    def _idw_predict_time(self, v_kmh: float, L_m: float) -> Tuple[float, float]:
        """보정 표 기반 IDW 추정. (예상시간, 기준점까지의 최소 정규화거리) 반환"""
        if not self.timer_calib:
            return float("nan"), float("inf")
        eps = 1e-6
        num = 0.0
        den = 0.0
        min_d = float("inf")
        for p in self.timer_calib:
            dv = (v_kmh - p["v"]) / max(eps, self.timer_norm_v)
            dL = (L_m   - p["L"]) / max(eps, self.timer_norm_L)
            d = (dv*dv + dL*dL) ** 0.5
            min_d = min(min_d, d)
            w = 1.0 / ((d + eps) ** self.timer_idw_power)
            num += w * p["t"]
            den += w
        t_idw = num / max(eps, den)
        return t_idw, min_d

    def _formula_time(self, v0_kmh: float, L_m: float) -> float:
        """
        기본 공식 기반 시간 = L / v_eff + buffer
        v_eff는 v_target과 v0의 완만한 혼합(안정적 추정).
        """
        v_target = self.timer_v_target_kmh
        # 거리 비율로 혼합 가중(멀수록 v_target 비중 ↑) — 0~1로 스케일
        r = min(1.0, max(0.0, L_m / self.timer_norm_L))  # 300m 기준
        # 너무 느린 v0는 하한, 너무 빠른 v0는 상한
        v0_clip = min(self.timer_max_effective_v_kmh,
                      max(self.timer_min_effective_v_kmh, v0_kmh))
        v_eff = (1.0 - 0.35*r) * v0_clip + (0.35*r) * v_target  # r↑일수록 v_target 쪽
        v_ms = max(0.1, v_eff / 3.6)
        return float(L_m / v_ms + self.timer_buffer_s)

    def _compute_time_budget_auto(self, v_kmh: float, L_m: float) -> float:
        # 1) 보정 표(IDW)ㅋ
        t_idw, min_d = self._idw_predict_time(v_kmh, L_m)

        # 2) 강화된 공식 기반
        t_formula = self._formula_time(v_kmh, L_m)

        # 3) 블렌딩(기준점과 멀수록 공식 비중↑)
        if not self.timer_calib or math.isnan(t_idw):
            t = t_formula
        else:
            if min_d < self.timer_blend_threshold:
                # 기준점 근방 → 보정 표 신뢰
                t = t_idw
            else:
                # 먼 이상치 → 선형으로 공식 비중↑
                alpha = max(0.0, 1.0 - (min_d / (self.timer_blend_threshold * 2.0)))
                # 공식 비중 하한(너무 멀면 공식 최소 35% 반영)
                formula_weight = max(1.0 - alpha, self.timer_far_outlier_scale)
                t = (1.0 - formula_weight) * t_idw + formula_weight * t_formula

        # 4) 최종 클램핑
        t = max(self.timer_min_s, min(self.timer_max_s, t))
        return t

    def _compute_time_budget(self) -> float:
        """스테이지 시작 시 부여할 제한시간(초) 계산"""
        if not self.state.timer_enabled:
            return 0.0

        v0_kmh = self.scn.v0 * 3.6
        L_m = self.scn.L

        # A) 보정 표가 있으면 자동 산출 우선
        if self.timer_calib:
            return self._compute_time_budget_auto(v0_kmh, L_m)

        # B) 정적 테이블 매핑 사용 시
        if self.timer_use_table and self.timer_table:
            v0_round = round(v0_kmh)
            key = min(self.timer_table.keys(), key=lambda k: abs(int(k) - v0_round))
            return float(self.timer_table[key])

        # C) 그 외엔 공식 기반
        return self._formula_time(v0_kmh, L_m)

    # ----------------- Physics helpers -----------------

    # def _effective_brake_accel(self, notch: int, v: float) -> float:
    #     if notch >= len(self.veh.notch_accels):
    #         return 0.0
    #     if notch >= 0:
    #         base = float(self.veh.notch_accels[notch])  # 브레이크
    #     else:
    #         idx = -notch - 1
    #         base = float(self.veh.forward_notch_accels[idx])  # 전진 notch
    #     k_srv = 0.85
    #     k_eb = 0.98
    #     is_eb = (notch == (self.veh.notches - 1))
    #     k_adh = k_eb if is_eb else k_srv
    #     a_cap = -k_adh * float(self.scn.mu) * 9.81
    #     a_eff = max(base, a_cap)
    #     if a_eff <= a_cap + 1e-6:
    #         scale = 0.90 if v > 8.0 else 0.85
    #         a_eff = a_cap * scale
    #     return a_eff

    # def _effective_brake_accel(self, notch: int, v: float) -> float:
    #     # ✅ 악셀(음수) 또는 N(0)에서는 '브레이크 없음'
    #     if notch <= 0:
    #         return 0.0

    #     if notch >= len(self.veh.notch_accels):
    #         return 0.0

    #     base = float(self.veh.notch_accels[notch]) # 음수여야 정상(제동)
    #     k_srv = 0.85
    #     k_eb = 0.98
    #     is_eb = (notch == self.veh.notches - 1)
    #     k_adh = k_eb if is_eb else k_srv
    #     a_cap = -k_adh * float(self.scn.mu) * 9.81 # 음수

    #     a_eff = max(base, a_cap)
    #     if a_eff <= a_cap + 1e-6:
    #         scale = 0.90 if v > 8.0 else 0.85
    #         a_eff = a_cap * scale
    #     return a_eff

    def _effective_brake_accel(self, notch: int, v: float) -> float:
        # ✅ 악셀(음수) 또는 N(0)에서는 '브레이크 없음'
        if notch <= 0:
            return 0.0

        if notch >= len(self.veh.notch_accels):
            return 0.0

        v_kmh = v * 3.6

        # 기본 제동 가속도(음수)
        base = float(self.veh.notch_accels[notch]) # 음수여야 정상(제동)
    
        # 저속(<=10 km/h)에서는 마찰/밸브 한계로 제동력이 감소 (0.8~1.0 범위)
        # 저속에서 제동력 계수 적용 (12->8km/h 구간 선형, 8km/h 이하 0.90, <=3km/h는 0.85)
        # 기본 factor 계산
        if v_kmh >= air_brake_vi:
            factor = 1.0
        elif v_kmh > 8.0:
            # 8~air_brake_vi km/h 사이 선형 보간 (8→0.95, air_brake_vi→1.0)
            factor = 0.95 + (v_kmh - 8.0) * 0.05 / max(1e-6, (air_brake_vi - 8.0))
        elif v_kmh > 5.0:
            factor = 0.90
        else:
            # 5km/h 이하
            factor = 0.80 if notch == 1 else 0.90  # B1은 특별히 더 낮게

        # base 보정
        if notch == 1 and v_kmh <= 5.0:
            base = min(base * factor, -0.15)  # 지나친 미끄럼 방지 위해 최소 제동력을 -0.15로 클램핑
        else:
            base *= factor


        k_srv = 0.85
        k_eb = 0.98
        is_eb = (notch == self.veh.notches - 1)
        k_adh = k_eb if is_eb else k_srv
        a_cap = -k_adh * float(self.scn.mu) * 9.81 # 음수

        a_eff = max(base, a_cap)
        if a_eff <= a_cap + 1e-6:
            scale = 0.90 if v > 8.0 else 0.85
            a_eff = a_cap * scale
        return a_eff

    def _grade_accel(self) -> float:
        return -9.81 * (self.scn.grade_percent / 100.0)

    def _davis_accel(self, v: float) -> float:
        """Davis 저항을 가속도로 환산 (A0/B1/C2는 차량 객체의 최신값 사용)"""
        A0 = self.veh.A0 * self.rr_factor
        B1 = self.veh.B1 * self.rr_factor
        C2 = self.veh.C2
        F = A0 + B1 * v + C2 * v * v  # N
        return -F / self.veh.mass_kg if v != 0 else 0.0

    # ----------------- 기타 헬퍼 -----------------
    def _taus_for_speed(self, v: float, is_eb: bool):
        v_kmh = v * 3.6
        if is_eb:
            return 0.15, 0.45
        if v_kmh >= 15.0:
            return 0.18, 0.40
        elif v_kmh >= 10.0:
            return 0.30, 0.60
        else:
            return 0.55, 0.80

    def _blend_w_regen(self, v: float) -> float:
        v_kmh = v * 3.6
        if v_kmh >= 20.0: return 1.0
        if v_kmh <= 8.0:  return 0.0
        return (v_kmh - 8.0) / 12.0

    def _update_brake_dyn_split(self, a_total_cmd: float, v: float, is_eb: bool, dt: float):
        w = self._blend_w_regen(v)
        a_cmd_e = a_total_cmd * w
        a_cmd_a = a_total_cmd * (1.0 - w)
        tau_e_apply, tau_e_rel = (0.18, 0.40) if v * 3.6 >= 15 else (0.30, 0.50)
        tau_a_apply, tau_a_rel = (0.45, 0.75) if v * 3.6 < 10 else (0.30, 0.60)
        if is_eb:
            tau_a_apply, tau_a_rel = 0.15, 0.45
        e_stronger = (a_cmd_e < self.brk_elec)
        a_stronger = (a_cmd_a < self.brk_air)
        tau_e = tau_e_apply if e_stronger else tau_e_rel
        tau_a = tau_a_apply if a_stronger else tau_a_rel
        self.brk_elec += (a_cmd_e - self.brk_elec) * (dt / max(1e-6, tau_e))
        self.brk_air  += (a_cmd_a - self.brk_air ) * (dt / max(1e-6, tau_a))
        self.brk_accel = self.brk_elec + self.brk_air

    def _wsp_update(self, v: float, a_demand: float, dt: float):
        a_cap = -0.85 * self.scn.mu * 9.81
        margin = 0.05
        if self.wsp_state == "normal":
            if a_demand < (a_cap - margin) and v * 3.6 > 3.0:
                self.wsp_state = "release"
                self.wsp_timer = 0.12
                return min(a_demand, 0.5 * a_cap)
            return a_demand
        elif self.wsp_state == "release":
            self.wsp_timer -= dt
            if self.wsp_timer <= 0.0:
                self.wsp_state = "reapply"
                self.wsp_timer = 0.15
            return min(a_demand, 0.3 * a_cap)
        else:
            self.wsp_timer -= dt
            if self.wsp_timer <= 0.0:
                self.wsp_state = "normal"
            return min(a_demand, 0.8 * a_cap)

    # ----------------- Controls -----------------

    def _clamp_notch(self, n: int) -> int:
        # forward_notches 길이만큼 음수 허용
        min_notch = -len(self.veh.forward_notch_accels)  # 예: -2
        max_notch = len(self.veh.notch_accels) - 1       # EB 인덱스
        return max(min_notch, min(max_notch, n))


    def queue_command(self, name: str, val: int = 0):
        self._cmd_queue.append(
            {"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val}
        )

    def _apply_command(self, cmd: dict):
        st = self.state
        name = cmd["name"]
        val = cmd["val"]

        # ▼ TASC가 'active'인 상태에서 수동 개입이 들어오면 즉시 TASC를 OFF
        if self.tasc_enabled and self.tasc_active and name in ("stepNotch", "applyNotch", "release", "emergencyBrake"):
            self.tasc_enabled = False
            self.tasc_active = False
            self.tasc_armed = False
            if DEBUG:
                print("[TASC] manual intervention while ACTIVE -> TASC OFF")

        # ▼ 이하 기존 로직(lever_notch 직접 조작) 유지
        if name == "stepNotch":
            old_notch = st.lever_notch
            st.lever_notch = self._clamp_notch(st.lever_notch + val)
            if DEBUG:
                print(f"Applied stepNotch: {old_notch} -> {st.lever_notch}")
        elif name == "release":
            st.lever_notch = 0
        elif name == "emergencyBrake":
            st.lever_notch = self.veh.notches - 1
            self.eb_used = True
        elif name == "setNotch":
            st.lever_notch = self._clamp_notch(val)


    # ----------------- Lifecycle -----------------

    def reset(self):
        # ▼ 기존 상태의 timer_enabled를 보존(없으면 False)
        prev_timer_enabled = getattr(self.state, "timer_enabled", False)



        # 계획 속도는 시나리오의 v0를 따로 들고 있고, 대기 상태에는 v=0으로 둔다
        self._planned_v0 = self.scn.v0

        self.state = State(
            t=0.0, s=0.0, v=0.0, a=0.0, lever_notch=0, finished=False
        )
        self.running = False
        self._cmd_queue.clear()

        self.first_brake_start = None
        self.first_brake_done = False
        self.eb_used = False
        self.notch_history.clear()
        self.time_history.clear()

        self.prev_a = 0.0
        self.jerk_history = []

        # self.manual_override = False
        self._tasc_last_change_t = 0.0
        if not self.tasc_active:
            self._tasc_phase = "build"
            self._tasc_peak_notch = 1

        self.tasc_active = False
        self.tasc_armed = bool(self.tasc_enabled)

        self._tasc_pred_cache.update({
            "t": -1.0, "v": -1.0, "notch": -1,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        })
        self._tasc_last_pred_t = -1.0

        self._need_b5_last_t = -1.0
        self._need_b5_last = False

        self.brk_accel = 0.0
        self.brk_elec = 0.0
        self.brk_air  = 0.0

        self.wsp_state = "normal"
        self.wsp_timer = 0.0

        self._a_cmd_filt = 0.0

        self.rr_factor = 1.0

        # ▼ 보존해 둔 타이머 플래그 복원
        self.state.timer_enabled = prev_timer_enabled

        # ▼ 타이머 초기화 (예산시간/남은시간 세팅)
        if self.state.timer_enabled:
            self.state.time_budget_s = self._compute_time_budget()
            self.state.time_remaining_s = self.state.time_budget_s
        else:
            self.state.time_budget_s = 0.0
            self.state.time_remaining_s = 0.0

        # ▼ 정수 표시 초기화
        self.state.time_remaining_int = math.floor(self.state.time_remaining_s)
        self.state.time_overrun_s = 0.0
        self.state.time_overrun_int = 0
        self.state.time_overrun_started = False

        if DEBUG:
            print(f"Simulation reset | timer_enabled={self.state.timer_enabled} "
                  f"| budget={self.state.time_budget_s:.2f}s | L={self.scn.L} v0={self.scn.v0*3.6:.1f}km/h")

    def start(self):
        self.reset()
        self.state.v = float(self._planned_v0) 
        self.running = True
        self._t_start = time.time()  # sim_loop에서 참조 가능
        if DEBUG:
            print("Simulation started")


    def compute_power_accel(self, lever_notch: int, v: float) -> float:
        if lever_notch >= 0 or v <= 0.0:
            return 0.0

        n_notches = len(self.veh.forward_notch_accels)
        idx = max(0, min(-lever_notch - 1, n_notches - 1))
        base_accel = self.veh.forward_notch_accels[idx]

        v_max_total = max(1e-6, self.veh.maxSpeed_kmh / 3.6)
        if idx < 1:
            v_cap = (v_max_total * (idx + 1) / n_notches) + (10 / 3.6)
        else:
            v_cap = (v_max_total * (idx + 1) / n_notches)

        fade_start = 0.1 * v_cap
        #0.85 0.20
        min_factor = 0.05  # 캡 근처에서도 5%는 남음

        if v <= fade_start:
            factor = 1.0
        else:
            x = (v - fade_start) / (v_cap - fade_start)
            factor = 1.0 - (1.0 - min_factor) * x
            factor = max(factor, min_factor)  # clamp 하한값

        return base_accel * factor


    def eb_used_from_history(self) -> bool:
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # ------ stopping distance helpers ------

    def _estimate_stop_distance(self, notch: int, v0: float) -> float:
        # if notch <= 0:
        #     return float('inf')



        dt = 0.03
        v = max(0.0, v0)
        a = float(self.state.a)
        s = 0.0



        brk_elec = float(self.brk_elec)
        brk_air  = float(self.brk_air)
        wsp_state = self.wsp_state
        wsp_timer = float(self.wsp_timer)
        a_cmd_filt = float(self._a_cmd_filt)

        rem_now = self.scn.L - self.state.s
        limit = float(rem_now + 8.0)

        ctrl_delay = max(self._tasc_pred_interval, self.tasc_hold_min_s)
        latency_margin = v * ctrl_delay

        for _ in range(2400):

                                # 동력 가속도 계산 (전진 notch)
            if notch < 0:  # P1~P5
                pwr_accel = self.compute_power_accel(notch, v)
            else:
                pwr_accel = 0.0
            is_eb = (notch == self.veh.notches - 1)
            a_cmd_total = self._effective_brake_accel(notch, v)

            w = self._blend_w_regen(v)
            a_cmd_e = a_cmd_total * w
            a_cmd_a = a_cmd_total * (1.0 - w)

            tau_e_apply, tau_e_rel = (0.18, 0.40) if v * 3.6 >= 15 else (0.30, 0.50)
            tau_a_apply, tau_a_rel = (0.45, 0.75) if v * 3.6 < 10 else (0.30, 0.60)
            if is_eb:
                tau_a_apply, tau_a_rel = 0.15, 0.45

            e_stronger = (a_cmd_e < brk_elec)
            a_stronger = (a_cmd_a < brk_air)

            tau_e = tau_e_apply if e_stronger else tau_e_rel
            tau_a = tau_a_apply if a_stronger else tau_a_rel

            brk_elec += (a_cmd_e - brk_elec) * (dt / max(1e-6, tau_e))
            brk_air  += (a_cmd_a - brk_air ) * (dt / max(1e-6, tau_a))
            a_brake = brk_elec + brk_air

            a_cap = -0.85 * self.scn.mu * 9.81
            margin = 0.05
            if wsp_state == "normal":
                if a_brake < (a_cap - margin) and v * 3.6 > 3.0:
                    wsp_state = "release"
                    wsp_timer = 0.12
                    a_brake = min(a_brake, 0.5 * a_cap)
            elif wsp_state == "release":
                wsp_timer -= dt
                if wsp_timer <= 0.0:
                    wsp_state = "reapply"
                    wsp_timer = 0.15
                a_brake = min(a_brake, 0.3 * a_cap)
            else:
                wsp_timer -= dt
                if wsp_timer <= 0.0:
                    wsp_state = "normal"
                    wsp_timer = 0.0
                a_brake = min(a_brake, 0.8 * a_cap)

            a_grade = self._grade_accel()
            a_davis = self._davis_accel(v)
            a_target = pwr_accel + a_brake + a_grade + a_davis

            if notch == 0:
                a_target = self._grade_accel() + self._davis_accel(v)

            # E233계열은 회생제동 우선 제어 방식을 사용하며, 속도 약 7~10 km/h 이하에서 회생제동이 실질적으로 사라집니다.

            # 이때 공기제동이 완전히 takeover
            # 공기압 밸브 제어에 따른 지연이 필연적으로 존재합니다.
            # 일반적으로 응답상수 τ ≈ 0.5초 내외로 알려져 있습니다.

            # 반면 회생제동의 경우 전류 제어 응답이 수백 ms(0.2~0.3s) 수준이라
            # 체감상 약 1.5~2배 느리다고 볼 수 있습니다.

            # 제동력 자체는 저속 시 마찰제동의 압력 제한 및 마찰계수 변화로 인해
            # 약 0.7~0.8배 수준으로 감소합니다.

            # # (신규) 속도 기반 소프트 스톱
            rem_pred = max(0.0, rem_now - s)
            # v_kmh = v * 3.6
            # if v_kmh <= soft_stop_di and notch > 0:
            #     alpha = max(0.0, min(1.0, v_kmh / soft_stop_di))
            #     a_soft = (-0.30) * alpha + (soft_stop_const) * (1.0 - alpha)
            #     w_soft = 1.0 - alpha
            #     a_target = (1.0 - w_soft) * a_target + w_soft * a_soft

            ### NEW NEW NEW
            # if notch == 1 or rem_pred <= 0.0:
            #     a_target = min(a_target, 0.0)

            # # 응답시간(tau_brk)을 저속에서 늘려서 밸브 지연 반영
            # v_kmh_local = v * 3.6
            # effective_tau_brk = self.veh.tau_brk * 1.5 if v_kmh_local <= air_brake_vi else self.veh.tau_brk

            # a_cmd_filt += (a_target - a_cmd_filt) * (dt / max(1e-6, effective_tau_brk))
            ### NEW NEW NEW

            if notch == 1 or rem_pred <= 0.0:
                a_target = min(a_target, 0.0)

            a_cmd_filt += (a_target - a_cmd_filt) * (dt / max(1e-6, self.veh.tau_brk))

            max_da = self.veh.j_max * dt
            v_kmh = v * 3.6
            if v_kmh <= 5.0:
                scale = 0.25 + 0.75 * (v_kmh / 5.0)
                max_da *= scale

            da = a_cmd_filt - a
            if da > max_da:
                da = max_da
            elif da < -max_da:
                da = -max_da
            a += da

            v = max(0.0, v + a * dt)
            s += v * dt + 0.5 * a * dt * dt

            if v <= 0.01:
                break
            if s > limit:
                break

        return s + latency_margin

    def _stopping_distance(self, notch: int, v: float) -> float:
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v)

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
            return (
                self._tasc_pred_cache["s_cur"],
                self._tasc_pred_cache["s_up"],
                self._tasc_pred_cache["s_dn"],
            )

        max_normal_notch = self.veh.notches - 2
        s_cur = self._stopping_distance(cur_notch, v) if cur_notch > 0 else float("inf")
        s_up = self._stopping_distance(cur_notch + 1, v) if cur_notch + 1 <= max_normal_notch else 0.0
        s_dn = self._stopping_distance(cur_notch - 1, v) if cur_notch - 1 >= 1 else float("inf")

        self._tasc_pred_cache.update(
            {"t": st.t, "v": v, "notch": cur_notch, "s_cur": s_cur, "s_up": s_up, "s_dn": s_dn}
        )
        self._tasc_last_pred_t = st.t
        return s_cur, s_up, s_dn

    def _need_B5_now(self, v: float, remaining: float) -> bool:
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last

        v0_kmh = self.scn.v0 * 3.6
        if v0_kmh < 75.0:
            n_ref = 2
        elif v0_kmh < 85.0:
            n_ref = 3
        elif v0_kmh < 95.0:
            n_ref = 3
        else:
            n_ref = 4

        s_ref = self._stopping_distance(n_ref, v)
        need = s_ref > (remaining + self.tasc_deadband_m)
        self._need_b5_last = need
        self._need_b5_last_t = st.t
        return need

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
                if self.first_brake_start is None and not self.tasc_active:
                    self.first_brake_start = self.state.t

                elif (st.t - self.first_brake_start) >= 1.0:
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ▼ 타이머(카운트다운): 0 아래로도 계속 진행
        if st.timer_enabled and not st.finished:
            st.time_remaining_s -= dt
            st.time_remaining_int = math.floor(st.time_remaining_s)
            if st.time_remaining_s < 0.0:
                st.time_overrun_s = -st.time_remaining_s
                st.time_overrun_int = abs(st.time_remaining_int)
                if not st.time_overrun_started:
                    st.time_overrun_started = True
                    st.issues = getattr(st, "issues", {})
                    st.issues["timeout_started"] = True
            else:
                st.time_overrun_s = 0.0
                st.time_overrun_int = 0

        # ---------- TASC ----------
        if self.tasc_enabled and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            cur = st.lever_notch
            max_normal_notch = self.veh.notches - 2

            if self.tasc_armed and not self.tasc_active:
                takeover_on = self.tasc_takeover_rem_m
                hyst = self.tasc_takeover_hyst_m
                if rem_now <= (takeover_on + hyst):
                    self.tasc_active = True
                    self.tasc_armed = False
                    self._tasc_last_change_t = st.t

            if self.tasc_active:
                if not self.first_brake_done:
                    self.first_brake_done = True
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
                        # if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m):
                        #     if dwell_ok:
                        #         st.lever_notch = self._clamp_notch(cur - 1)
                        #         self._tasc_last_change_t = st.t
                        if cur > 1:
                            target_notch = cur - 1
                            # 변경: "릴렉스되어 내려갈 목표 노치(target_notch)가 4 이상"일 때만 마진/홀드를 적용
                            # (즉, 5→4, 6→5 처럼 결과가 여전히 4 이상인 경우에만 지연)
                            if cur >= test_notch:
                                # 더 보수적으로 완화하려면 추가 마진 요구
                                relax_allowed = (s_dn <= (rem_now + self.tasc_deadband_m - self.tasc_relax_margin_m))
                                time_since_change = st.t - self._tasc_last_change_t
                                if relax_allowed and dwell_ok and (time_since_change >= self.tasc_relax_hold_s):
                                    st.lever_notch = self._clamp_notch(target_notch)
                                    self._tasc_last_change_t = st.t
                            else:
                                # 목표 노치가 3 이하(3,2,1 등)면 기존 즉시 완화 규칙 유지
                                if s_dn <= (rem_now + self.tasc_deadband_m) and dwell_ok:
                                    st.lever_notch = self._clamp_notch(target_notch)
                                    self._tasc_last_change_t = st.t
        # ---------- Dynamics ----------

                
        pwr_accel = self.compute_power_accel(st.lever_notch, st.v)


        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v)
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self._update_brake_dyn_split(a_cmd_brake, st.v, is_eb, dt)
        a_brake = self._wsp_update(st.v, self.brk_accel, dt)

        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)

        a_target = pwr_accel + a_brake + a_grade + a_davis

        rem_now = self.scn.L - st.s
        v_kmh = st.v * 3.6
        # --- 속도 기반 소프트 스톱 ---
        # if v_kmh <= soft_stop_di and st.lever_notch > 0:
        #     alpha = max(0.0, min(1.0, v_kmh / soft_stop_di))     # 5km/h→1, 0km/h→0
        #     # -0.08
        #     a_soft = (-0.30) * alpha + (soft_stop_const) * (1.0 - alpha)
        #     w_soft = 1.0 - alpha                         # 속도가 낮을수록 소프트 비중↑
        #     a_target = (1.0 - w_soft) * a_target + w_soft * a_soft
        
        ### NEW NEW NEW 
        # if st.lever_notch >= 1:
        #     a_target = min(a_target, 0.0)

        # # 저속(<=10 km/h)일 때 tau_brk를 2배로 사용하여 공기제동 밸브 지연을 반영
        # v_kmh = st.v * 3.6
        # effective_tau_brk = self.veh.tau_brk * 1.5 if v_kmh <= air_brake_vi else self.veh.tau_brk

        # self._a_cmd_filt += (a_target - self._a_cmd_filt) * (dt / max(1e-6, effective_tau_brk))
        ### NEW NEW NEW 

        if st.lever_notch >= 1:
            a_target = min(a_target, 0.0)

        self._a_cmd_filt += (a_target - self._a_cmd_filt) * (dt / max(1e-6, self.veh.tau_brk))        

        max_da = self.veh.j_max * dt
        if v_kmh <= 5.0:
            scale = 0.25 + 0.75 * (v_kmh / 5.0)
            max_da *= scale

        da = self._a_cmd_filt - st.a
        if da > max_da:
            da = max_da
        elif da < -max_da:
            da = -max_da
        st.a += da

        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        # ---------- Finish ----------
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
                score += 400

            st.issues["early_brake_too_short"] = not self.first_brake_done
            st.issues["step_brake_incomplete"] = not self.is_stair_pattern(self.notch_history)
            st.issues["stop_error_m"] = st.stop_error_m

            jerk = abs((st.a - self.prev_a) / dt)
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            # ▼ 타임오버/정확 도착 보정 (정수 초 기준)
            if st.timer_enabled:
                # 정수 0초(내림) 도착 → 보너스
                if st.time_remaining_int == 0:
                    score += int(self.timer_exact_bonus)
                    st.issues["timer_exact_hit"] = True
                    st.issues["timer_exact_bonus"] = int(self.timer_exact_bonus)
                    st.time_overrun_int = 0
                    st.time_overrun_s = 0.0
                elif st.time_remaining_int < 0:
                    over_s = abs(st.time_remaining_int)  # 정수 초
                    overtime_pen = min(
                        self.timer_overtime_penalty_per_s * over_s,
                        self.timer_overtime_penalty_cap
                    )
                    score -= int(overtime_pen)
                    st.issues["timeout_overrun_s"] = over_s
                    st.issues["timeout_penalty"] = int(overtime_pen)
                # 남은 시간이 양수(조기 도착)인 경우는 보너스/페널티 없음

            st.score = score
            self.running = False
            if DEBUG:
                print(f"Avg jerk: {avg_jerk:.4f}, jerk_score: {jerk_score:.2f}, final score: {score}")
                print(f"Simulation finished: stop_error={st.stop_error_m:.3f} m, score={score}")

    def is_stair_pattern(self, notches: List[int]) -> bool:
        if len(notches) < 3:
            return False

        # # 첫 브레이크가 1 또는 2인지 확인
        # first_brake_notch = next((n for n in notches if n > 0), None)
        # if first_brake_notch not in (1, 2):
        #     return False

        peak_reached = False
        prev = notches[0]

        for cur in notches[1:]:
            if not peak_reached:
                if cur < prev:  # 내려가기 시작하면 피크 도달
                    peak_reached = True
            else:
                if cur > prev:  # 피크 이후 다시 올라가면 실패
                    return False
            prev = cur

        # 마지막은 1로 끝나야 함
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

        # 12 이하 → 500점, 12~30 → 선형 감소, 30 이상 → 0점
        low_bound = 13.3
        if adjusted_jerk <= low_bound:
            jerk_score = 500
        elif adjusted_jerk <= 30:
            jerk_score = 500 * (30 - adjusted_jerk) / (30 - low_bound)  # 500 * (30 - jerk)/18
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
            "tasc_armed": getattr(self, "tasc_armed", False),
            "tasc_active": getattr(self, "tasc_active", False),

            # HUD/디버그용 (업데이트된 Davis 확인 가능)
            "mu": float(self.scn.mu),
            "rr_factor": float(self.rr_factor),
            "davis_A0": self.veh.A0,
            "davis_B1": self.veh.B1,
            "davis_C2": self.veh.C2,

            # ▼ 타이머 표시용
            "timer_enabled": st.timer_enabled,
            "time_budget_s": st.time_budget_s,
            "time_remaining_s": st.time_remaining_s,     # float 원본(음수 가능)
            "time_remaining_int": st.time_remaining_int, # 정수 표시(내림)
            "time_overrun_s": st.time_overrun_s,
            "time_overrun_int": st.time_overrun_int,
            "time_overrun_started": st.time_overrun_started,

            # 입력 보정 정보(서버 클램프)
            "input_sanitized": getattr(self, "last_input_sanitized", {}),
        }


# ------------------------------------------------------------
# FastAPI app
# ------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")

app = FastAPI()

# /static 경로 제공
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
#yes
@app.get("/")
async def root():
    return HTMLResponse(open(os.path.join(STATIC_DIR, "index.html"), "r", encoding="utf-8").read())

from fastapi.responses import FileResponse

@app.get("/favicon.ico")
async def favicon():
    return FileResponse(os.path.join(STATIC_DIR, "favicon.ico"))


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    cur_length = 10
    cur_load_rate = 0.70
    vehicle_json_path = os.path.join(STATIC_DIR, "e233_1000.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))
    vehicle.notches = len(vehicle.notch_accels)

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.reset() # 초기화

    # ✅ 최초 연결 직후: 기본 편성/탑승률을 즉시 반영해 총중량/데이비스 재계산
    sim.veh.update_mass(cur_length) # 1차: 편성 반영

    base_1c_t = sim.veh.mass_t # 1량(공차) 톤
    pax_1c_t = 10.5 # 1량 만차 탑승 톤 (가정값)
    total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)

    sim.veh.mass_kg = total_tons * 1000.0
    sim.veh.recompute_davis(sim.veh.mass_kg)

    if DEBUG:
        print(f"[INIT] len={cur_length}, load={cur_load_rate*100:.1f}% "
              f"-> base_1c_t={base_1c_t:.3f} t, pax_1c_t={pax_1c_t:.2f} t "
              f"| total={total_tons:.2f} t, mass_kg={sim.veh.mass_kg:.0f} "
              f"| A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}")

    sim.reset() # ✅ 재계산 반영된 상태로 다시 초기화(처음부터 일관)
    sim.running = False


    # 전송 속도: 30Hz
    send_interval = 1.0 / 30.0

    # ---- 분리된 비동기 루프들 ----
    async def recv_loop():
        # vehicle(바깥 스코프 변수)에 재할당 가능하게
        nonlocal vehicle, cur_length, cur_load_rate
        try:
            while True:
                msg = await ws.receive_text()
                try:
                    data = json.loads(msg)
                except Exception:
                    if DEBUG:
                        print("Invalid JSON received.")
                    continue

                if data.get("type") != "cmd":
                    continue

                payload = data.get("payload", {})
                name = payload.get("name")

                if name == "setInitial":
                    speed = payload.get("speed")
                    dist = payload.get("dist")
                    grade = payload.get("grade", 0.0) / 10.0
                    mu = float(payload.get("mu", 1.0))
                    if speed is not None and dist is not None:
                        # ▼ 서버 측 이중 방어(클램프) — 프론트와 동일
                        v_kmh_raw = float(speed)
                        L_raw = float(dist)
                        v_kmh = max(40.0,  min(300.0, v_kmh_raw))
                        L_m   = max(150.0, min(900.0,  L_raw))

                        sim.scn.v0 = v_kmh / 3.6
                        sim.scn.L = L_m
                        sim.scn.grade_percent = float(grade)
                        sim.scn.mu = mu

                        # 클램프 여부 기록
                        sim.last_input_sanitized = {
                            "speed_input": v_kmh_raw, "speed_used": v_kmh,
                            "dist_input": L_raw, "dist_used": L_m,
                            "clamped": (v_kmh != v_kmh_raw) or (L_m != L_raw)
                        }

                        if DEBUG:
                            print(f"setInitial: v0={v_kmh:.1f}km/h ({v_kmh_raw}), "
                                  f"L={L_m:.0f}m ({L_raw}), grade={grade}%, mu={mu}")
                        sim.reset()  # reset()이 timer_enabled 보존 + budget 재계산

                elif name == "start":
                    sim.start()

                elif name in ("stepNotch", "applyNotch"):
                    delta = int(payload.get("delta", 0))
                    sim.queue_command("stepNotch", delta)

                elif name == "release":
                    sim.queue_command("release", 0)

                elif name == "emergencyBrake":
                    sim.queue_command("emergencyBrake", 0)
                elif name == "setNotch":
    # 'val'이나 'delta'에 상관없이 value가 있다면 우선
                    val = payload.get("val", payload.get("delta", payload.get("value", 0)))
                    sim.queue_command("setNotch", val)



                elif name == "setTrainLength":
                    length = int(payload.get("length", 8))
                    cur_length = length # ✅ 상태 저장

                    # 길이 반영
                    sim.veh.update_mass(cur_length)

                    # 탑승률이 이미 있다면 총중량 덮어쓰기 + 재계산
                    base_1c_t = sim.veh.mass_t
                    pax_1c_t = 10.5
                    total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)
                    sim.veh.mass_kg = total_tons * 1000.0
                    sim.veh.recompute_davis(sim.veh.mass_kg)

                    if DEBUG:
                        print(f"[Length] {cur_length} cars | load={cur_load_rate*100:.1f}% "
                            f"-> mass_kg={sim.veh.mass_kg:.0f}, A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}")
                    sim.reset()

                elif name == "setMassTons":
                    mass_tons = float(payload.get("mass_tons", 200.0))
                    sim.veh.mass_t = mass_tons / int(payload.get("length", 8))
                    sim.veh.mass_kg = mass_tons * 1000.0
                    sim.veh.recompute_davis(sim.veh.mass_kg) # ✅ 새 질량으로 재계산
                    if DEBUG:
                        print(
                            f"총중량={mass_tons:.2f} t -> "
                            f"A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}"
                        )
                    sim.reset()

                elif name == "setLoadRate":
                    cur_load_rate = float(payload.get("loadRate", 0.0)) / 100.0 # ✅ 상태 저장

                    # 길이/탑승률로 총중량 재산출
                    base_1c_t = sim.veh.mass_t
                    pax_1c_t = 10.5
                    total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)

                    sim.veh.update_mass(cur_length) # 1차 (길이 반영)
                    sim.veh.mass_kg = total_tons * 1000.0 # 실제 총중량 덮어쓰기
                    sim.veh.recompute_davis(sim.veh.mass_kg) # 최종 재계산

                    if DEBUG:
                        print(f"[LoadRate] length={cur_length}, load={cur_load_rate*100:.1f}% "
                            f"-> mass_kg={sim.veh.mass_kg:.0f}, A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}")
                    sim.reset()



                elif name == "setTASC":
                    enabled = bool(payload.get("enabled", False))
                    sim.tasc_enabled = enabled
                    if enabled:
                        sim.manual_override = False
                        sim._tasc_last_change_t = sim.state.t
                        sim._tasc_phase = "build"
                        sim._tasc_peak_notch = 1
                        sim.tasc_armed = True
                        sim.tasc_active = False
                    if DEBUG:
                        print(f"TASC set to {enabled}")

                elif name == "setMu":
                    value = float(payload.get("value", 1.0))
                    sim.scn.mu = value
                    if DEBUG:
                        print(f"마찰계수(mu)={value}")
                    sim.reset()


                elif name == "setVehicleFile":
                    rel = payload.get("file", "")
                    if rel:
                        try:
                            # 경로 정규화
                            rel_norm = rel.strip()
                            if rel_norm.startswith("/static/"): rel_norm = rel_norm[len("/static/"):]
                            elif rel_norm.startswith("static/"): rel_norm = rel_norm[len("static/"):]
                            path = os.path.join(STATIC_DIR, rel_norm)

                            if not os.path.isfile(path):
                                raise FileNotFoundError(path)

                            newv = Vehicle.from_json(path)
                            newv.notch_accels = list(reversed(newv.notch_accels))
                            newv.notches = len(newv.notch_accels)
                            newv.recompute_davis(newv.mass_kg)

                            sim.veh = newv
                            vehicle = newv

                            # 🔒 차량 교체 직후, 현재 길이/탑승률 재적용 (순서 무관 일관성 보장)
                            sim.veh.update_mass(cur_length)
                            base_1c_t = sim.veh.mass_t
                            pax_1c_t = 10.5
                            total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)
                            sim.veh.mass_kg = total_tons * 1000.0
                            sim.veh.recompute_davis(sim.veh.mass_kg)

                            sim.reset()

                            if DEBUG:
                                print(f"[Vehicle] switched -> {rel} ({path}) | len={cur_length}, load={cur_load_rate*100:.1f}% "
                                    f"| mass_kg={sim.veh.mass_kg:.0f} A0={sim.veh.A0:.1f} B1={sim.veh.B1:.2f} C2={sim.veh.C2:.2f}")
                        except Exception as e:
                            if DEBUG: print(f"[Vehicle] load failed: {rel} -> {e}")


                elif name == "reset":
                    sim.reset()

                # ---------- 타이머/페널티/보너스/보정 설정 ----------
                elif name == "setTimerFormula":
                    # payload: { "enabled": true, "v_target_kmh": 70, "buffer_s": 0 }
                    sim.timer_use_table = False
                    sim.state.timer_enabled = bool(payload.get("enabled", True))
                    sim.timer_v_target_kmh = float(payload.get("v_target_kmh", 70))
                    sim.timer_buffer_s = float(payload.get("buffer_s", 0.0))
                    sim.reset()

                elif name == "setTimerTable":
                    # payload: { "enabled": true, "table": { "60":35, "70":30, "80":26 } }
                    tbl = payload.get("table", {})
                    sim.timer_use_table = True
                    sim.state.timer_enabled = bool(payload.get("enabled", True))
                    sim.timer_table = {int(k): float(v) for k, v in tbl.items()}
                    sim.reset()

                elif name == "toggleTimer":
                    # payload: { "enabled": false }
                    sim.state.timer_enabled = bool(payload.get("enabled", False))
                    sim.reset()

                elif name == "setTimerPenalty":
                    # payload: { "per_s": 20, "cap": 400 }
                    sim.timer_overtime_penalty_per_s = float(payload.get("per_s", 20.0))
                    sim.timer_overtime_penalty_cap = float(payload.get("cap", 400.0))

                elif name == "setTimerExactBonus":
                    # payload: {"bonus": 100}
                    sim.timer_exact_bonus = float(payload.get("bonus", 100))

                elif name == "setTimerCalib":
                    # payload 예시:
                    # {
                    #   "points":[
                    #     {"v":60, "L":200, "t":23},
                    #     {"v":70, "L":200, "t":28},
                    #     {"v":90, "L":400, "t":30}
                    #   ],
                    #   "norm_v": 100, "norm_L": 300,
                    #   "idw_power": 2.0, "blend_threshold": 1.5
                    # }
                    pts = payload.get("points", [])
                    sim.set_timer_calibration(
                        points=pts,
                        norm_v=payload.get("norm_v"),
                        norm_L=payload.get("norm_L"),
                        idw_power=payload.get("idw_power"),
                        blend_threshold=payload.get("blend_threshold"),
                    )
                    # 자동 산출이 적용되도록 리셋
                    sim.state.timer_enabled = True
                    sim.reset()
                
                else:
                     cmd_val = payload.get("val", payload.get("delta", 0))
                     sim.queue_command(name, cmd_val)    
                

        except WebSocketDisconnect:
            if DEBUG:
                print("WebSocket disconnected (recv_loop).")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            if DEBUG:
                print(f"Error during receive: {e}")

    async def sim_loop():
        dt = sim.scn.dt
        step_count = 0
        t_start = None  # 시작 시점은 start() 눌렀을 때 설정
        was_running = False

        while True:
            if sim.running:
                if not was_running:
                    t_start = time.time()
                    step_count = 0

                t_now = time.time()
                expected_steps = int((t_now - t_start) / dt)

            # 누적된 스텝만큼만 진행
                for _ in range(step_count, expected_steps):
                    sim.step()

                step_count = expected_steps
                was_running = True

            else:
            # ★ 정지 상태에서는 기준값들을 항상 초기화
                was_running = False
                t_start = None
                step_count = 0

            await asyncio.sleep(dt / 2)


    async def send_loop():
        try:
            while True:
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                await asyncio.sleep(send_interval)
        except WebSocketDisconnect:
            if DEBUG:
                print("WebSocket disconnected (send_loop).")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            if DEBUG:
                print(f"Error during send: {e}")

    tasks = [
        asyncio.create_task(recv_loop()),
        asyncio.create_task(sim_loop()),
        asyncio.create_task(send_loop()),
    ]

    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for t in tasks:
            t.cancel()
        try:
            await ws.close()
        except RuntimeError:
            pass