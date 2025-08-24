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
DEBUG = False  # 디버그 로그를 보고 싶으면 True

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

    # Davis 계수 (열차 전체) : F = A0 + B1 * v + C2 * v^2 [N], v[m/s]
    A0: float = 1200.0
    B1: float = 30.0
    C2: float = 8.0

    # (공기계수 등은 참고로 유지)
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
        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, finished=False)
        self.running = False
        self.vref = build_vref(scn.L, 0.8 * veh.a_max)
        self._cmd_queue = deque()

        # 초기 제동(B1/B2) 판정
        self.first_brake_start = None
        self.first_brake_done = False

        # 기록
        self.notch_history: List[int] = []
        self.time_history: List[float] = []

        # EB 사용 여부
        self.eb_used = False

        # 저크 계산
        self.prev_a = 0.0
        self.jerk_history: List[float] = []

        # ---------- TASC ----------
        self.tasc_enabled = False
        self.manual_override = False
        self.tasc_deadband_m = 0.1
        self.tasc_hold_min_s = 0.05
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"  # "build" → "relax"
        self._tasc_peak_notch = 1
        # 대기/활성 상태
        self.tasc_armed = False
        self.tasc_active = False

        # 코스팅 보정(μ와 분리)
        self.coast_factor = 1.0  # 0.95~1.05 사이로 UI에서만 미세 조정 권장

        # ---- 성능 최적화: TASC 예측 캐시/스로틀 ----
        self._tasc_pred_cache = {
            "t": -1.0, "v": -1.0, "notch": -1, "keys": None,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        }
        self._tasc_pred_interval = 0.15  # ★ 50ms -> 150ms (부하↓)
        self._tasc_last_pred_t = -1.0
        self._tasc_speed_eps = 0.3  # m/s
        self._tasc_last_wall = 0.0  # ★ 실시간 스로틀(벽시계)

        # ---- B5 필요 여부 캐시/스로틀 ----
        self._need_b5_last_t = -1.0
        self._need_b5_last = False
        self._need_b5_interval = 0.05

        # 브레이크 동역학 상태 변수/시정수 (채널 분리)
        self.brk_accel = 0.0
        self.brk_elec = 0.0
        self.brk_air  = 0.0
        self.tau_apply = 0.25
        self.tau_release = 0.8
        self.tau_apply_eb = 0.15
        self.tau_release_lowv = 0.8

        # WSP(활주) 상태
        self.wsp_state = "normal"
        self.wsp_timer = 0.0

        # 저크 체인용 필터된 명령 가속도
        self._a_cmd_filt = 0.0

    # ----------------- Physics helpers -----------------

    def _grade_accel(self) -> float:
        """
        경사 가속도(정식).
        원 코드 컨벤션 유지: 내리막(+)이면 '감속 방향(-)'로 가게끔 음수 반환.
        """
        return -9.81 * (self.scn.grade_percent / 100.0)

    def _davis_accel(self, v: float) -> float:
        """Davis 저항을 가속도로 환산. μ와 분리되어 coast_factor로만 조정."""
        A0 = self.veh.A0 * self.coast_factor
        B1 = self.veh.B1 * self.coast_factor
        C2 = self.veh.C2
        F = A0 + B1 * v + C2 * v * v  # N
        return -F / self.veh.mass_kg if v > 0 else 0.0

    def _effective_brake_accel(self, notch: int, v: float) -> float:
        """
        노치별 장비 목표감속과 접착 한계(μg)를 클램프.
        (m/s^2, 음수 반환)
        """
        if notch >= len(self.veh.notch_accels):
            return 0.0
        base = float(self.veh.notch_accels[notch])  # 음수(0은 N)

        # 접착 한계 (서비스/EB 분기)
        k_srv = 0.85
        k_eb = 0.98
        is_eb = (notch == (self.veh.notches - 1))
        k_adh = k_eb if is_eb else k_srv
        a_cap = -k_adh * float(self.scn.mu) * 9.81  # 음수

        a_eff = max(base, a_cap)  # 절대값 작은 쪽 선택(안전)
        if a_eff <= a_cap + 1e-6:
            # 접착을 딱 치면 약간 덜 잡히게(보수적 스케일)
            scale = 0.90 if v > 8.0 else 0.85
            a_eff = a_cap * scale
        return a_eff

    # --- 속도별 τ (apply/release) ---

    def _taus_for_speed(self, v: float, is_eb: bool):
        v_kmh = v * 3.6
        if is_eb:
            return 0.15, 0.45  # apply, release (급충/급완)
        if v_kmh >= 15.0:        # 회생 우세
            return 0.18, 0.40
        elif v_kmh >= 10.0:      # 블렌딩 구간
            return 0.30, 0.60
        else:                    # 저속 공기 우세
            return 0.55, 0.80

    # --- 회생↔공기 블렌딩 가중치 ---

    def _blend_w_regen(self, v: float) -> float:
        v_kmh = v * 3.6
        if v_kmh >= 20.0: return 1.0    # 전기 100%
        if v_kmh <= 8.0:  return 0.0    # 공기 100%
        return (v_kmh - 8.0) / 12.0     # 8~20 선형

    # --- 밸브 동역학(채널 분리) ---

    def _update_brake_dyn_split(self, a_total_cmd: float, v: float, is_eb: bool, dt: float):
        w = self._blend_w_regen(v)
        a_cmd_e = a_total_cmd * w
        a_cmd_a = a_total_cmd * (1.0 - w)

        # 속도별 τ (전기/공기)
        tau_e_apply, tau_e_rel = (0.18, 0.40) if v * 3.6 >= 15 else (0.30, 0.50)
        tau_a_apply, tau_a_rel = (0.45, 0.75) if v * 3.6 < 10 else (0.30, 0.60)
        if is_eb:
            tau_a_apply, tau_a_rel = 0.15, 0.45

        # 강/약 판정
        e_stronger = (a_cmd_e < self.brk_elec)
        a_stronger = (a_cmd_a < self.brk_air)

        tau_e = tau_e_apply if e_stronger else tau_e_rel
        tau_a = tau_a_apply if a_stronger else tau_a_rel

        self.brk_elec += (a_cmd_e - self.brk_elec) * (dt / max(1e-6, tau_e))
        self.brk_air  += (a_cmd_a - self.brk_air ) * (dt / max(1e-6, tau_a))
        self.brk_accel = self.brk_elec + self.brk_air

    # --- 간이 WSP 사이클 ---

    def _wsp_update(self, v: float, a_demand: float, dt: float):
        a_cap = -0.85 * self.scn.mu * 9.81  # 서비스 접착한계(보수적)
        margin = 0.05  # m/s²

        if self.wsp_state == "normal":
            if a_demand < (a_cap - margin) and v * 3.6 > 3.0:
                self.wsp_state = "release"
                self.wsp_timer = 0.12           # 120ms 놓기
                return min(a_demand, 0.5 * a_cap)
            return a_demand

        elif self.wsp_state == "release":
            self.wsp_timer -= dt
            if self.wsp_timer <= 0.0:
                self.wsp_state = "reapply"
                self.wsp_timer = 0.15           # 150ms 재가압
            return min(a_demand, 0.3 * a_cap)

        else:  # "reapply"
            self.wsp_timer -= dt
            if self.wsp_timer <= 0.0:
                self.wsp_state = "normal"
            return min(a_demand, 0.8 * a_cap)

    # ----------------- Controls -----------------

    def _clamp_notch(self, n: int) -> int:
        # EB 인덱스 = 마지막 유효 인덱스 (배열 길이 신뢰)
        max_index = len(self.veh.notch_accels) - 1
        return max(0, min(max_index, n))

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
            if DEBUG:
                print(f"Applied stepNotch: {old_notch} -> {st.lever_notch}")
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
        self.tasc_active = False
        self.tasc_armed = bool(self.tasc_enabled)

        # 예측 캐시 초기화
        self._tasc_pred_cache.update({
            "t": -1.0, "v": -1.0, "notch": -1, "keys": None,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        })
        self._tasc_last_pred_t = -1.0

        # B5 필요 여부 캐시 초기화
        self._need_b5_last_t = -1.0
        self._need_b5_last = False

        # 브레이크 동역학 상태 초기화
        self.brk_accel = 0.0
        self.brk_elec = 0.0
        self.brk_air  = 0.0

        # WSP 초기화
        self.wsp_state = "normal"
        self.wsp_timer = 0.0

        # 저크 명령 필터 초기화
        self._a_cmd_filt = 0.0

        if DEBUG:
            print("Simulation reset")

    def start(self):
        self.reset()
        self.running = True
        if DEBUG:
            print("Simulation started")

    def eb_used_from_history(self) -> bool:
        """기록에서 EB(최대 인덱스) 사용 여부 확인"""
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # ------ stopping distance helpers (TASC) ------

    def _estimate_stop_distance(self, notch: int, v0: float) -> float:
        """
        새 파이프라인: 두 채널 블렌딩 + 속도별 τ + WSP + 저크 + μ/소프트스톱.
        실제 step()과 동일한 로직으로 예측해 TASC 오차를 최소화.
        """
        if notch <= 0:
            return float('inf')

        # ----- seed from live state -----
        dt = 0.05  # ★ 0.03 -> 0.05 (부하↓)
        v  = max(0.0, v0)
        a  = float(self.state.a)

        # 채널/밸브/저크/ WSP 시드 (실제 상태를 복사만)
        brk_elec = float(getattr(self, "brk_elec", 0.0))
        brk_air  = float(getattr(self, "brk_air",  0.0))
        wsp_state = getattr(self, "wsp_state", "normal")
        wsp_timer = float(getattr(self, "wsp_timer", 0.0))

        L_target = self.scn.L
        s = 0.0
        rem_now = L_target - self.state.s
        limit = float(rem_now + 8.0)

        ctrl_delay = max(self._tasc_pred_interval, self.tasc_hold_min_s)
        latency_margin = v * ctrl_delay

        # ★ 동적 스텝 상한 (현재 v와 노치 기반 간이 추정)
        base_mag = abs(self.veh.notch_accels[min(notch, len(self.veh.notch_accels)-1)])
        amin = max(0.5, min(1.3, base_mag))          # 보수적 서비스 감속 상한
        t_est = v / max(0.3, amin) + 1.0             # 여유 1초
        steps = int(min(1800, max(60, t_est / dt * 1.4)))

        # 메인 루프
        for _ in range(steps):
            # (1) 명령 제동가속도 (서비스/EB 캡 포함)
            is_eb = (notch == self.veh.notches - 1)
            a_cmd_total = self._effective_brake_accel(notch, v)

            # (2) 전기/공기 블렌딩 목표 분해
            w = self._blend_w_regen(v)
            a_cmd_e = a_cmd_total * w
            a_cmd_a = a_cmd_total * (1.0 - w)

            # (3) 속도별 τ로 채널 동역학 업데이트
            e_stronger = (a_cmd_e < brk_elec)
            a_stronger = (a_cmd_a < brk_air)

            # 전기 τ
            if v * 3.6 >= 15.0:
                tau_e_apply, tau_e_rel = 0.18, 0.40
            else:
                tau_e_apply, tau_e_rel = 0.30, 0.50

            # 공기 τ
            if is_eb:
                tau_a_apply, tau_a_rel = 0.15, 0.45
            else:
                if v * 3.6 < 10.0:
                    tau_a_apply, tau_a_rel = 0.45, 0.75
                else:
                    tau_a_apply, tau_a_rel = 0.30, 0.60

            tau_e = tau_e_apply if e_stronger else tau_e_rel
            tau_a = tau_a_apply if a_stronger else tau_a_rel

            brk_elec += (a_cmd_e - brk_elec) * (dt / max(1e-6, tau_e))
            brk_air  += (a_cmd_a - brk_air ) * (dt / max(1e-6, tau_a))
            a_brake = brk_elec + brk_air

            # (4) WSP 예측 적용 (상태는 예측 내부에서만 변경)
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
            else:  # reapply
                wsp_timer -= dt
                if wsp_timer <= 0.0:
                    wsp_state = "normal"
                    wsp_timer = 0.0
                a_brake = min(a_brake, 0.8 * a_cap)

            # (5) 외력
            a_grade = self._grade_accel()      # 메인과 동일 부호/정의 유지
            a_davis = self._davis_accel(v)     # μ와 분리된 coast_factor 반영

            # (6) 목표 a
            a_target = a_brake + a_grade + a_davis

            # (7) 소프트스톱 (마지막 1 m)
            rem_pred = max(0.0, rem_now - s)
            if rem_pred <= 1.0:
                safe_rem = max(0.05, rem_pred)
                a_need = -(v * v) / (2.0 * safe_rem)
                a_soft = max(-0.40, min(-0.12, a_need))
                w_soft = 1.0 if rem_pred <= 0.0 else max(0.0, min(1.0, 1.0 - rem_pred))
                a_target = (1.0 - w_soft) * a_target + w_soft * a_soft

            if notch >= 1 or rem_pred <= 0.0:
                a_target = min(a_target, 0.0)

            # (8) 저크 제한 (메인과 동일 규칙)
            max_da = self.veh.j_max * dt
            v_kmh = v * 3.6
            if v_kmh <= 5.0:
                max_da *= 0.25 + 0.75 * (v_kmh / 5.0)

            da = a_target - a
            if da > max_da:   da = max_da
            elif da < -max_da: da = -max_da
            a += da

            # (9) 적분
            v = max(0.0, v + a * dt)
            s += v * dt + 0.5 * a * dt * dt

            # (10) 종료 조건
            if v <= 0.01:
                break
            if s > limit:
                break

        return s + latency_margin

    def _stopping_distance(self, notch: int, v: float) -> float:
        """보수적 예측: 위의 수치예측 사용"""
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v)

    def _tasc_predict(self, cur_notch: int, v: float):
        """TASC 정지거리 예측 스로틀링 + 캐시(부하 완화)"""
        st = self.state
        need = False
        if (st.t - self._tasc_last_pred_t) >= self._tasc_pred_interval:
            need = True
        if abs(v - self._tasc_pred_cache["v"]) >= self._tasc_speed_eps:
            need = True
        if cur_notch != self._tasc_pred_cache["notch"]:
            need = True

        # ★ 캐시 키 양자화(버킷)으로 히트율↑
        def _q(x, q=0.05):
            return round(float(x) / q) * q

        keys = (
            round(self.scn.mu, 2),
            round(self.scn.grade_percent, 2),
            _q(getattr(self, "brk_elec", 0.0), 0.05),
            _q(getattr(self, "brk_air", 0.0), 0.05),
            getattr(self, "wsp_state", "normal"),
        )
        if keys != self._tasc_pred_cache.get("keys"):
            need = True

        # ★ 실시간(벽시계) 스로틀: 120ms 이내엔 캐시 재사용
        if need:
            now = time.perf_counter()
            if (now - self._tasc_last_wall) < 0.12 and self._tasc_pred_cache["t"] >= 0.0:
                c = self._tasc_pred_cache
                return (c["s_cur"], c["s_up"], c["s_dn"])
            self._tasc_last_wall = now

        if not need:
            c = self._tasc_pred_cache
            return (c["s_cur"], c["s_up"], c["s_dn"])

        max_normal_notch = self.veh.notches - 2  # EB 제외(마지막)
        s_cur = self._stopping_distance(cur_notch, v) if cur_notch > 0 else float("inf")
        s_up  = self._stopping_distance(min(cur_notch + 1, max_normal_notch), v) if cur_notch > 0 else \
                self._stopping_distance(1, v)
        s_dn  = self._stopping_distance(cur_notch - 1, v) if cur_notch - 1 >= 1 else float("inf")

        # 모노토닉 보정
        if s_up > s_cur: s_up = s_cur
        if s_cur > s_dn and s_dn != float("inf"): s_cur = s_dn

        self._tasc_pred_cache.update({
            "t": st.t, "v": v, "notch": cur_notch,
            "s_cur": s_cur, "s_up": s_up, "s_dn": s_dn,
            "keys": keys
        })
        self._tasc_last_pred_t = st.t
        return s_cur, s_up, s_dn

    # v0(시작속도) 기반으로 기준 노치 선택
    def _need_B5_now(self, v: float, remaining: float) -> bool:
        """
        'B?가 필요하냐?' 판정.
        v0(시작속도) 기준으로 참조 노치 선택:
        v0<75 → B2, <85 → B3, <95 → B3, 그 외 → B4  (보수적)
        """
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

        # 예약된 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # 기록 & 초제동(B1/B2) 판정 체크 (시뮬 시간 기반)
        self.notch_history.append(st.lever_notch)
        self.time_history.append(st.t)
        if not self.first_brake_done:
            if st.lever_notch in (1, 2):  # B1 또는 B2
                if self.first_brake_start is None:
                    self.first_brake_start = st.t
                elif (st.t - self.first_brake_start) >= 1.0:  # 초제동 1초
                    self.first_brake_done = True
            else:
                self.first_brake_start = None

        # ---------- TASC 자동 제동 ----------
        if self.tasc_enabled and not self.manual_override and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            speed_kmh = st.v * 3.6
            cur = st.lever_notch
            max_normal_notch = self.veh.notches - 2  # EB-1까지

            # (A) B? 필요 시점 감지 → 그때 TASC 활성화(초제동 시퀀스 시작)
            if self.tasc_armed and not self.tasc_active:
                if self._need_B5_now(st.v, rem_now):
                    self.tasc_active = True
                    self.tasc_armed = False
                    self._tasc_last_change_t = st.t

            if self.tasc_active:
                # (B) 초제동 유지: 활성화 이후에만 B1/B2를 1초간 강제
                if not self.first_brake_done:
                    desired = 2 if speed_kmh >= 70.0 else 1
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
                        if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur - 1)
                                self._tasc_last_change_t = st.t

        # ====== 동역학 ======
        # 제동 명령 → 밸브 동역학(채널 분리) → 실제 제동가속도
        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v)
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self._update_brake_dyn_split(a_cmd_brake, st.v, is_eb, dt)  # 회생/공기 분리 응답
        a_brake = self._wsp_update(st.v, self.brk_accel, dt)        # 간이 WSP 적용

        # 경사/저항
        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)

        # 목표 가속도
        a_target = a_brake + a_grade + a_davis

        # 잔여거리
        rem_now = self.scn.L - st.s

        # ★ 소프트스톱 (마지막 1 m): 필요 감속과 승차감 한계 혼합
        if rem_now <= 1.0:
            safe_rem = max(0.05, rem_now)
            a_need = -(st.v * st.v) / (2.0 * safe_rem)
            a_soft = max(-0.40, min(-0.12, a_need))
            w_soft = 1.0 if rem_now <= 0.0 else max(0.0, min(1.0, 1.0 - rem_now))
            a_target = (1.0 - w_soft) * a_target + w_soft * a_soft

        # 정지 직전/오버런 구간 (+가속 금지)
        if st.lever_notch >= 1 or rem_now <= 0.0:
            a_target = min(a_target, 0.0)

        # --- 저크 체인 (단일화): τ 필터 대신 명령 필터+저크로만 가속 업데이트 ---
        # (1) 명령 가속도 1차 필터
        self._a_cmd_filt += (a_target - self._a_cmd_filt) * (dt / max(1e-6, self.veh.tau_brk))
        # (2) 저크 제한
        max_da = self.veh.j_max * dt
        v_kmh = st.v * 3.6
        if v_kmh <= 5.0:
            max_da *= 0.25 + 0.75 * (v_kmh / 5.0)
        da = self._a_cmd_filt - st.a
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

            # 저크 기록 (※ 지금은 종료 시점만 반영)
            jerk = abs((st.a - self.prev_a) / dt) if dt > 0 else 0.0
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            # 저크 점수 반영
            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False
            if DEBUG:
                print(f"Avg jerk: {avg_jerk:.4f}, jerk_score: {jerk_score:.2f}, final score: {score}")
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
            "coast_factor": float(self.coast_factor),
            "davis_A0": self.veh.A0,
            "davis_B1": self.veh.B1,
            "davis_C2": self.veh.C2,
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
    # EB 유령 방지: 실제 배열 길이로 동기화
    vehicle.notches = len(vehicle.notch_accels)

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.start()

    # 전송 속도: 30Hz
    send_interval = 1.0 / 30.0

    # ---- 분리된 비동기 루프들 ----
    async def recv_loop():
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
                    grade = payload.get("grade", 0.0) / 10.0  # 기존 UI 호환(기존 코드 유지)
                    mu = float(payload.get("mu", 1.0))
                    if speed is not None and dist is not None:
                        sim.scn.v0 = float(speed) / 3.6
                        sim.scn.L = float(dist)
                        sim.scn.grade_percent = float(grade)
                        sim.scn.mu = mu
                        if DEBUG:
                            print(
                                f"setInitial: v0={speed}km/h, L={dist}m, grade={grade}%, mu={mu}, coast_factor={sim.coast_factor:.3f}"
                            )
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
                    if DEBUG:
                        print(f"Train length set to {length} cars.")
                    sim.reset()

                elif name == "setMassTons":
                    mass_tons = float(payload.get("mass_tons", 200.0))
                    # 차량 1량 질량 갱신(정보 목적), 총질량은 mass_kg에 반영
                    vehicle.mass_t = mass_tons / int(payload.get("length", 8))
                    vehicle.mass_kg = mass_tons * 1000.0
                    if DEBUG:
                        print(f"차량 전체 중량을 {mass_tons:.2f} 톤으로 업데이트 했습니다.")
                    sim.reset()

                elif name == "setLoadRate":
                    load_rate = float(payload.get("loadRate", 0.0)) / 100.0  # 예: 85 → 0.85
                    length = int(payload.get("length", 8))

                    # 1량 기본 질량(톤): vehicle.json의 mass_t가 '1량 기준'이라고 가정
                    base_1c_t = vehicle.mass_t
                    pax_1c_t = 10.5  # 만석 시 1량당 승객 질량(톤). 필요하면 vehicle.json로 빼도 됨.

                    total_tons = length * (base_1c_t + pax_1c_t * load_rate)

                    # 총질량 반영
                    vehicle.update_mass(length)             # mass_kg = base_1c_t * 1000 * length
                    vehicle.mass_kg = total_tons * 1000.0   # 여기에 탑승률 반영값으로 덮어씀

                    if DEBUG:
                        print(f"[LoadRate] length={length}, load={load_rate*100:.1f}%, total={total_tons:.1f} t")

                    sim.reset()

                elif name == "setTASC":
                    enabled = bool(payload.get("enabled", False))
                    sim.tasc_enabled = enabled
                    if enabled:
                        sim.manual_override = False
                        sim._tasc_last_change_t = sim.state.t
                        sim._tasc_phase = "build"
                        sim._tasc_peak_notch = 1
                        # 즉시 작동 금지: B? 필요 시점까지 '대기'
                        sim.tasc_armed = True
                        sim.tasc_active = False
                    if DEBUG:
                        print(f"TASC set to {enabled}")

                elif name == "setMu":
                    value = float(payload.get("value", 1.0))
                    sim.scn.mu = value
                    # μ는 접착/WSP 전용. 코스팅은 coast_factor로만 조정(여기서는 손대지 않음)
                    if DEBUG:
                        print(f"마찰계수(mu)={value} 로 설정 (coast_factor={sim.coast_factor:.3f})")
                    sim.reset()

                elif name == "setCoastFactor":
                    cf = float(payload.get("value", 1.0))
                    sim.coast_factor = max(0.8, min(1.2, cf))
                    if DEBUG:
                        print(f"coast_factor = {sim.coast_factor:.3f}")
                    sim.reset()

                elif name == "setVehicleFile":
                    rel = payload.get("file", "")
                    if rel:
                        try:
                            # 상대경로 처리 (./e233_0000.json 같은 형식)
                            STATIC_DIR = os.path.join(BASE_DIR, "static")
                            # 예: rel="./e233_0000.json" -> static/e233_0000.json
                            path = os.path.join(STATIC_DIR, rel.lstrip("./"))

                            # 파일이 없으면 바로 예외
                            if not os.path.isfile(path):
                                raise FileNotFoundError(path)
                            newv = Vehicle.from_json(path)

                            # 역순 적용 (프론트와 일치)
                            newv.notch_accels = list(reversed(newv.notch_accels))

                            # notches 동기화 (유령 B9 방지)
                            newv.notches = len(newv.notch_accels)

                            # 현재 vehicle 객체 갱신
                            vehicle.__dict__.update(newv.__dict__)

                            # 시뮬레이터에 반영
                            sim.veh = vehicle
                            sim.reset()

                            if DEBUG:
                                print(f"[Vehicle] switched to {rel} / notches={vehicle.notches}")
                        except Exception as e:
                            if DEBUG:
                                print(f"[Vehicle] load failed: {rel} -> {e}")

                elif name == "reset":
                    sim.reset()

        except WebSocketDisconnect:
            if DEBUG:
                print("WebSocket disconnected (recv_loop).")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            if DEBUG:
                print(f"Error during receive: {e}")

    async def sim_loop():
        try:
            dt = sim.scn.dt  # 시뮬 dt 그대로 사용
            while True:
                if sim.running:
                    sim.step()
                await asyncio.sleep(dt)
        except asyncio.CancelledError:
            pass

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

    # 세 루프를 동시에 실행하다가 하나라도 종료되면 전체 종료
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