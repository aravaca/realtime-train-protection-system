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
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0,
                           lever_notch=0, finished=False)
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

    # --------------------------------------------------------
    # Core brake model (공용): 실시간/예측 모두 여기 사용
    # predict_mode=True면 전역 상태를 절대 바꾸지 않음.
    # enable_b1_pi=False면 B1 P+I 비활성(명목 감속만 사용)
    # --------------------------------------------------------
    def _effective_brake_accel_core(
        self,
        notch: int,
        v: float,
        rem_now: float,
        dt_used: float,
        air_boost_state: float,
        b1_i: float,
        predict_mode: bool,
        enable_b1_pi: bool,
    ):
        # 유효 범위
        if notch >= len(self.veh.notch_accels):
            return 0.0, air_boost_state, b1_i

        base = float(self.veh.notch_accels[notch])  # 음수

        # 전기/공기 블렌딩
        blend_cutoff_speed = 40.0 / 3.6
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))
        speed_kmh = v * 3.6
        air_boost = 0.72 if speed_kmh <= 3.0 else 1.0

        # -------- B1 P+I (옵션) --------
        if enable_b1_pi and notch == 1:
            # 명목 B1 기준으로 에러 구성: 현재 잔여거리(rem_now) - 단순 모델 정지거리
            # 단순 모델: 현재 속도에서 B1(base)만 적용(블렌딩 반영)한다고 보고 v^2/2a 추정
            # a_nom = |base|의 전기/공기 블렌딩을 반영한 크기(양수화)
            a_nom_mag = abs(base * (regen_frac + (1 - regen_frac) * air_boost))
            a_nom_mag = max(0.05, a_nom_mag)  # 분모 보호

            s_nom = (v * v) / (2.0 * a_nom_mag)

            error_m = rem_now - s_nom

            # Gains
            k_base, k_near = 0.25, 0.08
            scale = min(1.0, max(0.0, abs(error_m) / 0.8))
            k = k_near + (k_base - k_near) * scale

            ki, leak = 0.4, 0.985
            b1_i = (b1_i * leak) + (ki * error_m * dt_used)
            b1_i = max(-0.25, min(0.60, b1_i))

            adjust = 1.0 - k * error_m
            target_boost = max(0.25, min(1.35, adjust))
            target_boost *= (1.0 + b1_i)

            if rem_now < 1.5 and v < 1.2:
                target_boost = max(0.28, target_boost - 0.04)

            alpha = min(0.65, dt_used / 0.022)
            air_boost_state += alpha * (target_boost - air_boost_state)

            air_boost *= air_boost_state
        else:
            # PI 비활성 시에는 현재 상태만 반영(업데이트 없음)
            air_boost *= air_boost_state

        # 최종 블렌딩
        blended_accel = base * (regen_frac + (1 - regen_frac) * air_boost)

        # 접착 한계 + 간이 WSP
        k_srv, k_eb = 0.85, 0.98
        is_eb = (notch == self.veh.notches - 1)
        k_adh = k_eb if is_eb else k_srv
        a_cap = -k_adh * float(self.scn.mu) * 9.81
        a_eff = max(blended_accel, a_cap)
        if a_eff <= a_cap + 1e-6:
            a_eff = a_cap * (0.90 if v > 8.0 else 0.85)

        return a_eff, air_boost_state, b1_i

    # ----------------- 실시간용 wrapper -----------------
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        rem_now = self.scn.L - self.state.s
        a_eff, new_boost, new_b1i = self._effective_brake_accel_core(
            notch=notch,
            v=v,
            rem_now=rem_now,
            dt_used=self.scn.dt,
            air_boost_state=self._b1_air_boost_state,
            b1_i=self._b1_i,
            predict_mode=False,
            enable_b1_pi=True,   # 실시간: B1 PI 활성
        )
        # 예측 중이 아니고, 진행중이면 전역 상태 갱신
        if (not self._in_predict) and (self.state is not None) and (not self.state.finished):
            self._b1_air_boost_state = new_boost
            self._b1_i = new_b1i
        return a_eff

    def _grade_accel(self) -> float:
        return -9.81 * (self.scn.grade_percent / 100.0)

    def _davis_accel(self, v: float) -> float:
        A0 = self.veh.A0 * self.rr_factor
        B1 = self.veh.B1 * self.rr_factor
        C2 = self.veh.C2
        F = A0 + B1 * v + C2 * v * v
        return -F / self.veh.mass_kg if v != 0 else 0.0

    def _update_brake_dyn_local(self, a_cmd: float, v: float, is_eb: bool, dt: float, brk_accel_cur: float):
        """예측/실시간 공용: 비대칭 τ"""
        going_stronger = (a_cmd < brk_accel_cur)
        if going_stronger:
            tau = self.tau_apply_eb if is_eb else self.tau_apply
        else:
            tau = self.tau_release_lowv if v < 3.0 else self.tau_release
        alpha = dt / max(1e-6, tau)
        return brk_accel_cur + (a_cmd - brk_accel_cur) * alpha

    # ------ stopping distance helpers (예측: 실시간과 동일 물리, 단 B1 PI 비활성) ------
    def _estimate_stop_distance(self, notch: int, v0: float, include_margin: bool = True) -> float:
        """
        고정 노치(notch)로 간다는 가정의 정지거리 예측.
        - 실시간 step과 동일한 순서/모델(비대칭 τ + 저크 한계 + 외력).
        - 전역 상태는 절대 변경하지 않음.
        - 성능을 위해 dt_pred = max(0.01, 2*dt).
        - B1 PI는 비활성(명목 감속)로 계산 → 재귀/발산 방지, 가볍고 안정.
        """
        # 로컬 복제
        v = max(0.0, v0)
        a = self.state.a  # 현재 가속도에서 시작 → 일치성 향상
        s = 0.0
        brk_accel_cur = self.brk_accel
        air_boost_state = self._b1_air_boost_state
        b1_i = self._b1_i

        dt_pred = max(0.01, 2.0 * self.scn.dt)
        rem0 = self.scn.L - self.state.s
        limit = float(rem0 + 5.0)

        self._in_predict = True
        try:
            for _ in range(10000):  # 안전 상한
                rem_now = max(0.0, rem0 - s)

                a_cmd, _, _ = self._effective_brake_accel_core(
                    notch=notch,
                    v=v,
                    rem_now=rem_now,
                    dt_used=dt_pred,
                    air_boost_state=air_boost_state,
                    b1_i=b1_i,
                    predict_mode=True,
                    enable_b1_pi=False,  # 예측: B1 PI 비활성(명목 B1)
                )
                is_eb = (notch == self.veh.notches - 1)
                brk_accel_cur = self._update_brake_dyn_local(a_cmd, v, is_eb, dt_pred, brk_accel_cur)

                a_grade = self._grade_accel()
                a_davis = self._davis_accel(v)
                a_target = brk_accel_cur + a_grade + a_davis

                # 저크 제한 (실시간과 동일)
                max_da = self.veh.j_max * dt_pred
                da = a_target - a
                if da > max_da:
                    da = max_da
                elif da < -max_da:
                    da = -max_da
                a += da

                # 적분
                v = max(0.0, v + a * dt_pred)
                s += v * dt_pred + 0.5 * a * dt_pred * dt_pred

                if v <= 0.01 or s > limit:
                    break
        finally:
            self._in_predict = False

        return s

    def _stopping_distance(self, notch: int, v: float, include_margin: bool = True) -> float:
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v, include_margin=include_margin)

    # ----------------- Main step -----------------
    def step(self):
        st = self.state
        dt = self.scn.dt

        # 예약된 명령 처리
        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())

        # notch 기록 & 초제동 판정
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

        # ====== TASC ======
        if self.tasc_enabled and not self.manual_override and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            speed_kmh = st.v * 3.6
            cur = st.lever_notch
            max_normal_notch = self.veh.notches - 2  # EB-1까지

            # (A) B5 필요 감지 → 활성화
            if self.tasc_armed and not self.tasc_active:
                if self._need_B5_now(st.v, rem_now):
                    self.tasc_active = True
                    self.tasc_armed = False
                    self._tasc_last_change_t = st.t

            if self.tasc_active:
                # (B) 초제동 유지(2초)
                if not self.first_brake_done:
                    desired = 2 if speed_kmh >= 70.0 else 1
                    if dwell_ok and cur != desired:
                        stepv = 1 if desired > cur else -1
                        st.lever_notch = self._clamp_notch(cur + stepv)
                        self._tasc_last_change_t = st.t
                else:
                    # (C) 빌드/릴랙스: 예측값 사용(명목 모델)
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
        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v)
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self.brk_accel = self._update_brake_dyn_local(a_cmd_brake, st.v, is_eb, dt, self.brk_accel)

        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)
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

        # B1 I항 소거
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

            jerk = abs((st.a - self.prev_a) / dt)
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            st.score = score
            self.running = False
            if DEBUG:
                print(f"Simulation finished: stop_error={st.stop_error_m:.3f} m, score={score}")

    # ---------- TASC helpers ----------
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
        s_b4 = self._stopping_distance(2, v)  # B4 정지거리(명목)
        need = s_b4 > (remaining + self.tasc_deadband_m)
        self._need_b5_last = need
        self._need_b5_last_t = st.t
        return need

    # ---------- 기타 ----------
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
            "rr_factor": float(self.rr_factor),
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
    # 프론트는 EB→...→N으로 보내고, 서버는 N→...→EB로 쓰기 위해 반전
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.start()

    last_sim_time = time.perf_counter()
    last_send = 0.0
    send_interval = 1.0 / 30.0  # 30Hz 송신 제한

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - last_sim_time

            # 클라이언트 입력 처리
            try:
                msg = await asyncio.wait_for(ws.receive_text(), timeout=0.01)
                data = json.loads(msg)
                if data.get("type") == "cmd":
                    payload = data["payload"]
                    name = payload.get("name")

                    if name == "setInitial":
                        speed = payload.get("speed")
                        dist = payload.get("dist")
                        grade = payload.get("grade", 0.0) / 10.0  # 퍼밀→퍼센트(프론트에서 나눠서 옴)
                        mu = float(payload.get("mu", 1.0))
                        if speed is not None and dist is not None:
                            sim.scn.v0 = float(speed) / 3.6
                            sim.scn.L = float(dist)
                            sim.scn.grade_percent = float(grade)
                            sim.scn.mu = mu
                            sim.rr_factor = _mu_to_rr_factor(mu)
                            if DEBUG:
                                print(f"setInitial: v0={speed}km/h, L={dist}m, grade={grade}%, mu={mu}")
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
                        if DEBUG:
                            print(f"Train length set to {length} cars.")
                        sim.reset()

                    elif name == "setMassTons":
                        mass_tons = float(payload.get("mass_tons", 200.0))
                        vehicle.mass_t = mass_tons / int(payload.get("length", 8))
                        vehicle.mass_kg = mass_tons * 1000.0
                        if DEBUG:
                            print(f"차량 전체 중량을 {mass_tons:.2f} 톤으로 업데이트했습니다.")
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
                        sim.rr_factor = _mu_to_rr_factor(value)
                        if DEBUG:
                            print(f"마찰계수(mu)={value}")
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

            # 시뮬 시간 흐름
            dt = sim.scn.dt
            while elapsed >= dt:
                if sim.running:
                    sim.step()
                last_sim_time += dt
                elapsed -= dt

            # 상태 전송
            if (now - last_send) >= send_interval:
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                last_send = now

            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass