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
    """
    μ가 낮을수록(미끄러울수록) 코스팅 저항(A0/B1)을 조금 낮춰 더 미끄러지는 감각을 줌.
    1.0(맑음) → 1.0, 0.6(비) → ~0.88, 0.3(눈) → ~0.79
    """
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
        self.tasc_deadband_m = 1.5
        self.tasc_hold_min_s = 0.20
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"
        self._tasc_peak_notch = 1
        self.tasc_armed = False
        self.tasc_active = False
        # __init__ 안 어딘가 TASC 관련 설정들 아래에
        self.tasc_target_bias_m = 0.35  # +면 ‘목표를 조금 더 앞으로 가게’ 만듦(언더런 교정)


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

    # ----------------- Physics helpers -----------------
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        """
        노치별 장비 목표감속 + 속도 기반 전기/공기 블렌딩 적용
        """
        if notch >= len(self.veh.notch_accels):
            return 0.0
        base = float(self.veh.notch_accels[notch]) # 음수

        # ----- 블렌딩 비율 계산 -----
        blend_cutoff_speed = 40.0 / 3.6 # m/s (40km/h)
        regen_frac = max(0.0, min(1.0, v / blend_cutoff_speed))

        # 저속(5 km/h 이하)에서 공기 제동 살짝 강화
        
        air_boost = 1.0
        if v < (5.0/3.6):
            air_boost = 0.8  # 저속에서 공기제동을 살~짝 약화 → 덜 일찍 선다
        blended_accel = base * (regen_frac + (1 - regen_frac) * air_boost)

       
        # ----- 접착 한계 적용 -----
        k_srv = 0.85
        k_eb = 0.98
        is_eb = (notch == (self.veh.notches - 1))
        k_adh = k_eb if is_eb else k_srv
        a_cap = -k_adh * float(self.scn.mu) * 9.81 # 음수

        a_eff = max(blended_accel, a_cap)

        # 간단 WSP
        if a_eff <= a_cap + 1e-6:
            scale = 0.90 if v > 8.0 else 0.85
            a_eff = a_cap * scale
    
 
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
        # TASC가 켜져 있었다면 '대기'로만 두고 즉시 작동하지 않음
        self.tasc_active = False
        self.tasc_armed = bool(self.tasc_enabled)

        # 예측 캐시 초기화
        self._tasc_pred_cache.update({"t": -1.0, "v": -1.0, "notch": -1,
                                      "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')})
        self._tasc_last_pred_t = -1.0

        # B5 필요 여부 캐시 초기화
        self._need_b5_last_t = -1.0
        self._need_b5_last = False

        # (추가) 제동장치 상태 초기화
        self.brk_accel = 0.0

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

    # ------ stopping distance helpers ------

    def _estimate_stop_distance(self, notch: int, v0: float) -> float:
        """
        현재 속도 v0에서 '해당 노치 고정'으로 가정하고 실제 동역학과 동일한 모델로
        간단 수치적분으로 정지거리 추정. (TASC 예측에 사용)
        - jerk/명령지연은 생략, 제동 응답 tau_brk는 1차 지연으로 반영
        - 성능 최적화: 더 큰 dt, 조기 종료
        """
        dt = 0.03 # 더 큰 스텝(≈33Hz)
        v = max(0.0, v0)
        a = 0.0
        s = 0.0
        tau = max(0.15, self.veh.tau_brk) # 응답 느림 반영

        # 조기 종료용: 현재 정지점까지 남은 거리 + 버퍼
        rem_now = self.scn.L - self.state.s
        limit = float(rem_now + 5.0)

        for _ in range(1200): # 최대 ≈36s
            a_brk = self._effective_brake_accel(notch, v)
            a_grade = self._grade_accel()
            a_davis = self._davis_accel(v)
            a_target = a_brk + a_grade + a_davis
            # 1차 지연 응답
            a += (a_target - a) * (dt / tau)
            # 적분
            v = max(0.0, v + a * dt)
            s += v * dt + 0.5 * a * dt * dt
            if v <= 0.01:
                break # 정지
            if s > limit:
                break # 충분히 큼 → 더 계산하지 않음
        return s + 0.3

    def _stopping_distance(self, notch: int, v: float) -> float:
        """보수적 예측: 위의 수치예측 사용"""
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v)

    def _tasc_predict(self, cur_notch: int, v: float):
        """TASC 정지거리 예측 스로틀링 + 캐시"""
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
        B5 필요 ↔ B4로는 못 멈춘다 ↔ s4 > remaining(+deadband)
        50ms 스로틀 및 캐시 적용.
        """
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last
        s4 = self._stopping_distance(2, v) # B4 정지거리만 계산
        need = s4 > (remaining + self.tasc_deadband_m)
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
                elif (st.t - self.first_brake_start) >= 1.2: # 1.2초로 연장
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
                    self._tasc_last_change_t = st.t # 활성화 시각

            if self.tasc_active:
                # (B) 초제동 유지: 활성화 이후에만 B1/B2를 2초간 강제
                if not self.first_brake_done:
                    desired = 2 if speed_kmh >= 70.0 else 1
                    if dwell_ok and cur != desired:
                        step = 1 if desired > cur else -1
                        st.lever_notch = self._clamp_notch(cur + step)
                        self._tasc_last_change_t = st.t
                else:
                    # (C) 빌드/릴렉스 로직 (기존 유지)
                    s_cur, s_up, s_dn = self._tasc_predict(cur, st.v)

                    changed = False

                    if self._tasc_phase == "build":
                        # 더 강한 제동이 필요하면 한 단계 강화
                        if cur < max_normal_notch and s_cur > (rem_now - self.tasc_deadband_m + self.tasc_target_bias_m):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur + 1)
                                self._tasc_last_change_t = st.t
                                self._tasc_peak_notch = max(self._tasc_peak_notch, st.lever_notch)
                                changed = True
                        else:
                            # 충분히 맞아떨어지면 relax로 전환
                            self._tasc_phase = "relax"

                    if self._tasc_phase == "relax" and not changed:
                        # 더 약한 제동으로도 충분하면 한 단계 완해
                        if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m + 0.1 + self.tasc_target_bias_m):
                            if dwell_ok:
                                st.lever_notch = self._clamp_notch(cur - 1)
                                self._tasc_last_change_t = st.t

        # ====== 동역학 ======
        # (수정) 제동 명령 → 제동장치 동역학 추종(느리게)
        a_cmd_brake = self._effective_brake_accel(st.lever_notch, st.v) # 음수
        is_eb = (st.lever_notch == self.veh.notches - 1)
        self._update_brake_dyn(a_cmd_brake, st.v, is_eb, dt) # self.brk_accel 갱신

        # 경사/저항
        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)

        # 목표 가속도: 제동장치가 실제로 내고 있는 감속 + 외력
        a_target = self.brk_accel + a_grade + a_davis

        # (수정) 차량 가속도에는 저크 제한만 적용
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

            # ★ 0 cm 정차 보너스 (+100) : 절대값 1cm 미만이면 인정
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

            # 노치 증감은 1단계 이하
            if abs(diff) > 1:
                return False

            if not peak_reached:
                if cur < prev:
                    peak_reached = True
            else:
                if cur > prev:
                    return False

            prev = cur

        # 마지막은 반드시 B1
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

                    elif name == "setTASC":
                        enabled = bool(payload.get("enabled", False))
                        sim.tasc_enabled = enabled
                        if enabled:
                            sim.manual_override = False
                            sim._tasc_last_change_t = sim.state.t
                            sim._tasc_phase = "build"
                            sim._tasc_peak_notch = 1
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
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                last_send = now

            await asyncio.sleep(0)
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass
