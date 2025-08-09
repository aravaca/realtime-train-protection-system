# simcore/controller.py


class TASCController:
    def __init__(self, veh, kp=0.6, ki=0.15, notch_rate=4.0):
        self.veh = veh
        self.kp = kp
        self.ki = ki
        self.notch_rate = notch_rate  # notch 변화 속도 제한 (notch/sec)
        self.integral = 0.0
        self.last_error = None
        self.last_notch = 0

    def step(self, s, v, v_ref, dt):
        error = v_ref - v

        # 적분 누적 (anti-windup을 단순히 notch 범위로 제한)
        self.integral += error * dt
        self.integral = max(min(self.integral, 5), -5)

        # PI 계산
        output = self.kp * error + self.ki * self.integral

        # 목표 감속을 notch 단위로 변환 (음수니까 절댓값으로)
        # 0..notches-1 범위 내에서 가까운 notch를 찾음
        candidate_notch = 0
        min_diff = float("inf")
        for n, a_n in enumerate(self.veh.notch_accels):
            diff = abs(output - (-a_n))  # output은 양수, a_n 음수
            if diff < min_diff:
                min_diff = diff
                candidate_notch = n

        # notch 변화 제한 (notch/sec)
        max_delta = self.notch_rate * dt
        delta = candidate_notch - self.last_notch
        if delta > max_delta:
            candidate_notch = self.last_notch + max_delta
        elif delta < -max_delta:
            candidate_notch = self.last_notch - max_delta

        # notch는 정수여야 하므로 반올림
        candidate_notch = int(round(candidate_notch))

        # 범위 제한
        candidate_notch = max(0, min(candidate_notch, self.veh.notches - 1))

        self.last_notch = candidate_notch
        self.last_error = error

        return candidate_notch
