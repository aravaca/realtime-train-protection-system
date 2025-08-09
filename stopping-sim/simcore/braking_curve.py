import numpy as np


def build_tbc(L, v0, a_max, j_max, dt=0.01):
    """
    Target Braking Curve (TBC) 생성 - S-curve 감속 (역적분 방식)

    Args:
        L (float): 정지까지 거리 (m)
        v0 (float): 초기 속도 (m/s)
        a_max (float): 최대 감속 (m/s², 양수)
        j_max (float): 최대 저크 (m/s³, 양수)
        dt (float): 타임스텝 (s)

    Returns:
        Tuple of np.ndarray: s (거리), v (속도), a (가속도)
    """
    # 충분한 시간 확보
    t_max = 60
    N = int(t_max / dt)

    s = np.zeros(N)
    v = np.zeros(N)
    a = np.zeros(N)

    # 초기 조건 (정지점)
    s[-1] = L
    v[-1] = 0
    a[-1] = 0

    for i in reversed(range(N - 1)):
        # Phase 1: 감속을 서서히 증가 (-j_max)
        if a[i + 1] > -a_max:
            a[i] = a[i + 1] - j_max * dt
            if a[i] < -a_max:
                a[i] = -a_max
        else:
            # Phase 2: 최대 감속 유지
            a[i] = -a_max

        v[i] = v[i + 1] - a[i] * dt
        v[i] = max(v[i], 0)

        s[i] = s[i + 1] - v[i] * dt + 0.5 * a[i] * dt**2

        if s[i] < 0:
            # 역적분 완료 (0까지 도달)
            s = s[i:]
            v = v[i:]
            a = a[i:]
            break

    # 정방향 정렬
    s = s[::-1]
    v = v[::-1]
    a = a[::-1]

    return s, v, a


def build_notch_curves(L, v0, notch_accels, dt=0.01):
    """
    각 브레이크 노치에 해당하는 등가속 감속 곡선 생성

    Args:
        L (float): 거리 한계 (m)
        v0 (float): 초기 속도 (m/s)
        notch_accels (List[float]): 각 노치별 감속 (m/s², 음수 포함)
        dt (float): 타임스텝 (s)

    Returns:
        List[Tuple[np.ndarray, np.ndarray]]: [(s0, v0), (s1, v1), ...]
    """
    curves = []

    for a_cmd in notch_accels:
        s = [0.0]
        v = [v0]
        while v[-1] > 0 and s[-1] < L * 1.2:
            v_next = max(0.0, v[-1] + a_cmd * dt)
            s_next = s[-1] + v[-1] * dt + 0.5 * a_cmd * dt**2
            v.append(v_next)
            s.append(s_next)
        curves.append((np.array(s), np.array(v)))

    return curves
