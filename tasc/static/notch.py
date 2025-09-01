import numpy as np

# 기존 notch 인덱스
notches = np.array([0,1,2,3,4])
# 목표 최대값
max_accel = 0.694
# 최소값
min_accel = 0.10
factor = 1.6

# 정규화 0~1
x_norm = (notches - notches[0]) / (notches[-1] - notches[0])
# 비선형 적용
y_nonlin = min_accel + (max_accel - min_accel) * (x_norm**factor)

# 각 notch 간 간격
deltas = np.diff(y_nonlin)

# 출력 with commas
print("Notch values:[", ', '.join(f"{v:.3f}" for v in y_nonlin), "],")
print("Notch gaps:", ', '.join(f"{d:.3f}" for d in deltas))
