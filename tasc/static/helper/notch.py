import numpy as np

# 기존 notch 인덱스
num_notch = 23
list = []
for n in range(num_notch):
    list.append(n)

notches = np.array(list)
# 목표 최대값
max_accel = -0.18
# max_accel *= (5/18)/
# 최소값
min_accel = -1.38
factor = 1.0
# 정규화 0~1
x_norm = (notches - notches[0]) / (notches[-1] - notches[0])
# 비선형 적용
y_nonlin = min_accel + (max_accel - min_accel) * (x_norm**factor)

y_nonlin = y_nonlin
# 각 notch 간 간격
deltas = np.diff(y_nonlin)

# 출력 with commas
print("Notch values:[", ', '.join(f"{v:.3f}" for v in y_nonlin), "],")
print("Notch gaps:", ', '.join(f"{d:.3f}" for d in deltas))
