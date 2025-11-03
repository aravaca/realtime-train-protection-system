
values = [-1.44, -1.16, -1.11, -1.05, -0.81, -0.62, -0.50, -0.36, -0.18, 0.00]

# result = []
# for a, b in zip(values[1:], values[2:]):
#     for x in (a, round(a - (a - b) / 3, 2), round(a - 2 * (a - b) / 3, 2)):
#         result.append(x)

# 리스트 컴프리헨션으로 한 번에 생성
tasc_list = [values[0]] + [
    x
    for a, b in zip(values[1:], values[2:])
    for x in (a, round(a - (a - b) / 3, 2), round(a - 2 * (a - b) / 3, 2))
] + [0.00]

print(tasc_list)
print(len(tasc_list))