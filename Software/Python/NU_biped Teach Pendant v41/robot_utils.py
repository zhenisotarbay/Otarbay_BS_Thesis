from math import pi, sin, cos, radians  # this library is to work with math stuffs
from numpy import matrix

def forward_kinematics(*theta):
    alf = [0, (-pi) / 2, pi / 2, -pi / 2, 0, pi / 2]
    a = [0, 0, 0, 318, 318, 0]
    d = [0, 0, 0, 0, 0, 0]
    t = []
    for i in range(len(theta)):
        t.append(radians(theta[i]))
    t[1] += pi / 2
    T_keys = ["01", "12", "23", "34", "45", "56"]
    T_length = len(T_keys)
    T = {}
    for i in range(T_length):
        j = T_keys[i]
        T[j] = matrix([
            [cos(t[i]), -sin(t[i]), 0, a[i]],
            [sin(t[i]) * cos(alf[i]), cos(t[i]) * cos(alf[i]), -sin(alf[i]), -sin(alf[i]) * d[i]],
            [sin(t[i]) * sin(alf[i]), cos(t[i]) * sin(alf[i]), cos(alf[i]), cos(alf[i]) * d[i]],
            [0, 0, 0, 1]
        ])
    L5 = 50
    L6 = 100
    T["6F"] = matrix([
        [0, 0, -1, L5],
        [0, 1, 0, 0],
        [1, 0, 0, L6],
        [0, 0, 0, 1]
    ])
    for i in range(2, 7):
        T["0" + repr(i)] = T["0" + repr(i - 1)] * T[repr(i - 1) + repr(i)]
    T["0F"] = T["06"] * T["6F"]
    return T