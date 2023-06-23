"""
Dubins Path
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
# import draw as draw


# class for PATH element
class PATH:
    def __init__(self, L, mode, x, y, yaw):
        self.L = L  # total path length [float]
        self.mode = mode  # type of each part of the path [string]
        self.x = x  # final s positions [m]
        self.y = y  # final y positions [m]
        self.yaw = yaw  # final yaw angles [rad]


# utils
def pi_2_pi(theta):
    while theta > math.pi:
        theta -= 2.0 * math.pi

    while theta < -math.pi:
        theta += 2.0 * math.pi

    return theta


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / math.pi / 2.0)


def LSL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_lsl = 2 + dist ** 2 - 2 * cos_a_b + 2 * dist * (sin_a - sin_b)

    if p_lsl < 0:
        return None, None, None, ["L", "S", "L"]
    else:
        p_lsl = math.sqrt(p_lsl)

    denominate = dist + sin_a - sin_b
    t_lsl = mod2pi(-alpha + math.atan2(cos_b - cos_a, denominate))
    q_lsl = mod2pi(beta - math.atan2(cos_b - cos_a, denominate))

    return t_lsl, p_lsl, q_lsl, ["L", "S", "L"]


def RSR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_rsr = 2 + dist ** 2 - 2 * cos_a_b + 2 * dist * (sin_b - sin_a)

    if p_rsr < 0:
        return None, None, None, ["R", "S", "R"]
    else:
        p_rsr = math.sqrt(p_rsr)

    denominate = dist - sin_a + sin_b
    t_rsr = mod2pi(alpha - math.atan2(cos_a - cos_b, denominate))
    q_rsr = mod2pi(-beta + math.atan2(cos_a - cos_b, denominate))

    return t_rsr, p_rsr, q_rsr, ["R", "S", "R"]


def LSR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_lsr = -2 + dist ** 2 + 2 * cos_a_b + 2 * dist * (sin_a + sin_b)

    if p_lsr < 0:
        return None, None, None, ["L", "S", "R"]
    else:
        p_lsr = math.sqrt(p_lsr)

    rec = math.atan2(-cos_a - cos_b, dist + sin_a + sin_b) - math.atan2(-2.0, p_lsr)
    t_lsr = mod2pi(-alpha + rec)
    q_lsr = mod2pi(-mod2pi(beta) + rec)

    return t_lsr, p_lsr, q_lsr, ["L", "S", "R"]


def RSL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_rsl = -2 + dist ** 2 + 2 * cos_a_b - 2 * dist * (sin_a + sin_b)

    if p_rsl < 0:
        return None, None, None, ["R", "S", "L"]
    else:
        p_rsl = math.sqrt(p_rsl)

    rec = math.atan2(cos_a + cos_b, dist - sin_a - sin_b) - math.atan2(2.0, p_rsl)
    t_rsl = mod2pi(alpha - rec)
    q_rsl = mod2pi(beta - rec)

    return t_rsl, p_rsl, q_rsl, ["R", "S", "L"]


def RLR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    rec = (6.0 - dist ** 2 + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0

    if abs(rec) > 1.0:
        return None, None, None, ["R", "L", "R"]

    p_rlr = mod2pi(2 * math.pi - math.acos(rec))
    t_rlr = mod2pi(alpha - math.atan2(cos_a - cos_b, dist - sin_a + sin_b) + mod2pi(p_rlr / 2.0))
    q_rlr = mod2pi(alpha - beta - t_rlr + mod2pi(p_rlr))

    return t_rlr, p_rlr, q_rlr, ["R", "L", "R"]


def LRL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    rec = (6.0 - dist ** 2 + 2.0 * cos_a_b + 2.0 * dist * (sin_b - sin_a)) / 8.0

    if abs(rec) > 1.0:
        return None, None, None, ["L", "R", "L"]

    p_lrl = mod2pi(2 * math.pi - math.acos(rec))
    t_lrl = mod2pi(-alpha - math.atan2(cos_a - cos_b, dist + sin_a - sin_b) + p_lrl / 2.0)
    q_lrl = mod2pi(mod2pi(beta) - alpha - t_lrl + mod2pi(p_lrl))

    return t_lrl, p_lrl, q_lrl, ["L", "R", "L"]


def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):
    if m == "S":
        px[ind] = ox + l / maxc * math.cos(oyaw)
        py[ind] = oy + l / maxc * math.sin(oyaw)
        pyaw[ind] = oyaw
    else:
        ldx = math.sin(l) / maxc
        if m == "L":
            ldy = (1.0 - math.cos(l)) / maxc
        elif m == "R":
            ldy = (1.0 - math.cos(l)) / (-maxc)

        gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
        gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy

    if m == "L":
        pyaw[ind] = oyaw + l
    elif m == "R":
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions


def generate_local_course(L, lengths, mode, maxc, step_size):
    point_num = int(L / step_size) + len(lengths) + 3

    px = [0.0 for _ in range(point_num)]
    py = [0.0 for _ in range(point_num)]
    pyaw = [0.0 for _ in range(point_num)]
    directions = [0 for _ in range(point_num)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size

    ll = 0.0

    for m, l, i in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = -d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = \
                interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        px, py, pyaw, directions = \
            interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)

    if len(px) <= 1:
        return [], [], [], []

    # remove unused data
    while len(px) >= 1 and px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions


def planning_from_origin(gx, gy, gyaw, curv, step_size):
    D = math.hypot(gx, gy)
    d = D * curv

    theta = mod2pi(math.atan2(gy, gx))
    alpha = mod2pi(-theta)
    beta = mod2pi(gyaw - theta)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    best_cost = float("inf")
    bt, bp, bq, best_mode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)

        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if best_cost > cost:
            bt, bp, bq, best_mode = t, p, q, mode
            best_cost = cost
    lengths = [bt, bp, bq]

    x_list, y_list, yaw_list, directions = generate_local_course(
        sum(lengths), lengths, best_mode, curv, step_size)

    return x_list, y_list, yaw_list, best_mode, best_cost


def calc_dubins_path(sx, sy, syaw, gx, gy, gyaw, curv, step_size=0.1):
    gx = gx - sx
    gy = gy - sy

    l_rot = Rot.from_euler('z', syaw).as_matrix()[0:2, 0:2]
    le_xy = np.stack([gx, gy]).T @ l_rot
    le_yaw = gyaw - syaw

    lp_x, lp_y, lp_yaw, mode, lengths = planning_from_origin(
        le_xy[0], le_xy[1], le_yaw, curv, step_size)

    rot = Rot.from_euler('z', -syaw).as_matrix()[0:2, 0:2]
    converted_xy = np.stack([lp_x, lp_y]).T @ rot
    x_list = converted_xy[:, 0] + sx
    y_list = converted_xy[:, 1] + sy
    yaw_list = [pi_2_pi(i_yaw + syaw) for i_yaw in lp_yaw]
    return PATH(lengths, mode, x_list, y_list, yaw_list)


def dupin_smooth(states,max_c):
    path_x, path_y, yaw = [], [], []
    smooth_path=[]
    for i in range(len(states) - 1):
        s_x = states[i][0]
        s_y = states[i][1]
        s_yaw = np.deg2rad(states[i][2])
        g_x = states[i + 1][0]
        g_y = states[i + 1][1]
        g_yaw = np.deg2rad(states[i + 1][2])
        path_i = calc_dubins_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, max_c)
        for x, y, iyaw in zip(path_i.x, path_i.y, path_i.yaw):
            path_x.append(x)
            path_y.append(y)
            yaw.append(iyaw)
            smooth_path.append([x, y, iyaw])
    return smooth_path

def main():
    states=[(140, 60, -180), (133.29928172514263, 58.5167590004819, -180), (115.33426379805583, 57.808475396527946, -180), (104.0930187819094, 55.98716936709386, 150), (88.98626701784836, 65.62055995560259, 60), (92.0420309466869, 74.20024368854274, 60), (97.99943571654177, 84.64731921905555, 60), (103.38866503819168, 96.26407124522682, 90), (103.98470593505743, 109.27703359130524, 90), (105.6991055752328, 124.49670394466628, 120), (101.02577400106382, 138.56603820926884, -150), (88.63792553126687, 134.52046334040188, 180), (76.57374444681739, 135.00424772593226, 180), (63.15177019679338, 135.2077477504061, 180), (44.22619048400519, 135.73525469402702, 180), (34.681027718664524, 138.21196450608412, -150), (19.898836789502692, 127.89248997167769, -90), (17.472360570494953, 115.2825274087782, -120), (11.92871511537037, 99.51180866939477, -90), (11.240435006647864, 86.56619695510327, -90), (10.286948976620176, 72.86930029286283, -90), (13.31535495661096, 59.75658997491251, -60), (16.771774213097025, 49.2485026969138, -90), (20.49643505696428, 33.62701042643676, -30), (29.247629429062748, 26.010714966497684, 0), (42.592229699843514, 23.09877491608586, 0), (57.714228535426464, 23.28130084917897, -30), (64.95636177058716, 20.370611013415406, -30), (80.17251700217626, 15.487850912584594, 0), (96.94124903171542, 11.346094266818994, 0), (114.54364842488353, 13.115497006880783, 0), (124.48926953642489, 12.28350823083649, 0), (140, 10, 0)]
    dupin_smooth(states,0.22)


if __name__ == '__main__':
    main()
