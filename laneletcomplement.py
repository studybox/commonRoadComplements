import numpy as np
import math
from typing import *
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking

class VecE2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __add__(self, b):
        return VecE2(self.x+b.x, self.y+b.y)
    def __sub__(self, b):
        return VecE2(self.x-b.x, self.y-b.y)


class VecSE2(VecE2):
    def __init__(self, x, y, th):
        super().__init__(x, y)
        self.th = th
    def __str__(self):
        return "VecSE2: x {}, y {}, th {}".format(self.x, self.y, self.th)
    def __repr__(self):
        return "VecSE2: x {}, y {}, th {}".format(self.x, self.y, self.th)

    def proj_on_curve(self, curve):
        ind = self.index_closest_to_point(curve)
        curveind = None
        footpoint = None

        if ind > 0 and ind < len(curve)-1:
            t_lo = get_lerp_time(curve[ind-1].pos, curve[ind].pos, self)
            t_hi = get_lerp_time(curve[ind].pos, curve[ind+1].pos, self)

            p_lo = lerp_pos(curve[ind-1].pos, curve[ind].pos, t_lo)
            p_hi = lerp_pos(curve[ind].pos, curve[ind+1].pos, t_hi)

            d_lo = (p_lo.x - self.x)**2 + (p_lo.y - self.y)**2
            d_hi = (p_hi.x - self.x)**2 + (p_hi.y - self.y)**2

            if d_lo < d_hi:
                footpoint = p_lo
                curveind = CurveIndex(ind-1, t_lo)
            else:
                footpoint = p_hi
                curveind = CurveIndex(ind, t_hi)
        elif ind == 0:
            t = get_lerp_time(curve[0].pos, curve[1].pos, self)
            footpoint = lerp_pos( curve[0].pos, curve[1].pos, t)
            curveind = CurveIndex(ind, t)
        else: # ind == len(curve)-1
            t = get_lerp_time( curve[-2].pos, curve[-1].pos, self)
            footpoint = lerp_pos( curve[-2].pos, curve[-1].pos, t)
            curveind = CurveIndex(ind-1, t)
        curve_proj, x = self.get_curve_projection(footpoint, curveind)
        if curveind.i >= len(curve)-2 and curveind.t ==1.0:
            return curve_proj
        elif curveind.i == 0 and curveind.t == 0.0:
            return curve_proj
        if np.abs(x) >= 1.0:
            best_curve_proj = curve_proj
            best_x = x
            if x > 0.0:
                ds = 0.1
                while ds <= x:
                    new_curveind, _ = get_curve_index(curveind, curve, ds)
                    if new_curveind.t > 1.0:
                        break
                    new_footpoint = lerp_pos(curve[new_curveind.i].pos,curve[new_curveind.i+1].pos, new_curveind.t)
                    new_curve_proj, new_x = self.get_curve_projection(new_footpoint, new_curveind)
                    if np.abs(new_x) < np.abs(best_x):
                        best_x = new_x
                        best_curve_proj = new_curve_proj
                    ds += 0.1
            else:
                ds = -0.1
                while ds >= x:
                    new_curveind, _ = get_curve_index(curveind, curve, ds)
                    if new_curveind.t < 0.0:
                        break
                    new_footpoint = lerp_pos(curve[new_curveind.i].pos,curve[new_curveind.i+1].pos, new_curveind.t)
                    new_curve_proj, new_x = self.get_curve_projection(new_footpoint, new_curveind)
                    if np.abs(new_x) < np.abs(best_x):
                        best_x = new_x
                        best_curve_proj = new_curve_proj
                    ds -= 0.1
            return best_curve_proj
        return curve_proj

    def index_closest_to_point(self, curve):

        a = 0
        b = len(curve)-1
        c = (a+b)//2

        sqdist_a = np.hypot(curve[a].pos.x - self.x, curve[a].pos.y - self.y)
        sqdist_b = np.hypot(curve[b].pos.x - self.x, curve[b].pos.y - self.y)
        sqdist_c = np.hypot(curve[c].pos.x - self.x, curve[c].pos.y - self.y)

        while True:
            if b == a:
                return a
            elif b == a + 1:
                return b if sqdist_b < sqdist_a else a
            elif a + 1 == c and b - 1 == c:
                if sqdist_a < sqdist_b and sqdist_a < sqdist_c:
                    return a
                elif sqdist_b < sqdist_a and sqdist_b < sqdist_c:
                    return b
                else:
                    return c
            left = (a+c)//2
            sqdist_l = np.hypot(curve[left].pos.x - self.x, curve[left].pos.y - self.y)

            right = (c+b)//2
            sqdist_r = np.hypot(curve[right].pos.x - self.x, curve[right].pos.y - self.y)

            if sqdist_l < sqdist_r:
                b = c
                sqdist_b = sqdist_c
                c = left
                sqdist_c = sqdist_l
            else:
                a = c
                sqdist_a = sqdist_c
                c = right
                sqdist_c = sqdist_r

    def get_curve_projection(self, footpoint, curveind):
        F = self.inertial2body(footpoint)
        #print("curve", F.x, " ind ", curveind.i, " t ", curveind.t)
        return CurveProjection(curveind, F.y, F.th), F.x

    def inertial2body(self, reference):
        s, c = np.sin(reference.th), np.cos(reference.th)
        deltax = self.x - reference.x
        deltay = self.y - reference.y
        return VecSE2(c*deltax + s*deltay, c*deltay - s*deltax, self._mod2pi(self.th-reference.th))

    def _mod2pi(self, x):
        if x > math.pi:
            return self._mod2pi(x-2*math.pi)
        if x <-math.pi:
            return self._mod2pi(x+2*math.pi)
        return x

    def proj(self, lanelet, lanelet_network, move_along_curves=True):
        curveproj = self.proj_on_curve(lanelet.center_curve)
        retid = lanelet.lanelet_id
        # TODO Check this
        #if curveproj.ind.is_start() and len(lanelet.predecessor)>0 :
        #elif curveproj.ind.is_end() and len(lanelet.successor)>0 :
        return LaneletProjection(curveproj, retid)

    def projF(self, lanelet_network, clues):
        best_dist2 = float('inf')
        best_proj = LaneletProjection(CurveProjection(CurveIndex(-1, -1.0),
                                                   float('nan'), float('nan')), -1)
        if clues is None:
            candidates = lanelet_network.lanelets
        else:
            candidates = []
            for c in clues:
                candidates.append(lanelet_network.find_lanelet_by_id(c))
        for lanelet in candidates:
            roadproj = self.proj(lanelet, lanelet_network, move_along_curves=False)
            footpoint = lanelet.get_curvePt_by_curveid(roadproj.curveproj.ind)
            dist2 =  (self.x - footpoint.pos.x)**2 + (self.y - footpoint.pos.y)**2

            if dist2 < best_dist2:
                best_dist2 = dist2
                best_proj = roadproj
        return best_proj
    def projCurvePt(self, curvePt_reference):
        F = self.inertial2body(curvePt_reference.pos)
        return F.x, F.y

    def move_along(self, lanelet_network, delta_s, delta_d, posF):
        curve_ind, lanelet_id  = posF.move_along(lanelet_network, delta_s, delta_d)
        curve = lanelet_network.find_lanelet_by_id(lanelet_id).center_curve
        footpoint = lerp_curve(curve[curve_ind.i], curve[curve_ind.i+1], curve_ind.t)
        th = footpoint.pos.th + 0.5*np.pi
        posG = VecSE2(footpoint.pos.x + self.d*np.cos(th), footpoint.pos.y + self.d*np.sin(th), footpoint.pos.th + posF.phi)
        return posG
 ################
 # On Curves



class CurvePt:
    def __init__(self, pos, s, k, kd):
        self.pos = pos
        self.s = s
        self.k = k
        self.kd = kd
    def __str__(self):
        return "CurvePt: pos {}, disp {}, k {}, kd {}".format(self.pos, self.s, self.k, self.kd)
    def __repr__(self):
        return "CurvePt: pos {}, disp {}, k {}, kd {}".format(self.pos, self.s, self.k, self.kd)

class CurveIndex:
    def __init__(self, ind, t):
        self.i = ind
        self.t = t
    def __str__(self):
        return "Index: i {}, t {}".format(self.i, self.t)
    def __repr__(self):
        return "Index: i {}, t {}".format(self.i, self.t)

class CurveProjection:
    def __init__(self, curveindex, d, phi):
        self.ind = curveindex
        self.d = d
        self.phi = phi

class LaneletProjection:
    def __init__(self, curveprojection, lanelet_id):
        self.curveproj = curveprojection
        self.lanelet_id = lanelet_id
    def get_lanelet_id(self):
        return self.lanelet_id

class Grid():
    def __init__(self, gid):
        self.id = gid
        self.pos_list,self.ind_list = [], []
        self.front_list,self.back_list = [],[]
        self.left_list, self.right_list = [], []
        self.conflict_list = []
    def add_pos(self, pos):
        self.pos_list.append(pos)
    def add_ind(self, ind):
        self.ind_list.append(ind)
    def add_front(self, gid):
        if gid not in self.front_list:
            self.front_list.append(gid)
    def add_back(self, gid):
        if gid not in self.back_list:
            self.back_list.append(gid)
    def add_left(self, gid):
        if gid not in self.left_list:
            self.left_list.append(gid)
    def add_right(self, gid):
        if gid not in self.right_list:
            self.right_list.append(gid)
    def add_conflict(self, gid):
        if gid not in self.conflict_list:
            self.conflict_list.append(gid)
    def in_grid(self, ind, pos):
        has_rear_pt = False
        has_front_pt = False
        same_lane = False
        for gind in self.ind_list:
            if gind[1] == ind[1]:
                same_lane = True
                if gind[0].i < ind[0].i or (gind[0].i==ind[0].i and gind[0].t<=ind[0].t):
                    has_rear_pt = True
                else:
                    has_front_pt = True
        if not same_lane:
            return False
        if has_front_pt and has_rear_pt:
            return True
        least_dist2 = 20
        for gpos in self.pos_list:
            if (gpos.x-pos.x)**2+(gpos.y-pos.y)**2 < least_dist2:
                return True
        return False

    def __str__(self):
        return "Grid {}".format(self.id)
    def __repr__(self):
        return "Grid {}".format(self.id)


def proj_on_line(a : VecE2, b : VecE2):
    # dot(a,b) / dot(b,b) ⋅ b
    s = (a.x*b.x + a.y*b.y) / (b.x*b.x + b.y*b.y)
    return VecE2(s*b.x, s*b.y)

def get_lerp_time_unclamped(A : VecE2, B : VecE2, Q : VecE2):
    a = Q - A
    b = B - A
    c = proj_on_line(a, b)
    if b.x != 0.0:
        t = c.x / b.x
    elif b.y != 0.0:
        t = c.y / b.y
    else:
        t = 0.0
    return t

def get_lerp_time(A : VecSE2, B : VecSE2, Q : VecSE2):
    return np.clip(get_lerp_time_unclamped(VecE2(A.x, A.y), VecE2(B.x, B.y), VecE2(Q.x, Q.y)), 0.0, 1.0)

def lerp_angle(a : float, b : float, t: float):
    return a + np.arctan(np.sin(b-a)/np.cos(b-a))*t


def lerp_pos(a : VecSE2, b : VecSE2, t : float):
    x = a.x + (b.x-a.x)*t
    y = a.y + (b.y-a.y)*t
    th = lerp_angle(a.th, b.th, t)
    return VecSE2(x,y,th)

def lerp_curve(a : CurvePt, b : CurvePt, t : float):
    return CurvePt(lerp_pos(a.pos, b.pos, t), a.s + (b.s - a.s)*t, a.k + (b.k - a.k)*t, a.kd + (b.kd - a.kd)*t)

def get_curve_index(ind : CurveIndex, curve, delta_s):

    L = len(curve)
    ind_lo, ind_hi = ind.i, ind.i+1

    s_lo = curve[ind_lo].s
    s_hi = curve[ind_hi].s

    s = s_lo + (s_hi - s_lo)*ind.t

    if delta_s >= 0.0:
        if s + delta_s >= s_hi and ind_hi < L-1:
            while s + delta_s >= s_hi and ind_hi < L-1:
                delta_s -= s_hi - s
                s = s_hi
                ind_lo += 1
                ind_hi += 1
                s_lo = curve[ind_lo].s
                s_hi = curve[ind_hi].s
        else:
            delta_s = s + delta_s - s_lo
        t = delta_s/(s_hi - s_lo)
        return CurveIndex(ind_lo, t), s_lo+delta_s
    else:
        while s + delta_s < s_lo and ind_lo > 0:
            delta_s += (s - s_lo)
            s = s_lo
            ind_lo -= 1
            ind_hi -= 1
            s_lo = curve[ind_lo].s
            s_hi = curve[ind_hi].s

        delta_s = s + delta_s - s_lo
        t = delta_s/(s_hi-s_lo)
        return CurveIndex(ind_lo, t), s_lo+delta_s

##################
# On Lanelet

class LaneLetCurve(Lanelet):
    def __init__(self, left_vertices: np.ndarray, center_vertices: np.ndarray, right_vertices: np.ndarray,
                 center_curve,
                 lanelet_id,
                 predecessor=None, predecessor_connections=None, successor=None, successor_connections=None,
                 adjacent_left=None, adjacent_left_same_direction=None,
                 adjacent_right=None, adjacent_right_same_direction=None,
                 speed_limit=np.infty, line_marking_left_vertices=None,
                 line_marking_right_vertices=None, lanelet_type=set()):
        super().__init__(left_vertices, center_vertices, right_vertices,
                 lanelet_id,
                 predecessor=predecessor, successor=successor,
                 adjacent_left=adjacent_left, adjacent_left_same_direction=adjacent_left_same_direction,
                 adjacent_right=adjacent_right, adjacent_right_same_direction=adjacent_right_same_direction,
                  line_marking_left_vertices=line_marking_left_vertices,
                 line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type)
        self.center_curve = center_curve
        self.speed_limit = speed_limit

        if predecessor_connections is None:
            self.predecessor_connection = {}
        else:
            self.predecessor_connections = predecessor_connections

        if successor_connections is None:
            self.successor_connection = {}
        else:
            self.successor_connections = successor_connections

    def get_curvePt_by_curveid(self, curveindex):
        return lerp_curve(self.center_curve[curveindex.i], self.center_curve[curveindex.i+1], curveindex.t)
    def _convert(self, vertices):
        #TODO
        pass

def make_lanelet_curve_network(lanelet_network, speed_limit_dict):
    new_lanelets = []
    for lanelet in lanelet_network.lanelets:

        ths = []
        dss = []
        ss = [0]
        for i_pt in range(len(lanelet.center_vertices)-1):
            dx = lanelet.center_vertices[i_pt+1,0] - lanelet.center_vertices[i_pt,0]
            dy = lanelet.center_vertices[i_pt+1,1] - lanelet.center_vertices[i_pt,1]
            ths.append(np.arctan2(dy, dx))
            dss.append(np.sqrt(dx**2 + dy**2))
            ss.append(np.sqrt(dx**2 + dy**2))
        ths.append(ths[-1])
        dss.append(dss[-1])
        ss = np.cumsum(ss)

        ks = []
        for i_pt in range(len(lanelet.center_vertices)-1):
            ks.append((ths[i_pt+1] - ths[i_pt])/dss[i_pt])
        ks.append(ks[-1])
        kds = []
        for i_pt in range(len(lanelet.center_vertices)-1):
            kds.append((ks[i_pt+1] - ks[i_pt])/dss[i_pt])
        kds.append(kds[-1])
        center_curve = []
        for i_pt in range(len(lanelet.center_vertices)):
            center_curve.append(CurvePt(VecSE2(lanelet.center_vertices[i_pt,0],lanelet.center_vertices[i_pt,1],ths[i_pt]),
                                        ss[i_pt], ks[i_pt], kds[i_pt]))
        predecessor_connections = {}
        for pred in lanelet.predecessor:
            pred_lanelet = lanelet_network.find_lanelet_by_id(pred)
            mylane = CurveIndex(int(0), 0.0)
            targetlane = CurveIndex(len(pred_lanelet.center_vertices)-2, 1.0)
            predecessor_connections[pred] = (mylane, targetlane)
        successor_connections = {}
        for succ in lanelet.successor:
            mylane = CurveIndex(len(lanelet.center_vertices)-2, 1.0)
            targetlane = CurveIndex(int(0), 0.0)
            successor_connections[succ] = (mylane, targetlane)

        new_lanelet = LaneLetCurve(
                       left_vertices=lanelet.left_vertices, center_vertices=lanelet.center_vertices, right_vertices=lanelet.right_vertices, center_curve=center_curve,
                       lanelet_id=lanelet.lanelet_id,
                       predecessor=lanelet.predecessor, predecessor_connections=predecessor_connections, successor=lanelet.successor, successor_connections=successor_connections,
                       adjacent_left=lanelet.adj_left, adjacent_left_same_direction=lanelet.adj_left_same_direction,
                       adjacent_right=lanelet.adj_right, adjacent_right_same_direction=lanelet.adj_right_same_direction,
                       speed_limit=speed_limit_dict[lanelet.lanelet_id],
                       line_marking_left_vertices=lanelet.line_marking_left_vertices,
                       line_marking_right_vertices=lanelet.line_marking_right_vertices,
                       lanelet_type=lanelet.lanelet_type)
        new_lanelets.append(new_lanelet)
    return LaneletNetwork.create_from_lanelet_list(new_lanelets)
