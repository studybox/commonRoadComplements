import numpy as np
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.scenario.laneletcomplement import VecSE2, lerp_curve, CurveIndex, get_curve_index
import math
'''
 `lanelet_id`: road index
 `s`: distance along lane
 `d`: lane offset, positive is to left. zero point is the centerline of the lane.
 `phi`: lane relative heading
'''
class Frenet:
    def __init__(self, roadproj, lanelet_network, tuple_form = None):

        if tuple_form is not None:
            i,t,lid,s,d,phi = tuple_form
            self.ind = (CurveIndex(i, t), lid)
            self.s = s
            self.d = d
            self.phi = phi
        else:
            curve_ind = roadproj.curveproj.ind
            lanelet_id = roadproj.get_lanelet_id()
            self.ind = (curve_ind, lanelet_id)
            curve = lanelet_network.find_lanelet_by_id(lanelet_id).center_curve
            self.s = lerp_curve(curve[curve_ind.i], curve[curve_ind.i+1], curve_ind.t).s
            self.d = roadproj.curveproj.d
            #self.phi = roadproj.curveproj.phi
            self.phi = self._mod2pi(roadproj.curveproj.phi)

    def _mod2pi(self, x):
        if x > math.pi:
            return self._mod2pi(x-2*math.pi)
        if x <-math.pi:
            return self._mod2pi(x+2*math.pi)
        return x

    def get_posG_from_posF(self, lanelet_network):
        curve_ind, lanelet_id = self.ind

        curve = lanelet_network.find_lanelet_by_id(lanelet_id).center_curve

        footpoint = lerp_curve(curve[curve_ind.i], curve[curve_ind.i+1], curve_ind.t)

        th = footpoint.pos.th + 0.5*np.pi
        posG = VecSE2(footpoint.pos.x + self.d*np.cos(th), footpoint.pos.y + self.d*np.sin(th), footpoint.pos.th + self.phi)

        return posG
    def move_along(self, lanelet_network, delta_s, delta_d, new_phi, target_lanelet=None):
        curve_ind, lanelet_id = self.ind
        lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
        curve = lanelet.center_curve
        curvePt = lerp_curve(curve[curve_ind.i], curve[curve_ind.i+1], curve_ind.t)
        result_posF_adjust = result_posF = None
        #print("gaga ?", curvePt.s + delta_s)
        if curvePt.s + delta_s < -1e-4:#0.0:
            dist_s = [0.0]
            cands = [lanelet_id]
            while len(cands) > 0:
                current_cand = cands.pop(0)
                current_dist_s = dist_s.pop(0)
                current_lane = lanelet_network.find_lanelet_by_id(current_cand)
                if current_cand == target_lanelet:
                    new_delta_s =  current_dist_s + curvePt.s + delta_s
                    current_curve = current_lane.center_curve
                    final_ind, new_s = get_curve_index(CurveIndex(0,0.0), current_curve, new_delta_s)
                    final_lid = current_cand
                    infos = (final_ind.i, final_ind.t, final_lid, new_s, self.d + delta_d, new_phi)
                    result_posF = Frenet(None, None, infos)
                    break
                else:
                    if curvePt.s + delta_s + current_dist_s < 0.0:
                        for pred in current_lane.predecessor:
                            predecessor_lane = lanelet_network.find_lanelet_by_id(pred)
                            cands.append(pred)
                            dist_s.append(current_dist_s + predecessor_lane.center_curve[-1].s)
                    else:
                        new_delta_s = current_dist_s + curvePt.s + delta_s
                        current_curve = current_lane.center_curve
                        final_ind, new_s = get_curve_index(CurveIndex(0,0.0), current_curve, new_delta_s)
                        final_lid = current_cand
                        infos = (final_ind.i, final_ind.t, final_lid, new_s, self.d + delta_d, new_phi)
                        new_result_posF = Frenet(None, None, infos)
                        if target_lanelet is not None:
                            new_result_posG = new_result_posF.get_posG_from_posF(lanelet_network)
                            new_result_posF_adjust = Frenet(new_result_posG.projF(lanelet_network, [target_lanelet]), lanelet_network)
                            if result_posF_adjust is None:
                                result_posF = new_result_posF
                                result_posF_adjust = new_result_posF_adjust
                            elif target_lanelet is not None and np.abs(new_result_posF_adjust.d) < np.abs(result_posF_adjust.d):
                                result_posF = new_result_posF
                                result_posF_adjust = new_result_posF_adjust
                        else:
                            result_posF = new_result_posF

            #current_s = curvePt.s + delta_s

            #while len(lanelet.predecessor) > 0:
            #    new_curve = lanelet_network.find_lanelet_by_id(lanelet.predecessor[0]).center_curve
            #    s_all = new_curve[-1].s
            #    if -current_s > s_all:
                    # next next

            #        lanelet = lanelet_network.find_lanelet_by_id(lanelet.predecessor[0])
            #        current_s += lanelet.center_curve[-1].s
            #    else:
            #        new_delta_s = current_s + s_all - new_curve[-2].s
            #        new_ind = len(new_curve)-2

            #        final_ind, new_s = get_curve_index(CurveIndex(new_ind,0.0), new_curve, new_delta_s)
            #        final_lid = lanelet.predecessor[0]
            #        break
            #if len(lanelet.predecessor) == 0:
            #    # if no predecessor
            #    final_ind = CurveIndex(0, 0.0)
            #    final_lid = lanelet.lanelet_id
            #    new_s = 0.0
        elif curvePt.s + delta_s > curve[-1].s:
            dist_s = [0.0]
            cands = [lanelet_id]
            while len(cands) > 0:
                current_cand = cands.pop(0)
                current_dist_s = dist_s.pop(0)
                current_lane = lanelet_network.find_lanelet_by_id(current_cand)
                if current_cand == target_lanelet:
                    new_delta_s = curvePt.s + delta_s - current_dist_s
                    current_curve = current_lane.center_curve
                    final_ind, new_s = get_curve_index(CurveIndex(0,0.0), current_curve, new_delta_s)
                    final_lid = current_cand
                    infos = (final_ind.i, final_ind.t, final_lid, new_s, self.d + delta_d, new_phi)
                    result_posF = Frenet(None, None, infos)
                    break
                else:
                    if curvePt.s + delta_s  > current_dist_s + current_lane.center_curve[-1].s:
                        for succ in current_lane.successor:
                            successor_lane = lanelet_network.find_lanelet_by_id(succ)
                            cands.append(succ)
                            dist_s.append(current_dist_s + current_lane.center_curve[-1].s)
                    else:
                        new_delta_s = curvePt.s + delta_s - current_dist_s
                        current_curve = current_lane.center_curve
                        final_ind, new_s = get_curve_index(CurveIndex(0,0.0), current_curve, new_delta_s)
                        final_lid = current_cand
                        infos = (final_ind.i, final_ind.t, final_lid, new_s, self.d + delta_d, new_phi)
                        new_result_posF = Frenet(None, None, infos)
                        if target_lanelet is not None:
                            new_result_posG = new_result_posF.get_posG_from_posF(lanelet_network)
                            new_result_posF_adjust = Frenet(new_result_posG.projF(lanelet_network, [target_lanelet]), lanelet_network)
                            if result_posF_adjust is None:
                                result_posF = new_result_posF
                                result_posF_adjust = new_result_posF_adjust
                            elif target_lanelet is not None and np.abs(new_result_posF_adjust.d) < np.abs(result_posF_adjust.d):
                                result_posF = new_result_posF
                                result_posF_adjust = new_result_posF_adjust
                        else:
                            result_posF = new_result_posF

            #current_s = curvePt.s + delta_s
            #while len(lanelet.successor) > 0:
            #    new_curve = lanelet_network.find_lanelet_by_id(lanelet.successor[0]).center_curve
            #    s_all = new_curve[-1].s
            #    if current_s - lanelet.center_curve[-1].s > s_all:
                    # next
            #        current_s -= lanelet.center_curve[-1].s
            #        lanelet = lanelet_network.find_lanelet_by_id(lanelet.successor[0])
            #    else:
            #        new_delta_s = current_s - lanelet.center_curve[-1].s
            #        new_ind = 0
            #        final_ind, new_s = get_curve_index(CurveIndex(new_ind, 0.0), new_curve, new_delta_s)
            #        final_lid = lanelet.successor[0]
            #        break
            #if len(lanelet.successor) == 0:
                # no successor
            #    final_ind = CurveIndex(len(lanelet.center_curve)-2, 1.0)
            #    final_lid = lanelet.lanelet_id
            #    new_s = lanelet.center_curve[-1].s
        else:
            #print("gaga")
            final_ind, new_s = get_curve_index(curve_ind, curve, delta_s)
            final_lid = lanelet_id
            infos = (final_ind.i, final_ind.t, final_lid, new_s, self.d + delta_d, new_phi)
            result_posF = Frenet(None, None, infos)
            result_lane = lanelet_network.find_lanelet_by_id(result_posF.ind[1])

        #infos = (final_ind.i, final_ind.t, final_lid, new_s, self.d + delta_d, new_phi)
        #result_posF = Frenet(None, None, infos)
        #result_lane = lanelet_network.find_lanelet_by_id(result_posF.ind[1])
        if target_lanelet == lanelet.adj_left or target_lanelet == lanelet.adj_right and target_lanelet is not None:
            result_posG = result_posF.get_posG_from_posF(lanelet_network)
            result_posF = Frenet(result_posG.projF(lanelet_network, [target_lanelet, lanelet.lanelet_id]), lanelet_network)
        #if result_posF.d >= 3.0 *0.5 and result_lane.adj_left is not None:
        #    result_posG = result_posF.get_posG_from_posF(lanelet_network)
        #    result_posF = Frenet(result_posG.projF(lanelet_network, [result_posF.ind[1], result_lane.adj_left]), lanelet_network)
        #elif result_posF.d <= -3.0 *0.5 and result_lane.adj_right is not None:
        #    result_posG = result_posF.get_posG_from_posF(lanelet_network)
        #    result_posF = Frenet(result_posG.projF(lanelet_network, [result_posF.ind[1], result_lane.adj_right]), lanelet_network)

        return result_posF


class FrenetState(State):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.posG = VecSE2(self.position[0], self.position[1], self.orientation)
        self.posF = None
    def get_posG(self):
        if self.posG is None:
            self.posG = VecSE2(self.position[0], self.position[1], self.orientation)
        return self.posG
    def set_posF(self, lanelet_network, clues=None):
        posG = self.get_posG()
        self.posF = Frenet(posG.projF(lanelet_network, clues), lanelet_network)
    def get_posG_from_posF(self, lanelet_network, posF=None):

        if posF is None:
            if self.posF is None:
                print("posF not set")
                self.set_posF(lanelet_network)
            posF = self.posF

        curve_ind, lanelet_id = posF.ind

        curve = lanelet_network.find_lanelet_by_id(lanelet_id).center_curve

        footpoint = lerp_curve(curve[curve_ind.i], curve[curve_ind.i+1], curve_ind.t)

        th = footpoint.pos.th + 0.5*np.pi
        posG = VecSE2(footpoint.pos.x + posF.d*np.cos(th), footpoint.pos.y + posF.d*np.sin(th), footpoint.pos.th + posF.phi)

        return posG
    def get_footpoint(self):
        posG = self.get_posG()
        return VecSE2(posG.x+self.posF.d*np.cos(self.posG.th-self.posF.phi-np.pi*0.5), posG.y+self.posF.d*np.sin(self.posG.th-self.posF.phi-np.pi*0.5), posG.th)
