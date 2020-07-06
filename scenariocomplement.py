import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectorycomplement import FrenetState, Frenet
from commonroad.scenario.laneletcomplement import *
from copy import deepcopy
# Items

LANECURVATURE = 1
LINE_MARKING_LEFT_DIST = 2
LINE_MARKING_RIGHT_DIST = 3
TIMEGAP = 4
TTC = 5
NEIGHBOR_FORE_ALONG_LANE = 6
NEIGHBOR_FORE_ALONG_LEFT_LANE = 7
NEIGHBOR_FORE_ALONG_RIGHT_LANE = 8
NEIGHBOR_REAR_ALONG_LANE = 9
NEIGHBOR_REAR_ALONG_LEFT_LANE = 10
NEIGHBOR_REAR_ALONG_RIGHT_LANE = 11
NEIGHBOR_FORE_REAR_LEFT_MID_RIGHT = 15
IS_COLLIDING = 12
ROAD_DIST_LEFT = 13
ROAD_DIST_RIGHT = 14

class ScenarioWrapper:
    def __init__(self, dt, lanelet_network, lanelet_network_id, obstacles, collision_objects):
        self.dt = dt
        self.lanelet_network = lanelet_network
        self.lanelet_network_id = lanelet_network_id
        self.obstacles = obstacles
        self.collision_objects = collision_objects
    def commonroad_scenario_at_time_step(self, time_step, ego_vehids, neighbor_vehids):
        cr_scenario = Scenario(self.dt, '1')
        cr_scenario.lanelet_network = self.lanelet_network
        obstacles = []
        ego_vehs = []
        nei_vehs = []
        for veh_id, obs in self.obstacles[time_step].items():
            cobs = deepcopy(obs)
            cobs.initial_state.time_step = 0
            if veh_id in ego_vehids:
                ego_vehs.append(cobs)
            elif veh_id in neighbor_vehids:
                nei_vehs.append(cobs)
            else:
                obstacles.append(cobs)
        cr_scenario.add_objects(obstacles)
        return cr_scenario, ego_vehs, nei_vehs

    def _get_cr_obstacles(time_step):
        obstacles = []
        for veh_id, obs in self.obstacles[time_step].items():
            obstacles.append(obs)
        return obstacles
    def get_attribute(self, item, veh_id, time_step):
        if item == LANECURVATURE:
            return self.get_lane_curvature(veh_id, time_step)
        if item == LINE_MARKING_LEFT_DIST:
            return self.get_lane_marking_left_distance(veh_id, time_step)
        if item == LINE_MARKING_RIGHT_DIST:
            return self.get_lane_marking_right_distance(veh_id, time_step)
        if item == TIMEGAP:
            return self.get_time_gap(veh_id, time_step)
        if item == TTC:
            return self.get_inverse_time_to_collision(veh_id, time_step)
        if item == NEIGHBOR_FORE_ALONG_LANE:
            return self.get_front_neighbor_along_lane(veh_id, time_step, max_range=150.0, max_veh=2)
        if item == NEIGHBOR_FORE_ALONG_LEFT_LANE:
            return self.get_front_neighbor_along_left_lane(veh_id, time_step, max_range=150.0, max_veh=2)
        if item == NEIGHBOR_FORE_ALONG_RIGHT_LANE:
            return self.get_front_neighbor_along_right_lane(veh_id, time_step, max_range=150.0, max_veh=2)
        if item == NEIGHBOR_REAR_ALONG_LANE:
            return self.get_rear_neighbor_along_lane(veh_id, time_step, max_range=50.0, max_veh=1)
        if item == NEIGHBOR_REAR_ALONG_LEFT_LANE:
            return self.get_rear_neighbor_along_left_lane(veh_id, time_step, max_range=50.0, max_veh=1)
        if item == NEIGHBOR_REAR_ALONG_RIGHT_LANE:
            return self.get_rear_neighbor_along_right_lane(veh_id, time_step, max_range=50.0, max_veh=1)
        if item == NEIGHBOR_FORE_REAR_LEFT_MID_RIGHT:
            return self.get_fore_rear_left_mid_right(veh_id, time_step)

        if item == IS_COLLIDING:
            return self.is_colliding(veh_id, time_step)
        if item == ROAD_DIST_LEFT:
            return self.get_left_road_distance(veh_id, time_step)
        if item == ROAD_DIST_RIGHT:
            return self.get_right_road_distance(veh_id, time_step)

    def get_vector_between_cars(self, vehicle_id, veh_id, startframe, along_lanelet=False):
        if vehicle_id not in self.obstacles[startframe] or veh_id not in self.obstacles[startframe]:
            return float("inf"), float("inf")
        ego = self.obstacles[startframe][vehicle_id]
        ego_lane_id = ego.initial_state.posF.ind[1]
        ego_lane = self.lanelet_network.find_lanelet_by_id(ego_lane_id)
        veh = self.obstacles[startframe][veh_id]
        if along_lanelet:

            veh_lane_id = veh.initial_state.posF.ind[1]
            veh_lane = self.lanelet_network.find_lanelet_by_id(ego_lane_id)
            if ego_lane_id == veh_lane_id:
                delta_d = veh.initial_state.posF.d - ego.initial_state.posF.d
                delta_s = veh.initial_state.posF.s - ego.initial_state.posF.s
            else:
                posG = veh.initial_state.get_posG()
                posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_lane_id]), self.lanelet_network)
                max_ind = len(ego_lane.center_curve)-2
                if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0 and len(ego_lane.predecessor) > 0:
                    # rematch
                    dist_s = 0
                    current_lane = ego_lane
                    while len(current_lane.predecessor) > 0:
                        posF_adjust = Frenet(posG.projF(self.lanelet_network, [current_lane.predecessor[0]]), self.lanelet_network)
                        predecessor_lane = self.lanelet_network.find_lanelet_by_id(current_lane.predecessor[0])
                        dist_s += predecessor_lane.center_curve[-1].s
                        if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0:
                            current_lane = predecessor_lane
                        else:
                            break
                    #cands = []
                    #for pred_id in ego_lane.predecessor:
                    #    if ego_lane.predecessor_connections[pred_id][0].i == 0:
                    #        cands.append(pred_id)
                    #posF_adjust = Frenet(posG.projF(self.lanelet_network, cands), self.lanelet_network)
                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                    delta_s = posF_adjust.s - (ego.initial_state.posF.s + dist_s)
                    #delta_s = posF_adjust.s - (ego.initial_state.posF.s + self.lanelet_network.find_lanelet_by_id(posF_adjust.ind[1]).center_curve[-1].s)
                elif posF_adjust.ind[0].i >= max_ind and posF_adjust.ind[0].t == 1.0 and len(ego_lane.successor) > 0:
                    #rematch
                    dist_s = 0
                    current_lane = ego_lane
                    while len(current_lane.successor) > 0:
                        posF_adjust = Frenet(posG.projF(self.lanelet_network, [current_lane.successor[0]]), self.lanelet_network)
                        successor_lane = self.lanelet_network.find_lanelet_by_id(current_lane.successor[0])
                        dist_s += current_lane.center_curve[-1].s
                        if posF_adjust.ind[0].i >= len(successor_lane.center_curve)-2 and posF_adjust.ind[0].t == 1.0:
                            current_lane = successor_lane
                        else:
                            break
                    #cands = []
                    #for succ_id in ego_lane.successor:
                    #    if ego_lane.successor_connections[succ_id][0].i >= max_ind-1:
                    #        cands.append(succ_id)

                    #posF_adjust = Frenet(posG.projF(self.lanelet_network, cands), self.lanelet_network)
                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                    #delta_s = posF_adjust.s + ego_lane.center_curve[-1].s - ego.initial_state.posF.s
                    delta_s = posF_adjust.s + dist_s - ego.initial_state.posF.s
                else:
                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                    delta_s = posF_adjust.s - ego.initial_state.posF.s
            return delta_s, delta_d
        else:
            #TODO instant not along lanelet
            curve_ind, lanelet_id = ego.initial_state.posF.ind
            curve = ego_lane.center_curve
            instant_curvePt = lerp_curve(curve[curve_ind.i], curve[curve_ind.i+1], curve_ind.t)
            veh_posG = veh.initial_state.get_posG()
            ego_posG = ego.initial_state.get_posG()
            F = veh_posG.inertial2body(ego_posG)
            delta_s, delta_d = F.x, F.y
            #delta_s, delta_d = veh_posG.projCurvePt(instant_curvePt)
            return delta_s, delta_d


    def get_lane_curvature(self, veh_id, time_step):
        vehicle = self.obstacles[time_step][veh_id]
        curve_ind, lanelet_id = vehicle.initial_state.posF.ind
        curvept = self.lanelet_network.find_lanelet_by_id(lanelet_id).get_curvePt_by_curveid(curve_ind)
        val = curvept.k
        return val
    def get_lane_marking_left_distance(self, veh_id, time_step):
        vehicle = self.obstacles[time_step][veh_id]
        curve_ind, lanelet_id = vehicle.initial_state.posF.ind
        offset = vehicle.initial_state.posF.d
        width = 3.2
        val = width*0.5 - offset
        return val
    def get_lane_marking_right_distance(self, veh_id, time_step):
        vehicle = self.obstacles[time_step][veh_id]
        curve_ind, lanelet_id = vehicle.initial_state.posF.ind
        offset = vehicle.initial_state.posF.d
        width = 3.2
        val = width*0.5 + offset
        return val
    def get_time_gap(self, veh_id, time_step):
        front_neighbor, front_neighbor_delta_s,_ = self.get_front_neighbor_along_lane(veh_id, time_step, max_range=250.0, max_veh=1)

        vel = self.obstacles[time_step][veh_id].initial_state.velocity

        if vel <= 0.0 or front_neighbor[0] is None:
            return 20.0, 0.0
        else:
            #len_ego = self.obstacles[time_step][veh_id].obstacle_shape.length
            #len_oth = self.obstacles[time_step][front_neighbor].obstacle_shape.length
            delta_s = front_neighbor_delta_s[0] #- len_oth #TODO check this
            if delta_s > 0.0:
                return delta_s / vel, 1.0
            else:
                return 0.0, 1.0
    def get_inverse_time_to_collision(self, veh_id, time_step):
        front_neighbor, front_neighbor_delta_s, _ = self.get_front_neighbor_along_lane(veh_id, time_step, max_range=250.0, max_veh=1)
        if front_neighbor[0] is None:
            return 0.0, 0.0
        else:
            veh_fore = self.obstacles[time_step][veh_id]
            veh_rear = self.obstacles[time_step][front_neighbor[0]]

            #len_fore = veh_fore.obstacle_shape.length
            #len_rear = veh_rear.obstacle_shape.length
            delta_s = front_neighbor_delta_s[0]# - len_fore

            delta_v = veh_fore.initial_state.velocity - veh_rear.initial_state.velocity

            if delta_s < 0.0:
                return 20.0, 1.0
            elif delta_v > 0.0:
                return 0.0, 0.0
            else:
                f = -delta_v/delta_s
                return f, 1.0
    def is_colliding(self, ego_id, time_step):
        #ego_posG = self.obstacles[time_step][ego_id].initial_state.get_posG()
        ego = self.obstacles[time_step][ego_id]
        ego_r = np.sqrt(ego.obstacle_shape.length**2 + (0.5*ego.obstacle_shape.width)**2)

        for veh_id, veh in self.obstacles[time_step].items():
            if veh_id != ego_id:
                #veh_posG = veh.initial_state.get_posG()
                veh_r = np.sqrt(veh.obstacle_shape.length**2 + (0.5*veh.obstacle_shape.width)**2)
                delta_pos = ego.initial_state.position-veh.initial_state.position
                dist2 = delta_pos[0]**2 + delta_pos[1]**2
                if dist2 <= ego_r**2 + veh_r**2 + 2*ego_r*veh_r:
                    if self.collision_objects[time_step][ego_id].collide(self.collision_objects[time_step][veh_id]):
                        return True
        return False

    def get_left_road_distance(self, veh_id, time_step):
        # the distance to left road boundary
        veh = self.obstacles[time_step][veh_id]
        offset = veh.initial_state.posF.d
        footpoint = veh.initial_state.get_footpoint()
        lane = self.lanelet_network.find_lanelet_by_id(veh.initial_state.posF.ind[1])
        while lane.adj_left is not None:
            lane = self.lanelet_network.find_lanelet_by_id(lane.adj_left)
        roadproj = veh.initial_state.get_posG().projF(self.lanelet_network, [lane.lanelet_id])
        curvept = lane.get_curvePt_by_curveid(roadproj.curveproj.ind)
        return 3.2*0.5 + np.sqrt((curvept.pos.x-footpoint.x)**2+(curvept.pos.y-footpoint.y)**2) - offset

    def get_right_road_distance(self, veh_id, time_step):
        # the distance to right road boundary
        veh = self.obstacles[time_step][veh_id]
        offset = veh.initial_state.posF.d
        footpoint = veh.initial_state.get_footpoint()
        lane = self.lanelet_network.find_lanelet_by_id(veh.initial_state.posF.ind[1])
        while lane.adj_right is not None:
            lane = self.lanelet_network.find_lanelet_by_id(lane.adj_right)
        roadproj = veh.initial_state.get_posG().projF(self.lanelet_network, [lane.lanelet_id])
        curvept = lane.get_curvePt_by_curveid(roadproj.curveproj.ind)
        return 3.2*0.5 + np.sqrt((curvept.pos.x-footpoint.x)**2+(curvept.pos.y-footpoint.y)**2) + offset


    def get_rear_neighbor_along_right_lane(self, ego_id, time_step, max_range=250.0, max_veh=1):
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_range for _ in range(max_veh)]
        best_offsets = [None for _ in range(max_veh)]

        ego_vehicle = self.obstacles[time_step][ego_id]
        ego_lanelet_id = ego_vehicle.initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
        right_lane_id = None
        if ego_lanelet.adj_right is not None:
            right_lane_id = ego_lanelet.adj_right
        else:
            if self.lanelet_network_id == "i80" and ego_lanelet_id == 101:
                # i - 80
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [201]), self.lanelet_network) # magic, i just know!
                if posF_try.d <= 150:
                    right_lane_id = 201
            elif self.lanelet_network_id == "u101" and ego_lanelet_id == 401:
                # u- 101
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [501]), self.lanelet_network) # magic, i just know!
                if posF_try.d <= 150:
                    right_lane_id = 501
        if right_lane_id is not None:
            target_lanelet_id = start_lanelet_id  = right_lane_id
            ego_target_lanelet_id = ego_start_lanelet_id  = ego_lanelet_id
            #posG = ego_vehicle.initial_state.get_posG()
            posF_base = ego_vehicle.initial_state.posF #Frenet(posG.projF(self.lanelet_network, [target_lanelet_id]), self.lanelet_network)
            s_base = posF_base.s + self.get_targetpoint_delta("front_center", ego_vehicle, posF_base)

            dist_searched = 0.0
            while dist_searched < max_range:
                target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
                ego_target_lanelet = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id)
                for veh_id, veh in self.obstacles[time_step].items():
                    if veh_id != ego_id:
                        s_adjust = None
                        posF_adjust = None
                        if veh.initial_state.posF.ind[1] == target_lanelet_id:
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        if s_adjust is not None:
                            s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist_valid = s_base - s_valid + dist_searched
                            if dist_valid > 0.0:
                                s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                                dist = s_base - s_primary + dist_searched
                                if dist < best_dists[-1]:
                                    for i in range(max_veh):
                                        if dist < best_dists[i]:
                                            for j in range(max_veh-1, i, -1):
                                                best_dists[j] = best_dists[j-1]
                                                best_inds[j] = best_inds[j-1]
                                                best_offsets[j] = best_offsets[j-1]
                                            best_dists[i] = dist
                                            best_inds[i] = veh_id
                                            best_offsets[i] = posF_adjust.d - posF_base.d
                                            break

                if best_inds[-1] is not None: # all neighbors are located
                    break
                if len(ego_target_lanelet.predecessor) == 0 or (ego_target_lanelet_id==ego_start_lanelet_id and dist_searched != 0.0): # no where to search
                    break
                dist_searched += s_base
                s_base = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet.predecessor[0]).center_curve[-1].s #TODO check this

                ego_target_lanelet_id = ego_target_lanelet.predecessor[0]

                target_lanelet_id = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id).adj_right
                if target_lanelet_id is None:
                    break
        return best_inds, best_dists, best_offsets


    def get_front_neighbor_along_right_lane(self, ego_id, time_step, max_range=250.0, max_veh=1):
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_range for _ in range(max_veh)]
        best_offsets = [max_range for _ in range(max_veh)]

        ego_vehicle = self.obstacles[time_step][ego_id]
        ego_lanelet_id = ego_vehicle.initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
        right_lane_id = None
        if ego_lanelet.adj_right is not None:
            right_lane_id = ego_lanelet.adj_right
        else:
            if self.lanelet_network_id == "i80" and ego_lanelet_id == 101:
                # i - 80
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [201]), self.lanelet_network) # magic, i just know!
                if posF_try.d <= 20:
                    right_lane_id = 201
            elif self.lanelet_network_id == "u101" and ego_lanelet_id == 401:
                # u- 101
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [501]), self.lanelet_network) # magic, i just know!
                if posF_try.d <= 20:
                    right_lane_id = 501
        if right_lane_id is not None:
            target_lanelet_id = start_lanelet_id  = right_lane_id
            ego_target_lanelet_id = ego_start_lanelet_id  = ego_lanelet_id
            posF_base = ego_vehicle.initial_state.posF#Frenet(posG.projF(self.lanelet_network, [target_lanelet_id]), self.lanelet_network)
            s_base = posF_base.s + self.get_targetpoint_delta("front_center", ego_vehicle, posF_base)

            dist_searched = 0.0
            while dist_searched < max_range:
                target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
                ego_target_lanelet = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id)
                for veh_id, veh in self.obstacles[time_step].items():
                    if veh_id != ego_id:
                        s_adjust = None
                        posF_adjust = None
                        if veh.initial_state.posF.ind[1] == target_lanelet_id:
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        if s_adjust is not None:
                            s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist_valid = s_valid - s_base + dist_searched
                            if dist_valid >= 0.0:
                                s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                                dist = s_primary - s_base + dist_searched
                                if dist < best_dists[-1]:
                                    for i in range(max_veh):
                                        if dist < best_dists[i]:
                                            for j in range(max_veh-1, i, -1):
                                                best_dists[j] = best_dists[j-1]
                                                best_inds[j] = best_inds[j-1]
                                                best_offsets[j] = best_offsets[j-1]
                                            best_dists[i] = dist
                                            best_inds[i] = veh_id
                                            best_offsets[i] = posF_adjust.d - posF_base.d
                                            break

                if best_inds[-1] is not None: # all neighbors are located
                    break
                if len(ego_target_lanelet.successor) == 0 or (ego_target_lanelet_id==ego_start_lanelet_id and dist_searched != 0.0): # no where to search
                    break
                dist_searched += ego_target_lanelet.center_curve[-1].s - s_base
                s_base = 0.0
                ego_target_lanelet_id = ego_target_lanelet.successor[0]

                target_lanelet_id = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id).adj_right
                if target_lanelet_id is None:
                    break
        return best_inds, best_dists, best_offsets



    def get_rear_neighbor_along_left_lane(self, ego_id, time_step, max_range=250.0, max_veh=1):
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_range for _ in range(max_veh)]
        best_offsets = [None for _ in range(max_veh)]

        ego_vehicle = self.obstacles[time_step][ego_id]
        ego_lanelet_id = ego_vehicle.initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)

        left_lane_id = None
        if ego_lanelet.adj_left is not None:
            left_lane_id = ego_lanelet.adj_left
        else:
            if self.lanelet_network_id == "i80" and ego_lanelet_id == 201:
                # i - 80
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [101]), self.lanelet_network) # magic, i just know!
                if np.abs(posF_try.d) <= 20:
                    left_lane_id = 101
            elif self.lanelet_network_id == "u101" and ego_lanelet_id == 501:
                # u- 101
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [401]), self.lanelet_network) # magic, i just know!
                if np.abs(posF_try.d) <= 20:
                    left_lane_id = 401

        if left_lane_id is not None:
            target_lanelet_id = start_lanelet_id  = left_lane_id
            ego_target_lanelet_id = ego_start_lanelet_id = ego_lanelet_id
            #posG = ego_vehicle.initial_state.get_posG()
            posF_base = ego_vehicle.initial_state.posF#Frenet(posG.projF(self.lanelet_network, [target_lanelet_id]), self.lanelet_network)
            s_base = posF_base.s + self.get_targetpoint_delta("front_center", ego_vehicle, posF_base)

            dist_searched = 0.0
            while dist_searched < max_range:
                target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
                ego_target_lanelet = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id)
                for veh_id, veh in self.obstacles[time_step].items():
                    if veh_id != ego_id:
                        s_adjust = None
                        posF_adjust = None
                        if veh.initial_state.posF.ind[1] == target_lanelet_id:
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        if s_adjust is not None:
                            s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist_valid = s_base - s_valid + dist_searched
                            if dist_valid >= 0.0:
                                s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                                dist = s_base - s_primary + dist_searched
                                if dist < best_dists[-1]:
                                    for i in range(max_veh):
                                        if dist < best_dists[i]:
                                            for j in range(max_veh-1, i, -1):
                                                best_dists[j] = best_dists[j-1]
                                                best_inds[j] = best_inds[j-1]
                                                best_offsets[j] = best_offsets[j-1]
                                            best_dists[i] = dist
                                            best_inds[i] = veh_id
                                            best_offsets[i] = posF_adjust.d - posF_base.d
                                            break

                if best_inds[-1] is not None: # all neighbors are located
                    break
                if len(ego_target_lanelet.predecessor) == 0 or (ego_target_lanelet_id==ego_start_lanelet_id and dist_searched != 0.0): # no where to search
                    break
                dist_searched += s_base
                s_base = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet.predecessor[0]).center_curve[-1].s #TODO check this
                ego_target_lanelet_id = ego_target_lanelet.predecessor[0]

                target_lanelet_id = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id).adj_left
                if target_lanelet_id is None:
                    break

        if self.lanelet_network_id == "i80" and ego_lanelet_id == 201:
            # some more magic
            posG = ego_vehicle.initial_state.get_posG()
            posF_try  = Frenet(posG.projF(self.lanelet_network, [101]), self.lanelet_network) # magic, i just know!
            if np.abs(posF_try.d) <= 2.0:
                left_lane_id = 102

                target_lanelet_id = start_lanelet_id  = left_lane_id
                ego_target_lanelet_id = ego_start_lanelet_id = 101

                posF_base = posF_try
                s_base = posF_base.s + self.get_targetpoint_delta("front_center", ego_vehicle, posF_base)

                dist_searched = 0.0
                target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
                ego_target_lanelet = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id)
                for veh_id, veh in self.obstacles[time_step].items():
                    if veh_id != ego_id:
                        s_adjust = None
                        posF_adjust = None
                        if veh.initial_state.posF.ind[1] == target_lanelet_id:
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        if s_adjust is not None:
                            s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist_valid = s_base - s_valid + dist_searched
                            if dist_valid >= 0.0:
                                s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                                dist = s_base - s_primary + dist_searched
                                if dist < best_dists[-1]:
                                    for i in range(max_veh):
                                        if dist < best_dists[i]:
                                            for j in range(max_veh-1, i, -1):
                                                best_dists[j] = best_dists[j-1]
                                                best_inds[j] = best_inds[j-1]
                                                best_offsets[j] = best_offsets[j-1]
                                            best_dists[i] = dist
                                            best_inds[i] = veh_id
                                            best_offsets[i] = posF_adjust.d - posF_base.d
                                            break


        return best_inds, best_dists, best_offsets

    def get_front_neighbor_along_left_lane(self, ego_id, time_step, max_range=250.0, max_veh=1):
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_range for _ in range(max_veh)]
        best_offsets = [None for _ in range(max_veh)]

        ego_vehicle = self.obstacles[time_step][ego_id]
        ego_lanelet_id = ego_vehicle.initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)

        left_lane_id = None
        if ego_lanelet.adj_left is not None:
            left_lane_id = ego_lanelet.adj_left
        else:
            if self.lanelet_network_id == "i80" and ego_lanelet_id == 201:
                # i - 80
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [101]), self.lanelet_network) # magic, i just know!
                if np.abs(posF_try.d) <= 20:
                    left_lane_id = 101
            elif self.lanelet_network_id == "u101" and ego_lanelet_id == 501:
                # u- 101
                posG = ego_vehicle.initial_state.get_posG()
                posF_try  = Frenet(posG.projF(self.lanelet_network, [401]), self.lanelet_network) # magic, i just know!
                if np.abs(posF_try.d) <= 20:
                    left_lane_id = 401

        if left_lane_id is not None:
            target_lanelet_id = start_lanelet_id  = left_lane_id
            ego_target_lanelet_id = ego_start_lanelet_id = ego_lanelet_id
            posG = ego_vehicle.initial_state.get_posG()
            posF_base = ego_vehicle.initial_state.posF
            #posF_base = Frenet(posG.projF(self.lanelet_network, [target_lanelet_id]), self.lanelet_network)
            s_base = posF_base.s + self.get_targetpoint_delta("front_center", ego_vehicle, posF_base)

            dist_searched = 0.0
            while dist_searched < max_range:
                target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
                ego_target_lanelet = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id)
                for veh_id, veh in self.obstacles[time_step].items():
                    if veh_id != ego_id:
                        s_adjust = None
                        posF_adjust = None
                        if veh.initial_state.posF.ind[1] == target_lanelet_id:
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        if s_adjust is not None:
                            s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist_valid = s_valid - s_base + dist_searched
                            if dist_valid >= 0.0:
                                s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                                dist = s_primary - s_base + dist_searched
                                if dist < best_dists[-1]:
                                    for i in range(max_veh):
                                        if dist < best_dists[i]:
                                            for j in range(max_veh-1, i, -1):
                                                best_dists[j] = best_dists[j-1]
                                                best_inds[j] = best_inds[j-1]
                                                best_offsets[j] = best_offsets[j-1]
                                            best_dists[i] = dist
                                            best_inds[i] = veh_id
                                            best_offsets[i] = posF_adjust.d - posF_base.d
                                            break

                if best_inds[-1] is not None: # all neighbors are located
                    break
                if len(ego_target_lanelet.successor) == 0 or (ego_target_lanelet_id==ego_start_lanelet_id and dist_searched != 0.0): # no where to search
                    break
                dist_searched += ego_target_lanelet.center_curve[-1].s - s_base
                s_base = 0.0
                ego_target_lanelet_id = ego_target_lanelet.successor[0]

                target_lanelet_id = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id).adj_left
                if target_lanelet_id is None:
                    break
        if self.lanelet_network_id == "i80" and ego_lanelet_id == 201:
            # some more magic
            posG = ego_vehicle.initial_state.get_posG()
            posF_try  = Frenet(posG.projF(self.lanelet_network, [101]), self.lanelet_network) # magic, i just know!
            if np.abs(posF_try.d) <= 2.0:
                left_lane_id = 102

                target_lanelet_id = start_lanelet_id  = left_lane_id
                ego_target_lanelet_id = ego_start_lanelet_id = 101

                posF_base = posF_try
                s_base = posF_base.s + self.get_targetpoint_delta("front_center", ego_vehicle, posF_base)

                dist_searched = 0.0
                target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
                ego_target_lanelet = self.lanelet_network.find_lanelet_by_id(ego_target_lanelet_id)
                for veh_id, veh in self.obstacles[time_step].items():
                    if veh_id != ego_id:
                        s_adjust = None
                        posF_adjust = None
                        if veh.initial_state.posF.ind[1] == target_lanelet_id:
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)

                        elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                            s_adjust = 0.0
                            posG = veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [ego_target_lanelet_id]), self.lanelet_network)
                        if s_adjust is not None:
                            s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist_valid = s_valid - s_base + dist_searched
                            if dist_valid >= 0.0:
                                s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                                dist = s_primary - s_base + dist_searched
                                if dist < best_dists[-1]:
                                    for i in range(max_veh):
                                        if dist < best_dists[i]:
                                            for j in range(max_veh-1, i, -1):
                                                best_dists[j] = best_dists[j-1]
                                                best_inds[j] = best_inds[j-1]
                                                best_offsets[j] = best_offsets[j-1]
                                            best_dists[i] = dist
                                            best_inds[i] = veh_id
                                            best_offsets[i] = posF_adjust.d - posF_base.d
                                            break


        return best_inds, best_dists, best_offsets

    def get_rear_neighbor_along_lane(self, ego_id, time_step, max_range=250.0, max_veh=1):
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_range for _ in range(max_veh)]
        best_offsets = [None for _ in range(max_veh)]
        ego_vehicle = self.obstacles[time_step][ego_id]
        target_lanelet_id = start_lanelet_id = ego_vehicle.initial_state.posF.ind[1]
        s_base = ego_vehicle.initial_state.posF.s + self.get_targetpoint_delta("front_center", ego_vehicle, ego_vehicle.initial_state.posF)

        dist_searched = 0.0
        while dist_searched < max_range:
            target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
            for veh_id, veh in self.obstacles[time_step].items():
                if veh_id != ego_id:
                    s_adjust = None
                    posF_adjust = None
                    if veh.initial_state.posF.ind[1] == target_lanelet_id:
                        s_adjust = 0.0
                        posF_adjust = veh.initial_state.posF
                    elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                        s_adjust = 0.0
                        posG = veh.initial_state.get_posG()
                        posF_adjust = Frenet(posG.projF(self.lanelet_network, [target_lanelet.lanelet_id]), self.lanelet_network)

                    elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                        s_adjust = 0.0
                        posG = veh.initial_state.get_posG()
                        posF_adjust = Frenet(posG.projF(self.lanelet_network, [target_lanelet.lanelet_id]), self.lanelet_network)
                    if s_adjust is not None:
                        s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                        dist_valid = s_base - s_valid + dist_searched
                        if dist_valid >= 0.0:
                            s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist = s_base - s_primary + dist_searched
                            if dist < best_dists[-1]:
                                for i in range(max_veh):
                                    if dist < best_dists[i]:
                                        for j in range(max_veh-1, i, -1):
                                            best_dists[j] = best_dists[j-1]
                                            best_inds[j] = best_inds[j-1]
                                            best_offsets[j] = best_offsets[j-1]
                                        best_dists[i] = dist
                                        best_inds[i] = veh_id
                                        best_offsets[i] = posF_adjust.d - ego_vehicle.initial_state.posF.d
                                        break

            if best_inds[-1] is not None: # all neighbors are located
                break
            if len(target_lanelet.predecessor) == 0 or (target_lanelet_id==start_lanelet_id and dist_searched != 0.0): # no where to search
                break
            dist_searched += s_base
            s_base = self.lanelet_network.find_lanelet_by_id(target_lanelet.predecessor[0]).center_curve[-1].s
            target_lanelet_id = target_lanelet.predecessor[0]
        return best_inds, best_dists, best_offsets


    def get_front_neighbor_along_lane(self, ego_id, time_step, max_range=250.0, max_veh=1):
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_range for _ in range(max_veh)]
        best_offsets = [None for _ in range(max_veh)]
        ego_vehicle = self.obstacles[time_step][ego_id]
        target_lanelet_id = start_lanelet_id = ego_vehicle.initial_state.posF.ind[1]
        s_base = ego_vehicle.initial_state.posF.s + self.get_targetpoint_delta("front_center", ego_vehicle, ego_vehicle.initial_state.posF)

        dist_searched = 0.0
        while dist_searched < max_range:
            target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)
            for veh_id, veh in self.obstacles[time_step].items():
                if veh_id != ego_id:
                    s_adjust = None
                    posF_adjust = None
                    if veh.initial_state.posF.ind[1] == target_lanelet_id:
                        s_adjust = 0.0
                        posF_adjust = veh.initial_state.posF
                    elif self.is_in_entrances(veh, target_lanelet): # when there is car on lanelet linking to target lanelet
                        s_adjust = 0.0
                        posG = veh.initial_state.get_posG()
                        posF_adjust = Frenet(posG.projF(self.lanelet_network, [target_lanelet.lanelet_id]), self.lanelet_network)

                    elif self.is_in_exits(veh, target_lanelet): # when there is car on lanelet linking from target lanelet
                        s_adjust = 0.0
                        posG = veh.initial_state.get_posG()
                        posF_adjust = Frenet(posG.projF(self.lanelet_network, [target_lanelet.lanelet_id]), self.lanelet_network)

                    if s_adjust is not None:
                        s_valid = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                        dist_valid = s_valid - s_base + dist_searched
                        #print("dist ", dist_valid)
                        if dist_valid >= 0.0:
                            s_primary = posF_adjust.s + self.get_targetpoint_delta("front_center", veh, posF_adjust)
                            dist = s_primary - s_base + dist_searched

                            if dist < best_dists[-1]:
                                for i in range(max_veh):
                                    if dist < best_dists[i]:
                                        for j in range(max_veh-1, i, -1):
                                            best_dists[j] = best_dists[j-1]
                                            best_inds[j] = best_inds[j-1]
                                            best_offsets[j] = best_offsets[j-1]
                                        best_dists[i] = dist
                                        best_inds[i] = veh_id
                                        best_offsets[i] = posF_adjust.d - ego_vehicle.initial_state.posF.d
                                        break

            if best_inds[-1] is not None: # all neighbors are located
                break
            if len(target_lanelet.successor) == 0 or (target_lanelet_id==start_lanelet_id and dist_searched != 0.0) or (self.lanelet_network_id == "i80" and target_lanelet_id==201): # no where to search
                break
            dist_searched += target_lanelet.center_curve[-1].s - s_base
            s_base = 0.0
            target_lanelet_id = target_lanelet.successor[0]
        return best_inds, best_dists, best_offsets
    def is_in_entrances(self, veh, target_lanelet):
        # convert a vehicle at the entrance of target_lanelet to this target_lanelet
        veh_curve_ind, veh_lanelet_id = veh.initial_state.posF.ind
        veh_lanelet = self.lanelet_network.find_lanelet_by_id(veh_lanelet_id)
        if veh_lanelet_id in target_lanelet.predecessor:
            con = target_lanelet.predecessor_connections[veh_lanelet_id]
            t_x, t_y = target_lanelet.center_curve[0].pos.x, target_lanelet.center_curve[0].pos.y
            v_x, v_y = veh_lanelet.center_curve[-1].pos.x, target_lanelet.center_curve[-1].pos.y
            con_dist = np.sqrt((t_x-v_x)**2 + (t_y-v_y)**2)
            if veh_curve_ind.i >= con[1].i and veh_curve_ind.t == 1.0 and con_dist >= 0.0:#if np.abs(veh_curve_ind.i - con[0].i) <= 5:
                # need to project to check
                posG = veh.initial_state.get_posG()
                projF = posG.projF(self.lanelet_network, [target_lanelet.lanelet_id])
                if np.abs(projF.curveproj.d) <= 3.2*0.5:
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False
    def is_in_exits(self, veh, target_lanelet):
        # convert a vehicle at the exit of target_lanelet to this target_lanelet
        veh_curve_ind, veh_lanelet_id = veh.initial_state.posF.ind
        veh_lanelet = self.lanelet_network.find_lanelet_by_id(veh_lanelet_id)
        if veh_lanelet_id in target_lanelet.successor:
            con = target_lanelet.successor_connections[veh_lanelet_id]
            t_x, t_y = target_lanelet.center_curve[-1].pos.x, target_lanelet.center_curve[-1].pos.y
            v_x, v_y = veh_lanelet.center_curve[0].pos.x, target_lanelet.center_curve[0].pos.y
            con_dist = np.sqrt((t_x-v_x)**2 + (t_y-v_y)**2)
            if veh_curve_ind.i == 0 and veh_curve_ind.t==0.0 and con_dist >=0.0:#np.abs(veh_curve_ind.i - con[0].i) <= 5:
                # need to project to check
                posG = veh.initial_state.get_posG()
                projF = posG.projF(self.lanelet_network, [target_lanelet.lanelet_id])
                if np.abs(projF.curveproj.d) <= 3.2*0.5:
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False
    def get_targetpoint_delta(self, ref, veh, posF):
        if ref == "front_center":
            return 0.0
        else:
            return -veh.obstacle_shape.length*np.cos(posF.phi)
    '''
    def get_neighbors(self, ego_id, max_radius, max_veh, startframe):
        #print(ego_id)
        #print(startframe)
        #print(self.obstacles[startframe].keys())
        ego_posG = self.obstacles[startframe][ego_id].initial_state.get_posG()
        best_inds = [None for _ in range(max_veh)]
        best_dists = [max_radius**2 for _ in range(max_veh)]
        for veh_id, veh in self.obstacles[startframe].items():
            if veh_id != ego_id:
                veh_posG = self.obstacles[startframe][veh_id].initial_state.get_posG()
                distance2 = (veh_posG.x-ego_posG.x)**2 + (veh_posG.y-ego_posG.y)**2
                if distance2 < best_dists[-1]:
                    for i in range(max_veh):
                        if distance2 < best_dists[i]:
                            for j in range(max_veh-1, i, -1):
                                best_dists[j] = best_dists[j-1]
                                best_inds[j] = best_inds[j-1]
                            best_dists[i] = distance2
                            best_inds[i] = veh_id
                            break
        return best_inds
    '''
    def get_neighbors(self, ego_id, startframe, along_lanelet=False, max_radius=40, front=40, side=30,rear=30):
        #TODO make the reference straight instead of along lanelet
        ego_posG = self.obstacles[startframe][ego_id].initial_state.get_posG()
        ego_lanelet_id = self.obstacles[startframe][ego_id].initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
        best_ids = []
        for veh_id, veh in self.obstacles[startframe].items():
            if veh_id != ego_id:
                veh_posG = self.obstacles[startframe][veh_id].initial_state.get_posG()
                distance2 = (veh_posG.x-ego_posG.x)**2 + (veh_posG.y-ego_posG.y)**2
                if distance2 < max_radius**2:
                    best_ids.append(veh_id)
        final_ids = []
        delta_step = int(1/self.dt)
        for veh_id in best_ids:
            delta_s, delta_d = self.get_vector_between_cars(ego_id, veh_id, startframe, along_lanelet)
            if delta_s <= front and delta_s >= - rear:
                if delta_d <= side and delta_d >= -side:
                    final_ids.append(veh_id)
            #else:
            #    if ego_id in self.obstacles[startframe-delta_step] and veh_id in self.obstacles[startframe-delta_step]:
            #        past_delta_s, past_delta_d = self.get_vector_between_cars(ego_id, veh_id, startframe, along_lanelet)
            #        if past_delta_s <= front+20 and past_delta_s >= - rear-20:
            #            if past_delta_d <= side and past_delta_d >= -side:
            #                final_ids.append(veh_id)

        return final_ids
    def get_fore_rear_left_mid_right(self, ego_id, startframe, max_radius=40, front=40, side=30,rear=30):
        ego_posG = self.obstacles[startframe][ego_id].initial_state.get_posG()
        ego_lanelet_id = self.obstacles[startframe][ego_id].initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
        best_ids = []
        for veh_id, veh in self.obstacles[startframe].items():
            if veh_id != ego_id:
                veh_posG = self.obstacles[startframe][veh_id].initial_state.get_posG()
                distance2 = (veh_posG.x-ego_posG.x)**2 + (veh_posG.y-ego_posG.y)**2
                if distance2 < max_radius**2:
                    best_ids.append(veh_id)
        final_ids = []
        delta_step = int(1/self.dt)
        width = 4.0

        front_id = -100
        front_s = float("inf")
        front_d = -100

        best_left_ids = [-100 for _ in range(3)]
        best_left_s = [float("inf") for _ in range(3)] # closest to furthest
        best_left_d = [-100 for _ in range(3)]

        best_right_ids = [-100 for _ in range(3)]
        best_right_s = [float("inf") for _ in range(3)]
        best_right_d = [-100 for _ in range(3)]
        for veh_id in best_ids:
            delta_s, delta_d = self.get_vector_between_cars(ego_id, veh_id, startframe, along_lanelet=False)
            if delta_s <= front and delta_s >= - rear:
                if delta_d <= width/2 and delta_d >= -width/2 and delta_s >= 0.0:
                     if delta_s < front_s:
                         front_id = veh_id
                         front_s = delta_s
                         front_d = delta_d
                elif delta_d > width/2 and delta_d <= side:
                    if np.abs(delta_s) < np.abs(best_left_s[-1]):
                        for idx in range(len(best_left_s)):
                            if np.abs(delta_s) < np.abs(best_left_s[idx]):
                                best_left_s.insert(idx, delta_s)
                                best_left_s = best_left_s[:-1]
                                best_left_ids.insert(idx, veh_id)
                                best_left_ids = best_left_ids[:-1]
                                best_left_d.insert(idx, delta_d)
                                best_left_d = best_left_d[:-1]
                                break

                elif delta_d < -width/2 and delta_d >= -side:
                    if np.abs(delta_s) < np.abs(best_right_s[-1]):
                        for idx in range(len(best_right_s)):
                            if np.abs(delta_s) < np.abs(best_right_s[idx]):
                                best_right_s.insert(idx, delta_s)
                                best_right_s = best_right_s[:-1]
                                best_right_ids.insert(idx, veh_id)
                                best_right_ids = best_right_ids[:-1]
                                best_right_d.insert(idx, delta_d)
                                best_right_d = best_right_d[:-1]
                                break
        ret = {}
        if front_id != -100:
            delta_s_along_lane, delta_d_along_lane = self.get_vector_between_cars(ego_id, front_id, startframe, along_lanelet=True)
            ret["front"] = (front_id, delta_s_along_lane, delta_d_along_lane)
        else:
            ret["front"] = (-100, -100, -100)

        best_left_s_along_lane = []
        best_left_d_along_lane = []
        for left_veh_id in best_left_ids:
            if left_veh_id != -100:
                delta_s_along_lane, delta_d_along_lane = self.get_vector_between_cars(ego_id, left_veh_id, startframe, along_lanelet=True)
                best_left_s_along_lane.append(delta_s_along_lane)
                best_left_d_along_lane.append(delta_d_along_lane)
            else:
                best_left_s_along_lane.append(-100)
                best_left_d_along_lane.append(-100)
        ret["left"] = (best_left_ids, best_left_s_along_lane, best_left_d_along_lane)

        best_right_s_along_lane = []
        best_right_d_along_lane = []
        for right_veh_id in best_right_ids:
            if right_veh_id != -100:
                delta_s_along_lane, delta_d_along_lane = self.get_vector_between_cars(ego_id, right_veh_id, startframe, along_lanelet=True)
                best_right_s_along_lane.append(delta_s_along_lane)
                best_right_d_along_lane.append(delta_d_along_lane)
            else:
                best_right_s_along_lane.append(-100)
                best_right_d_along_lane.append(-100)
        ret["right"] = (best_right_ids, best_right_s_along_lane, best_right_d_along_lane)
        return ret
    '''
    def get_neighbors(self, ego_id, startframe, max_radius=120, front=120, side=40, rear=80):
        ego_posG = self.obstacles[startframe][ego_id].initial_state.get_posG()
        best_ids = []
        for veh_id, veh in self.obstacles[startframe].items():
            if veh_id != ego_id:
                veh_posG = self.obstacles[startframe][veh_id].initial_state.get_posG()
                distance2 = (veh_posG.x-ego_posG.x)**2 + (veh_posG.y-ego_posG.y)**2
                if distance2 < max_radius**2:
                    best_ids.append(veh_id)
        final_ids = []
        for veh_id in best_ids:
            delta_s, delta_d = self.get_vector_between_cars(ego_id, veh_id, startframe)
            if delta_s <= front and delta_s >= - rear:
                if delta_d <= side and delta_d >= -side:
                    final_ids.append(veh_id)
        return final_ids
    '''




class CoreFeatureExtractor:
    def __init__(self, delta_step = 10):
        self.delta_step = delta_step
        self.num_features = 10
        self.features = np.zeros(self.num_features, dtype=np.float32)

        self.feature_info = { "egoid":{"high":3000, "low":0},
                              "relative_offset":{"high":2.,    "low":-2.},
                              "velocity_s":{"high":40.,    "low":0.},
                              "velocity_d":{"high":40.,    "low":0.},
                              "length":{"high":30.,    "low":2.},
                              "width":{"high":3.,     "low":.9},
                              "target_delta_s":{"high":40, "low":0},
                              "target_delta_d":{"high":40, "low":0},
                              "target_velocity_s":{"high":40, "low":0},
                              "target_velocity_d":{"high":40, "low":0}
                              }

        self.feature_index = {"egoid":0,
                              "relative_offset":1,
                              "velocity_s":2,
                              "velocity_d":3,
                              "length":4,
                              "width":5,
                              "target_delta_s":6,
                              "target_delta_d":7,
                              "target_velocity_s":8,
                              "target_velocity_d":9
                              }
    def get_delta(self, cr_scenario, vehicle_id, startframe, step):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        current_state = ego_vehicle.initial_state
        current_posF = current_state.posF
        current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_posF.ind[1])
        if vehicle_id in cr_scenario.obstacles[startframe+step].keys():
            target_state = cr_scenario.obstacles[startframe+step][vehicle_id].initial_state

        else:
            ego_vehicle = cr_scenario.obstacles[startframe-step][vehicle_id]
            current_state = ego_vehicle.initial_state
            current_posF = current_state.posF
            current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_posF.ind[1])
            target_state = cr_scenario.obstacles[startframe][vehicle_id].initial_state

        target_posF = target_state.posF
        if target_posF.ind[1] == current_posF.ind[1]:
            delta_s = target_posF.s - current_posF.s
            delta_d = target_posF.d - current_posF.d
        else:
            posG = target_state.get_posG()
            posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [current_posF.ind[1]]), cr_scenario.lanelet_network)
            max_ind = len(current_lane.center_curve)-2
            if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0 and len(current_lane.predecessor) > 0:
                # normally this should not happen
                delta_s = 0.0
                delta_d = posF_adjust.d - current_posF.d
            elif posF_adjust.ind[0].i == max_ind and posF_adjust.ind[0].t == 1.0 and len(current_lane.successor) > 0:
                #TODO In the roundabout, this should be different
                cands = []
                for succ_id in current_lane.successor:
                    if current_lane.successor_connections[succ_id][0].i >= max_ind-1:
                        cands.append(succ_id)
                posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [cands[0]]), cr_scenario.lanelet_network)
                delta_s = posF_adjust.s + current_lane.center_curve[-1].s - current_posF.s
                delta_d = posF_adjust.d - current_posF.d
            else:
                delta_s = posF_adjust.s - current_posF.s
                delta_d = posF_adjust.d - current_posF.d
        return delta_s, delta_d
    def get_features(self, cr_scenario, vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        self.features[0] = vehicle_id
        self.features[1] = ego_vehicle.initial_state.posF.d

        if vehicle_id in cr_scenario.obstacles[startframe-1].keys():
            delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe-1, 2)
            self.features[2] = delta_s/(cr_scenario.dt*2)
            self.features[3] = delta_d/(cr_scenario.dt*2)
        else:
            delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe, 1)
            self.features[2] = delta_s/cr_scenario.dt
            self.features[3] = delta_d/cr_scenario.dt

        self.features[4] = ego_vehicle.obstacle_shape.length
        self.features[5] = ego_vehicle.obstacle_shape.width
        delta_step = int(1/cr_scenario.dt) # should be 10 if dt = 0.1
        if vehicle_id in cr_scenario.obstacles[startframe+delta_step].keys():
            self.features[6],self.features[7] = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step)
        elif vehicle_id in cr_scenario.obstacles[startframe+1].keys():
            i = 1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe+delta_step-i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step-i)
                    self.features[6] = delta_s*delta_step/(delta_step-i)
                    self.features[7] = delta_d*delta_step/(delta_step-i)
                    break
                i += 1
        else:
            i = 0
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe-delta_step+i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe-delta_step+i, delta_step-i)
                    self.features[6] = delta_s*delta_step/(delta_step-i)
                    self.features[7] = delta_d*delta_step/(delta_step-i)
                    break
                i += 1

        if vehicle_id in cr_scenario.obstacles[startframe+delta_step-1].keys():
            delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe+delta_step-1, 2)
            self.features[8] = delta_s/(cr_scenario.dt*2)
            self.features[9] = delta_d/(cr_scenario.dt*2)
        else:
            i = 1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe+delta_step-1-i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe+delta_step-1-i, 2)
                    self.features[8] = delta_s/(cr_scenario.dt*2)
                    self.features[9] = delta_d/(cr_scenario.dt*2)
                    break
                i += 1

        return self.features

class CoreFeatureExtractor1:
    def __init__(self, delta_step = 10):
        self.delta_step = delta_step
        self.num_features = 10
        self.features = np.zeros(self.num_features, dtype=np.float32)

        self.feature_info = { "egoid":{"high":3000, "low":0},
                              "relative_offset":{"high":2.,    "low":-2.},
                              "velocity_s":{"high":40.,    "low":0.},
                              "velocity_d":{"high":40.,    "low":0.},
                              "length":{"high":30.,    "low":2.},
                              "width":{"high":3.,     "low":.9},
                              "target_delta_s":{"high":40, "low":0},
                              "target_delta_d":{"high":40, "low":0},
                              "target_velocity_s":{"high":40, "low":0},
                              "target_velocity_d":{"high":40, "low":0}
                              }

        self.feature_index = {"egoid":0,
                              "relative_offset":1,
                              "velocity_s":2,
                              "velocity_d":3,
                              "length":4,
                              "width":5,
                              "target_delta_s":6,
                              "target_delta_d":7,
                              "target_velocity_s":8,
                              "target_velocity_d":9
                              }

    def get_delta(self, cr_scenario, vehicle_id, startframe, step):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        current_state = ego_vehicle.initial_state
        current_posF = current_state.posF
        current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_posF.ind[1])
        if vehicle_id in cr_scenario.obstacles[startframe+step].keys():
            target_state = cr_scenario.obstacles[startframe+step][vehicle_id].initial_state

        else:
            ego_vehicle = cr_scenario.obstacles[startframe-step][vehicle_id]
            current_state = ego_vehicle.initial_state
            current_posF = current_state.posF
            current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_posF.ind[1])
            target_state = cr_scenario.obstacles[startframe][vehicle_id].initial_state

        target_posF = target_state.posF
        if target_posF.ind[1] == current_posF.ind[1]:
            delta_s = target_posF.s - current_posF.s
            delta_d = target_posF.d - current_posF.d
        else:
            posG = target_state.get_posG()
            posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [current_posF.ind[1]]), cr_scenario.lanelet_network)
            max_ind = len(current_lane.center_curve)-2
            if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0 and len(current_lane.predecessor) > 0:
                # normally this should not happen
                delta_s = 0.0
                delta_d = posF_adjust.d - current_posF.d
            elif posF_adjust.ind[0].i == max_ind and posF_adjust.ind[0].t == 1.0 and len(current_lane.successor) > 0:
                #TODO In the roundabout, this should be different
                cands = []
                for succ_id in current_lane.successor:
                    if current_lane.successor_connections[succ_id][0].i >= max_ind-1:
                        cands.append(succ_id)
                posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [cands[0]]), cr_scenario.lanelet_network)
                delta_s = posF_adjust.s + current_lane.center_curve[-1].s - current_posF.s
                delta_d = posF_adjust.d - current_posF.d
            else:
                delta_s = posF_adjust.s - current_posF.s
                delta_d = posF_adjust.d - current_posF.d
        return delta_s, delta_d

    def get_features(self, cr_scenario, vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        self.features[0] = vehicle_id
        self.features[1] = ego_vehicle.initial_state.posF.d

        self.features[2] = np.cos(ego_vehicle.initial_state.posF.phi)*ego_vehicle.initial_state.velocity
        self.features[3] = np.sin(ego_vehicle.initial_state.posF.phi)*ego_vehicle.initial_state.velocity

        self.features[4] = ego_vehicle.obstacle_shape.length
        self.features[5] = ego_vehicle.obstacle_shape.width
        delta_step = int(1/cr_scenario.dt) # should be 10 if dt = 0.1
        if vehicle_id in cr_scenario.obstacles[startframe+delta_step].keys():
            self.features[6],self.features[7] = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step)
            target_state = cr_scenario.obstacles[startframe+delta_step][vehicle_id].initial_state
            self.features[8] = np.cos(target_state.posF.phi)*target_state.velocity
            self.features[9] = np.sin(target_state.posF.phi)*target_state.velocity
        elif vehicle_id in cr_scenario.obstacles[startframe+1].keys():
            i = 1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe+delta_step-i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step-i)
                    self.features[6] = delta_s*delta_step/(delta_step-i)
                    self.features[7] = delta_d*delta_step/(delta_step-i)

                    target_state = cr_scenario.obstacles[startframe+delta_step-i][vehicle_id].initial_state
                    self.features[8] = np.cos(target_state.posF.phi)*target_state.velocity
                    self.features[9] = np.sin(target_state.posF.phi)*target_state.velocity
                    break
                i += 1
        else:
            i = delta_step-1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe-delta_step+i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe-delta_step+i, delta_step-i)
                    self.features[6] = delta_s*delta_step/(delta_step-i)
                    self.features[7] = delta_d*delta_step/(delta_step-i)

                    target_state = cr_scenario.obstacles[startframe-delta_step+i][vehicle_id].initial_state
                    self.features[8] = np.cos(target_state.posF.phi)*target_state.velocity
                    self.features[9] = np.sin(target_state.posF.phi)*target_state.velocity
                    break
                i -= 1

        return self.features

class CoreFeatureExtractor0:
    def __init__(self):
        self.num_features = 3
        self.features = np.zeros(self.num_features, dtype=np.float32)

        self.feature_info = { "egoid":{"high":3000,    "low":0},
                              "length":{"high":30.,    "low":2.},
                              "width":{"high":3.,     "low":.9},
                              }

        self.feature_index = {"egoid":0,
                              "length":1,
                              "width":2
                              }
    def get_features(self, cr_scenario, vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        self.features[0] = vehicle_id
        self.features[1] = ego_vehicle.obstacle_shape.length
        self.features[2] = ego_vehicle.obstacle_shape.width
        return self.features


class TemporalFeatureExtractor:
    def __init__(self):
        self.num_features = 5
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {"accel" : {"high":9.,    "low":-9.},
                             "timegap" :  {"high":30.,  "low":0.},
                             "timegap_is_avail" :  {"high":1.,   "low":0.},
                             "inv_time_to_collision" : {"high":30.,  "low":0.},
                             "inv_time_to_collision_is_avail" : {"high":1.,   "low":0.}}
        self.feature_index = {"accel" : 0,
                             "timegap" : 1,
                             "timegap_is_avail" : 2,
                             "inv_time_to_collision" : 3,
                             "inv_time_to_collision_is_avail" : 4}

    def get_features(self, cr_scenario, vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        ego_vehicle2 = cr_scenario.obstacles[startframe-1][vehicle_id]
        self.features[0] = (ego_vehicle.initial_state.velocity - ego_vehicle2.initial_state.velocity)/0.1
        self.features[1], self.features[2] = cr_scenario.get_attribute(TIMEGAP, vehicle_id, startframe)
        self.features[3], self.features[4] = cr_scenario.get_attribute(TTC, vehicle_id, startframe)
        return self.features

class NeighborFeatureExtractor:
    def __init__(self):
        self.num_features = 3 * 8
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {}
        self.feature_names = feature_names = ["_veh_id", "_dist", "_offset"]
        feature_ranges = [{"high":2000,"low":1}, {"high":250.0, "low":-100.0}, {"high":4.0, "low":-4.0}]
        self.neighbor_names = neighbor_names = ["front1", "front2", "left_front1", "left_front2", "right_front1", "right_front2",
                          "left_rear1", "right_rear1"]
        self.feature_index = {}
        index = 0
        for nn in neighbor_names:
            for i in range(len(feature_names)):
                self.feature_info[nn+feature_names[i]] = feature_ranges[i]
                self.feature_index[nn+feature_names[i]] = index
                index += 1

    def get_features(self, cr_scenario, vehicle_id, startframe):
        self.features = np.zeros(self.num_features, dtype=np.float32)
        fore_Ms, fore_Mdists, fore_Moffsets = cr_scenario.get_attribute(NEIGHBOR_FORE_ALONG_LANE, vehicle_id, startframe)
        fore_Ls, fore_Ldists, fore_Loffsets = cr_scenario.get_attribute(NEIGHBOR_FORE_ALONG_LEFT_LANE, vehicle_id, startframe)
        fore_Rs, fore_Rdists, fore_Roffsets = cr_scenario.get_attribute(NEIGHBOR_FORE_ALONG_RIGHT_LANE, vehicle_id, startframe)

        rear_Ls, rear_Ldists, rear_Loffsets = cr_scenario.get_attribute(NEIGHBOR_REAR_ALONG_LEFT_LANE, vehicle_id, startframe)
        rear_Rs, rear_Rdists, rear_Roffsets = cr_scenario.get_attribute(NEIGHBOR_REAR_ALONG_RIGHT_LANE, vehicle_id, startframe)
        # neighbors features: rel_pos, vel, length, width, rel_heading

        index = 0
        for i in range(len(fore_Ms)):
            veh_id = fore_Ms[i]
            if veh_id is not None:
                self.features[index] = veh_id
                self.features[index+1] = fore_Mdists[i]
                self.features[index+2] = fore_Moffsets[i]

            else:
                self.features[index:index+3] = -100
            index += 3

        for i in range(len(fore_Ls)):
            veh_id = fore_Ls[i]
            if veh_id is not None:
                self.features[index] = veh_id
                self.features[index+1] = fore_Ldists[i]
                self.features[index+2] = fore_Loffsets[i]

            else:
                self.features[index:index+3] = -100
            index += 3

        for i in range(len(fore_Rs)):
            veh_id = fore_Rs[i]
            if veh_id is not None:
                self.features[index] = veh_id
                self.features[index+1] = fore_Rdists[i]
                self.features[index+2] = fore_Roffsets[i]
            else:
                self.features[index:index+3] = -100
            index += 3

        for i in range(len(rear_Ls)):
            veh_id = rear_Ls[i]
            if veh_id is not None:
                self.features[index] = veh_id
                self.features[index+1] = -rear_Ldists[i]
                self.features[index+2] = rear_Loffsets[i]

            else:
                self.features[index:index+3] = -100
            index += 3

        for i in range(len(rear_Rs)):
            veh_id = rear_Rs[i]
            if veh_id is not None:
                self.features[index] = veh_id
                self.features[index+1] = -rear_Rdists[i]
                self.features[index+2] = rear_Roffsets[i]

            else:
                self.features[index:index+3] = -100
            index += 3

        return self.features

class NeighborFeatureExtractor1:
    def __init__(self):
        self.num_features = 3 * 7
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {}
        self.feature_names = feature_names = ["_veh_id", "_dist", "_offset"]
        feature_ranges = [{"high":2000,"low":1}, {"high":250.0, "low":-100.0}, {"high":4.0, "low":-4.0}]
        self.neighbor_names = neighbor_names = ["front1", "left1", "left2", "left3", "right1", "right2", "right3"]
        self.feature_index = {}
        index = 0
        for nn in neighbor_names:
            for i in range(len(feature_names)):
                self.feature_info[nn+feature_names[i]] = feature_ranges[i]
                self.feature_index[nn+feature_names[i]] = index
                index += 1

    def get_features(self, cr_scenario, vehicle_id, startframe):
        self.features = np.zeros(self.num_features, dtype=np.float32)
        ret = cr_scenario.get_attribute(NEIGHBOR_FORE_REAR_LEFT_MID_RIGHT, vehicle_id, startframe)
        # neighbors features: rel_pos, vel, length, width, rel_heading
        fore_M, fore_Mdist, fore_Moffset = ret["front"]
        veh_id = fore_M
        if veh_id is not None:
            self.features[0] = veh_id
            self.features[1] = fore_Mdist
            self.features[2] = fore_Moffset
        else:
            self.features[0] = -100
            self.features[1] = -100
            self.features[2] = -100
        Ls, Ldists, Loffsets = ret["left"]
        veh_id = Ls[0]
        if veh_id is not None:
            self.features[3] = veh_id
            self.features[4] = Ldists[0]
            self.features[5] = Loffsets[0]
        else:
            self.features[3] = -100
            self.features[4] = -100
            self.features[5] = -100
        veh_id = Ls[1]
        if veh_id is not None:
            self.features[6] = veh_id
            self.features[7] = Ldists[1]
            self.features[8] = Loffsets[1]
        else:
            self.features[6] = -100
            self.features[7] = -100
            self.features[8] = -100
        veh_id = Ls[2]
        if veh_id is not None:
            self.features[9] = veh_id
            self.features[10] = Ldists[2]
            self.features[11] = Loffsets[2]
        else:
            self.features[9] = -100
            self.features[10] = -100
            self.features[11] = -100
        Rs, Rdists, Roffsets = ret["right"]
        veh_id = Rs[0]
        if veh_id is not None:
            self.features[12] = veh_id
            self.features[13] = Rdists[0]
            self.features[14] = Roffsets[0]
        else:
            self.features[12] = -100
            self.features[13] = -100
            self.features[14] = -100
        veh_id = Rs[1]
        if veh_id is not None:
            self.features[15] = veh_id
            self.features[16] = Rdists[1]
            self.features[17] = Roffsets[1]
        else:
            self.features[15] = -100
            self.features[16] = -100
            self.features[17] = -100
        veh_id = Rs[2]
        if veh_id is not None:
            self.features[18] = veh_id
            self.features[19] = Rdists[2]
            self.features[20] = Roffsets[2]
        else:
            self.features[18] = -100
            self.features[19] = -100
            self.features[20] = -100
        return self.features

class NeighborFeatureExtractor0:
    def __init__(self):
        self.num_features = 8 * 3 * 6
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {}
        self.feature_names = feature_names = ["_veh_id", "_dist", "_offset"]
        feature_ranges = [{"high":2000,"low":1}, {"high":250.0, "low":-100.0}, {"high":4.0, "low":-4.0}]
        self.history_names = history_names = ["_0sec", "_1sec", "_2sec", "_3sec", "_4sec", "_5sec"]
        self.neighbor_names = neighbor_names = ["front1", "front2", "left_front1", "left_front2", "right_front1", "right_front2", "left_rear1", "right_rear1"]
        self.feature_index = {}
        index = 0
        for nn in neighbor_names:
            for i in range(len(feature_names)):
                for j in range(len(history_names)):
                    self.feature_info[nn+feature_names[i]+history_names[j]] = feature_ranges[i]
                    self.feature_index[nn+feature_names[i]+history_names[j]] = index
                    index += 1

    def get_features(self, cr_scenario, vehicle_id, startframe):
        self.features = np.zeros(self.num_features, dtype=np.float32)
        fore_Ms, fore_Mdists, fore_Moffsets = cr_scenario.get_attribute(NEIGHBOR_FORE_ALONG_LANE, vehicle_id, startframe)
        fore_Ls, fore_Ldists, fore_Loffsets = cr_scenario.get_attribute(NEIGHBOR_FORE_ALONG_LEFT_LANE, vehicle_id, startframe)
        fore_Rs, fore_Rdists, fore_Roffsets = cr_scenario.get_attribute(NEIGHBOR_FORE_ALONG_RIGHT_LANE, vehicle_id, startframe)

        #rear_Ms, rear_Mdists, rear_Moffsets = cr_scenario.get_attribute(NEIGHBOR_REAR_ALONG_LANE, vehicle_id, startframe)
        rear_Ls, rear_Ldists, rear_Loffsets = cr_scenario.get_attribute(NEIGHBOR_REAR_ALONG_LEFT_LANE, vehicle_id, startframe)
        rear_Rs, rear_Rdists, rear_Roffsets = cr_scenario.get_attribute(NEIGHBOR_REAR_ALONG_RIGHT_LANE, vehicle_id, startframe)

        index = 0
        for i in range(len(fore_Ms)):
            veh_id = fore_Ms[i]
            if veh_id is not None:
                veh = cr_scenario.obstacles[startframe][veh_id]
                self.features[index:index+6] = veh_id
                self.features[index+6] = fore_Mdists[i]
                self.features[index+12] = fore_Moffsets[i]
                for j in range(1,6):
                    self.features[index+6+j], self.features[index+12+j] = cr_scenario.get_vector_between_cars(vehicle_id, veh_id, startframe-10*j)
            else:
                self.features[index:index+18] = -100
            index += 18

        for i in range(len(fore_Ls)):
            veh_id = fore_Ls[i]
            if veh_id is not None:
                veh = cr_scenario.obstacles[startframe][veh_id]
                self.features[index:index+6] = veh_id
                self.features[index+6] = fore_Ldists[i]
                self.features[index+12] = fore_Loffsets[i]
                for j in range(1,6):
                    self.features[index+6+j], self.features[index+12+j] = cr_scenario.get_vector_between_cars(vehicle_id, veh_id, startframe-10*j)
            else:
                self.features[index:index+18] = -100
            index += 18

        for i in range(len(fore_Rs)):
            veh_id = fore_Rs[i]
            if veh_id is not None:
                veh = cr_scenario.obstacles[startframe][veh_id]
                self.features[index:index+6] = veh_id
                self.features[index+6] = fore_Rdists[i]
                self.features[index+12] = fore_Roffsets[i]
                for j in range(1,6):
                    self.features[index+6+j], self.features[index+12+j] = cr_scenario.get_vector_between_cars(vehicle_id, veh_id, startframe-10*j)
            else:
                self.features[index:index+18] = -100
            index += 18

        for i in range(len(rear_Ls)):
            veh_id = rear_Ls[i]
            if veh_id is not None:
                veh = cr_scenario.obstacles[startframe][veh_id]
                self.features[index:index+6] = veh_id
                self.features[index+6] = -rear_Ldists[i]
                self.features[index+12] = rear_Loffsets[i]
                for j in range(1,6):
                    self.features[index+6+j], self.features[index+12+j] = cr_scenario.get_vector_between_cars(vehicle_id, veh_id, startframe-10*j)
            else:
                self.features[index:index+18] = -100
            index += 18

        for i in range(len(rear_Rs)):
            veh_id = rear_Rs[i]
            if veh_id is not None:
                veh = cr_scenario.obstacles[startframe][veh_id]
                self.features[index:index+6] = veh_id
                self.features[index+6] = -rear_Rdists[i]
                self.features[index+12] = rear_Roffsets[i]
                for j in range(1,6):
                    self.features[index+6+j], self.features[index+12+j] = cr_scenario.get_vector_between_cars(vehicle_id, veh_id, startframe-10*j)
            else:
                self.features[index:index+18] = -100
            index += 18

        return self.features


class LaneletNetworkFeatureExtractor:
    def __init__(self):
        self.num_features = 2
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {
                             "lanelet_id" : {"high":600, "low":100},
                             "lanelet_remaining" : {"high": 1000., "low":0.},
                             }
        self.feature_index = {
                             "lanelet_id" : 0,
                             "lanelet_remaining" : 1,
                             }
    def get_features(self, cr_scenario, vehicle_id, startframe):
        veh = cr_scenario.obstacles[startframe][vehicle_id]
        self.features[0] = veh.initial_state.posF.ind[1]
        lane = cr_scenario.lanelet_network.find_lanelet_by_id(veh.initial_state.posF.ind[1])
        self.features[1] = lane.center_curve[-1].s - veh.initial_state.posF.s
        #self.features[2] = lane.speed_limit
        return self.features

class WellBehavedFeatureExtractor:
    def __init__(self):
        self.num_features = 5
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {"is_colliding" : {"high":1.,    "low":0.},
                             "out_of_lane"  : {"high":1.,    "low":0.},
                             "negative_velocity" : {"high":1., "low":0.},
                             "distance_road_edge_left" : {"high":50.,    "low":-50.},
                             "distance_road_edge_right" : {"high":50.,    "low":-50.}}
        self.feature_index = {"is_colliding" : 0,
                             "out_of_lane"  : 1,
                             "negative_velocity" : 2,
                             "distance_road_edge_left" : 3,
                             "distance_road_edge_right" : 4}

    def get_features(self, cr_scenario,  vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        d_ml = cr_scenario.get_attribute(LINE_MARKING_LEFT_DIST, vehicle_id, startframe)
        d_mr = cr_scenario.get_attribute(LINE_MARKING_RIGHT_DIST, vehicle_id, startframe)
        self.features[0] = cr_scenario.get_attribute(IS_COLLIDING, vehicle_id, startframe)
        self.features[1] = d_ml < -0.8 or d_mr < -0.8
        self.features[2] = ego_vehicle.initial_state.velocity < 0.0
        self.features[3] = cr_scenario.get_attribute(ROAD_DIST_LEFT, vehicle_id, startframe)
        self.features[4] = cr_scenario.get_attribute(ROAD_DIST_RIGHT, vehicle_id, startframe)
        return self.features

class WellBehavedFeatureExtractor1:
    def __init__(self):
        self.num_features = 1
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {"is_colliding" : {"high":1.,    "low":0.}
                             }
        self.feature_index = {"is_colliding" : 0
                             }

    def get_features(self, cr_scenario,  vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        d_ml = cr_scenario.get_attribute(LINE_MARKING_LEFT_DIST, vehicle_id, startframe)
        d_mr = cr_scenario.get_attribute(LINE_MARKING_RIGHT_DIST, vehicle_id, startframe)
        self.features[0] = cr_scenario.get_attribute(IS_COLLIDING, vehicle_id, startframe)
        return self.features


class HistoryFeatureExtractor:
    def __init__(self):
        self.num_features = 5*5
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {}
        self.feature_names = feature_names = ["_relative_offset", "_relative_distance", "_velocity", "_relative_heading", "_time_step"]
        feature_ranges = [{"high":2.0, "low":-2.0}, {"high":40.0, "low":-40.0},
                          {"high":40.0, "low":0.0}, {"high":0.05, "low":-0.05},
                          {"high":5.0,"low":1.0}]
        self.history_names = history_names = ["1sec", "2sec", "3sec", "4sec", "5sec"]
        self.feature_index = {}
        index = 0
        for nn in history_names:
            for i in range(len(feature_names)):
                self.feature_info[nn+feature_names[i]] = feature_ranges[i]
                self.feature_index[nn+feature_names[i]] = index
                index += 1


    def get_features(self, cr_scenario,  vehicle_id, startframe):
        ego = cr_scenario.obstacles[startframe][vehicle_id]
        ego_lane_id = ego.initial_state.posF.ind[1]
        index = 0
        for t in range(1, 6):
            if vehicle_id in cr_scenario.obstacles[startframe - t*10].keys():
                past_veh = cr_scenario.obstacles[startframe-10*t][vehicle_id]
                past_lane_id = past_veh.initial_state.posF.ind[1]
                if ego_lane_id == past_lane_id:
                    self.features[index] = past_veh.initial_state.posF.d - ego.initial_state.posF.d
                    self.features[index+1] = past_veh.initial_state.posF.s - ego.initial_state.posF.s
                    self.features[index+2] = past_veh.initial_state.velocity
                    self.features[index+3] = past_veh.initial_state.posF.phi
                    self.features[index+4] = t*1.0
                else:
                    posG = past_veh.initial_state.get_posG()
                    posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [ego_lane_id]), cr_scenario.lanelet_network)
                    self.features[index] = posF_adjust.d - ego.initial_state.posF.d
                    self.features[index+1] = posF_adjust.s - ego.initial_state.posF.s
                    self.features[index+2] = past_veh.initial_state.velocity
                    self.features[index+3] = posF_adjust.phi
                    self.features[index+4] = t*1.0
            else:
                for i in range(1, 10):
                    if vehicle_id in cr_scenario.obstacles[startframe-(t-1)*10-(10-i)].keys():
                        past_veh = cr_scenario.obstacles[startframe-(t-1)*10-(10-i)][vehicle_id]
                        past_lane_id = past_veh.initial_state.posF.ind[1]
                        if ego_lane_id == past_lane_id:
                            self.features[index] = past_veh.initial_state.posF.d - ego.initial_state.posF.d
                            self.features[index+1] = past_veh.initial_state.posF.s - ego.initial_state.posF.s
                            self.features[index+2] = past_veh.initial_state.velocity
                            self.features[index+3] = past_veh.initial_state.posF.phi
                            self.features[index+4] = t-1 + (10-i)*0.1
                        else:
                            posG = past_veh.initial_state.get_posG()
                            posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [ego_lane_id]), cr_scenario.lanelet_network)
                            self.features[index] = posF_adjust.d - ego.initial_state.posF.d
                            self.features[index+1] = posF_adjust.s - ego.initial_state.posF.s
                            self.features[index+2] = past_veh.initial_state.velocity
                            self.features[index+3] = posF_adjust.phi
                            self.features[index+4] = t-1 + (10-i)*0.1
                        break
                for i in range(index, len(self.num_features)):
                    self.features[i] = -1
                break
            index += 5
        return self.features

class HistoryFeatureExtractor1:
    def __init__(self):
        self.num_features = 6*2
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {}
        self.feature_names = feature_names = ["_lane_relative_offset", "_lane_relative_distance"]
        feature_ranges = [{"high":3.2, "low":-3.2}, {"high":40.0, "low":-40.0}]
        self.history_names = history_names = ["0sec", "1sec", "2sec", "3sec", "4sec", "5sec"]
        self.feature_index = {}
        index = 0
        for nn in history_names:
            for i in range(len(feature_names)):
                self.feature_info[nn+feature_names[i]] = feature_ranges[i]
                self.feature_index[nn+feature_names[i]] = index
                index += 1


    def get_features(self, cr_scenario,  vehicle_id, startframe):
        ego = cr_scenario.obstacles[startframe][vehicle_id]
        ego_lane_id = ego.initial_state.posF.ind[1]
        ego_lane = cr_scenario.lanelet_network.find_lanelet_by_id(ego_lane_id)
        index = 0
        self.features[index] = ego.initial_state.posF.d
        lane_remain = ego_lane.center_curve[-1].s - ego.initial_state.posF.s
        self.features[index+1] = 100.0 if lane_remain > 100.0 else lane_remain # 0.0 TODO I chagned this
        index += 2
        for t in range(1, 6):
            if vehicle_id in cr_scenario.obstacles[startframe - t*10].keys():
                past_veh = cr_scenario.obstacles[startframe-10*t][vehicle_id]
                past_lane_id = past_veh.initial_state.posF.ind[1]
                if ego_lane_id == past_lane_id:
                    self.features[index] = past_veh.initial_state.posF.d# - ego.initial_state.posF.d
                    self.features[index+1] = past_veh.initial_state.posF.s - ego.initial_state.posF.s

                else:
                    posG = past_veh.initial_state.get_posG()
                    posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [ego_lane_id]), cr_scenario.lanelet_network)
                    if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0 and len(ego_lane.predecessor) > 0:
                        # rematch
                        cands = []
                        for pred_id in ego_lane.predecessor:
                            if ego_lane.predecessor_connections[pred_id][0].i == 0:
                                cands.append(pred_id)
                        posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, cands), cr_scenario.lanelet_network)
                        self.features[index] = posF_adjust.d# - ego.initial_state.posF.d
                        self.features[index+1] = posF_adjust.s - (ego.initial_state.posF.s + cr_scenario.lanelet_network.find_lanelet_by_id(posF_adjust.ind[1]).center_curve[-1].s)
                    else:
                        self.features[index] = posF_adjust.d# - ego.initial_state.posF.d
                        self.features[index+1] = posF_adjust.s - ego.initial_state.posF.s
            else:
                for i in range(index, self.num_features):
                    self.features[i] = -100
                break
            index += 2
        return self.features

class MultiFeatureExtractor:
    def __init__(self, extractors=[CoreFeatureExtractor1(),
                           WellBehavedFeatureExtractor1(),
                           LaneletNetworkFeatureExtractor(),
                           HistoryFeatureExtractor1(),
                           NeighborFeatureExtractor1()]):

        self.extractors = extractors
        #self.extractors = [CoreFeatureExtractor1(),
        #                   WellBehavedFeatureExtractor1(),
        #                   LaneletNetworkFeatureExtractor(),
        #                   HistoryFeatureExtractor1(),
                           #TemporalFeatureExtractor(),
        #                   NeighborFeatureExtractor1(),
                           #CarLidarFeatureExtractor(20, carlidar_max_range = 50.0),
                           #,
        #                   ]
        self.lengths = [subext.num_features for subext in self.extractors]
        self.num_features = np.sum(self.lengths)
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_index = {}
        for index, subext in enumerate(self.extractors):
            for feat_name, sub_index in subext.feature_index.items():
                if index == 0:
                    self.feature_index[feat_name] = sub_index
                else:
                    self.feature_index[feat_name] = sub_index + int(np.sum(self.lengths[0:index]))

    def get_features(self, cr_scenario, vehicle_id, startframe):
        feature_index = 0
        for subext, feature_len in zip(self.extractors, self.lengths):
            stop = feature_index + feature_len
            self.features[feature_index:stop] = subext.get_features(cr_scenario, vehicle_id, startframe)
            feature_index += feature_len
        return self.features
    def get_features_by_name(self, features, names):
        ret = []
        for n in names:
            if n in self.feature_index.keys():
                ret.append(features[self.feature_index[n]])
        return ret
