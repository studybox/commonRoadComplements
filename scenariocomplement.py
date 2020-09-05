import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectorycomplement import FrenetState, Frenet
from commonroad.scenario.laneletcomplement import *
from commonroad.scenario.lanelet import LineMarking, LaneletType
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
    def set_sensor_range(self, radius, front, rear, side):
        self.max_radius = radius
        self.max_front = front
        self.max_rear = rear
        self.max_side = side

    def add_grids(self, grids):
        self.grids = grids

    def locate_on_grid(self, time_step, vehicle_id, initial_cands=None):
        ego_vehicle = self.obstacles[time_step][vehicle_id]
        posF = ego_vehicle.initial_state.posF
        cands = []
        projections = {posF.ind[1]:(posF.s, posF.d)}
        best_dist2 = float('inf')
        best_projection = None
        for index, grid in enumerate(self.grids):
            # get the s and d
            if initial_cands is not None:
                if index not in initial_cands:
                    continue
            if grid.from_lanelet_id not in projections:
                projF = Frenet(ego_vehicle.initial_state.posG.projF(self.lanelet_network, [grid.from_lanelet_id]), self.lanelet_network)
                projections[grid.from_lanelet_id] = (projF.s, projF.d)
            if grid.center_lanelet_id not in projections:
                projF = Frenet(ego_vehicle.initial_state.posG.projF(self.lanelet_network, [grid.center_lanelet_id]), self.lanelet_network)
                projections[grid.center_lanelet_id] = (projF.s, projF.d)
            if grid.to_lanelet_id not in projections:
                projF = Frenet(ego_vehicle.initial_state.posG.projF(self.lanelet_network, [grid.to_lanelet_id]), self.lanelet_network)
                projections[grid.to_lanelet_id] = (projF.s, projF.d)

            if grid.is_in_grid(posF.ind[0], posF.ind[1]):
                cands.append(index)
        #print(time_step, vehicle_id, posF.ind[1], projections)
        #if time_step == 4923 and vehicle_id==127:
        #    print(posF.ind[1], posF.ind[0].i, posF.ind[0].t)
        if len(cands) > 4:
            print("there are {} grid for veh {} at {} {} {}".format(len(cands), vehicle_id, posF.ind[1], posF.ind[0].i, posF.ind[0].t))
            #for grid in self.grids:
            #    print(grid)
            #assert (len(cands) == 0 or len(cands) == 1 or len(cands) == 2)
        if len(cands) == 0:
            for index, grid in enumerate(self.grids):
                if initial_cands is not None:
                    if index not in initial_cands:
                        continue
                from_lanelet = self.lanelet_network.find_lanelet_by_id(grid.from_lanelet_id)
                from_curve = from_lanelet.center_curve
                from_curve_pt = lerp_curve(from_curve[grid.from_index.i], from_curve[grid.from_index.i+1], grid.from_index.t)
                new_s = projections[grid.from_lanelet_id][0]-from_curve_pt.s

                center_lanelet = self.lanelet_network.find_lanelet_by_id(grid.center_lanelet_id)
                center_curve = center_lanelet.center_curve
                center_curve_pt = lerp_curve(center_curve[grid.center_index.i], center_curve[grid.center_index.i+1], grid.center_index.t)
                c_dist2 = (center_curve_pt.s- projections[grid.center_lanelet_id][0])**2 + projections[grid.center_lanelet_id][1]**2
                if c_dist2 < best_dist2:
                    best_projection = (new_s, index)
                    best_dist2 = c_dist2
                to_lanelet = self.lanelet_network.find_lanelet_by_id(grid.to_lanelet_id)
                to_curve = to_lanelet.center_curve
                to_curve_pt = lerp_curve(to_curve[grid.to_index.i], to_curve[grid.to_index.i+1], grid.to_index.t)
                t_dist2 = (to_curve_pt.s- projections[grid.to_lanelet_id][0])**2 + projections[grid.to_lanelet_id][1]**2
                if t_dist2 < best_dist2:
                    best_projection = (new_s, index)
                    best_dist2 = t_dist2
                #print("grid {} c_dist2 {} t_dist2 {} cur_best {}".format(grid, c_dist2, t_dist2, self.grids[best_projection[1]]))
            return best_projection
        if len(cands) == 1:
            grid = self.grids[cands[0]]
            from_lanelet = self.lanelet_network.find_lanelet_by_id(grid.from_lanelet_id)
            from_curve = from_lanelet.center_curve
            from_curve_pt = lerp_curve(from_curve[grid.from_index.i], from_curve[grid.from_index.i+1], grid.from_index.t)
            if grid.from_lanelet_id == posF.ind[1]:
                new_s = posF.s - from_curve_pt.s
            else:
                new_s = posF.s + from_curve[-1].s - from_curve_pt.s
            assert new_s >= 0.0, "pos_s {} for veh {} at time {} and curve_pt s {} is negative laneletids from {} center {} to {} veh {} index i {} {} {} index t {} {} {}".format(posF.s,
                                                                                                           vehicle_id,
                                                                                                           time_step,
                                                                                                           from_curve_pt.s,
                                                                                                           grid.from_lanelet_id,
                                                                                                           grid.center_lanelet_id,
                                                                                                           grid.to_lanelet_id,
                                                                                                           posF.ind[1],
                                                                                                           grid.from_index.i,
                                                                                                           grid.center_index.i,
                                                                                                           grid.to_index.i,
                                                                                                           grid.from_index.t,
                                                                                                           grid.center_index.t,
                                                                                                           grid.to_index.t
                                                                                                           )
            return new_s, cands[0]
        else:
            best_from = self.grids[cands[0]].from_lanelet_id
            best_to = None
            best_to_dist = float("inf")
            # four cands
            for index in cands:
                grid = self.grids[index]
                t = 1
                while vehicle_id in self.obstacles[time_step-t]:
                    if self.obstacles[time_step-t][vehicle_id].initial_state.posF.ind[1] == grid.from_lanelet_id:
                        best_from = grid.from_lanelet_id
                        break
                    t += 1
                to_lanelet = self.lanelet_network.find_lanelet_by_id(grid.to_lanelet_id)
                to_curve = to_lanelet.center_curve
                to_curve_pt = lerp_curve(to_curve[grid.to_index.i], to_curve[grid.to_index.i+1], grid.to_index.t)
                distance2 = (to_curve_pt.pos.x-ego_vehicle.initial_state.posG.x)**2 + (to_curve_pt.pos.y-ego_vehicle.initial_state.posG.y)**2
                if distance2 < best_to_dist:
                    best_to = grid.to_lanelet_id
                    best_to_dist = distance2
            #print("best_from", best_from)
            #print("best_to", best_to)
            #print("cands", [self.grids[c] for c in cands])
            for index in cands:
                grid = self.grids[index]
                if grid.from_lanelet_id == best_from and grid.to_lanelet_id == best_to:
                    from_lanelet = self.lanelet_network.find_lanelet_by_id(grid.from_lanelet_id)
                    from_curve = from_lanelet.center_curve
                    from_curve_pt = lerp_curve(from_curve[grid.from_index.i], from_curve[grid.from_index.i+1], grid.from_index.t)
                    if grid.from_lanelet_id == posF.ind[1]:
                        new_s = posF.s - from_curve_pt.s
                    else:
                        new_s = posF.s + from_curve[-1].s - from_curve_pt.s
                    return new_s, index
            for index in cands:
                grid = self.grids[index]
                if grid.to_lanelet_id == best_to:
                    from_lanelet = self.lanelet_network.find_lanelet_by_id(grid.from_lanelet_id)
                    from_curve = from_lanelet.center_curve
                    from_curve_pt = lerp_curve(from_curve[grid.from_index.i], from_curve[grid.from_index.i+1], grid.from_index.t)
                    if grid.from_lanelet_id == posF.ind[1]:
                        new_s = posF.s - from_curve_pt.s
                    else:
                        new_s = posF.s + from_curve[-1].s - from_curve_pt.s
                    return new_s, index
        '''
        elif len(cands)==2:
            if self.grids[cands[0]].from_lanelet_id != self.grids[cands[1]].from_lanelet_id:
                if vehicle_id in self.obstacles[time_step-1]:
                    if self.obstacles[time_step-1][vehicle_id].initial_state.posF.ind[1] == self.grids[cands[1]].from_lanelet_id:
                        grid_index = cands[1]
                    else:
                        grid_index = cands[0]
                else:
                    grid_index = cands[0]
                grid = self.grids[grid_index]
                from_lanelet = self.lanelet_network.find_lanelet_by_id(grid.from_lanelet_id)
                from_curve = from_lanelet.center_curve
                from_curve_pt = lerp_curve(from_curve[grid.from_index.i], from_curve[grid.from_index.i+1], grid.from_index.t)
                if grid.from_lanelet_id == posF.ind[1]:
                    new_s = posF.s - from_curve_pt.s
                else:
                    new_s = posF.s + from_curve[-1].s - from_curve_pt.s
                assert new_s >= 0.0
                return new_s, grid_index
            else:
                grid1 = self.grids[cands[0]]
                to_lanelet1 = self.lanelet_network.find_lanelet_by_id(grid1.to_lanelet_id)
                to_curve1 = to_lanelet1.center_curve
                to_curve_pt1 = lerp_curve(to_curve1[grid1.to_index.i], to_curve1[grid1.to_index.i+1], grid1.to_index.t)
                distance1 = (to_curve_pt1.pos.x-ego_vehicle.initial_state.posG.x)**2 + (to_curve_pt1.pos.y-ego_vehicle.initial_state.posG.y)**2

                grid2 = self.grids[cands[1]]
                to_lanelet2 = self.lanelet_network.find_lanelet_by_id(grid2.to_lanelet_id)
                to_curve2 = to_lanelet2.center_curve
                to_curve_pt2 = lerp_curve(to_curve2[grid2.to_index.i], to_curve2[grid2.to_index.i+1], grid2.to_index.t)
                distance2 = (to_curve_pt2.pos.x-ego_vehicle.initial_state.posG.x)**2 + (to_curve_pt2.pos.y-ego_vehicle.initial_state.posG.y)**2
                if distance1 <= distance2:
                    from_lanelet1 = self.lanelet_network.find_lanelet_by_id(grid1.from_lanelet_id)
                    from_curve1 = from_lanelet1.center_curve
                    from_curve_pt1 = lerp_curve(from_curve1[grid1.from_index.i], from_curve1[grid1.from_index.i+1], grid1.from_index.t)
                    if grid1.from_lanelet_id == posF.ind[1]:
                        new_s = posF.s - from_curve_pt1.s
                    else:
                        new_s = posF.s + from_curve1[-1].s - from_curve_pt1.s

                    assert new_s >= 0.0
                    return new_s, cands[0]
                else:
                    from_lanelet2 = self.lanelet_network.find_lanelet_by_id(grid2.from_lanelet_id)
                    from_curve2 = from_lanelet2.center_curve
                    from_curve_pt2 = lerp_curve(from_curve2[grid2.from_index.i], from_curve2[grid2.from_index.i+1], grid2.from_index.t)
                    if grid2.from_lanelet_id == posF.ind[1]:
                        new_s = posF.s - from_curve_pt2.s
                    else:
                        new_s = posF.s + from_curve2[-1].s - from_curve_pt2.s
                    assert new_s >= 0.0
                    return new_s, cands[1]
        '''
    def locate_on_grid1(self, time_step, vehicle_id, initial_cands=None):
        ego_vehicle = self.obstacles[time_step][vehicle_id]
        ego_posG = ego_vehicle.initial_state.posG
        best_dist2 = float('inf')
        best_projection = None
        if initial_cands is None:
            initial_cands = np.arange(len(self.grids))
        for index in initial_cands:
            grid = self.grids[index]
            best_point_dist2 = float('inf')
            for pos in grid.pos_list:
                dist2 = (pos.x-ego_posG.x)**2 + (pos.y-ego_posG.y)**2
                if dist2 < best_point_dist2:
                    best_point_dist2 = dist2
            if best_point_dist2 < best_dist2:
                best_dist2 = best_point_dist2
                best_projection = index
        return index
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
            return self.get_fore_rear_left_mid_right(veh_id, time_step, max_radius=self.max_radius, front=self.max_front, side=self.max_side,rear=self.max_rear)

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
            delta_d = delta_s = None
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


                    dist_s = [0.0]
                    cands = [ego_lane_id]
                    while len(cands) > 0:
                        current_cand = cands.pop(0)
                        current_dist_s = dist_s.pop(0)
                        if current_cand == veh_lane_id:
                            posF_adjust = veh.initial_state.posF
                            delta_d = posF_adjust.d - ego.initial_state.posF.d
                            delta_s = posF_adjust.s -(ego.initial_state.posF.s + current_dist_s)
                            break
                        else:
                            current_lane = self.lanelet_network.find_lanelet_by_id(current_cand)
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [current_cand]), self.lanelet_network)
                            if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0:
                                for pred in current_lane.predecessor:
                                    predecessor_lane = self.lanelet_network.find_lanelet_by_id(pred)
                                    cands.append(pred)
                                    dist_s.append(current_dist_s+predecessor_lane.center_curve[-1].s)
                            else:
                                if delta_d is None:
                                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                                    delta_s = posF_adjust.s - (ego.initial_state.posF.s + current_dist_s)
                                elif np.abs(posF_adjust.d - ego.initial_state.posF.d) < np.abs(delta_d):
                                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                                    delta_s = posF_adjust.s - (ego.initial_state.posF.s + current_dist_s)


                elif posF_adjust.ind[0].i >= max_ind and posF_adjust.ind[0].t == 1.0 and len(ego_lane.successor) > 0:
                    #rematch

                    dist_s = [0.0]
                    cands = [ego_lane_id]
                    while len(cands) > 0:
                        current_cand = cands.pop(0)
                        current_dist_s = dist_s.pop(0)
                        if current_cand == veh_lane_id:
                            posF_adjust = veh.initial_state.posF
                            delta_d = posF_adjust.d - ego.initial_state.posF.d
                            delta_s = posF_adjust.s + current_dist_s - ego.initial_state.posF.s
                            break
                        else:
                            current_lane = self.lanelet_network.find_lanelet_by_id(current_cand)
                            posF_adjust = Frenet(posG.projF(self.lanelet_network, [current_cand]), self.lanelet_network)
                            if posF_adjust.ind[0].i >= len(current_lane.center_curve)-2 and posF_adjust.ind[0].t == 1.0:
                                for succ in current_lane.successor:
                                    successor_lane = self.lanelet_network.find_lanelet_by_id(succ)
                                    cands.append(succ)
                                    dist_s.append(current_dist_s+current_lane.center_curve[-1].s)
                            else:
                                if delta_d is None:
                                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                                    delta_s = posF_adjust.s + current_dist_s - ego.initial_state.posF.s
                                elif np.abs(posF_adjust.d - ego.initial_state.posF.d) < np.abs(delta_d):
                                    delta_d = posF_adjust.d - ego.initial_state.posF.d
                                    delta_s = posF_adjust.s + current_dist_s - ego.initial_state.posF.s

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
    def get_neighbors(self, ego_id, startframe, along_lanelet=False, max_radius=30, front=30, side=20,rear=25):
        #TODO make the reference straight instead of along lanelet
        ego_posG = self.obstacles[startframe][ego_id].initial_state.get_posG()
        ego_lanelet_id = self.obstacles[startframe][ego_id].initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
        best_ids = []
        for veh_id, veh in self.obstacles[startframe].items():
            if veh_id != ego_id:
                veh_posG = self.obstacles[startframe][veh_id].initial_state.get_posG()
                distance2 = (veh_posG.x-ego_posG.x)**2 + (veh_posG.y-ego_posG.y)**2
                is_in_grids = False
                for grid in self.grids:
                    if grid.is_in_grid(veh.initial_state.posF.ind[0], veh.initial_state.posF.ind[1]):
                        is_in_grids = True
                        break
                if distance2 < max_radius**2 and is_in_grids:
                    best_ids.append(veh_id)
        final_ids = []
        delta_step = int(1/self.dt)
        for veh_id in best_ids:
            delta_s, delta_d = self.get_vector_between_cars(ego_id, veh_id, startframe, along_lanelet)
            if delta_s <= front and delta_s >= - rear:
                if delta_d <= side and delta_d >= -side:
                    final_ids.append(veh_id)
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
        #delta_step = int(1/self.dt)
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
            ret["front"] = (front_id, delta_s_along_lane, delta_d_along_lane, front_s, front_d)
        else:
            ret["front"] = (-100, -100, -100, -100, -100)

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
        ret["left"] = (best_left_ids, best_left_s_along_lane, best_left_d_along_lane, best_left_s, best_left_d)

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
        ret["right"] = (best_right_ids, best_right_s_along_lane, best_right_d_along_lane, best_right_s, best_right_d)
        return ret
    def get_grids(self, ego_id, startframe, grid_length, max_disp_front=55, max_disp_rear=30, max_radius=55):
        ego_posG = self.obstacles[startframe][ego_id].initial_state.get_posG()
        ego_posF = self.obstacles[startframe][ego_id].initial_state.posF
        ego_curve_index = self.obstacles[startframe][ego_id].initial_state.posF.ind[0]
        ego_lanelet_id = self.obstacles[startframe][ego_id].initial_state.posF.ind[1]
        ego_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet_id)

        # this is the center of everything
        cand_indexs = [(ego_curve_index, ego_lanelet_id)]
        future_indexs = []
        all_lanelets = set()
        all_lanelets.add(ego_lanelet_id)
        #print("gaga", ego_lanelet_id)
        #future_indexs_backward = [(ego_curve_index, ego_lanelet_id, 0.0)]
        if ego_lanelet.adj_left is not None:
            left_ego_posF = Frenet(ego_posG.projF(self.lanelet_network, [ego_lanelet.adj_left]), self.lanelet_network)
            left_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet.adj_left)
            future_indexs.append((left_ego_posF.ind[0], ego_lanelet.adj_left, 0.0, "forward"))
            future_indexs.append((left_ego_posF.ind[0], ego_lanelet.adj_left, 0.0, "backward"))
            all_lanelets.add(ego_lanelet.adj_left)
            #future_indexs_backward.append((ego_curve_index, ego_lanelet.adj_left, 0.0))
            cand_indexs.append((left_ego_posF.ind[0], ego_lanelet.adj_left))
            if left_lanelet.adj_left is not None:
                left_left_ego_posF = Frenet(ego_posG.projF(self.lanelet_network, [left_lanelet.adj_left]), self.lanelet_network)
                future_indexs.append((left_left_ego_posF.ind[0], left_lanelet.adj_left, 0.0, "forward"))
                future_indexs.append((left_left_ego_posF.ind[0], left_lanelet.adj_left, 0.0, "backward"))
                cand_indexs.append((left_left_ego_posF.ind[0], left_lanelet.adj_left))
                all_lanelets.add(left_lanelet.adj_left)

        if ego_lanelet.adj_right is not None:
            right_ego_posF = Frenet(ego_posG.projF(self.lanelet_network, [ego_lanelet.adj_right]), self.lanelet_network)
            right_lanelet = self.lanelet_network.find_lanelet_by_id(ego_lanelet.adj_right)
            future_indexs.append((right_ego_posF.ind[0], ego_lanelet.adj_right, 0.0, "backward"))
            future_indexs.append((right_ego_posF.ind[0], ego_lanelet.adj_right, 0.0, "forward"))

            all_lanelets.add(ego_lanelet.adj_right)
            #future_indexs_backward.append((ego_curve_index, ego_lanelet.adj_right, 0.0))
            cand_indexs.append((right_ego_posF.ind[0], ego_lanelet.adj_right))
            if right_lanelet.adj_right is not None:
                right_right_ego_posF = Frenet(ego_posG.projF(self.lanelet_network, [right_lanelet.adj_right]), self.lanelet_network)
                future_indexs.append((right_right_ego_posF.ind[0], right_lanelet.adj_right, 0.0, "backward"))
                future_indexs.append((right_right_ego_posF.ind[0], right_lanelet.adj_right, 0.0, "forward"))
                cand_indexs.append((right_right_ego_posF.ind[0], right_lanelet.adj_right))
                all_lanelets.add(right_lanelet.adj_right)


        future_indexs.append((ego_curve_index, ego_lanelet_id, 0.0, "backward"))
        future_indexs.append((ego_curve_index, ego_lanelet_id, 0.0, "forward"))

        # forward and backward
        while len(future_indexs) > 0:
            current_index, current_lanelet_id, current_disp, dir = future_indexs.pop(-1)
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet_id)
            current_curve = current_lanelet.center_curve
            #print(len(current_curve), current_index.i, current_lanelet_id, dir, current_disp)
            current_curvePt = lerp_curve(current_curve[current_index.i], current_curve[current_index.i+1], current_index.t)
            distance2 = (current_curvePt.pos.x-ego_posG.x)**2 + (current_curvePt.pos.y-ego_posG.y)**2
            if (dir == "forward" and current_disp > max_disp_front) or (dir == "backward" and current_disp > max_disp_rear) or distance2 > max_radius**2:
                #print("out", current_disp + grid_length > max_disp_front, current_disp + grid_length > max_disp_rear, distance2>max_radius**2)
                continue
            elif current_disp != 0.0:
                #print("in", dir, current_lanelet_id, current_index.i, current_disp, np.sqrt(distance2))
                cand_indexs.append((current_index, current_lanelet_id))
            if dir == "forward":
                if current_curvePt.s + grid_length > current_curve[-1].s:
                    ori_new_s = current_curvePt.s+grid_length-current_curve[-1].s
                    next_lanes = []
                    for succ in current_lanelet.successor:
                        successor_lane = self.lanelet_network.find_lanelet_by_id(succ)
                        new_index, _ = get_curve_index(CurveIndex(0, 0.0), successor_lane.center_curve, ori_new_s)
                        if new_index.t > 1.0:
                            for succ_succ in successor_lane.successor:
                                if succ_succ not in all_lanelets:
                                    next_lanes.append((succ_succ, ori_new_s-successor_lane.center_curve[-1].s, succ))
                        elif succ not in all_lanelets:
                            next_lanes.append((succ, ori_new_s, succ))
                    for succ, new_s, ori_succ in next_lanes:
                        successor_lane = self.lanelet_network.find_lanelet_by_id(succ)
                        all_lanelets.add(succ)
                        new_index, _ = get_curve_index(CurveIndex(0, 0.0), successor_lane.center_curve, new_s)
                        future_indexs.append((new_index, succ, current_disp+grid_length, "forward"))
                        if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                            print("1 {}".format(future_indexs[-1]))
                        if current_lanelet.adj_left is None and successor_lane.adj_left is not None and successor_lane.adj_left not in all_lanelets:
                            new_curvePt = lerp_curve(successor_lane.center_curve[new_index.i],
                                                     successor_lane.center_curve[new_index.i+1], new_index.t)
                            left_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [successor_lane.adj_left]), self.lanelet_network)
                            future_indexs.append((left_posF.ind[0], successor_lane.adj_left, current_disp + grid_length, "forward"))
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("2 {}".format(future_indexs[-1]))
                            future_indexs.append((left_posF.ind[0], successor_lane.adj_left, 0.0, "backward"))
                            all_lanelets.add(successor_lane.adj_left)
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("3 {}".format(future_indexs[-1]))
                            left_lanelet = self.lanelet_network.find_lanelet_by_id(successor_lane.adj_left)
                            if left_lanelet.adj_left is not None and left_lanelet.adj_left not in all_lanelets:
                                all_lanelets.add(left_lanelet.adj_left)
                                left_left_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [left_lanelet.adj_left]), self.lanelet_network)
                                future_indexs.append((left_left_posF.ind[0], left_lanelet.adj_left, current_disp + grid_length, "forward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("4 {}".format(future_indexs[-1]))
                                future_indexs.append((left_left_posF.ind[0], left_lanelet.adj_left, 0.0, "backward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("5 {}".format(future_indexs[-1]))

                        if current_lanelet.adj_right is None and successor_lane.adj_right is not None and successor_lane.adj_right not in all_lanelets:
                            new_curvePt = lerp_curve(successor_lane.center_curve[new_index.i],
                                                     successor_lane.center_curve[new_index.i+1], new_index.t)
                            right_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [successor_lane.adj_right]), self.lanelet_network)
                            future_indexs.append((right_posF.ind[0], successor_lane.adj_right, current_disp + grid_length, "forward"))
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("6 {}".format(future_indexs[-1]))
                            future_indexs.append((right_posF.ind[0], successor_lane.adj_right, 0.0, "backward"))
                            all_lanelets.add(successor_lane.adj_right)
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("7 {}".format(future_indexs[-1]))
                            right_lanelet = self.lanelet_network.find_lanelet_by_id(successor_lane.adj_right)
                            if right_lanelet.adj_right is not None and right_lanelet.adj_right not in all_lanelets:
                                all_lanelets.add(right_lanelet.adj_right)
                                right_right_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [right_lanelet.adj_right]), self.lanelet_network)
                                future_indexs.append((right_right_posF.ind[0], right_lanelet.adj_right, current_disp + grid_length, "forward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("8 {}".format(future_indexs[-1]))
                                future_indexs.append((right_right_posF.ind[0], right_lanelet.adj_right, 0.0, "backward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("9 {}".format(future_indexs[-1]))
                        if len(successor_lane.predecessor) > 1: # there exists a conflict lane
                            for pred in successor_lane.predecessor:
                                if (succ == ori_succ and pred != current_lanelet_id and pred not in all_lanelets) or (succ != ori_succ and pred != ori_succ and pred not in all_lanelets):
                                    conflict_lanelet = self.lanelet_network.find_lanelet_by_id(pred)
                                    conflict_new_s = conflict_lanelet.center_curve[-1].s+new_s-grid_length
                                    if conflict_new_s < 0:
                                        for con_pred in conflict_lanelet.predecessor:
                                            if con_pred not in all_lanelets:
                                                pred_conflict_lanelet = self.lanelet_network.find_lanelet_by_id(con_pred)
                                                pred_conflict_new_s = conflict_new_s + pred_conflict_lanelet.center_curve[-1].s
                                                conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), pred_conflict_lanelet.center_curve, pred_conflict_new_s)
                                                future_indexs.append((conflict_new_index, con_pred, 5., "backward"))
                                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                                    print("11.0 {}".format(future_indexs[-1]))
                                                all_lanelets.add(con_pred)
                                    else:
                                        conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), conflict_lanelet.center_curve, conflict_new_s)
                                        future_indexs.append((conflict_new_index, pred, 5., "backward"))
                                        all_lanelets.add(pred)
                                        if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                            print("10 {}".format(future_indexs[-1]))
                                            print(pred, ori_succ, succ, new_s, grid_length)
                        if succ != ori_succ:
                            ori_succ_lane = self.lanelet_network.find_lanelet_by_id(ori_succ)
                            if len(ori_succ_lane.predecessor) > 1:
                                for pred in ori_succ_lane.predecessor:
                                    if pred != current_lanelet_id and pred not in all_lanelets:
                                        conflict_lanelet = self.lanelet_network.find_lanelet_by_id(pred)
                                        conflict_new_s = np.maximum(conflict_lanelet.center_curve[-1].s+new_s+ori_succ_lane.center_curve[-1].s-grid_length, 0.0)
                                        conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), conflict_lanelet.center_curve, conflict_new_s)
                                        future_indexs.append((conflict_new_index, pred, 5., "backward"))
                                        all_lanelets.add(pred)



                else:
                    new_index, _ = get_curve_index(current_index, current_curve, grid_length)
                    future_indexs.append((new_index, current_lanelet_id, current_disp + grid_length, "forward"))
                    if future_indexs[-1][0].t >=1.0:
                        print("11 {}".format(future_indexs[-1]))
            else:
                if current_curvePt.s - grid_length < 0.0:
                    next_lanes = []
                    for pred in current_lanelet.predecessor:
                        predecessor_lane = self.lanelet_network.find_lanelet_by_id(pred)
                        if predecessor_lane.center_curve[-1].s + current_curvePt.s - grid_length < 0:
                            for pred_pred in predecessor_lane.predecessor:
                                if pred_pred not in all_lanelets:
                                    pred_pred_lane = self.lanelet_network.find_lanelet_by_id(pred_pred)
                                    next_lanes.append((pred_pred, pred_pred_lane.center_curve[-1].s+predecessor_lane.center_curve[-1].s+current_curvePt.s-grid_length, pred))
                        elif pred not in all_lanelets:
                            next_lanes.append((pred, predecessor_lane.center_curve[-1].s+current_curvePt.s-grid_length, pred))

                    for pred, new_s, ori_pred in next_lanes:
                        predecessor_lane = self.lanelet_network.find_lanelet_by_id(pred)
                        all_lanelets.add(pred)
                        #new_s = predecessor_lane.center_curve[-1].s + current_curvePt.s - grid_length
                        new_index, _ = get_curve_index(CurveIndex(0,0.0), predecessor_lane.center_curve, new_s)
                        future_indexs.append((new_index, pred, current_disp + grid_length, "backward"))
                        if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                            print("12 {}".format(future_indexs[-1]))
                        if current_lanelet.adj_left is None and predecessor_lane.adj_left is not None and predecessor_lane.adj_left not in all_lanelets:
                            new_curvePt = lerp_curve(predecessor_lane.center_curve[new_index.i],
                                                     predecessor_lane.center_curve[new_index.i+1], new_index.t)
                            left_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [predecessor_lane.adj_left]), self.lanelet_network)
                            future_indexs.append((left_posF.ind[0], predecessor_lane.adj_left, current_disp + grid_length, "backward"))
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("13 {}".format(future_indexs[-1]))
                            future_indexs.append((left_posF.ind[0], predecessor_lane.adj_left, 0.0, "forward"))
                            all_lanelets.add(predecessor_lane.adj_left)
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("14 {}".format(future_indexs[-1]))
                            left_lanelet = self.lanelet_network.find_lanelet_by_id(predecessor_lane.adj_left)
                            if left_lanelet.adj_left is not None and left_lanelet.adj_left not in all_lanelets:
                                all_lanelets.add(left_lanelet.adj_left)
                                left_left_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [left_lanelet.adj_left]), self.lanelet_network)
                                future_indexs.append((left_left_posF.ind[0], left_lanelet.adj_left, current_disp + grid_length, "backward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("15 {}".format(future_indexs[-1]))
                                future_indexs.append((left_left_posF.ind[0], left_lanelet.adj_left, 0.0, "forward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("16 {}".format(future_indexs[-1]))
                        if current_lanelet.adj_right is None and predecessor_lane.adj_right is not None and predecessor_lane.adj_right not in all_lanelets:
                            new_curvePt = lerp_curve(predecessor_lane.center_curve[new_index.i],
                                                     predecessor_lane.center_curve[new_index.i+1], new_index.t)
                            right_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [predecessor_lane.adj_right]), self.lanelet_network)
                            future_indexs.append((right_posF.ind[0], predecessor_lane.adj_right, current_disp + grid_length, "backward"))
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("17 {}".format(future_indexs[-1]))
                            future_indexs.append((right_posF.ind[0], predecessor_lane.adj_right, 0.0, "forward"))
                            all_lanelets.add(predecessor_lane.adj_right)
                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                print("18 {}".format(future_indexs[-1]))
                            right_lanelet = self.lanelet_network.find_lanelet_by_id(predecessor_lane.adj_right)
                            if right_lanelet.adj_right is not None and right_lanelet.adj_right not in all_lanelets:
                                all_lanelets.add(right_lanelet.adj_right)
                                right_right_posF = Frenet(new_curvePt.pos.projF(self.lanelet_network, [right_lanelet.adj_right]), self.lanelet_network)
                                future_indexs.append((right_right_posF.ind[0], right_lanelet.adj_right, current_disp + grid_length, "backward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("19 {}".format(future_indexs[-1]))
                                future_indexs.append((right_right_posF.ind[0], right_lanelet.adj_right, 0.0, "forward"))
                                if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                    print("20 {}".format(future_indexs[-1]))
                        if len(predecessor_lane.successor) > 1: # conflict_lane
                            for succ in predecessor_lane.successor:
                                if pred == ori_pred:
                                    if succ != current_lanelet_id and succ not in all_lanelets:
                                        conflict_lanelet = self.lanelet_network.find_lanelet_by_id(succ)
                                        conflict_new_s = current_curvePt.s
                                        #TODO
                                        if conflict_new_s > conflict_lanelet.center_curve[-1].s:
                                            #assert len(conflict_lanelet.successor) == 1, "conflict {}".format(succ)
                                            for con_succ in conflict_lanelet.successor:
                                                if con_succ not in all_lanelets:
                                                    succ_conflict_new_s = conflict_new_s - conflict_lanelet.center_curve[-1].s
                                                    succ_conflict_lanelet = self.lanelet_network.find_lanelet_by_id(con_succ)
                                                    conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), succ_conflict_lanelet.center_curve, succ_conflict_new_s)
                                                    future_indexs.append((conflict_new_index, con_succ, 5., "forward"))
                                                    if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                                        print("21.0 {}".format(future_indexs[-1]))
                                                    all_lanelets.add(con_succ)

                                        else:
                                            conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), conflict_lanelet.center_curve, conflict_new_s)
                                            future_indexs.append((conflict_new_index, succ, 5., "forward"))
                                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                                print("21 {}".format(future_indexs[-1]))
                                            all_lanelets.add(succ)

                                else:
                                    if succ != ori_pred and succ not in all_lanelets:
                                        conflict_lanelet = self.lanelet_network.find_lanelet_by_id(succ)
                                        ori_pred_lane = self.lanelet_network.find_lanelet_by_id(ori_pred)
                                        conflict_new_s = current_curvePt.s + ori_pred_lane.center_curve[-1].s
                                        #TODO
                                        if conflict_new_s > conflict_lanelet.center_curve[-1].s:
                                            #print(conflict_new_s, conflict_lanelet.center_curve[-1].s)
                                            #assert len(conflict_lanelet.successor) == 1, "conflict {}".format(succ)
                                            for con_succ in conflict_lanelet.successor:
                                                if con_succ not in all_lanelets:
                                                    succ_conflict_new_s = conflict_new_s - conflict_lanelet.center_curve[-1].s
                                                    succ_conflict_lanelet = self.lanelet_network.find_lanelet_by_id(con_succ)
                                                    conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), succ_conflict_lanelet.center_curve, succ_conflict_new_s)
                                                    future_indexs.append((conflict_new_index, con_succ, 5., "forward"))
                                                    if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                                        print("22.0 {}".format(future_indexs[-1]))
                                                        print(conflict_new_s)
                                                    all_lanelets.add(con_succ)

                                        else:
                                            conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), conflict_lanelet.center_curve, conflict_new_s)
                                            future_indexs.append((conflict_new_index, succ, 5., "forward"))
                                            if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                                print("22 {}".format(future_indexs[-1]))
                                            all_lanelets.add(succ)
                                    #if current_lanelet.adj_left is None and conflict_lanelet.adj_left is not None:
                                    #    future_indexs.append((conflict_new_index, conflict_lanelet.adj_left, 10., "forward"))
                                    #if current_lanelet.adj_right is None and conflict_lanelet.adj_right is not None:
                                    #    future_indexs.append((conflict_new_index, conflict_lanelet.adj_right, 10., "forward"))
                        if pred != ori_pred:
                            ori_pred_lane = self.lanelet_network.find_lanelet_by_id(ori_pred)
                            if len(ori_pred_lane.successor) > 1:
                                for succ in ori_pred_lane.successor:
                                    if succ != current_lanelet_id and succ not in all_lanelets:
                                        conflict_lanelet = self.lanelet_network.find_lanelet_by_id(succ)
                                        ori_pred_lane = self.lanelet_network.find_lanelet_by_id(ori_pred)
                                        conflict_new_s = np.minimum(current_curvePt.s + ori_pred_lane.center_curve[-1].s, conflict_lanelet.center_curve[-1].s)
                                        conflict_new_index,_ = get_curve_index(CurveIndex(0,0.0), conflict_lanelet.center_curve, conflict_new_s)
                                        future_indexs.append((conflict_new_index, succ, 5., "forward"))
                                        if future_indexs[-1][0].t >1.0 or future_indexs[-1][0].t < 0.0:
                                            print("23 {}".format(future_indexs[-1]))
                                        all_lanelets.add(succ)

                else:
                    new_index, _ = get_curve_index(current_index, current_curve, -grid_length)
                    future_indexs.append((new_index,current_lanelet_id, current_disp + grid_length, "backward"))
                    if future_indexs[-1][0].t > 1.0 or future_indexs[-1][0].t < 0.0:
                        print("24 {}".format(future_indexs[-1]))

        # backward
        '''
        while len(future_indexs_backward) > 0:
            current_index, current_lanelet_id, current_disp = future_indexs_backward.pop(0)
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet_id)
            current_curve = current_lanelet.center_curve
            current_curvePt = lerp_curve(current_curve[current_index.i], current_curve[current_index.i+1], current_index.t)
            if current_disp + grid_length > max_disp:
                continue
            elif current_disp != 0.0:
                print("backward", current_lanelet_id, current_index.i)
                cand_indexs.append((current_index, current_lanelet_id))
            if current_curvePt.s - grid_length < 0.0:
                for pred in current_lanelet.predecessor:
                    predecessor_lane = self.lanelet_network.find_lanelet_by_id(pred)
                    new_s = predecessor_lane.center_curve[-1].s + current_curvePt.s - grid_length
                    new_index, _ = get_curve_index(CurveIndex(0,0.0), predecessor_lane.center_curve, new_s)
                    future_indexs_backward.append((new_index, pred, current_disp + grid_length))
            else:
                new_index, _ = get_curve_index(current_index, current_curve, -grid_length)
                future_indexs_backward.append((new_index,current_lanelet_id, current_disp + grid_length))
        '''
        grids = []
        for center_index, center_lanelet_id in cand_indexs:

            center_lanelet = self.lanelet_network.find_lanelet_by_id(center_lanelet_id)
            center_curve = center_lanelet.center_curve
            center_curvePt = lerp_curve(center_curve[center_index.i], center_curve[center_index.i+1], center_index.t)
            #distance2 = (center_curvePt.pos.x-ego_posG.x)**2 + (center_curvePt.pos.y-ego_posG.y)**2
            #if distance2 > max_radius**2:
            #    continue
            to_points = []
            if center_curvePt.s + grid_length * 0.5 > center_curve[-1].s:
                if len(center_lanelet.successor) > 0:
                    for succ in center_lanelet.successor:
                        succ_lanelet = self.lanelet_network.find_lanelet_by_id(succ)
                        new_index,_ = get_curve_index(CurveIndex(0,0.0), succ_lanelet.center_curve, center_curvePt.s+grid_length*0.5-center_curve[-1].s)
                        to_points.append((new_index, succ, grid_length*0.5))
                else:
                    new_index = CurveIndex(len(center_curve)-2, 1.0)
                    to_points.append((new_index,center_lanelet_id, center_curve[-1].s-center_curvePt.s))
            else:
                if center_curvePt.s + grid_length > center_curve[-1].s and len(center_lanelet.successor) == 0:
                    new_index = CurveIndex(len(center_curve)-2,1.0)
                    to_points.append((new_index,center_lanelet_id, center_curve[-1].s-center_curvePt.s))
                else:
                    new_index, _ = get_curve_index(center_index, center_curve, grid_length*0.5)
                    to_points.append((new_index,center_lanelet_id, grid_length * 0.5))

            from_points = []
            if center_curvePt.s - grid_length * 0.5 < 0.0:
                if len(center_lanelet.predecessor) > 0:
                    for pred in center_lanelet.predecessor:
                        pred_lanelet = self.lanelet_network.find_lanelet_by_id(pred)
                        new_s = pred_lanelet.center_curve[-1].s + center_curvePt.s - grid_length*0.5
                        new_index, _ = get_curve_index(CurveIndex(0,0.0), pred_lanelet.center_curve, new_s)
                        assert new_index.t <=1.0 , "pred_lanelet {} length {} new_s {}".format(pred, pred_lanelet.center_curve[-1].s, new_s)
                        from_points.append((new_index, pred, -grid_length * 0.5))
                else:
                    new_index = CurveIndex(0, 0.0)
                    from_points.append((new_index,center_lanelet_id, -center_curvePt.s))
            else:
                if center_curvePt.s - grid_length < 0.0 and len(center_lanelet.predecessor) == 0:
                    new_index = CurveIndex(0, 0.0)
                    from_points.append((new_index,center_lanelet_id, -center_curvePt.s))
                else:
                    new_index, _ = get_curve_index(center_index, center_curve, -grid_length*0.5)
                    from_points.append((new_index,center_lanelet_id, -grid_length * 0.5))

            for from_index,from_lanelet_id,from_disp in from_points:
                for to_index,to_lanelet_id,to_disp in to_points:
                    grids.append(Grid(from_index=from_index,
                                      to_index=to_index,
                                      from_lanelet_id=from_lanelet_id,
                                      to_lanelet_id=to_lanelet_id,
                                      center_index=center_index,
                                      center_lanelet_id=center_lanelet_id,
                                      from_disp=from_disp,
                                      to_disp=to_disp, lanelet_network=self.lanelet_network))
        return grids

class CoreFeatureExtractor:
    def __init__(self):
        self.num_features = 13
        self.features = np.zeros(self.num_features, dtype=np.float32)

        self.feature_info = { "egoid":{"high":3000, "low":0},
                              "relative_offset":{"high":2.,    "low":-2.},
                              "relative_displa":{"high":100.,    "low": 0.},
                              "velocity_s":{"high":40.,    "low":0.},
                              "velocity_d":{"high":40.,    "low":0.},
                              "grid_index" : {"high":None, "low":None},
                              "length":{"high":30.,    "low":2.},
                              "width":{"high":3.,     "low":.9},
                              "target_delta_s":{"high":40, "low":0},
                              "target_delta_d":{"high":40, "low":0},
                              "target_velocity_s":{"high":40, "low":0},
                              "target_velocity_d":{"high":40, "low":0},
                              "target_grid_index":{"high":None, "low":None}
                              }

        self.feature_index = {"egoid":0,
                              "relative_offset":1,
                              "relative_displa":2,
                              "velocity_s":3,
                              "velocity_d":4,
                              "grid_index":5,
                              "length":6,
                              "width":7,
                              "target_delta_s":8,
                              "target_delta_d":9,
                              "target_velocity_s":10,
                              "target_velocity_d":11,
                              "target_grid_index":12
                              }
    def set_step_length_and_maneuver_horizon(self, step_length, horizon):
        self.step_length = step_length
        self.horizon = horizon

    def get_delta(self, cr_scenario, vehicle_id, startframe, step):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        current_state = ego_vehicle.initial_state
        current_posF = current_state.posF
        current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_posF.ind[1])
        origin_current_lanelet_id = current_lane.lanelet_id
        if current_lane.adj_right is not None: # this is for rounD dataset
            current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_lane.adj_right)
            current_posF = Frenet(current_state.posG.projF(cr_scenario.lanelet_network, [current_lane.lanelet_id]), cr_scenario.lanelet_network)

        target_state = cr_scenario.obstacles[startframe+step][vehicle_id].initial_state
        target_posF = target_state.posF
        target_lane = cr_scenario.lanelet_network.find_lanelet_by_id(target_posF.ind[1])
        origin_target_lanelet_id = target_lane.lanelet_id
        if target_lane.adj_right is not None:
            target_lane = cr_scenario.lanelet_network.find_lanelet_by_id(target_lane.adj_right)
            target_posF = Frenet(target_state.posG.projF(cr_scenario.lanelet_network, [target_lane.lanelet_id]), cr_scenario.lanelet_network)

        if target_posF.ind[1] == current_posF.ind[1]:
            delta_s = target_posF.s - current_posF.s
            delta_d = target_posF.d - current_posF.d
            #target_lanelet_id = origin_target_lanelet_id
        else:
            posG = target_state.get_posG()
            posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [current_posF.ind[1]]), cr_scenario.lanelet_network)
            max_ind = len(current_lane.center_curve)-2
            if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0 and len(current_lane.predecessor) > 0:
                # normally this should not happen
                delta_s = 0.0
                delta_d = posF_adjust.d - current_posF.d
            elif posF_adjust.ind[0].i >= max_ind and posF_adjust.ind[0].t == 1.0 and len(current_lane.successor) > 0:
                #TODO In the roundabout, this should be different
                dist_s = [0.0]
                cands = [current_posF.ind[1]]
                delta_d = None
                while len(cands) > 0:
                    this_cand = cands.pop(0)
                    this_dist_s = dist_s.pop(0)
                    if this_cand == target_posF.ind[1]:
                        delta_s = target_posF.s + this_dist_s - current_posF.s
                        delta_d = target_posF.d - current_posF.d
                        #target_lanelet_id = origin_target_lanelet_id
                        break
                    else:
                        this_lane = cr_scenario.lanelet_network.find_lanelet_by_id(this_cand)
                        posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [this_cand]), cr_scenario.lanelet_network)
                        if posF_adjust.ind[0].i >= len(this_lane.center_curve)-2 and posF_adjust.ind[0].t == 1.0:
                            for succ in this_lane.successor:
                                successor_lane = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                                cands.append(succ)
                                dist_s.append(this_dist_s+this_lane.center_curve[-1].s)
                        else:
                            if delta_d is None:
                                delta_s = posF_adjust.s + this_dist_s - current_posF.s
                                delta_d = posF_adjust.d - current_posF.d
                                #target_lanelet_id = origin_target_lanelet_id

                            elif np.abs(posF_adjust.d - current_posF.d) < np.abs(delta_d):
                                delta_s = posF_adjust.s + this_dist_s - current_posF.s
                                delta_d = posF_adjust.d - current_posF.d
                                #target_lanelet_id = origin_target_lanelet_id
            else:
                delta_s = posF_adjust.s - current_posF.s
                delta_d = posF_adjust.d - current_posF.d
                #target_lanelet_id = origin_target_lanelet_id

        return delta_s, delta_d
    def get_features(self, cr_scenario, vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        grid_s, grid_index = cr_scenario.locate_on_grid(startframe, vehicle_id)
        self.features[self.feature_index["egoid"]] = vehicle_id
        self.features[self.feature_index["relative_offset"]] = ego_vehicle.initial_state.posF.d
        self.features[self.feature_index["relative_displa"]] = grid_s
        self.features[self.feature_index["velocity_s"]] = np.cos(ego_vehicle.initial_state.posF.phi)*ego_vehicle.initial_state.velocity
        self.features[self.feature_index["velocity_d"]] = np.sin(ego_vehicle.initial_state.posF.phi)*ego_vehicle.initial_state.velocity
        self.features[self.feature_index["grid_index"]] = grid_index
        self.features[self.feature_index["length"]] = ego_vehicle.obstacle_shape.length
        self.features[self.feature_index["width"]] = ego_vehicle.obstacle_shape.width
        delta_step = int(self.step_length/cr_scenario.dt) # should be 10 if dt = 0.1
        horizon_step = int(self.horizon/cr_scenario.dt)
        if vehicle_id in cr_scenario.obstacles[startframe+horizon_step].keys():
            delta_s,_ = self.get_delta(cr_scenario, vehicle_id, startframe, horizon_step)
            if delta_s >= 0:
                _, self.features[self.feature_index["target_grid_index"]] = cr_scenario.locate_on_grid(startframe+horizon_step, vehicle_id)
            else:
                self.features[self.feature_index["target_grid_index"]] = grid_index
        elif vehicle_id in cr_scenario.obstacles[startframe+1].keys():
            i = 1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe+horizon_step-i].keys():
                    delta_s,_ = self.get_delta(cr_scenario, vehicle_id, startframe, horizon_step-i)
                    if delta_s >= 0:
                        _, self.features[self.feature_index["target_grid_index"]] = cr_scenario.locate_on_grid(startframe+horizon_step-i, vehicle_id)
                    else:
                        self.features[self.feature_index["target_grid_index"]] = grid_index
                    break
                i += 1
        else:
            self.features[self.feature_index["target_grid_index"]] = grid_index

        if vehicle_id in cr_scenario.obstacles[startframe+delta_step].keys():
            self.features[self.feature_index["target_delta_s"]],\
            self.features[self.feature_index["target_delta_d"]] = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step)
            target_state = cr_scenario.obstacles[startframe+delta_step][vehicle_id].initial_state
            self.features[self.feature_index["target_velocity_s"]] = np.cos(target_state.posF.phi)*target_state.velocity
            self.features[self.feature_index["target_velocity_d"]] = np.sin(target_state.posF.phi)*target_state.velocity

        elif vehicle_id in cr_scenario.obstacles[startframe+1].keys():
            i = 1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe+delta_step-i].keys():
                    delta_s,\
                    delta_d = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step-i)
                    self.features[self.feature_index["target_delta_s"]] = delta_s*delta_step/(delta_step-i)
                    self.features[self.feature_index["target_delta_d"]] = delta_d*delta_step/(delta_step-i)

                    target_state = cr_scenario.obstacles[startframe+delta_step-i][vehicle_id].initial_state
                    self.features[self.feature_index["target_velocity_s"]] = np.cos(target_state.posF.phi)*target_state.velocity
                    self.features[self.feature_index["target_velocity_d"]] = np.sin(target_state.posF.phi)*target_state.velocity
                    break
                i += 1
            assert(i < delta_step)
        else:
            i = 0
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe-delta_step+i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe-delta_step+i, delta_step-i)
                    self.features[self.feature_index["target_delta_s"]] = delta_s*delta_step/(delta_step-i)
                    self.features[self.feature_index["target_delta_d"]] = delta_d*delta_step/(delta_step-i)

                    target_state = cr_scenario.obstacles[startframe-delta_step+i][vehicle_id].initial_state
                    self.features[self.feature_index["target_velocity_s"]] = np.cos(target_state.posF.phi)*target_state.velocity
                    self.features[self.feature_index["target_velocity_d"]] = np.sin(target_state.posF.phi)*target_state.velocity

                    break
                i += 1
            assert(i < delta_step)

        return self.features

class CoreFeatureExtractor1:
    def __init__(self):
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
                              "target_velocity_d":{"high":40, "low":0},
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
                              "target_velocity_d":9,
                              }
    def set_step_length_and_maneuver_horizon(self, step_length, horizon):
        self.step_length = step_length
        self.horizon = horizon

    def get_delta(self, cr_scenario, vehicle_id, startframe, step):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        current_state = ego_vehicle.initial_state
        current_posF = current_state.posF
        current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_posF.ind[1])
        origin_current_lanelet_id = current_lane.lanelet_id
        if current_lane.adj_right is not None: # this is for rounD dataset
            current_lane = cr_scenario.lanelet_network.find_lanelet_by_id(current_lane.adj_right)
            current_posF = Frenet(current_state.posG.projF(cr_scenario.lanelet_network, [current_lane.lanelet_id]), cr_scenario.lanelet_network)

        target_state = cr_scenario.obstacles[startframe+step][vehicle_id].initial_state
        target_posF = target_state.posF
        target_lane = cr_scenario.lanelet_network.find_lanelet_by_id(target_posF.ind[1])
        origin_target_lanelet_id = target_lane.lanelet_id
        if target_lane.adj_right is not None:
            target_lane = cr_scenario.lanelet_network.find_lanelet_by_id(target_lane.adj_right)
            target_posF = Frenet(target_state.posG.projF(cr_scenario.lanelet_network, [target_lane.lanelet_id]), cr_scenario.lanelet_network)

        if target_posF.ind[1] == current_posF.ind[1]:
            delta_s = target_posF.s - current_posF.s
            delta_d = target_posF.d - current_posF.d
            #target_lanelet_id = origin_target_lanelet_id
        else:
            posG = target_state.get_posG()
            posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [current_posF.ind[1]]), cr_scenario.lanelet_network)
            max_ind = len(current_lane.center_curve)-2
            if posF_adjust.ind[0].i == 0 and posF_adjust.ind[0].t == 0.0 and len(current_lane.predecessor) > 0:
                # normally this should not happen
                delta_s = 0.0
                delta_d = posF_adjust.d - current_posF.d
            elif posF_adjust.ind[0].i >= max_ind and posF_adjust.ind[0].t == 1.0 and len(current_lane.successor) > 0:
                #TODO In the roundabout, this should be different
                dist_s = [0.0]
                cands = [current_posF.ind[1]]
                delta_d = None
                while len(cands) > 0:
                    this_cand = cands.pop(0)
                    this_dist_s = dist_s.pop(0)
                    if this_cand == target_posF.ind[1]:
                        delta_s = target_posF.s + this_dist_s - current_posF.s
                        delta_d = target_posF.d - current_posF.d
                        #target_lanelet_id = origin_target_lanelet_id
                        break
                    else:
                        this_lane = cr_scenario.lanelet_network.find_lanelet_by_id(this_cand)
                        posF_adjust = Frenet(posG.projF(cr_scenario.lanelet_network, [this_cand]), cr_scenario.lanelet_network)
                        if posF_adjust.ind[0].i >= len(this_lane.center_curve)-2 and posF_adjust.ind[0].t == 1.0:
                            for succ in this_lane.successor:
                                successor_lane = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                                cands.append(succ)
                                dist_s.append(this_dist_s+this_lane.center_curve[-1].s)
                        else:
                            if delta_d is None:
                                delta_s = posF_adjust.s + this_dist_s - current_posF.s
                                delta_d = posF_adjust.d - current_posF.d
                                #target_lanelet_id = origin_target_lanelet_id

                            elif np.abs(posF_adjust.d - current_posF.d) < np.abs(delta_d):
                                delta_s = posF_adjust.s + this_dist_s - current_posF.s
                                delta_d = posF_adjust.d - current_posF.d
                                #target_lanelet_id = origin_target_lanelet_id
            else:
                delta_s = posF_adjust.s - current_posF.s
                delta_d = posF_adjust.d - current_posF.d
                #target_lanelet_id = origin_target_lanelet_id

        return delta_s, delta_d
    def get_features(self, cr_scenario, vehicle_id, startframe):
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        self.features[self.feature_index["egoid"]] = vehicle_id
        self.features[self.feature_index["relative_offset"]] = ego_vehicle.initial_state.posF.d
        self.features[self.feature_index["velocity_s"]] = np.cos(ego_vehicle.initial_state.posF.phi)*ego_vehicle.initial_state.velocity
        self.features[self.feature_index["velocity_d"]] = np.sin(ego_vehicle.initial_state.posF.phi)*ego_vehicle.initial_state.velocity
        self.features[self.feature_index["length"]] = ego_vehicle.obstacle_shape.length
        self.features[self.feature_index["width"]] = ego_vehicle.obstacle_shape.width
        delta_step = int(self.step_length/cr_scenario.dt) # should be 10 if dt = 0.1
        #horizon_step = int(self.horizon/cr_scenario.dt)
        if vehicle_id in cr_scenario.obstacles[startframe+delta_step].keys():
            self.features[self.feature_index["target_delta_s"]],\
            self.features[self.feature_index["target_delta_d"]] = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step)
            target_state = cr_scenario.obstacles[startframe+delta_step][vehicle_id].initial_state
            self.features[self.feature_index["target_velocity_s"]] = np.cos(target_state.posF.phi)*target_state.velocity
            self.features[self.feature_index["target_velocity_d"]] = np.sin(target_state.posF.phi)*target_state.velocity

        elif vehicle_id in cr_scenario.obstacles[startframe+1].keys():
            i = 1
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe+delta_step-i].keys():
                    delta_s,\
                    delta_d = self.get_delta(cr_scenario, vehicle_id, startframe, delta_step-i)
                    self.features[self.feature_index["target_delta_s"]] = delta_s*delta_step/(delta_step-i)
                    self.features[self.feature_index["target_delta_d"]] = delta_d*delta_step/(delta_step-i)

                    target_state = cr_scenario.obstacles[startframe+delta_step-i][vehicle_id].initial_state
                    self.features[self.feature_index["target_velocity_s"]] = np.cos(target_state.posF.phi)*target_state.velocity
                    self.features[self.feature_index["target_velocity_d"]] = np.sin(target_state.posF.phi)*target_state.velocity
                    break
                i += 1
            assert(i < delta_step)
        else:
            i = 0
            while True:
                if vehicle_id in cr_scenario.obstacles[startframe-delta_step+i].keys():
                    delta_s,delta_d = self.get_delta(cr_scenario, vehicle_id, startframe-delta_step+i, delta_step-i)
                    self.features[self.feature_index["target_delta_s"]] = delta_s*delta_step/(delta_step-i)
                    self.features[self.feature_index["target_delta_d"]] = delta_d*delta_step/(delta_step-i)

                    target_state = cr_scenario.obstacles[startframe-delta_step+i][vehicle_id].initial_state
                    self.features[self.feature_index["target_velocity_s"]] = np.cos(target_state.posF.phi)*target_state.velocity
                    self.features[self.feature_index["target_velocity_d"]] = np.sin(target_state.posF.phi)*target_state.velocity

                    break
                i += 1
            assert(i < delta_step)

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

class NeighborFeatureExtractor0:
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
    def set_step_length(self, step_length):
        self.step_length = step_length

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

class NeighborFeatureExtractor:
    def __init__(self):
        self.num_features = 5 * 7
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {}
        self.feature_names = feature_names = ["_veh_id", "_dist", "_offset", "_dist_ins", "_offset_ins"]
        feature_ranges = [{"high":2000,"low":1}, {"high":250.0, "low":-100.0}, {"high":4.0, "low":-4.0},{"high":250.0, "low":-100.0},{"high":250.0, "low":-100.0}]
        self.neighbor_names = neighbor_names = ["front1", "left1", "left2", "left3", "right1", "right2", "right3"]
        self.feature_index = {}
        index = 0
        for nn in neighbor_names:
            for i in range(len(feature_names)):
                self.feature_info[nn+feature_names[i]] = feature_ranges[i]
                self.feature_index[nn+feature_names[i]] = index
                index += 1
    def set_step_length_and_maneuver_horizon(self, step_length, horizon):
        self.step_length = step_length
        self.horizon = horizon
    def get_features(self, cr_scenario, vehicle_id, startframe):
        self.features = np.zeros(self.num_features, dtype=np.float32)
        ret = cr_scenario.get_attribute(NEIGHBOR_FORE_REAR_LEFT_MID_RIGHT, vehicle_id, startframe)
        # neighbors features: rel_pos, vel, length, width, rel_heading
        fore_M, fore_Mdist, fore_Moffset, fore_Mdist2, fore_Moffset2 = ret["front"]
        veh_id = fore_M
        if veh_id is not None:
            self.features[self.feature_index["front1_veh_id"]] = veh_id
            self.features[self.feature_index["front1_dist"]] = fore_Mdist
            self.features[self.feature_index["front1_offset"]] = fore_Moffset
            self.features[self.feature_index["front1_dist_ins"]] = fore_Mdist2
            self.features[self.feature_index["front1_offset_ins"]] = fore_Moffset2
        else:
            self.features[self.feature_index["front1_veh_id"]] = -100
            self.features[self.feature_index["front1_dist"]] = -100
            self.features[self.feature_index["front1_offset"]] = -100
            self.features[self.feature_index["front1_dist_ins"]] = -100
            self.features[self.feature_index["front1_offset_ins"]] = -100

        Ls, Ldists, Loffsets, Ldists2, Loffsets2 = ret["left"]
        veh_id = Ls[0]
        if veh_id is not None:
            self.features[self.feature_index["left1_veh_id"]] = veh_id
            self.features[self.feature_index["left1_dist"]] = Ldists[0]
            self.features[self.feature_index["left1_offset"]] = Loffsets[0]
            self.features[self.feature_index["left1_dist_ins"]] = Ldists2[0]
            self.features[self.feature_index["left1_offset_ins"]] = Loffsets2[0]

        else:
            self.features[self.feature_index["left1_veh_id"]] = -100
            self.features[self.feature_index["left1_dist"]] = -100
            self.features[self.feature_index["left1_offset"]] = -100
            self.features[self.feature_index["left1_dist_ins"]] = -100
            self.features[self.feature_index["left1_offset_ins"]] = -100

        veh_id = Ls[1]
        if veh_id is not None:
            self.features[self.feature_index["left2_veh_id"]] = veh_id
            self.features[self.feature_index["left2_dist"]] = Ldists[1]
            self.features[self.feature_index["left2_offset"]] = Loffsets[1]
            self.features[self.feature_index["left2_dist_ins"]] = Ldists2[1]
            self.features[self.feature_index["left2_offset_ins"]] = Loffsets2[1]
        else:
            self.features[self.feature_index["left2_veh_id"]] = -100
            self.features[self.feature_index["left2_dist"]] = -100
            self.features[self.feature_index["left2_offset"]] = -100
            self.features[self.feature_index["left2_dist_ins"]] = -100
            self.features[self.feature_index["left2_offset_ins"]] = -100

        veh_id = Ls[2]
        if veh_id is not None:
            self.features[self.feature_index["left3_veh_id"]] = veh_id
            self.features[self.feature_index["left3_dist"]] = Ldists[2]
            self.features[self.feature_index["left3_offset"]] = Loffsets[2]
            self.features[self.feature_index["left3_dist_ins"]] = Ldists2[2]
            self.features[self.feature_index["left3_offset_ins"]] = Loffsets2[2]

        else:
            self.features[self.feature_index["left3_veh_id"]] = -100
            self.features[self.feature_index["left3_dist"]] = -100
            self.features[self.feature_index["left3_offset"]] = -100
            self.features[self.feature_index["left3_dist_ins"]] = -100
            self.features[self.feature_index["left3_offset_ins"]] = -100

        Rs, Rdists, Roffsets, Rdists2, Roffsets2 = ret["right"]
        veh_id = Rs[0]
        if veh_id is not None:
            self.features[self.feature_index["right1_veh_id"]] = veh_id
            self.features[self.feature_index["right1_dist"]] = Rdists[0]
            self.features[self.feature_index["right1_offset"]] = Roffsets[0]
            self.features[self.feature_index["right1_dist_ins"]] = Rdists2[0]
            self.features[self.feature_index["right1_offset_ins"]] = Roffsets2[0]

        else:
            self.features[self.feature_index["right1_veh_id"]] = -100
            self.features[self.feature_index["right1_dist"]] = -100
            self.features[self.feature_index["right1_offset"]] = -100
            self.features[self.feature_index["right1_dist_ins"]] = -100
            self.features[self.feature_index["right1_offset_ins"]] = -100

        veh_id = Rs[1]
        if veh_id is not None:
            self.features[self.feature_index["right2_veh_id"]] = veh_id
            self.features[self.feature_index["right2_dist"]] = Rdists[1]
            self.features[self.feature_index["right2_offset"]] = Roffsets[1]
            self.features[self.feature_index["right2_dist_ins"]] = Rdists2[1]
            self.features[self.feature_index["right2_offset_ins"]] = Roffsets2[1]
        else:
            self.features[self.feature_index["right2_veh_id"]] = -100
            self.features[self.feature_index["right2_dist"]] = -100
            self.features[self.feature_index["right2_offset"]] = -100
            self.features[self.feature_index["right2_dist_ins"]] = -100
            self.features[self.feature_index["right2_offset_ins"]] = -100
        veh_id = Rs[2]
        if veh_id is not None:
            self.features[self.feature_index["right3_veh_id"]] = veh_id
            self.features[self.feature_index["right3_dist"]] = Rdists[2]
            self.features[self.feature_index["right3_offset"]] = Roffsets[2]
            self.features[self.feature_index["right3_dist_ins"]] = Rdists2[2]
            self.features[self.feature_index["right3_offset_ins"]] = Roffsets2[2]
        else:
            self.features[self.feature_index["right3_veh_id"]] = -100
            self.features[self.feature_index["right3_dist"]] = -100
            self.features[self.feature_index["right3_offset"]] = -100
            self.features[self.feature_index["right3_dist_ins"]] = -100
            self.features[self.feature_index["right3_offset_ins"]] = -100

        return self.features

class VehicleToLaneletFeatureExtractor:
    def __init__(self):
        self.num_features = 10
        self.feature_info = {"isCurrent":{"high":1, "low":0},
                             "isTarget":{"high":1, "low":0},
                             "xs":{"high":40., "low":-40.},
                             "ys":{"high":40., "low":-40.},}
        self.feature_index = {"isCurrent":0,
                              "isTarget":1,
                              "xs":[2,3,4,5],
                              "ys":[6,7,8,9]}
    def set_range_and_horizon(self, max_s, horizon):
        self.max_s = max_s
        self.horizon = horizon
    def get_features(self, cr_scenario, vehicle_id, startframe):
        feature_inds = []
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        ego_posG = ego_vehicle.initial_state.posG
        ego_posF = ego_vehicle.initial_state.posF
        ego_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_posF.ind[1])
        ego_curve = ego_lanelet.center_curve
        _, current_index = cr_scenario.locate_on_grid(startframe, vehicle_id)
        ego_curvePt = lerp_curve(ego_curve[ego_posF.ind[0].i], ego_curve[ego_posF.ind[0].i+1], ego_posF.ind[0].t)
        lanelets_from_to_index = {}
        if ego_curvePt.s + self.max_s > ego_curve[-1].s:
            lanelets_from_to_index[ego_lanelet.lanelet_id] = (ego_posF.ind[0], CurveIndex(len(ego_curve)-2,1.0))
            for succ in ego_lanelet.successor:
                succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                succ_curve = succ_lanelet.center_curve
                if ego_curvePt.s +self.max_s > ego_curve[-1].s + succ_curve[-1].s:
                    lanelets_from_to_index[succ] = (CurveIndex(0,0.0), CurveIndex(len(succ_curve)-2,1.0))
                    if ego_lanelet.adj_left is None and succ_lanelet.adj_left is not None:
                        succ_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_left)
                        succ_left_curve = succ_left_lanelet.center_curve
                        lanelets_from_to_index[succ_lanelet.adj_left] = (CurveIndex(0,0.0), CurveIndex(len(succ_left_curve)-2,1.0))
                    if ego_lanelet.adj_right is None and succ_lanelet.adj_right is not None:
                        succ_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_right)
                        succ_right_curve = succ_right_lanelet.center_curve
                        lanelets_from_to_index[succ_lanelet.adj_right] = (CurveIndex(0,0.0), CurveIndex(len(succ_right_curve)-2,1.0))

                    for succ_succ in succ_lanelet.successor:
                        succ_succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ)
                        succ_succ_curve = succ_succ_lanelet.center_curve
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s-succ_curve[-1].s)
                        lanelets_from_to_index[succ_succ] = (CurveIndex(0,0.0), to_index)
                        if succ_succ_lanelet.adj_left is not None:
                            succ_succ_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ_lanelet.adj_left)
                            succ_succ_left_curve = succ_succ_left_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_left_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ_lanelet.adj_left] = (CurveIndex(0,0.0), to_index)
                        if succ_succ_lanelet.adj_right is not None:
                            succ_succ_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ_lanelet.adj_right)
                            succ_succ_right_curve = succ_succ_right_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_right_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ_lanelet.adj_right] = (CurveIndex(0,0.0), to_index)
                else:
                    to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s)
                    lanelets_from_to_index[succ] = (CurveIndex(0,0.0), to_index)
                    if ego_lanelet.adj_left is None and succ_lanelet.adj_left is not None:
                        succ_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_left)
                        succ_left_curve = succ_left_lanelet.center_curve
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_left_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s)
                        lanelets_from_to_index[succ_lanelet.adj_left] = (CurveIndex(0,0.0), to_index)
                    if ego_lanelet.adj_right is None and succ_lanelet.adj_right is not None:
                        succ_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_right)
                        succ_right_curve = succ_right_lanelet.center_curve
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_right_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s)
                        lanelets_from_to_index[succ_lanelet.adj_right] = (CurveIndex(0,0.0), to_index)
        else:
            to_index, _ = get_curve_index(ego_posF.ind[0], ego_curve, self.max_s)
            lanelets_from_to_index[ego_lanelet.lanelet_id] = (ego_posF.ind[0], to_index)
        if ego_lanelet.adj_left is not None:
            ego_left_posF = Frenet(ego_posG.projF(cr_scenario.lanelet_network, [ego_lanelet.adj_left]), cr_scenario.lanelet_network)
            ego_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_left_posF.ind[1])
            ego_left_curve = ego_left_lanelet.center_curve
            ego_left_curvePt = lerp_curve(ego_left_curve[ego_left_posF.ind[0].i], ego_left_curve[ego_left_posF.ind[0].i+1], ego_left_posF.ind[0].t)
            if ego_left_curvePt.s + self.max_s > ego_left_curve[-1].s:
                lanelets_from_to_index[ego_left_lanelet.lanelet_id] = (ego_left_posF.ind[0], CurveIndex(len(ego_left_curve)-2,1.0))
                for succ in ego_left_lanelet.successor:
                    succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                    succ_curve = succ_lanelet.center_curve
                    if ego_left_curvePt.s +self.max_s > ego_left_curve[-1].s + succ_curve[-1].s:
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), CurveIndex(len(succ_curve)-2,1.0))
                        for succ_succ in succ_lanelet.successor:
                            succ_succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ)
                            succ_succ_curve = succ_succ_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_curve, ego_left_curvePt.s+self.max_s-ego_left_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ] = (CurveIndex(0,0.0), to_index)
                    else:
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_curve, ego_left_curvePt.s+self.max_s-ego_left_curve[-1].s)
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), to_index)
            else:
                to_index, _ = get_curve_index(ego_left_posF.ind[0], ego_left_curve, self.max_s)
                lanelets_from_to_index[ego_left_lanelet.lanelet_id] = (ego_left_posF.ind[0], to_index)
        if ego_lanelet.adj_right is not None:
            ego_right_posF = Frenet(ego_posG.projF(cr_scenario.lanelet_network, [ego_lanelet.adj_right]), cr_scenario.lanelet_network)
            ego_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_right_posF.ind[1])
            ego_right_curve = ego_right_lanelet.center_curve
            ego_right_curvePt = lerp_curve(ego_right_curve[ego_right_posF.ind[0].i], ego_right_curve[ego_right_posF.ind[0].i+1], ego_right_posF.ind[0].t)
            if ego_right_curvePt.s + self.max_s > ego_right_curve[-1].s:
                lanelets_from_to_index[ego_right_lanelet.lanelet_id] = (ego_right_posF.ind[0], CurveIndex(len(ego_right_curve)-2,1.0))
                for succ in ego_right_lanelet.successor:
                    succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                    succ_curve = succ_lanelet.center_curve
                    if ego_right_curvePt.s +self.max_s > ego_right_curve[-1].s + succ_curve[-1].s:
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), CurveIndex(len(succ_curve)-2,1.0))
                        for succ_succ in succ_lanelet.successor:
                            succ_succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ)
                            succ_succ_curve = succ_succ_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_curve, ego_right_curvePt.s+self.max_s-ego_right_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ] = (CurveIndex(0,0.0), to_index)
                    else:
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_curve, ego_right_curvePt.s+self.max_s-ego_right_curve[-1].s)
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), to_index)
            else:
                to_index, _ = get_curve_index(ego_right_posF.ind[0], ego_right_curve, self.max_s)
                lanelets_from_to_index[ego_right_lanelet.lanelet_id] = (ego_right_posF.ind[0], to_index)
        #print("from_tos", lanelets_from_to_index)
        for index in range(len(cr_scenario.grids)):
            grid = cr_scenario.grids[index]
            if grid.from_lanelet_id in lanelets_from_to_index:
                from_index, to_index = lanelets_from_to_index[grid.from_lanelet_id]
                if to_index.i < grid.from_index.i or (to_index.i==grid.from_index.i and to_index.t < grid.from_index.t):
                    continue
                elif grid.to_lanelet_id == grid.from_lanelet_id:
                    if from_index.i > grid.to_index.i or (from_index.i==grid.to_index.i and from_index.t > grid.to_index.t):
                        continue
                    else:
                        feature_inds.append(index)
                else:
                    feature_inds.append(index)
            elif grid.to_lanelet_id in lanelets_from_to_index and grid.points[1][1] in lanelets_from_to_index:
                from_index, to_index = lanelets_from_to_index[grid.to_lanelet_id]
                if from_index.i > grid.to_index.i or (from_index.i==grid.to_index.i and from_index.t > grid.to_index.t):
                    continue
                elif grid.to_lanelet_id == grid.from_lanelet_id:
                    if to_index.i < grid.from_index.i or (to_index.i==grid.from_index.i and to_index.t < grid.from_index.t):
                        continue
                    else:
                        feature_inds.append(index)
                else:
                    feature_inds.append(index)
            elif index == current_index:
                feature_inds.append(index)
            else:
                continue
        horizon_step = int(self.horizon/cr_scenario.dt)
        if vehicle_id in cr_scenario.obstacles[startframe+horizon_step].keys():
            _, target_index = cr_scenario.locate_on_grid(startframe+horizon_step, vehicle_id, initial_cands=feature_inds)
        else:
            _, target_index = cr_scenario.locate_on_grid(startframe, vehicle_id, initial_cands=feature_inds)


        assert current_index in feature_inds
        assert target_index in feature_inds
        final_features = {}
        for feature_ind in feature_inds:
            if current_index == feature_ind:
                final_features[feature_ind] = [1]
            else:
                final_features[feature_ind] = [0]
            if target_index == feature_ind:
                final_features[feature_ind].append(1)
            else:
                final_features[feature_ind].append(0)
            xs = []
            ys = []
            for pindex, planelet_id in cr_scenario.grids[feature_ind].points:
                planelet = cr_scenario.lanelet_network.find_lanelet_by_id(planelet_id)
                p_curve = planelet.center_curve
                pcurve_pt = lerp_curve(p_curve[pindex.i],p_curve[pindex.i+1],pindex.t)
                pproj = pcurve_pt.pos.inertial2body(ego_posG)
                xs.append(pproj.x)
                ys.append(pproj.y)
            final_features[feature_ind].extend(xs)
            final_features[feature_ind].extend(ys)
        return final_features

class VehicleToLaneletFeatureExtractor1:
    def __init__(self):
        self.num_features = 10
        self.feature_info = {"isCurrent":{"high":1, "low":0},
                             "isTarget":{"high":1, "low":0},
                             "xs":{"high":40., "low":-40.},
                             "ys":{"high":40., "low":-40.},}
        self.feature_index = {"isCurrent":0,
                              "isTarget":1,
                              "xs":[2,3,4,5],
                              "ys":[6,7,8,9]}
    def set_range_and_horizon(self, max_s, horizon):
        self.max_s = max_s
        self.horizon = horizon
    def get_features(self, cr_scenario, vehicle_id, startframe):
        feature_inds = []
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        ego_posG = ego_vehicle.initial_state.posG
        ego_posF = ego_vehicle.initial_state.posF
        ego_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_posF.ind[1])
        ego_curve = ego_lanelet.center_curve
        current_index = cr_scenario.locate_on_grid1(startframe, vehicle_id)
        ego_curvePt = lerp_curve(ego_curve[ego_posF.ind[0].i], ego_curve[ego_posF.ind[0].i+1], ego_posF.ind[0].t)
        lanelets_from_to_index = {}
        if ego_curvePt.s + self.max_s > ego_curve[-1].s:
            lanelets_from_to_index[ego_lanelet.lanelet_id] = (ego_posF.ind[0], CurveIndex(len(ego_curve)-2,1.0))
            for succ in ego_lanelet.successor:
                succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                succ_curve = succ_lanelet.center_curve
                if ego_curvePt.s +self.max_s > ego_curve[-1].s + succ_curve[-1].s:
                    lanelets_from_to_index[succ] = (CurveIndex(0,0.0), CurveIndex(len(succ_curve)-2,1.0))
                    if ego_lanelet.adj_left is None and succ_lanelet.adj_left is not None:
                        succ_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_left)
                        succ_left_curve = succ_left_lanelet.center_curve
                        lanelets_from_to_index[succ_lanelet.adj_left] = (CurveIndex(0,0.0), CurveIndex(len(succ_left_curve)-2,1.0))
                    if ego_lanelet.adj_right is None and succ_lanelet.adj_right is not None:
                        succ_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_right)
                        succ_right_curve = succ_right_lanelet.center_curve
                        lanelets_from_to_index[succ_lanelet.adj_right] = (CurveIndex(0,0.0), CurveIndex(len(succ_right_curve)-2,1.0))

                    for succ_succ in succ_lanelet.successor:
                        succ_succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ)
                        succ_succ_curve = succ_succ_lanelet.center_curve
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s-succ_curve[-1].s)
                        lanelets_from_to_index[succ_succ] = (CurveIndex(0,0.0), to_index)
                        if succ_succ_lanelet.adj_left is not None:
                            succ_succ_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ_lanelet.adj_left)
                            succ_succ_left_curve = succ_succ_left_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_left_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ_lanelet.adj_left] = (CurveIndex(0,0.0), to_index)
                        if succ_succ_lanelet.adj_right is not None:
                            succ_succ_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ_lanelet.adj_right)
                            succ_succ_right_curve = succ_succ_right_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_right_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ_lanelet.adj_right] = (CurveIndex(0,0.0), to_index)
                else:
                    to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s)
                    lanelets_from_to_index[succ] = (CurveIndex(0,0.0), to_index)
                    if ego_lanelet.adj_left is None and succ_lanelet.adj_left is not None:
                        succ_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_left)
                        succ_left_curve = succ_left_lanelet.center_curve
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_left_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s)
                        lanelets_from_to_index[succ_lanelet.adj_left] = (CurveIndex(0,0.0), to_index)
                    if ego_lanelet.adj_right is None and succ_lanelet.adj_right is not None:
                        succ_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_lanelet.adj_right)
                        succ_right_curve = succ_right_lanelet.center_curve
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_right_curve, ego_curvePt.s+self.max_s-ego_curve[-1].s)
                        lanelets_from_to_index[succ_lanelet.adj_right] = (CurveIndex(0,0.0), to_index)
        else:
            to_index, _ = get_curve_index(ego_posF.ind[0], ego_curve, self.max_s)
            lanelets_from_to_index[ego_lanelet.lanelet_id] = (ego_posF.ind[0], to_index)
        if ego_lanelet.adj_left is not None:
            ego_left_posF = Frenet(ego_posG.projF(cr_scenario.lanelet_network, [ego_lanelet.adj_left]), cr_scenario.lanelet_network)
            ego_left_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_left_posF.ind[1])
            ego_left_curve = ego_left_lanelet.center_curve
            ego_left_curvePt = lerp_curve(ego_left_curve[ego_left_posF.ind[0].i], ego_left_curve[ego_left_posF.ind[0].i+1], ego_left_posF.ind[0].t)
            if ego_left_curvePt.s + self.max_s > ego_left_curve[-1].s:
                lanelets_from_to_index[ego_left_lanelet.lanelet_id] = (ego_left_posF.ind[0], CurveIndex(len(ego_left_curve)-2,1.0))
                for succ in ego_left_lanelet.successor:
                    succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                    succ_curve = succ_lanelet.center_curve
                    if ego_left_curvePt.s +self.max_s > ego_left_curve[-1].s + succ_curve[-1].s:
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), CurveIndex(len(succ_curve)-2,1.0))
                        for succ_succ in succ_lanelet.successor:
                            succ_succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ)
                            succ_succ_curve = succ_succ_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_curve, ego_left_curvePt.s+self.max_s-ego_left_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ] = (CurveIndex(0,0.0), to_index)
                    else:
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_curve, ego_left_curvePt.s+self.max_s-ego_left_curve[-1].s)
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), to_index)
            else:
                to_index, _ = get_curve_index(ego_left_posF.ind[0], ego_left_curve, self.max_s)
                lanelets_from_to_index[ego_left_lanelet.lanelet_id] = (ego_left_posF.ind[0], to_index)
        if ego_lanelet.adj_right is not None:
            ego_right_posF = Frenet(ego_posG.projF(cr_scenario.lanelet_network, [ego_lanelet.adj_right]), cr_scenario.lanelet_network)
            ego_right_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_right_posF.ind[1])
            ego_right_curve = ego_right_lanelet.center_curve
            ego_right_curvePt = lerp_curve(ego_right_curve[ego_right_posF.ind[0].i], ego_right_curve[ego_right_posF.ind[0].i+1], ego_right_posF.ind[0].t)
            if ego_right_curvePt.s + self.max_s > ego_right_curve[-1].s:
                lanelets_from_to_index[ego_right_lanelet.lanelet_id] = (ego_right_posF.ind[0], CurveIndex(len(ego_right_curve)-2,1.0))
                for succ in ego_right_lanelet.successor:
                    succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ)
                    succ_curve = succ_lanelet.center_curve
                    if ego_right_curvePt.s +self.max_s > ego_right_curve[-1].s + succ_curve[-1].s:
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), CurveIndex(len(succ_curve)-2,1.0))
                        for succ_succ in succ_lanelet.successor:
                            succ_succ_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(succ_succ)
                            succ_succ_curve = succ_succ_lanelet.center_curve
                            to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_succ_curve, ego_right_curvePt.s+self.max_s-ego_right_curve[-1].s-succ_curve[-1].s)
                            lanelets_from_to_index[succ_succ] = (CurveIndex(0,0.0), to_index)
                    else:
                        to_index, _ = get_curve_index(CurveIndex(0,0.0), succ_curve, ego_right_curvePt.s+self.max_s-ego_right_curve[-1].s)
                        lanelets_from_to_index[succ] = (CurveIndex(0,0.0), to_index)
            else:
                to_index, _ = get_curve_index(ego_right_posF.ind[0], ego_right_curve, self.max_s)
                lanelets_from_to_index[ego_right_lanelet.lanelet_id] = (ego_right_posF.ind[0], to_index)

        for index in range(len(cr_scenario.grids)):
            grid = cr_scenario.grids[index]
            if index == current_index:
                feature_inds.append(index)
                continue
            for ind in grid.ind_list:
                if ind[1] in lanelets_from_to_index:
                    from_index, to_index = lanelets_from_to_index[ind[1]]
                    if from_index.i < ind[0].i or (from_index.i==ind[0].i and from_index.t < ind[0].t):
                        if to_index.i > ind[0].i or (to_index.i==ind[0].i and to_index.t > ind[0].t):
                            feature_inds.append(index)
                            break

        horizon_step = int(self.horizon/cr_scenario.dt)
        if vehicle_id in cr_scenario.obstacles[startframe+horizon_step].keys():
            target_index = cr_scenario.locate_on_grid1(startframe+horizon_step, vehicle_id, initial_cands=feature_inds)
        else:
            target_index = cr_scenario.locate_on_grid1(startframe, vehicle_id, initial_cands=feature_inds)


        assert current_index in feature_inds
        assert target_index in feature_inds
        final_features = {}
        for feature_ind in feature_inds:
            final_features[feature_ind] = {}
            if current_index == feature_ind:
                final_features[feature_ind]["current"] = 1
            else:
                final_features[feature_ind]["current"] = 0
            if target_index == feature_ind:
                final_features[feature_ind]["target"] = 1
            else:
                final_features[feature_ind]["target"] = 0
            points = []
            for pos in cr_scenario.grids[feature_ind].pos_list:
                pproj = pos.inertial2body(ego_posG)
                points.append((pproj.x, pproj.y, pproj.th))

            final_features[feature_ind]["point"] = points
        return final_features


class LaneletToLaneletFeatureExtractor:
    def __init__(self):
        self.num_features = 5
        self.feature_info = {
                              "index":{"high":None, "low":None},
                              #"is_target": {"high":1., "low":0.},
                              "relation" : {"high":1., "low":0.},
        }
        self.feature_index = {"index" : 0,
                              #"is_target": 1,
                              "relation" : [1,2,3,4],
        }
    def set_grid_length(self, grid_length):
        self.max_distance2 = (grid_length+2)**2
        self.grid_length = grid_length
    def get_features(self, cr_scenario, grid_index):
        features = []
        ego_grid = cr_scenario.grids[grid_index]
        ego_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ego_grid.center_lanelet_id)
        ego_curve = ego_lanelet.center_curve
        ego_curve_pt = lerp_curve(ego_curve[ego_grid.center_index.i], ego_curve[ego_grid.center_index.i+1], ego_grid.center_index.t)
        for index in range(len(cr_scenario.grids)):
            if index == grid_index:
                continue
            feature = np.zeros(self.num_features, dtype=np.float32)
            nei_grid = cr_scenario.grids[index]
            nei_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(nei_grid.center_lanelet_id)
            nei_curve = nei_lanelet.center_curve
            nei_curve_pt = lerp_curve(nei_curve[nei_grid.center_index.i], nei_curve[nei_grid.center_index.i+1], nei_grid.center_index.t)

            distance2 = (ego_curve_pt.pos.x-nei_curve_pt.pos.x)**2+(ego_curve_pt.pos.y-nei_curve_pt.pos.y)**2
            if distance2 > self.max_distance2:
                continue
            feature[self.feature_index["index"]] = index
            if ego_grid.to_lanelet_id == nei_grid.from_lanelet_id and ego_grid.to_index.i == nei_grid.from_index.i:
                feature[self.feature_index["relation"]] = [0,0,1,0]
                features.append(feature)
                continue

            if ego_lanelet.adj_left == nei_lanelet.lanelet_id:
                projF = Frenet(ego_curve_pt.pos.projF(cr_scenario.lanelet_network, [nei_lanelet.lanelet_id]), cr_scenario.lanelet_network)
                if projF.s-nei_curve_pt.s >= -self.grid_length and projF.s-nei_curve_pt.s <= self.grid_length:
                    feature[self.feature_index["relation"]] = [1,0,0,0]
                    features.append(feature)
                    continue

            if ego_lanelet.adj_right == nei_lanelet.lanelet_id:
                projF = Frenet(ego_curve_pt.pos.projF(cr_scenario.lanelet_network, [nei_lanelet.lanelet_id]), cr_scenario.lanelet_network)
                if projF.s-nei_curve_pt.s >= -self.grid_length and projF.s-nei_curve_pt.s <= self.grid_length:
                    feature[self.feature_index["relation"]] = [0,1,0,0]
                    features.append(feature)
                    continue

            intersect = [lanelet_id for lanelet_id in ego_lanelet.successor if lanelet_id in nei_lanelet.successor]
            if len(intersect) != 0:
                projF = Frenet(ego_curve_pt.pos.projF(cr_scenario.lanelet_network, [nei_lanelet.lanelet_id]), cr_scenario.lanelet_network)
                if projF.s-nei_curve_pt.s >= -self.grid_length and projF.s-nei_curve_pt.s <= self.grid_length:
                    feature[self.feature_index["relation"]] = [0,0,0,1]
                    features.append(feature)
                    continue
        return features

class LaneletToLaneletFeatureExtractor1:
    def __init__(self):
        self.num_features = 5
        self.feature_info = {
                              "index":{"high":None, "low":None},
                              "relation" : {"high":1., "low":0.},
        }
        self.feature_index = {"index" : 0,
                              "relation" : [1,2,3,4],
        }
    def set_grid_length(self, grid_length):
        self.max_distance2 = (grid_length+2)**2
        self.grid_length = grid_length
    def get_features(self, cr_scenario, grid_index):
        features = []
        ego_grid = cr_scenario.grids[grid_index]
        for front in ego_grid.front_list:
            feature = np.zeros(self.num_features, dtype=np.float32)
            feature[self.feature_index["index"]] = front
            feature[self.feature_index["relation"]] = [0,0,1,0]
            features.append(feature)
        for left in ego_grid.left_list:
            feature = np.zeros(self.num_features, dtype=np.float32)
            feature[self.feature_index["index"]] = left
            feature[self.feature_index["relation"]] = [1,0,0,0]
            features.append(feature)
        for right in ego_grid.right_list:
            feature = np.zeros(self.num_features, dtype=np.float32)
            feature[self.feature_index["index"]] = right
            feature[self.feature_index["relation"]] = [0,1,0,0]
            features.append(feature)
        for conflict in ego_grid.conflict_list:
            feature = np.zeros(self.num_features, dtype=np.float32)
            feature[self.feature_index["index"]] = conflict
            feature[self.feature_index["relation"]] = [0,0,0,1]
            features.append(feature)
        return features

class LaneletFeatureExtractor:
    def __init__(self):
        self.num_features = 7#15
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {
                             "marker_left" : {"high": 1., "low":0.},
                             "marker_right" : {"high": 1., "low":0.},
                             "priority" : {"high":1., "low":0.},
                             }
        self.feature_index = {

                             "marker_left" : [0,1,2],
                             "marker_right" : [3,4,5],
                             "priority" : 6,
                             }
    def get_features(self, cr_scenario, grid_index, vehicle_id, startframe):
        self.features = np.zeros(self.num_features, dtype=np.float32)
        grid = cr_scenario.grids[grid_index]
        lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(grid.center_lanelet_id)
        if lanelet.line_marking_left_vertices ==  LineMarking.DASHED:
            self.features[self.feature_index["marker_left"]] = [1,0,0]
        elif lanelet.line_marking_left_vertices ==  LineMarking.SOLID:
            self.features[self.feature_index["marker_left"]] = [0,1,0]
        elif lanelet.line_marking_left_vertices ==  LineMarking.NO:
            self.features[self.feature_index["marker_left"]] = [0,0,1]

        if lanelet.line_marking_right_vertices ==  LineMarking.DASHED:
            self.features[self.feature_index["marker_right"]] = [1,0,0]
        elif lanelet.line_marking_right_vertices ==  LineMarking.SOLID:
            self.features[self.feature_index["marker_right"]] = [0,1,0]
        elif lanelet.line_marking_right_vertices ==  LineMarking.NO:
            self.features[self.feature_index["marker_right"]] = [0,0,1]

        if  LaneletType.ACCESS_RAMP in lanelet.lanelet_type or LaneletType.EXIT_RAMP in lanelet.lanelet_type:
            self.features[self.feature_index["priority"]] = 0.0
        else:
            self.features[self.feature_index["priority"]] = 1.0
        ###
        '''
        ego_vehicle = cr_scenario.obstacles[startframe][vehicle_id]
        ego_posG = ego_vehicle.initial_state.posG
        start_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(grid.from_lanelet_id)
        start_curve = start_lanelet.center_curve
        start_curve_pt = lerp_curve(start_curve[grid.from_index.i], start_curve[grid.from_index.i+1], grid.from_index.t)
        start_proj = start_curve_pt.pos.inertial2body(ego_posG)
        self.features[self.feature_index["start_x"]] = start_proj.x
        self.features[self.feature_index["start_y"]] = start_proj.y

        middle_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(grid.center_lanelet_id)
        middle_curve = middle_lanelet.center_curve
        middle_curve_pt = lerp_curve(middle_curve[grid.center_index.i], middle_curve[grid.center_index.i+1], grid.center_index.t)
        middle_proj = middle_curve_pt.pos.inertial2body(ego_posG)
        self.features[self.feature_index["middle_x"]] = middle_proj.x
        self.features[self.feature_index["middle_y"]] = middle_proj.y
        #self.features[self.feature_index["middle_s"]] = grid.center_s

        end_lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(grid.to_lanelet_id)
        end_curve = end_lanelet.center_curve
        end_curve_pt = lerp_curve(end_curve[grid.to_index.i], end_curve[grid.to_index.i+1], grid.to_index.t)
        end_proj = end_curve_pt.pos.inertial2body(ego_posG)
        self.features[self.feature_index["end_x"]] = end_proj.x
        self.features[self.feature_index["end_y"]] = end_proj.y
        #self.features[self.feature_index["end_s"]] = grid.to_s
        '''
        return self.features

class LaneletFeatureExtractor1:
    def __init__(self):
        self.num_features = 7#15
        self.features = np.zeros(self.num_features, dtype=np.float32)
        self.feature_info = {
                             "marker_left" : {"high": 1., "low":0.},
                             "marker_right" : {"high": 1., "low":0.},
                             "priority" : {"high":1., "low":0.},
                             }
        self.feature_index = {

                             "marker_left" : [0,1,2],
                             "marker_right" : [3,4,5],
                             "priority" : 6,
                             }
    def get_features(self, cr_scenario, grid_index, vehicle_id, startframe):
        self.features = np.zeros(self.num_features, dtype=np.float32)
        grid = cr_scenario.grids[grid_index]
        marker_left_list = []
        marker_right_list = []
        priority_list = []
        for ind in grid.ind_list:
            lanelet = cr_scenario.lanelet_network.find_lanelet_by_id(ind[1])
            if lanelet.line_marking_left_vertices ==  LineMarking.DASHED:
                marker_left_list.append(1)
            elif lanelet.line_marking_left_vertices ==  LineMarking.SOLID:
                marker_left_list.append(2)
            elif lanelet.line_marking_left_vertices ==  LineMarking.NO:
                marker_left_list.append(3)

            if lanelet.line_marking_right_vertices ==  LineMarking.DASHED:
                marker_right_list.append(1)
            elif lanelet.line_marking_right_vertices ==  LineMarking.SOLID:
                marker_right_list.append(2)
            elif lanelet.line_marking_right_vertices ==  LineMarking.NO:
                marker_right_list.append(3)

            if  LaneletType.ACCESS_RAMP in lanelet.lanelet_type or LaneletType.EXIT_RAMP in lanelet.lanelet_type:
                priority_list.append(0)
            else:
                priority_list.append(1)
        try:
            marker_left = max(set(marker_left_list), key=marker_left_list.count)
        except:
            print(grid.ind_list, grid.pos_list, grid.id)
            raise
        marker_right = max(set(marker_right_list), key=marker_right_list.count)
        priority = max(set(priority_list), key=priority_list.count)
        if marker_left == 1:
            self.features[self.feature_index["marker_left"]] = [1,0,0]
        elif marker_left == 2:
            self.features[self.feature_index["marker_left"]] = [0,1,0]
        elif marker_left == 3:
            self.features[self.feature_index["marker_left"]] = [0,0,1]

        if marker_right == 1:
            self.features[self.feature_index["marker_right"]] = [1,0,0]
        elif marker_right == 2:
            self.features[self.feature_index["marker_right"]] = [0,1,0]
        elif marker_right == 3:
            self.features[self.feature_index["marker_right"]] = [0,0,1]

        self.features[self.feature_index["priority"]] = priority
        ###

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
    def __init__(self, extractors=[CoreFeatureExtractor(),
                           WellBehavedFeatureExtractor(),
                           LaneletFeatureExtractor(),
                           HistoryFeatureExtractor(),
                           NeighborFeatureExtractor()]):

        self.extractors = extractors
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
    def set_step_length_and_maneuver_horizon(self, step_length, horizon):
        for subext in self.extractors:
            subext.set_step_length_and_maneuver_horizon(step_length, horizon)

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
                try:
                    ret.extend(features[self.feature_index[n]])
                except:
                    ret.append(features[self.feature_index[n]])
        return ret
