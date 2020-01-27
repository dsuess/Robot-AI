"""
Solver for Assignment 2 by team Love Triangle
For COMP3702, Semester 2, 2019, University of Queensland
"""
#!/usr/bin/env python3
from support import angle, obstacle, problem_spec, robot_config
import tester
import argparse, time, sys, math, random, queue
from numpy import arange

# Handle cmd line args
parser = argparse.ArgumentParser()
parser.add_argument("input_file", type=str, help="initial configuration of robot and map")
parser.add_argument("output_file", type=str, help="name of output file containing all the moves")
args = parser.parse_args()

if not (args.input_file.endswith('.txt')):
    parser.print_help()
    sys.exit(1)

input_file = args.input_file
output_file = open(args.output_file, 'w')

problem:problem_spec.ProblemSpec = problem_spec.ProblemSpec(input_file)
grapples = problem.grapple_points
obstacles = problem.obstacles
init_robot:robot_config.RobotConfig = problem.initial
goal_config:robot_config.RobotConfig = problem.goal

PRIMITIVE_STEP = 1e-3
RECURSION_DEPTH = 1e-4
TOLERANCE = 1e-5
N = 10
INTERPOLATE_N = 500
RAD_THRESH_GLOBAL = 0.3 #between 0 and 1
RAD_THRESH_MIN = 0.01
LEN_THRESH_GLOBAL = 0.1
LEN_THRESH_MIN = 0.01
DELTA_THRESH = 0.01
DISCREETE_STEPS = 0.001
DISCREETE_TOLERANCE = 0.05
COLLISION_TOLERANCE = 0.001
MAX_RADIANS = (11 * math.pi / 12)

#Find path from start to goal
moves = {
    'l': (DISCREETE_STEPS,0),
    'r': (-DISCREETE_STEPS, 0),
    'u': (0, DISCREETE_STEPS),
    'd': (0, -DISCREETE_STEPS)
}
possible_moves = ['l', 'r', 'u', 'd']

def make_move(coord:tuple, move:str):
    coord_delta = moves[move]
    return (coord[0] + coord_delta[0], coord[1] + coord_delta[1])

def man_dist(init_coord:tuple, end_coord:tuple):
    # init_coord = (round(init_coord[0], 5), round(init_coord[1], 5))
    # end_coord = (round(end_coord[0], 5), round(end_coord[1], 5))
    
    return round(abs(init_coord[0]-end_coord[0]) + abs(init_coord[1]-end_coord[1]), 6)

def close_enough(start_coord:tuple, end_coord:tuple):
    x = start_coord[0]
    y = start_coord[1]
    return (x<end_coord[0]+DISCREETE_TOLERANCE) and (x>end_coord[0]-DISCREETE_TOLERANCE)\
        and (y<end_coord[1]+DISCREETE_TOLERANCE) and (y>end_coord[1]-DISCREETE_TOLERANCE)

def valid_point(coord:tuple):
    x = coord[0]
    y = coord[1]
    if x>1 or x<0 or y>1 or y<0:
        return False 
    res = True
    for obstacle in obstacles:
        # print("obstacle y1: ", obstacle.y1)
        # print("obstacle y2: ", obstacle.y2)
        if (obstacle.x1==obstacle.x2):
            if (y<obstacle.y2) and (y>obstacle.y1):
                res = False
        elif (obstacle.y1==obstacle.y2):
            if (x<obstacle.x2) and (x>obstacle.x1):
                res = False
        elif (x<obstacle.x2 ) and (x>obstacle.x1 ) and\
            (y<obstacle.y2 ) and (y>obstacle.y1 ):
            res = False
    
    return res 


#change moves to path
def get_path(start:tuple, moves:str):
    res_path = [start]
    for move in moves:
        start = make_move(start, move)
        res_path.append(start)

    return res_path

# discreete path finder. deprecated
# def find_path(start_coord:tuple, end_coord:tuple):

#     path_counter = 0
#     path_q = queue.PriorityQueue()
#     path_q.put((1, path_counter, '', init_coord))
#     solutions = []
#     explored = []

#     while(True):
#         # print(len(path_q.queue))
#         if (len(path_q.queue)==0):
#             print("failed to find path... exiting")
#             exit()
#         else:
#             man, path_count, move_str, current_coord = path_q.get()
#             # # print(man)
#             # temp_bot = tracking_robot(current_coord)
#             # output_file.write(str(temp_bot) + '\n')
#             # print('current_cooord: ', current_coord, \
#             #     'target_grap: ', target_grapple,\
#             #         close_enough(current_coord, target_grapple))
#             if (close_enough(current_coord, target_grapple)):
#                 solutions.append(move_str)
#                 break
            
#             if move_str in explored:
#                 continue
#             else:
#                 explored.append(move_str)
#                 for move in possible_moves:
#                     next_coord = make_move(current_coord, move)
#                     if valid_point(next_coord):
#                         current_moves = move_str + move
#                         path_counter = path_counter +1
#                         man = man_dist(next_coord, target_grapple)
#                         path_q.put((man, path_count, current_moves, next_coord))

#     test_path = get_path(init_coord, min(solutions))

#     return test_path


#make visualisation for path finder:
#make a robot with ee2 end at path point
def tracking_robot(coord:tuple):
    new_angles = []
    for idx, angObj in enumerate(init_robot.ee1_angles):
        
        new_angle = angle.Angle(radians=0)

        new_angles.append(new_angle)

    new_len = problem.min_lengths

    new_robot = robot_config.make_robot_config_from_ee2(coord[0], coord[1], new_angles, new_len, False, False)
    return new_robot

# Custom collision tests:
def custom_test_environment_bounds(config):
    # return true for pass, false for fail
    for x, y in config.points:
        if not DISCREETE_TOLERANCE <= x <= 1.0 -DISCREETE_TOLERANCE:
            return False
        if not DISCREETE_TOLERANCE <= y <= 1.0 -DISCREETE_TOLERANCE:
            return False
    return True

def custom_test_bounding_box(p1, q1, p2, q2):
    # return true for collision possible, false otherwise
    p1x, p1y = p1
    q1x, q1y = q1
    p2x, p2y = p2
    q2x, q2y = q2
    x1_min = min(p1x, q1x)
    x1_max = max(p1x, q1x)
    x2_min = min(p2x, q2x)
    x2_max = max(p2x, q2x)
    if x1_max < x2_min or x2_max < x1_min:
        return False

    y1_min = min(p1y, q1y)
    y1_max = max(p1y, q1y)
    y2_min = min(p2y, q2y)
    y2_max = max(p2y, q2y)
    if y1_max < y2_min or y2_max < y1_min:
        return False

    return True

def custom_test_obstacle_collision(config:robot_config.RobotConfig, spec:problem_spec.ProblemSpec, obstacles):
    # return true for pass, false for fail
    for i in range(spec.num_segments):
        p = config.points[i]
        q = config.points[i+1]
        for o in obstacles:
            line_start_x = p[0]
            line_end_x = q[0]
            line_start_y = p[1]
            line_end_y = q[1]

            for j in range(12):
                delta_x = (line_end_x -line_start_x)/(j+1)
                delta_y = (line_end_y -line_start_y)/(j+1)
                if j==0:
                    delta_x = 0
                    delta_y = 0
                elif j==11:
                    delta_x = (line_end_x -line_start_x)
                    delta_y = (line_end_y -line_start_y)

                x = line_start_x+delta_x
                y = line_start_y+delta_y

                if (o.x1==o.x2):
                    if (y<o.y2 +COLLISION_TOLERANCE) and (y>o.y1 -COLLISION_TOLERANCE):
                        return False
                elif (o.y1==o.y2):
                    if (x<o.x2 +COLLISION_TOLERANCE) and (x>o.x1 -COLLISION_TOLERANCE):
                        return False
                elif (x<o.x2 +COLLISION_TOLERANCE) and (x>o.x1 -COLLISION_TOLERANCE) and\
                    (y<o.y2 +COLLISION_TOLERANCE) and (y>o.y1 -COLLISION_TOLERANCE):
                    return False
            # bounding box check
            if not custom_test_bounding_box(p, q, (o.x1, o.y1), (o.x2, o.y2)):
                continue

            # full edge check
            for e in o.edges:
                if tester.test_line_collision((p, q), e):
                    # collision between robot segment and obstacle edge
                    return False
    return True
def get_lenient_obstacle_bounds(o, spec):
    """
    This method should only be used by tester. To avoid unexpected errors in your solution caused by floating point
    noise, you should not use this method in your solver.
    """
    # shrink obstacle by TOLERANCE in each direction
    return obstacle.Obstacle(o.x1 - COLLISION_TOLERANCE, o.y1 - COLLISION_TOLERANCE,
                    o.x2 + COLLISION_TOLERANCE, o.y2 + COLLISION_TOLERANCE)


def custom_get_lenient_obstacles(spec):
    """
    This method should only be used by tester. To avoid unexpected errors in your solution caused by floating point
    noise, you should not use this method in your solver.
    """
    # shrink all obstacles by TOLERANCE
    obstacles = []
    for o in spec.obstacles:
        obstacles.append(get_lenient_obstacle_bounds(o, spec))
    return obstacles

#Robot stuff
def config_validity(config:robot_config.RobotConfig, spec:problem_spec.ProblemSpec):
    #tester.test_obstacle_collision(config, spec, spec.obstacles
    return (custom_test_obstacle_collision(config, problem, custom_get_lenient_obstacles(spec))\
        and tester.test_self_collision(config, spec)\
        and tester.test_length_constraints(config, spec)\
        and tester.test_angle_constraints(config, spec)\
        and tester.test_environment_bounds(config))

def distance_to_grapple(config:robot_config.RobotConfig, grapple_coord:tuple):
    grap_x = grapple_coord[0]
    grap_y = grapple_coord[1]
    robot = config.get_ee1() if config.ee2_grappled else config.get_ee2()
    robot_x = robot[0]
    robot_y = robot[1]

    return math.sqrt(math.pow(grap_x-robot_x, 2) + math.pow(grap_y-robot_y, 2))

# def can_move_between(c1:robot_config.RobotConfig, c2:robot_config.RobotConfig):
#     res = True
#     return res
def inter_config_validity(c1:robot_config.RobotConfig, c2:robot_config.RobotConfig, spec:problem_spec.ProblemSpec):
    
    if(inter_config_distance_heuristic(c1, c2)<0.01): #PRIMITIVE_STEP
        return True
    else:
        inter1 = inter_config(c1, c2, spec)
        if config_validity(c1, spec):
            return inter_config_validity(c1, inter1, spec) and inter_config_validity(c2, inter1, spec)
        else:
            return False

    # inter1 = inter_config(c1, c2, spec)
    # inter2a = inter_config(inter1, c2, spec)
    # inter2b = inter_config(c1, inter1, spec)
    # return config_validity(inter1, spec) and config_validity(inter2a, spec) and config_validity(inter2b, spec)
def recursive_interpol_robot_list(c1:robot_config.RobotConfig, c2:robot_config.RobotConfig):
    res = []
    if(tester.test_config_distance(c1, c2, problem)):#inter_config_distance_heuristic(c1, c2)<PRIMITIVE_STEP):
        return res
    else:
        inter1 = inter_config(c1, c2, problem)
        res = [inter1]
        return [c1] + recursive_interpol_robot_list(c1, inter1) + res + recursive_interpol_robot_list(inter1, c2) +[c2]

#transition configs
# def transition_configs(c1:robot_config.RobotConfig, c2:robot_config.RobotConfig):
#     angle_rad_deltas = [a2.in_radians() - a1.in_radians(), for a1, a2 in zip(c1.ee1_angles, c2.ee1_angles)]


#returns config that's between two configs
def inter_config(c1:robot_config.RobotConfig, c2:robot_config.RobotConfig, spec:problem_spec.ProblemSpec):
    
    c1angles = c1.ee1_angles
    c1lengths = c1.lengths
    ee = c1.get_ee1()

    #interpolate mid between confs
    newAngles = []
    for idx, angObj in enumerate(c1angles):
        curr_rad = angObj.in_radians()
        
        end_rad = c2.ee1_angles[idx].in_radians()
        mid_rad = (curr_rad+end_rad)/2
        newAngle = angle.Angle(radians=mid_rad)

        newAngles.append(newAngle)
    
    newLen = []
    for idx, segment in enumerate(c1lengths):
        newSeg = (segment+ c2.lengths[idx])/2
        newLen.append(newSeg)

    newConf = robot_config.make_robot_config_from_ee1(ee[0], ee[1], newAngles, newLen, True, False)
    
    return newConf

#create N random configs dependent on config and spec
#assumes ee1 hooked
def random_steps(config:robot_config.RobotConfig, spec:problem_spec.ProblemSpec, goal_coord:tuple, dist_to_grap:float):
    RAD_THRESH = RAD_THRESH_GLOBAL
    LEN_THRESH = LEN_THRESH_GLOBAL
    
    ee = config.get_ee1() if config.ee1_grappled else config.get_ee2()
    lengths = config.lengths
    angles = config.ee1_angles if config.ee1_grappled else config.ee2_angles
    rand_configs = []
    total_bots = 0
    while(len(rand_configs)<N and total_bots<100):
        # print(len(rand_configs))
        newAngles = []
        for angObj in angles:
            curr_rad = angObj.in_radians()
            
            lower_limit = (curr_rad-math.pi*RAD_THRESH) if ((curr_rad-math.pi*RAD_THRESH)>=(-MAX_RADIANS) and\
                                                        (curr_rad-math.pi*RAD_THRESH)<=MAX_RADIANS) else -MAX_RADIANS
            upper_limit = (curr_rad+math.pi*RAD_THRESH) if ((curr_rad+math.pi*RAD_THRESH)>=(-MAX_RADIANS) and\
                                                        (curr_rad+math.pi*RAD_THRESH)<=MAX_RADIANS) else MAX_RADIANS
            # lower_limit = -math.pi
            # upper_limit = math.pi
            newAngle = angle.Angle(radians=(random.uniform(lower_limit, upper_limit)))

            newAngles.append(newAngle)
        
        newLen = []
        for idx, segment in enumerate(lengths):
            lower_limit = (segment-LEN_THRESH_GLOBAL) if (segment-LEN_THRESH_GLOBAL)>spec.min_lengths[idx] else spec.min_lengths[idx]
            upper_limit = (segment+LEN_THRESH_GLOBAL) if (segment+LEN_THRESH_GLOBAL)<spec.max_lengths[idx] else spec.max_lengths[idx]
            # lower_limit = spec.min_lengths[idx]
            # upper_limit = spec.max_lengths[idx]
            newSeg = random.uniform(lower_limit, upper_limit)
            newLen.append(newSeg)

        newConf = robot_config.make_robot_config_from_ee1(ee[0], ee[1], newAngles, newLen, True, False)
        
        # man_dist_to_grapple = heuristic(newConf, goal_config)
        #goal_points = goal_config.get_ee2()
        man_dist_to_grapple = distance_to_grapple(newConf, goal_coord)
        # print(man_dist_to_grapple)
        # print(len(rand_configs))
        total_bots +=1
        # print(total_bots)
        if ((config_validity(newConf, spec) and \
            (man_dist_to_grapple<dist_to_grap))): #and \inter_config_validity(config, newConf, spec)):
            rand_configs.append(newConf)
            
    
    return rand_configs

def inter_config_distance_heuristic(current_conf:robot_config.RobotConfig, goal_config:robot_config.RobotConfig):
    max_delta = 0 ##Assume zero
    len_points = len(current_conf.points)
    for idx, point in enumerate(current_conf.points):
        goal_point = goal_config.points[idx]
        max_delta += math.sqrt(math.pow(point[0]-goal_point[0], 2) + math.pow(point[1]-goal_point[1], 2))
    # print(max_delta/len_points)
    return max_delta #/len_points

def interpolate_steps_between(start_conf:robot_config.RobotConfig, end_conf:robot_config.RobotConfig):
    end_lengths = end_conf.lengths
    end_angles = end_conf.ee1_angles if end_conf.ee1_grappled else end_conf.ee2_angles

    ee = start_conf.get_ee1() if start_conf.ee1_grappled else start_conf.get_ee2()
    eex = ee[0]
    eey = ee[1]

    iters = 0
    
    output_file.write(str(start_conf) +'\n')
    # changes = False
    current_conf = start_conf
    while(iters<INTERPOLATE_N):

        curr_angles = current_conf.ee1_angles #if current_conf.ee1_grappled else current_conf.ee2_angles
        curr_len = current_conf.lengths

        temp_angles = []
        for idx, ang_obj in enumerate(curr_angles):
            init_rad = ang_obj.in_radians()
            goal_rad = end_angles[idx].in_radians()

            if (init_rad<=(goal_rad + TOLERANCE) and init_rad>=(goal_rad - TOLERANCE)):
                # print('no change in rad')
                temp_angles.append(ang_obj)
                continue
            # changes = True
            step = (0-PRIMITIVE_STEP) if (init_rad > goal_rad) else (0+PRIMITIVE_STEP)
            
            new_angle = angle.Angle(radians=((init_rad+step)))
            temp_angles.append(new_angle)
        
        temp_lens = []
        for idx, line_len in enumerate(curr_len):
            init_len = line_len
            goal_len = end_lengths[idx]
            
            if (init_len<=(goal_len + TOLERANCE) and init_len>=(goal_len - TOLERANCE)):
                # print('no change in len')
                temp_lens.append(init_len)
                continue
            # changes = True
            step = (0-PRIMITIVE_STEP) if (init_len > goal_len) else (0+PRIMITIVE_STEP)
            temp_lens.append(init_len+step)
        
        new_conf = robot_config.RobotConfig(temp_lens, eex, eey, temp_angles)
        current_conf = new_conf
        iters +=1
        # print(changes)
        # if tester.test_config_distance(current_conf, end_conf, problem):
        #     break
        output_file.write(str(new_conf) + '\n')
    
    output_file.write(str(end_conf) +'\n')

def snapshot_robot(config:robot_config.RobotConfig):
    points = config.points
    for idx, point in enumerate(points):
        points[idx] = (round(point[0], 4), round(point[1], 4))

    lengths = config.lengths
    for idx, line in enumerate(lengths):
        lengths[idx] = round(line, 4)
    
    return str(points) + str(lengths)

def get_solution_to_target(initial_config:robot_config.RobotConfig, target:tuple):
    RAD_THRESH = RAD_THRESH_GLOBAL
    LEN_THRESH = LEN_THRESH_GLOBAL
    robot_config_list = [initial_config]
    robot_q = queue.PriorityQueue()
    robot_count = 0
    robot_q.put((distance_to_grapple(init_robot, target_grapple), 0, 0.9, robot_count, robot_config_list, initial_config))
    solutions_robot = []
    explored_robots = []

    while(True):
        if (len(robot_q.queue)==0):
            print('failed')
            exit()
            # #exit()
            # robot_q.put((distance_to_grapple(init_robot, target_grapple), 0, 0.9, robot_count, robot_config_list, initial_config))
            # RAD_THRESH = RAD_THRESH_GLOBAL
            # LEN_THRESH = LEN_THRESH_GLOBAL
    
        else:
            grap_dist, inter_dist, delta_num, rc_n, curr_robot_list, curr_robot = robot_q.get()
            print(grap_dist)
            
            if (grap_dist<0.02):
                solutions_robot.append(curr_robot_list)
                break
            else:
                snap = snapshot_robot(curr_robot)

                if snap in explored_robots:
                    continue
                else:
                    explored_robots.append(snap)
                    RAD_THRESH = (RAD_THRESH - 0.00001) if (RAD_THRESH - 0.00001)>RAD_THRESH_MIN else RAD_THRESH_MIN
                    LEN_THRESH = (LEN_THRESH - 0.00001) if (LEN_THRESH - 0.00001)>LEN_THRESH_MIN else LEN_THRESH_MIN
                    for next_bot in random_steps(curr_robot, problem, target_grapple, delta_num -DELTA_THRESH):
                        if(inter_config_validity(curr_robot, next_bot, problem)):
                            new_robot_list = curr_robot_list + [next_bot]
                            dist_to_grap = round(distance_to_grapple(next_bot, target_grapple), 4)
                            dist_between = round(inter_config_distance_heuristic(curr_robot, next_bot), 4)
                            robot_count += 1
                            robot_q.put((dist_to_grap, dist_between, delta_num-DELTA_THRESH, robot_count, new_robot_list, next_bot ))

    return min(solutions_robot)

def get_bridge_to_target(initial_config:robot_config.RobotConfig, target:tuple):
    RAD_THRESH = RAD_THRESH_GLOBAL
    LEN_THRESH = LEN_THRESH_GLOBAL
    robot_config_list = [initial_config]
    robot_q = queue.PriorityQueue()
    robot_count = 0
    robot_q.put((distance_to_grapple(init_robot, target_grapple), 0, 0.9, robot_count, robot_config_list, initial_config))
    solutions_robot = []
    explored_robots = []

    while(True):
        if (len(robot_q.queue)==0):
            print('failed')
            exit()
        else:
            grap_dist, inter_dist, delta_num, rc_n, curr_robot_list, curr_robot = robot_q.get()
            print(grap_dist)
            
            if (grap_dist<PRIMITIVE_STEP):
                solutions_robot.append(curr_robot_list)
                break
            else:
                snap = snapshot_robot(curr_robot)

                if snap in explored_robots:
                    continue
                else:
                    explored_robots.append(snap)
                    RAD_THRESH = (RAD_THRESH - 0.0001) if (RAD_THRESH - 0.0001)>RAD_THRESH_MIN else RAD_THRESH_MIN
                    LEN_THRESH = (LEN_THRESH - 0.0001) if (LEN_THRESH - 0.0001)>LEN_THRESH_MIN else LEN_THRESH_MIN
                    for next_bot in random_steps(curr_robot, problem, target_grapple, delta_num -DELTA_THRESH):
                        if(inter_config_validity(curr_robot, next_bot, problem)):
                            new_robot_list = curr_robot_list + [next_bot]
                            dist_to_grap = round(distance_to_grapple(next_bot, target_grapple), 4)
                            dist_between = round(inter_config_distance_heuristic(curr_robot, next_bot), 4)
                            robot_count += 1
                            robot_q.put((dist_to_grap, dist_between, delta_num-DELTA_THRESH, robot_count, new_robot_list, next_bot ))

    return min(solutions_robot)

def generate_bridge_bots(start_conf:robot_config.RobotConfig, spec:problem_spec.ProblemSpec):
    RAD_THRESH = RAD_THRESH_GLOBAL
    LEN_THRESH = LEN_THRESH_GLOBAL

    valid_bridges = []
    ee = start_conf.get_ee1()
    lengths = start_conf.lengths
    angles = start_conf.ee1_angles

    while(len(valid_bridges)<1):

        newAngles = []
        for angObj in angles:
            curr_rad = angObj.in_radians()
            
            lower_limit = (curr_rad-math.pi*RAD_THRESH) if ((curr_rad-math.pi*RAD_THRESH)>=(-MAX_RADIANS) and\
                                                        (curr_rad-math.pi*RAD_THRESH)<=MAX_RADIANS) else -MAX_RADIANS
            upper_limit = (curr_rad+math.pi*RAD_THRESH) if ((curr_rad+math.pi*RAD_THRESH)>=(-MAX_RADIANS) and\
                                                        (curr_rad+math.pi*RAD_THRESH)<=MAX_RADIANS) else MAX_RADIANS
            
            newAngle = angle.Angle(radians=(random.uniform(lower_limit, upper_limit)))

            newAngles.append(newAngle)
        
        newLen = []
        for idx, segment in enumerate(lengths):
            lower_limit = (segment-LEN_THRESH_GLOBAL) if (segment-LEN_THRESH_GLOBAL)>spec.min_lengths[idx] else spec.min_lengths[idx]
            upper_limit = (segment+LEN_THRESH_GLOBAL) if (segment+LEN_THRESH_GLOBAL)<spec.max_lengths[idx] else spec.max_lengths[idx]
            # lower_limit = spec.min_lengths[idx]
            # upper_limit = spec.max_lengths[idx]
            newSeg = random.uniform(lower_limit, upper_limit)
            newLen.append(newSeg)

        newConf = robot_config.make_robot_config_from_ee1(ee[0], ee[1], newAngles, newLen, True, False)
        
        man_dist_to_grapple = inter_config_distance_heuristic(newConf, start_conf)
        #goal_points = goal_config.get_ee2()
        # man_dist_to_grapple = distance_to_grapple(newConf, goal_coord)
        # print(man_dist_to_grapple)
        # print(len(rand_configs))
        # print(total_bots)
        if ((config_validity(newConf, spec) and \
            (man_dist_to_grapple<0.01))): #and \inter_config_validity(config, newConf, spec)):
            valid_bridges.append(newConf)
    
    return valid_bridges

# def brige_to_target_by_moving_ee2(start_conf:robot_config.RobotConfig, target:tuple):
#     while(!close_enough(start_conf.get_ee2(), target)):
#         angles = start_conf.ee1_angles



def output_min_solution_to_file(final_path:list):
    for idx, robo in enumerate(final_path):
        if (idx==0):
            output_file.write(str(robo)+'\n')
            continue
        else:
            output_file.write(str(final_path[idx-1]) + '\n')
            recurse_list = recursive_interpol_robot_list(final_path[idx-1], robo)
            for id2, ele in enumerate(recurse_list):
                output_file.write(str(ele)+'\n')
                
            output_file.write(str(robo) + '\n')

def merge_to_final_bot(last_bot:robot_config.RobotConfig, goal_config:robot_config.RobotConfig):
    final_recurse_list = recursive_interpol_robot_list(last_bot, goal_config)
    for ele in final_recurse_list:
        output_file.write(str(ele) + '\n')

    output_file.write(str(goal_config))



#Actual script
target_grapple = None
if len(grapples)==1:
    # Single grapple solutions
    target_grapple = goal_config.get_ee2()
    final_path = get_solution_to_target(init_robot, target_grapple)
    output_min_solution_to_file(final_path)
    last_bot = final_path[len(final_path)-1]
    merge_to_final_bot(last_bot, goal_config)

else:
    print("Total grapples =", len(grapples))
    targets = grapples
    
    # targets.append(goal_config.get_ee2())
    last_target_idx = len(targets)-1
    current_init_robot = init_robot
    bridge_bot = None

    for idx,grapple in enumerate(targets[1:]):
        target_grapple = grapple
        final_path = get_solution_to_target(current_init_robot, target_grapple)
        output_min_solution_to_file(final_path)
        # last_bot = final_path[len(final_path)-1]
        # bridge_path = get_bridge_to_target(last_bot, target_grapple)
        # output_min_solution_to_file(bridge_path)

        #switch ees
        bridge_bot:robot_config.RobotConfig = final_path[len(final_path)-1]
        ee = bridge_bot.get_ee2()
        angles = bridge_bot.ee2_angles
        # x, y = bridge_bot.points[0]
        # angles = bridge_bot.ee1_angles[:]
        lengths = bridge_bot.lengths
        lengths.reverse()

        proto_next_bot = robot_config.make_robot_config_from_ee1(ee[0], ee[1], angles, lengths, True, False)
        next_bot = proto_next_bot
        while(not config_validity(next_bot, problem)):
            print("invalid :(")
            exit()
        
        current_init_robot = next_bot

    final_path_final = get_solution_to_target(current_init_robot, goal_config.get_ee2())
    output_min_solution_to_file(final_path_final)
    last_bot_final = final_path_final[len(final_path_final) -1]
    merge_to_final_bot(last_bot_final, goal_config)


