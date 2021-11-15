import numpy as np
import networkx as nx
from copy import copy
import collections
import itertools
import heapq
import gtdynamics as gtd
from gtdynamics import ContactGoal, PointOnLink, Slice, Interval

robot = gtd.CreateRobotFromFile(gtd.URDF_PATH + "quad_climb_robot.urdf")

class PriorityQueue(object):
    def __init__(self):
        self.queue = []
        self.entry_dict = collections.defaultdict(list)
        self.counter = itertools.count()


    def pop(self):
        while self.queue:
            priority, val = heapq.heappop(self.queue)
            if val != 'REMOVED':
                self.entry_dict[(priority[0], val)] = self.entry_dict[(priority[0], val)][1:]
                return (priority[0], val)

            
    def remove(self, node):
        vals = self.entry_dict[node]
        for val in vals:
            val[-1] = 'REMOVED'

        

    def remove_node_value(self, node):
        keys = self.entry_dict.keys()
        relevant_keys = [i for i in keys if i[1] == node]
        for k in relevant_keys:
            vals = self.entry_dict[k]
            for val in vals:
                val[-1] = 'REMOVED'


    def __iter__(self):
        return iter(sorted(self.queue))


    def __str__(self):
        return 'PQ:%s' % self.queue


    def append(self, node):
        priority = node[0]
        val = node[1]
        count = next(self.counter)
        entry = [(priority, count), val]
        self.entry_dict[node].append(entry)
        heapq.heappush(self.queue, entry)

        

    def __contains__(self, key):
        return key in [n[-1] for n in self.queue]


    def __eq__(self, other):
        return self.queue == other.queue


    def size(self):
        return len([i for i in self.queue if i[1] != 'REMOVED'])


    def clear(self):
        self.queue = []


    def top(self):
        self.queue = [i for i in self.queue if i[-1] != 'REMOVED']
        heapq.heapify(self.queue)
        return self.queue[0]


def get_possible_holds(climbing_face, current_configuration):
    def multiplyWTCom(pose3, points3):
        return np.matmul(points3, pose3.rotation().matrix()) + pose3.translation()

    possible_configurations = []
    for i in range(len(current_configuration)):
        for j in range(len(climbing_face)):
            current_point = current_configuration[i]
            new_hold = climbing_face[j]

            dist = np.linalg.norm(np.asarray(current_point) - np.asarray(new_hold))

            current_point_4x3 = np.zeros((4, 3))
            current_point_4x3[i] = current_point
            new_point_4x3 = np.zeros((4, 3))
            new_point_4x3[i] = new_hold
            new_configuration = np.asarray(current_configuration) - current_point_4x3 + new_point_4x3
            new_body_com = np.mean(new_configuration, axis=0)
            is_valid_com = all([np.linalg.norm(new_body_com - current_configuration[j]) < 1.5 for j in range(len(current_configuration)) if j != i])
            
            is_on_correct_side = True
            for k in range(4):
                curr_hold = new_configuration[k]
                if k == 0:
                    is_on_correct_side = is_on_correct_side and (curr_hold[0] < new_body_com[0] - .25) and curr_hold[1] < new_body_com[1]  - .25
                elif k == 1:
                    is_on_correct_side = is_on_correct_side and curr_hold[0] < new_body_com[0] - .25 and curr_hold[1] > new_body_com[1] + .25
                elif k == 2:
                    is_on_correct_side = is_on_correct_side and curr_hold[0] > new_body_com[0] + .25 and curr_hold[1] < new_body_com[1] - .25
                else:
                    is_on_correct_side = is_on_correct_side and curr_hold[0] > new_body_com[0] + .25 and curr_hold[1] > new_body_com[1] + .25


            if is_on_correct_side and dist <= 2.0 and is_valid_com and tuple(new_hold) not in [tuple(k) for k in current_configuration]:
                # Inverse kinematics check
                const_for_robot_size = 32
                np_current_configuration = np.asarray(current_configuration)
                LH = PointOnLink(robot.link("l_gripper"), multiplyWTCom(robot.link("l_gripper").wTcom(), np_current_configuration[2, :] / const_for_robot_size))
                LF = PointOnLink(robot.link("l_ankle"), multiplyWTCom(robot.link("l_ankle").wTcom(), np_current_configuration[0, :] / const_for_robot_size))
                RH = PointOnLink(robot.link("r_gripper"), multiplyWTCom(robot.link("r_gripper").wTcom(), np_current_configuration[3, :] / const_for_robot_size))
                RF = PointOnLink(robot.link("r_ankle"), multiplyWTCom(robot.link("r_ankle").wTcom(), np_current_configuration[1, :] / const_for_robot_size))

                contact_goals = [
                    ContactGoal(LH, multiplyWTCom(robot.link("l_gripper").wTcom(), new_configuration[2, :] / const_for_robot_size)),
                    ContactGoal(LF, multiplyWTCom(robot.link("l_ankle").wTcom(),  new_configuration[0, :] / const_for_robot_size)),
                    ContactGoal(RH, multiplyWTCom(robot.link("r_gripper").wTcom(), new_configuration[3, :] / const_for_robot_size)),
                    ContactGoal(RF, multiplyWTCom(robot.link("r_ankle").wTcom(), new_configuration[1, :] / const_for_robot_size)),
                ]
                kinematics = gtd.Kinematics(robot)
                result = kinematics.inverseSlice(Slice(4), contact_goals)
                passes_ik = all([goal.satisfied(result,k=4,tol=1e-3) for goal in contact_goals])

                if passes_ik:
                    possible_configurations.append((current_point, tuple(new_hold), i))
    return possible_configurations

def uniform_cost_search(climbing_face, initial_holds, goal_hold):
    def get_4x3_of_3_vector(vec, index):
        current_point_4x3 = np.zeros((4, 3))
        current_point_4x3[index] = vec
        return current_point_4x3

    def get_configuration_neighbors(configuration):
        possible_configurations = get_possible_holds(list(map(tuple, climbing_face)), list(map(tuple, configuration)))
        configuration_changes = {(np.asarray(configuration) - get_4x3_of_3_vector(i[0], i[2]) + get_4x3_of_3_vector(i[1], i[2])).tostring() : np.linalg.norm(np.mean(np.asarray(configuration) - get_4x3_of_3_vector(i[0], i[2]) + get_4x3_of_3_vector(i[1], i[2])) - np.mean(configuration)) for i in possible_configurations}
        return configuration_changes

    def heuristic(current_configuration, goal):
        return np.linalg.norm(np.mean(current_configuration) - np.asarray(goal))

    path_dict = collections.defaultdict(list)
    frontier = PriorityQueue()
    frontier.append((0, initial_holds.tostring()))
    while frontier.queue:
        cost, curr = frontier.pop()
        cost = cost
        if goal_hold in list(map(tuple, np.fromstring(curr).reshape((4, 3)))):
            print(len(path_dict[curr].keys()))
            return (path_dict[curr][0], path_dict[curr][1])

        if curr not in path_dict:
            path_dict[curr] = (path_dict[curr] + [curr], cost)
        print(len(path_dict[curr]))
        neighbors = get_configuration_neighbors(np.fromstring(curr).reshape((4, 3)))
        print(np.fromstring(curr).reshape((4, 3)))
        print([np.fromstring(i).reshape((4, 3)) for i in neighbors])
        # exit(0)
        for neighbor, weight in neighbors.items():
            neighbor_cost = cost + weight
            node = (neighbor_cost , neighbor)
            if neighbor not in path_dict:
                path_dict[neighbor] = (path_dict[curr][0] + [neighbor], neighbor_cost)
                frontier.append(node)
            else:
                if path_dict[neighbor][1] > neighbor_cost:
                    frontier.remove((path_dict[neighbor][1], neighbor))
                    path_dict[neighbor] = (path_dict[curr][0] + [neighbor], neighbor_cost)
                    frontier.append(node)

    return []


def shortest_path(climbing_face, initial_holds=[(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 1.0, 0.0), (1.0, 0.0, 0.0)], goal_hold=(0.0, 0.0, 0.0)):
    current_configuration = initial_holds
    G = nx.Graph()
    G.add_nodes_from(initial_holds)
    ucs(G, climbing_face, initial_holds, goal_hold)
    return G

def ucs(G, climbing_face, initial_holds=[(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 1.0, 0.0), (1.0, 0.0, 0.0)], goal_hold=(0.0, 0.0, 0.0), max_depth=25, curr_depth=0):
    def get_4x3_of_3_vector(vec, index):
        current_point_4x3 = np.zeros((4, 3))
        current_point_4x3[index] = vec
        return current_point_4x3

    if goal_hold in initial_holds or curr_depth >= max_depth:
        return
    
    possible_configurations = get_possible_holds(climbing_face, initial_holds)
    configuration_change = [(np.asarray(initial_holds), np.asarray(initial_holds) - get_4x3_of_3_vector(i[0], i[2]) + get_4x3_of_3_vector(i[1], i[2])) for i in possible_configurations]
    new_nodes = [i[1].tostring() for i in configuration_change if not G.has_node(i[1].tostring())]
    new_possible_configurations = [possible_configurations[i] for i in range(len(possible_configurations)) if not G.has_node(configuration_change[i][1].tostring())]

    if len(new_nodes) == 0:
        return

    new_edges = [(i[0].tostring(), i[1].tostring()) for i in configuration_change if not G.has_node(i[1].tostring())]
    edge_attrs = {new_edges[i]: {'point_change': new_possible_configurations[i]} for i in range(len(new_edges))}
    G.add_nodes_from(new_nodes)
    G.add_edges_from(new_edges)
    nx.set_edge_attributes(G, edge_attrs)
    for i in configuration_change:
        updated_initial_holds = list(map(tuple, i[1]))
        ucs(G, climbing_face, updated_initial_holds, goal_hold, curr_depth=curr_depth + 1)