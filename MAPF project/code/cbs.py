import copy
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    maxTimestep = max(len(path1), len(path2))

    # run through each timestep in the paths -> length of the longest path
    for timestep in range(maxTimestep):
        # if the two locations at a time are the same, vertex collision -> and return in a list
        # check vertex edge cases
        if get_location(path1, timestep) == get_location(path2, timestep):
            return {'loc': get_location(path1, timestep), 'timestep': timestep}
        # check negative edge cases
        # looking at exp2_1.txt
        # path1: [1,3] at 3 and path2:[1,4] at 3 -> path2 cannot be at [1,4] at timestep 2 otherwise path1 (agent 1) will not have a collision free path
        # then
        elif get_location(path1, timestep) == get_location(path2, timestep-1) and get_location(path1, timestep-1) == get_location(path2, timestep):
            # print(get_location(path2, timestep - 1))
            # print(get_location(path2, timestep))
            return {'loc': [get_location(path2, timestep), get_location(path2, timestep-1)], 'timestep': timestep}

    return {}


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    # collisions = []

    # return collisions
    agents = len(paths)

    collisions = []

    # need to look at both agents -> nested for loop to check both
    # i.e. agents = [agent0, agent1]
    # detect collisions using both paths of the agent
    for i in range(agents - 1):
        for j in range(i+1, agents):
            collision_loc = detect_collision(paths[i], paths[j])
            if collision_loc:
                collisions.append(
                    {'a1': i, 'a2': j, 'loc': collision_loc['loc'], 'timestep': collision_loc['timestep']})

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    # vertex collision -> when there is only one cell in collision['loc']
    # edge collision -> when there is more than one cell in collision['loc']

    #  [{'agent': 0, 'loc': [(1, 4)], 'timestep': 3},
    #  {'agent': 1, 'loc': [(1, 4)], 'timestep': 3}]
    #  constraints should look like this

    location = collision['loc']
    timestep = collision['timestep']
    agent1, agent2 = collision['a1'], collision['a2']
    newConstraints = list()

    # vertex constraint
    if len(location) == 1:
        newConstraints.append(
            {'agent': agent1, 'loc': location, 'timestep': timestep})
        newConstraints.append(
            {'agent': agent2, 'loc': location, 'timestep': timestep})

        return newConstraints

    # negative edge constraint
    else:
        newConstraints.append({'agent': agent1, 'loc': [(
            location[0], location[1])], 'timestep': timestep})

        newConstraints.append({'agent': agent2, 'loc': [(
            location[0], location[1])], 'timestep': timestep})

        return newConstraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    location = collision['loc']
    timestep = collision['timestep']
    agent1, agent2 = collision['a1'], collision['a2']
    newConstraints = list()

    if random.randint(0, 1) == 0:
        agent = 'a1'
    else:
        agent = 'a2'

    # vertex collision -> one agent has positive constraint, one has negative
    if len(location) == 1:
        newConstraints.append(
            {'agent': collision['agent'], 'loc': location, 'timestep': timestep, 'positive': True})
        newConstraints.append(
            {'agent': collision['agent'], 'loc': location, 'timestep': timestep, 'positive': False})
    elif agent == 'a1':
        newConstraints.append({'agent': agent1, 'loc': [
                              (location[0], location[1])], 'timestep': timestep, 'positive': True})
        newConstraints.append({'agent': agent1, 'loc': [
                              (location[0], location[1])], 'timestep': timestep, 'positive': False})
    else:
        newConstraints.append({'agent': agent2, 'loc': [
                              (location[1], location[0])], 'timestep': timestep, 'positive': True})
        newConstraints.append({'agent': agent2, 'loc': [
                              (location[1], location[0])], 'timestep': timestep, 'positive': False})

    return newConstraints


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(
            node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(
                self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # loop while open_list is not empty
        while len(self.open_list) > 0:

            # pop node from the open list
            node = self.pop_node()
            print("Expanded Nodes: " + str(self.num_of_expanded))
            # check if there are any collisions
            if len(node['collisions']) == 0:
                return node['paths']

            # choose the first collision
            collision = node['collisions'][0]
            # use standard_splitting to create a list of constraints
            # constraints = standard_splitting(collision)
            # use disjoint_splitting to create a list of constraints
            constraints = disjoint_splitting(collision)

            for constraint in constraints:
                # Q is new node -> constraints, paths, collisions, cost
                Q = {'constraints': [],
                     'paths': [],
                     'collisions': [],
                     'cost': 0}

                # was overwriting all ['constraints'] by not creating a deep copy lmao
                Q['constraints'] = copy.deepcopy(node['constraints'])
                Q['constraints'].append(constraint)
                Q['paths'] = copy.deepcopy(node['paths'])
                agent = constraint['agent']

                path = a_star(
                    self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, Q['constraints'])

                # print("Path: " + str(path))

                if path is not None:
                    Q['paths'][agent] = path
                    # use paths_violate_constraint to create a new path list of the paths that violate constraint
                    if constraint['positive'] == True:
                        constraintPath = paths_violate_constraint(
                            constraint, Q['paths'])
                        # print("here")
                        # print(constraintPath)
                        for disjointAgent in constraintPath:
                            constraint = {
                                'agent': disjointAgent, 'loc': constraint['loc'], 'timestep': constraint['timestep']}
                            Q['constraints'].append(constraint)
                            disjointPath = a_star(
                                self.my_map, self.starts[disjointAgent], self.goals[disjointAgent], self.heuristics[disjointAgent], disjointAgent, Q['constraints'])

                            # Condition to not add constraint if no such path exists
                            # break and push the new node Q
                            if disjointPath is None:
                                break

                    # keep same
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])

                    self.push_node(Q)

        raise BaseException('No Solutions')
        self.print_results(root)
        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
