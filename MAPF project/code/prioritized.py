import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        # {'agent': 0, 'loc': [(1, 5)], 'timestep': 4}, {'agent': 1, 'loc': [(1,2),(1,3)], 'timestep':1},{'agent': 0, 'loc': [(1, 5)], 'timestep': 10}

        # Sum of costs = 8
        #    {'agent': 1, 'loc': [(1, 3)], 'timestep':2},
        #    {'agent': 1, 'loc': [(1, 3), (1, 2)], 'timestep':2},
        #    {'agent': 1, 'loc': [(1, 4)], 'timestep':3},
        #    {'agent': 1, 'loc': [(1, 4), (1, 3)], 'timestep':3},
        #    {'agent': 1, 'loc': [(1, 5)], 'timestep':4},
        #    {'agent': 1, 'loc': [(1, 5), (1, 4)], 'timestep':4}
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################

            # for loop -> iterate over num of agents
            if len(constraints) == 0:
                for i in range(self.num_of_agents):
                    # for loop -> add constraint for the agent
                    for j in range(len(path)):
                        # print("I value:" + str(i))
                        # print("path[j]:" + str(path[j]))
                        # print("J value:" + str(j))
                        # add constraints

                        # agent 0 and 1 starts moving at its initial step -> do not put constraint at agent 0 and 1's initial state
                        if (i != j):
                            constraints.append(
                                {'agent': i, 'loc': [path[j]], 'timestep': j})

                            if j > 0:
                                # print("path[j]:" + str(path[j]))
                                # print("path[j-1]:" + str(path[j-1]))
                                constraints.append(
                                    {'agent': i, 'loc': [path[j], path[j-1]], 'timestep': j})

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
