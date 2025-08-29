#---------------------#
# DO NOT MODIFY BEGIN #
#---------------------#

import logging

import util
from problems.q1c_problem import q1c_problem

#-------------------#
# DO NOT MODIFY END #
#-------------------#

def q1c_solver(problem: q1c_problem):
    # YOUR CODE HERE
    data = id_astar_initialise(problem)
    terminate = False
    while not terminate:
        terminate, result = id_astar_loop_body(problem, data)
    return result

class IDAStarData:
    # YOUR CODE HERE
    def __init__(self, start_state):
        self.current = None
        self.discovered = util.PriorityQueue()
        self.actions_list = []
        h = id_astar_heuristic(start_state.src, start_state.dest)
        self.discovered.push(start_state, (start_state.distance + h, h))
        start_state.discovered = True

def id_astar_initialise(problem: q1c_problem):
    # YOUR CODE HERE
    return IDAStarData(problem.getStartState())

def id_astar_loop_body(problem: q1c_problem, data: IDAStarData):
    # YOUR CODE HERE
    data.current = data.discovered.pop()
    
    h = id_astar_heuristic(data.current.src, data.current.dest)
    if h >= (len(data.current.dest) * 10) + 500:
        return data.discovered.isEmpty(), data.actions_list
    
    data.current.visited = True
    
    if data.current.src in data.current.dest:
        data.current.dest.remove(data.current.src)
        new_state = problem.resetGrid(data.current.src, data.current.dest)
        data.actions_list.extend(backtrack_action(data.current.previous))
        if problem.isGoalState(data.current):
            return True, data.actions_list
        else:
            data.discovered = util.PriorityQueue()
            h = id_astar_heuristic(new_state.src, new_state.dest)
            data.discovered.push(new_state, (new_state.distance + h, h)) 
            new_state.discovered = True
    else:
        for successor in problem.getSuccessors(data.current):
            s_state, action, cost = successor
            h = id_astar_heuristic(s_state.src, s_state.dest)
            if s_state.discovered is False:
                s_state.discovered = True
                s_state.distance = data.current.distance + cost
                s_state.previous = (data.current, action)
                data.discovered.push(s_state, (s_state.distance + h, h))
            elif s_state.visited is False:
                if s_state.distance > data.current.distance + cost:
                    s_state.distance = data.current.distance + cost
                    s_state.previous = (data.current, action)
                    data.discovered.update(s_state, (s_state.distance + h, h))
                        
    return data.discovered.isEmpty(), data.actions_list

def id_astar_heuristic(current, goals):
    # YOUR CODE HERE
    return min([util.manhattanDistance(current, goal) for goal in goals])

def backtrack_action(goal_state):
    action_list = []
    bt_state, bt_action = goal_state
    while bt_state is not None:
        action_list.append(bt_action)
        bt_state, bt_action = bt_state.previous
    action_list.reverse()
    return action_list  