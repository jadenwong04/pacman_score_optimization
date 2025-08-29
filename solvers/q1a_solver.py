#---------------------#
# DO NOT MODIFY BEGIN #
#---------------------#

import logging

import util
from problems.q1a_problem import q1a_problem

def q1a_solver(problem: q1a_problem):
    astarData = astar_initialise(problem)
    num_expansions = 0
    terminate = False
    while not terminate:
        num_expansions += 1
        terminate, result = astar_loop_body(problem, astarData)
    print(f'Number of node expansions: {num_expansions}')
    return result

#-------------------#
# DO NOT MODIFY END #
#-------------------#

class AStarData:
    # YOUR CODE HERE
    def __init__(self, start_state, food_coord):
        self.current = None
        self.destination = food_coord
        self.discovered = util.PriorityQueue()
        self.discovered.push(start_state, start_state.distance + astar_heuristic(start_state.src, food_coord))
        start_state.discovered = True
        
def astar_initialise(problem: q1a_problem):
    # YOUR CODE HERE
    return AStarData(problem.getStartState(), problem.food)

def astar_loop_body(problem: q1a_problem, astarData: AStarData):
    # YOUR CODE HERE
    astarData.current = astarData.discovered.pop()
    astarData.current.visited = True
    
    for successor in problem.getSuccessors(astarData.current):
        s_state, action, cost = successor
        h = astar_heuristic(s_state.src, problem.food)
        if s_state.discovered is False:
            s_state.discovered = True
            s_state.distance = astarData.current.distance + cost
            s_state.previous = (astarData.current, action)
            astarData.discovered.push(s_state, s_state.distance + h)
        elif s_state.visited is False:
            if s_state.distance > astarData.current.distance + cost:
                s_state.distance = astarData.current.distance + cost
                s_state.previous = (astarData.current, action)
                astarData.discovered.update(s_state, s_state.distance + h) 
    
    is_goal_state = problem.isGoalState(astarData.current)
    
    actions = backtrack_action(astarData.current.previous, is_goal_state)       
    
    return astarData.discovered.isEmpty() or is_goal_state, actions
    
def astar_heuristic(current, goal):
    # YOUR CODE HERE
    return util.manhattanDistance(current, goal)

def backtrack_action(goal_state, is_goal_state):
    action_list = []
    if is_goal_state:
        bt_state, bt_action = goal_state
        while bt_state is not None:
            action_list.append(bt_action)
            bt_state, bt_action = bt_state.previous
        action_list.reverse()
    return action_list