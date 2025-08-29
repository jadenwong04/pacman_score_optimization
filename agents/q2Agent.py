import logging
import random

import util
from game import Actions, Agent, Directions
from logs.search_logger import log_function
from pacman import GameState
from util import manhattanDistance

import numpy as np

def scoreEvaluationFunction(currentGameState: GameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    scared_ghost_position = [currentGameState.getGhostPosition(g_index) for g_index in range(1, currentGameState.getNumAgents()) if currentGameState.getGhostState(g_index).scaredTimer > 0]
    if len(scared_ghost_position) > 0:
        cost_to_closest_scared_ghost = AStar(currentGameState, scared_ghost_position).get_closest_distance()
        return (currentGameState.getScore(), -cost_to_closest_scared_ghost)
    else:
        cost_to_closest_dot = AStar(currentGameState, currentGameState.getFood().asList()).get_closest_distance()
        return (currentGameState.getScore(), -cost_to_closest_dot)

class Q2_Agent(Agent):
    
    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)
        
    @log_function
    def getAction(self, gameState: GameState):
        """
            Returns the minimax action from the current gameState using self.depth
            and self.evaluationFunction.

            Here are some method calls that might be useful when implementing minimax.

            gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

            gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

            gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        logger = logging.getLogger('root')
        logger.info('MinimaxAgent')
        "*** YOUR CODE HERE ***"
        _, action = self.alpha_beta_search(gameState, 0, 0)
        return action
    
    def alpha_beta_search(self, gameState, current_index, current_depth, alpha = (-np.inf,) * 2, beta = (np.inf,) * 2):
        if current_index == gameState.getNumAgents():
            current_index = self.index
            current_depth += 1
            
        if gameState.isWin() or gameState.isLose() or current_depth == self.depth:
            return self.evaluationFunction(gameState), None
        else:
            if current_index == 0:
                return self.get_max_value(gameState, alpha, beta, current_index, current_depth)
            else:
                return self.get_min_value(gameState, alpha, beta, current_index, current_depth)
                
    def get_max_value(self, state, alpha, beta, index, depth):
        max_value, max_action = (-np.inf,) * 2, None
        for action in state.getLegalActions(index):
            s_value, s_action = self.alpha_beta_search(state.generateSuccessor(index, action), index + 1, depth, alpha, beta)
            
            if s_value > max_value:
                max_value, max_action = s_value, action
                alpha = max(alpha, max_value)
            
            if max_value >= beta:
                return max_value, max_action
            
        return max_value, max_action
            
    def get_min_value(self, state, alpha, beta, index, depth):
        min_value, min_action = (np.inf,) * 2, None
        for action in state.getLegalActions(index):
            s_value, s_action = self.alpha_beta_search(state.generateSuccessor(index, action), index + 1, depth, alpha, beta)
            
            if s_value < min_value:
                min_value, min_action = s_value, action
                beta = min(beta, min_value) 
            
            if min_value <= alpha:
                return min_value, min_action
        
        return min_value, min_action

class AStar:
    def __init__(self, game_state: GameState, destinations):
        self.current = None
        self.destination = destinations
        self.grid = {}
        self.discovered = util.PriorityQueue()
        self.starting_game_state = game_state
        self.action = Actions._directions
        state = State(game_state.getPacmanPosition())
        self.grid[state.src] = state
        h = self.heuristic(state.src)
        self.discovered.push(state, (state.distance + h, h))
    
    def get_closest_distance(self):
        terminate = False
        while not terminate:
            terminate, distance = self.astar_loop_body()
        return distance
    
    def astar_loop_body(self):
        self.current = self.discovered.pop()
        self.current.visited = True
        
        for action in self.action:
            x = self.current.src[0] + self.action[action][0]
            y = self.current.src[1] + self.action[action][1]
            if not self.starting_game_state.hasWall(x, y):
                if self.grid.get((x, y)) is None:
                    self.grid[(x, y)] = State((x, y))
                successor = self.grid[(x, y)]
                h = self.heuristic(successor.src)
                if successor.discovered is False:
                    successor.discovered = True
                    successor.distance = self.current.distance + 1
                    self.discovered.push(successor, (successor.distance + h, h))
                elif successor.visited is False:
                    if successor.distance > self.current.distance + 1:
                        successor.distance = self.current.distance + 1
                        self.discovered.update(successor, (successor.distance + h, h))
        
        return self.discovered.isEmpty() or self.current.src in self.destination, self.current.distance
    
    def heuristic(self, current):
        all_distance = [util.manhattanDistance(current, goal) for goal in self.destination]
        return min(all_distance) if len(all_distance) > 0 else np.inf

class State:
    def __init__(self, pacman_coord):
        self.src = pacman_coord
        self.discovered = False
        self.visited = False
        self.distance = 0