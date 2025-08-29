import logging
import time
from typing import Tuple

import util
from game import Actions, Agent, Directions
from logs.search_logger import log_function
from pacman import GameState

class q1c_problem:
    """
    A search problem associated with finding a path that collects all of the
    food (dots) in a Pacman game.
    Some useful data has been included here for you
    """
    def __str__(self):
        return str(self.__class__.__module__)

    def __init__(self, gameState: GameState):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.startingGameState: GameState = gameState
        self.grid = dict()
        self.action = Actions._directions
    
    def resetGrid(self, pacman_coord, remaining_food):
        self.grid.clear()
        state = State(pacman_coord, remaining_food)
        self.grid[pacman_coord] = state
        return state
        
    @log_function
    def getStartState(self):
        "*** YOUR CODE HERE ***"
        state = State(self.startingGameState.getPacmanPosition(), self.startingGameState.getFood().asList())
        self.grid[state.src] = state
        return state

    @log_function
    def isGoalState(self, state):
        "*** YOUR CODE HERE ***"
        return len(state.dest) == 0

    @log_function
    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """
        "*** YOUR CODE HERE ***"
        successors = []
        for action in self.action:
            x = state.src[0] + self.action[action][0]
            y = state.src[1] + self.action[action][1]
            if not self.startingGameState.hasWall(x,y):
                if self.grid.get((x,y)) is None:
                    self.grid[(x,y)] = State((x,y), state.dest)
                successors.append((self.grid[(x,y)], action, 1))
        return successors

class State:
    def __init__(self, pacman_coord, foods_coord):
        self.src = pacman_coord
        self.dest = foods_coord
        self.discovered = False
        self.visited = False
        self.distance = 0
        self.previous = (None, None)