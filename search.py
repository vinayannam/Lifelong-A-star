# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Actions
from util import PriorityQueueLAS, PriorityQueueWithFunction

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

# the item in the priorityQ

def lifeLongAStarSearch(problem, heuristic):

    def calculateKey(state):
        g_rhs = min(problem.g[state], problem.rhs[state])
        return (g_rhs + heuristic(state, problem), g_rhs)

    def initialize():
        for state in problem.getStates():
            problem.rhs[state] = float('inf')
            problem.g[state] = float('inf')
        problem.rhs[problem.dynamicStartState] = 0
        problem.U.insert(problem.dynamicStartState, calculateKey(problem.dynamicStartState))

    def updateVertex(u):
        if u != problem.dynamicStartState:
            prevKeys = [float('inf')]
            for successor, _, cost in problem.getSuccessors(u):
                prevKeys.append(problem.g[successor]+cost)
            problem.rhs[u] = min(prevKeys)
        problem.U.remove(u)
        if problem.g[u] != problem.rhs[u]:
            problem.U.insert(u, calculateKey(u))

    def computeShortestPath():
        goal = problem.getGoalState()
        while problem.U.topKey() < calculateKey(goal) or problem.rhs[goal] != problem.g[goal]:
            u = problem.U.pop()
            if problem.g[u] > problem.rhs[u]:
                problem.g[u] = problem.rhs[u]
                for successor, _, _ in problem.getSuccessors(u):
                    updateVertex(successor)
            else:
                problem.g[u] = float('inf')
                updateVertex(u)
                for successor, _, _ in problem.getSuccessors(u):
                    updateVertex(successor)
    
    def shortestPath():
        path = []
        state = (problem.getGoalState(), None)
        path.append(state)
        while state[0] != problem.dynamicStartState:
            minimum = float('inf')
            for successor, action, _ in problem.getSuccessors(state[0]):
                if minimum > problem.g[successor]:
                    minimum = problem.g[successor]
                    state = (successor, Actions.reverseDirection(action))
            path.append(state)
        return path[::-1]

    def planning(pseudoPath):
        path = shortestPath()
        if len(path) == 1 and path[0][0] == problem.getGoalState(): 
            return pseudoPath, True
        for index in range(len(path)-1):
            currentState, currentAction = path[index]
            nextState, _ = path[index+1]
            pseudoPath.append(currentState)
            problem.finalPath.append((currentState, currentAction))
            print "--> " + str(nextState),
            if problem.isObstacle(nextState):
                pseudoPath = []
                print "\nObstacle @ "+ str(nextState)
                print "Replanning..."
                problem.insertObstacle(nextState)
                updateVertex(nextState)
                problem.dynamicStartState = currentState
                return pseudoPath, False
            elif nextState == problem.getGoalState():
                return pseudoPath, True

    def main():
        problem.U = PriorityQueueLAS()
        problem.g = {}
        problem.rhs = {}
        problem.finalPath = []
        problem.dynamicStartState = problem.getStartState()
        initialize()
        pseudoPath = []
        stop = False
        print 'The goal position is', problem.getGoalState()
        print "The path is: "
        print problem.dynamicStartState, 
        while (problem.dynamicStartState != problem.getGoalState())  and not stop:
            initialize()
            computeShortestPath()
            pseudoPath, stop = planning(pseudoPath)  
        problem.finalPath.append((problem.getGoalState(), None))
        print "\nDone Planning"
        actions = []
        states = []
        for index in range(len(problem.finalPath[:-1])):
            currentState, currentAction = problem.finalPath[index]
            nextState, _ = problem.finalPath[index+1]
            if currentState != nextState:
                actions.append(currentAction)
                states.append(currentState)
        problem.drawObstacles()
        problem.printPath(states)
        return actions

    return main()

def aStarSearch(problem, heuristic):
    def priorityFunction(node):
        state, actions_sequence, path_cost = node
        heuristic_cost = heuristic(state, problem)
        return path_cost+heuristic_cost
    frontier = PriorityQueueWithFunction(priorityFunction)
    return commonSearch(frontier, problem)

def commonSearch(frontier, problem):
    root = problem.getStartState()
    explored_set = set()
    actions_sequence = list()
    path_cost = 0
    frontier.push((root, actions_sequence, path_cost))
    while not frontier.isEmpty():
        parent, actions_sequence, path_cost = frontier.pop()
        if parent not in explored_set:
            if problem.isGoalState(parent):
                return actions_sequence
            explored_set.add(parent)
            for successor in problem.getSuccessors(parent):
                state, action, step_cost = successor
                new_actions_sequence = actions_sequence[:]
                new_actions_sequence += [action]
                cost = path_cost+step_cost
                frontier.push((state, new_actions_sequence, cost))

 # Abbreviations
las = lifeLongAStarSearch
astar = aStarSearch
