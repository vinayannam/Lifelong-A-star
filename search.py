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
import heapq
from game import Directions


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


class PriorityQueueLAS:

    def  __init__(self):
        self.heap = []
        self.count = 0

    def top(self):
        return self.heap[0]

    def topKey(self):
        if self.isEmpty():
            return (float('inf'), float('inf'))
        return self.heap[0][0]

    def insert(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.insert(item, priority)

    def remove(self, state):
        self.update(state, float('-inf'))
        a = self.pop()

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
        state = problem.getGoalState()
        path.append(state)
        while state != problem.dynamicStartState:
            minimum = float('inf')
            for successor, _, _ in problem.getSuccessors(state):
                if minimum > problem.g[successor]:
                    minimum = problem.g[successor]
                    state = successor
            path.append(state)
        return path[::-1]

    def main():
        problem.U = PriorityQueueLAS()
        problem.g = {}
        problem.rhs = {}
        problem.finalPath = []
        problem.dynamicStartState = problem.getStartState()
        initialize()
        pseudoPath = []
        stop = False
        while (problem.dynamicStartState != problem.getGoalState())  and not stop:
            initialize()
            computeShortestPath()
            path = shortestPath()
            print "Path: "
            print problem.getGoalState()
            if len(path) == 1 and path[0][0] == problem.getGoalState(): 
                break
            for index in range(len(path)-1):
                currentState = path[index]
                nextState = path[index+1]
                pseudoPath.append(currentState)
                problem.finalPath.append(currentState)
                print "-->" + str(nextState)
                if problem.isObstacle(nextState):
                    pseudoPath = []
                    print "Obstacle @ "+ str(nextState)
                    print "Replanning..."
                    problem.insertObstacle(nextState)
                    updateVertex(nextState)
                    problem.dynamicStartState = currentState
                    break
                elif nextState == problem.getGoalState():
                    stop = True
                    break
        problem.finalPath.append(problem.getGoalState())
        print "Done Planning"
        actions = []
        for index in range(len(problem.finalPath)-1):
            x, y = problem.finalPath[index]
            nextX, nextY = problem.finalPath[index+1]
            if x > nextX:
                actions.append(Directions.WEST)
            if x<nextX:
                actions.append(Directions.EAST)
            if y<nextY:
                actions.append(Directions.NORTH)
            if y>nextY:
                actions.append(Directions.SOUTH)
        print(actions)
        return actions

    return main()

 # Abbreviations
las = lifeLongAStarSearch
