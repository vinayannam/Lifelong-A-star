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


class PriorityQueueLAS:
    def __init__(self):
        self.heap = []
        self.count = 0

    def top(self):
        if self.isEmpty():
            return None
        (_, _, item) = self.heap[0]
        return item[0]

    def topKey(self):
        if self.isEmpty():
            return (float('inf'), float('inf'))
        (_, _, item) = self.heap[0]
        return item[1]

    def insert(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item[0]

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
        for index, item in enumerate(self.heap):
            if item[2][0] == state:
                del self.heap[index]
                heapq.heapify(self.heap)
                break


def lifeLongAStarSearch(problem, heuristic=nullHeuristic):
    """Life Long A * Search"""
    "*** YOUR CODE HERE ***"

    def calculateKey(state):
        g_rhs = min(problem.maze[state]['g'], problem.maze[state]['rhs'])
        return (g_rhs + heuristic(state, problem.goal), g_rhs)

    def initialize():
        problem.height = problem.walls.height
        problem.width = problem.walls.width
        problem.maze = {}
        for x in range(problem.height):
            for y in range(problem.width):
                problem.maze[(x, y)] = {'g': float('inf'), 'rhs': float('inf')}
        problem.maze[problem.startState]['g'] = float('inf')
        problem.maze[problem.startState]['rhs'] = 0.0
        problem.maze[problem.goal]['g'] = float('inf')
        problem.maze[problem.goal]['rhs'] = float('inf')
        problem.U = PriorityQueueLAS()
        key = calculateKey(problem.getStartState())
        problem.U.insert((problem.getStartState(), key), key[0]+key[1])

    def updateVertex(u):
        if u != problem.getStartState():
            prevKeys = []
            for successor in problem.getSuccessors(u):
                prevKeys.append(problem.maze[successor[0]]['g']+successor[2])
            problem.maze[u]['rhs'] = min(prevKeys)
        for item in problem.U.heap:
            if item[2][0] == u:
                problem.U.remove(u)
        if problem.maze[u]['g'] != problem.maze[u]['rhs']:
            key = calculateKey(u)
            problem.U.Insert((u, key), key)

    def computeShortestPath():
        goal = problem.maze[problem.goal]
        while problem.U.topKey() < calculateKey(problem.goal) or goal['rhs'] != goal['g']:
            u = problem.U.pop()
            if problem.maze[u]['g'] > problem.maze[u]['rhs']:
                problem.maze[u]['g'] = problem.maze[u]['rhs']
                for successor in problem.getSuccessors(u):
                    updateVertex(successor[0])
            else:
                problem.maze[u]['g'] = float('inf')
                updateVertex(u)
                for successor in problem.getSuccessors(u):
                    updateVertex(successor[0])

        initialize()
        computeShortestPath()
        for change in changes:
            updateVertex()

        # Initialize(); forever
        # ComputeShortestPath();
        # Wait for changes in edge costs;
        # for all directed edges (u, v) with changed edge costs
        # Update the edge cost c(u, v); UpdateVertex(v);


# Abbreviations
las = lifeLongAStarSearch
