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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def greedySearch(problem):
    import math
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST

    actions = []

    print "Start:", problem.getStartState()
    print "Goal:", problem.goal
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    curState = problem.getStartState()
    temp_step = 10000
    idx = 0
    while not problem.isGoalState(curState) and idx < temp_step:
        minDistance = float('inf')
        bestAction = None
        for successor in problem.getSuccessors(curState):
            pos = successor[0]
            distance = math.sqrt((pos[0] - problem.goal[0]) ** 2 + (pos[1] - problem.goal[1]) ** 2)
            if distance < minDistance:
                minDistance = distance
                bestAction = successor
        actions.append(bestAction[1])
        curState = bestAction[0]
        idx += 1

    return  actions

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    from game import Actions

    print "Start:", problem.getStartState()
    print "Goal:", problem.goal
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    stack = util.Stack()
    visited = []
    prev = {}

    stack.push(problem.getStartState())
    while not stack.isEmpty():
        curState = stack.pop()
        visited.append(curState)
        successors = problem.getSuccessors(curState)

        for successor in successors:
            if successor[0] not in visited:
                if problem.isGoalState(successor[0]):
                    visited.append(successor[0])
                    prev[successor[0]] = curState
                    stack.list = []
                    break
                prev[successor[0]] = curState
                stack.push(successor[0])

    res_path = []
    cur_pos = problem.goal
    res_path.append(cur_pos)
    while prev.get(cur_pos) is not None:
        cur_pos = prev[cur_pos]
        res_path.append(cur_pos)
    res_path = res_path[::-1]

    actions = []
    cur_pos = res_path[0]
    dir_dict = Actions._directions
    for pos in res_path[1:]:
        delta_pos = (pos[0] - cur_pos[0], pos[1] - cur_pos[1])
        dir = {i for i in dir_dict if dir_dict[i]==delta_pos}
        if len(dir) == 0:
            print pos, cur_pos, delta_pos, len(dir)
            print('Buggggggg')
            exit()
        actions.append(list(dir)[0])
        cur_pos = pos
    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from game import Actions

    print "Start:", problem.getStartState()
    print "Goal:", problem.goal
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    queue = util.Queue()
    visited = []
    prev = {}

    queue.push(problem.getStartState())
    while not queue.isEmpty():
        curState = queue.pop()
        visited.append(curState)
        successors = problem.getSuccessors(curState)

        for successor in successors:
            if successor[0] not in visited:
                if problem.isGoalState(successor[0]):
                    visited.append(successor[0])
                    prev[successor[0]] = curState
                    queue.list = []
                    break
                queue.push(successor[0])
                prev[successor[0]] = curState

    res_path = []
    cur_pos = problem.goal
    res_path.append(cur_pos)
    while prev.get(cur_pos) is not None:
        cur_pos = prev[cur_pos]
        res_path.append(cur_pos)
    res_path = res_path[::-1]

    actions = []
    cur_pos = res_path[0]
    dir_dict = Actions._directions

    for pos in res_path[1:]:
        delta_pos = (pos[0] - cur_pos[0], pos[1] - cur_pos[1])
        dir = {i for i in dir_dict if dir_dict[i]==delta_pos}
        if len(dir) == 0:
            print pos, cur_pos, delta_pos, len(dir)
            print('Buggggggg')
            exit()
        actions.append(list(dir)[0])
        cur_pos = pos
    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from game import Actions

    print "Start:", problem.getStartState()
    print "Goal:", problem.goal
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    visited = []
    neighbors = []
    prev = {}
    neighbors.append((problem.getStartState(), 0))

    def getMinCostPos(neighbors):
        minInd = 0
        minCost = 0
        idx = 0
        for pos, cost in neighbors:
            if cost < minCost:
                minInd = idx
                minCost = cost
            idx += 1
        return neighbors.pop(minInd)

    while len(neighbors) > 0:
        curState = getMinCostPos(neighbors)
        successors = problem.getSuccessors(curState[0])
        for successor in successors:
            if successor[0] not in visited:
                if problem.isGoalState(successor[0]):
                    prev[successor[0]] = curState[0]
                    neighbors = []
                    break
                else:
                    visited.append(successor[0])
                    prev[successor[0]] = curState[0]
                    neighbors.append((successor[0], curState[1] + successor[-1]))

    res_path = []
    cur_pos = problem.goal
    res_path.append(cur_pos)
    while cur_pos != problem.getStartState():
        cur_pos = prev[cur_pos]
        res_path.append(cur_pos)
    res_path = res_path[::-1]
    actions = []
    cur_pos = res_path[0]
    dir_dict = Actions._directions

    for pos in res_path[1:]:
        delta_pos = (pos[0] - cur_pos[0], pos[1] - cur_pos[1])
        dir = {i for i in dir_dict if dir_dict[i]==delta_pos}
        if len(dir) == 0:
            print pos, cur_pos, delta_pos, len(dir)
            print('Buggggggg')
            exit()
        actions.append(list(dir)[0])
        cur_pos = pos

    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
