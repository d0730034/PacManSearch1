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

"""
class Node:
    
    For building the tree node for the solution. The elements of the node will have:
 
    state: current position (x, y).
    path: the plan for move (x+p, y+p).
    gcost: the real cost between two nodes.
    heristic: the guessing distance to the goal node.
    
    def __init__(self, state, path, gcost=0, heuristic=0):
        self.state = state
        self.path = path
        self.gcost = gcost
        self.heuristic = heuristic
"""

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
    #Set up of direction
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST

    #basic set up element for DFS
    #current state 
    curr_state = problem.getStartState()
    #The dictionary for visited
    visited = {}
    #The list for visited in current state
    visited[curr_state] = []
    #the next possible movement 
    next_state_movement = util.Stack()

    #search goal state until hit the target
    while True:
        #look the every possible movement from getSuccessors
        for every_movement in problem.getSuccessors(curr_state):
            #only choose the un-visited state for next possible movement
            if every_movement[0] not in visited:
                 #build up the stack tree for un-visited state information
                 path = list(visited[curr_state])
                 path.append(every_movement[1])
                 next_state_movement.push((every_movement[0], path))
        #after getting the next state movement then pop it out
        next_state = next_state_movement.pop()
        #update the new information to the information's record
        visited[next_state[0]] = next_state[1]
        curr_state = next_state[0]
        #if hit the goal state then return the result
        if problem.isGoalState(curr_state):
            return visited[curr_state]

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #Set up of direction
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST
    #basic set up element for BFS
    #current state
    curr_state = problem.getStartState()
    #the next possible movement
    next_state_movement = util.Queue()
    #The dictionary for visited
    visited = {}
    #The list for visited in current state
    visited[curr_state] = []
    #search goal state until hit the goal state
    while True:
        #look the every possible movement from getSuccessors
        for every_movement in problem.getSuccessors(curr_state):
            #only choose the un-visited state for next possible movement
            if every_movement[0] not in visited:
                #build up the stack tree for un-visited state information
                path = list(visited[curr_state])
                path.append(every_movement[1])
                next_state_movement.push((every_movement[0], path))
                visited[every_movement[0]] = []
        #after getting the next state movement then pop it out
        next_state = next_state_movement.pop()
        #update the new information to the information's record
        visited[next_state[0]] = next_state[1]
        curr_state = next_state[0]
        #if hit the goal state then return the result
        if problem.isGoalState(curr_state):
            return visited[curr_state]
            

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST
    #basic set up element for UCS
    #next possible state movement
    next_state_movement = util.PriorityQueue()
    #current state
    curr_state = problem.getStartState()
    path_for_movement = []
    next_state_movement.push((curr_state, [], 0), 0)
    #search util hit the goal state
    while True:
        #pop out the information as given.
        (curr_state, visited, g_cost) = next_state_movement.pop()
        #check if current state if goal state before calculating  for special case
        if problem.isGoalState(curr_state):
            return visited
        #if current state is not in path then add it in.
        #For quick note UCS is depend on the least of cost of g
        if not curr_state in path_for_movement:
            path_for_movement.append(curr_state)
            for every_state, every_movement, every_g_cost in problem.getSuccessors(curr_state):
                next_state_movement.push((every_state, visited + [every_movement], g_cost + every_g_cost), g_cost + every_g_cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from game import Directions
    #basic set up element for A*
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST
    #next possible state movement
    next_state_movement = util.PriorityQueue()
    #current state
    curr_state = problem.getStartState()
    path_for_movement = []
    next_state_movement.push((curr_state, [], 0), 0)
    #search until hit the goal state
    while True:
        #pop out the information as given
        (curr_state, visited, g_cost) = next_state_movement.pop()
        #check if current state is goal state of not before the calculating
        if problem.isGoalState(curr_state):
            return visited
        #if current state is not in path then add it in.
        #For quick not A* search is combine of UCS and Greedy Search.
        #The movement of A* depend on least f(n) = g(n) + h(n)
        if not curr_state in path_for_movement:
            path_for_movement.append(curr_state)
            for every_state, every_movement, every_g_cost in problem.getSuccessors(curr_state):
                next_state_movement.push((every_state, visited + [every_movement], g_cost + every_g_cost), g_cost + every_g_cost + heuristic(every_state, problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
