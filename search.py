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
from pacman import Directions

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
    # path found
    valid_path=[]

    # store the processed state 
    close_state = []
    # put the starting state into close_state
    close_state.insert(0,problem.getStartState())

    # path find function
    def getValidPath(state):
        # if no succeeding state, return
        if len(state) == 0:
            return 1

        # process each succeeding state
        for item in reversed(state):
            # if the state hasn't been processed
            if item[0] not in close_state:
                # the state is Goal, put this Action into path, return
                if (problem.isGoalState(item[0])):
                    valid_path.insert(0,item[1])
                    return 0

                # the state has been processed, put into clost_state
                close_state.insert(0,item[0])

                # recursively processed successing state for this state
                p = getValidPath(problem.getSuccessors(item[0]))

                # if found the goal action into valid path
                if p==0 :
                    valid_path.insert(0,item[1])
                    return 0
        return 1

    # from the starting state, get valid path for its successor states
    p = getValidPath(problem.getSuccessors(problem.getStartState()))
    return valid_path 

    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # store the path 
    valid_path=[]
    
    # store the states that have been processed
    close_state = []
    
    # store the states that haven't been processed
    open_state = []  
    
    start_state = problem.getStartState()
   
    # if the starting point is the goal , return 
    if (problem.isGoalState(start_state)):
        return []
   
    #initialization: put the first state/action into queue    
    open_state.insert(0,start_state)  
    valid_path.insert(0,[])

    
    # set found flag = 0
    flag = 0 
    
    while (len(open_state)>0):
        state = open_state.pop()


        act=valid_path.pop()

        if (problem.isGoalState(state)):
             close_state.append(state)
             flag = 1            
             return act

        #print "------Process state:-----------",state       
        close_state.append(state)

        successors = problem.getSuccessors(state)
        
        for item in successors:				
            if item[0] not in close_state and item[0] not in open_state:
                #print "........check successor.....", item[0],item[1]
                new_act=list(act)
                new_act.append(item[1])

                open_state.insert(0, item[0])
                valid_path.insert(0,new_act)

 
    if flag == 0:  # no path found
        return []   


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    # store the path 
    valid_path=[]
    
    # store the states that have been processed
    close_state = []
    
    # store the states that haven't been processed
    open_state = util.PriorityQueue()    
    
    # store action seqence by which can be reach the state from starting poing
    # the seqence is presentented like follows:
    # {'(5,5)':['w,s,e,w',10],'(10,8)':['w,w,e,w,s,n,e',30]}
    action_seq = {}
      

    # if the starting point is the goal , return 
    start_state = problem.getStartState()  
    if (problem.isGoalState(start_state)):
        return []
   
    #put the first state into open_state queue with cost 0   
    open_state.push(start_state, 0) ;
    
    #assign the start start with empty action and cost 0      
    action_seq[str(tuple(start_state))] = []
    action_seq[str(tuple(start_state))]=([],0)

    # set found flag = 0
    flag = 0 

    
    while (open_state.isEmpty()==False):
        state = open_state.pop()

        #print "------------Parent's--------------",state
        # get parent state's action sequence and cost value
        act = action_seq[str(tuple(state))][0]
        cst = action_seq[str(tuple(state))][1]
        
        # if the state is Goal state, return the path
        if (problem.isGoalState(state)):
            #close_state.append(item[0])
            flag=1
            #print act
            return act  
 
        close_state.append(state)

        # get succeeding states
        successors = problem.getSuccessors(state)
        
        
        for item in successors:				
            # check if the succeeding state has been processed before
            if item[0] not in close_state:
                #print "........process....", item[0],item[1]

                # generate path for this succ state (from start to the state)
                new_act=list(act) 
                new_act.append(item[1])
                #print "child's action:",new_act
 
                # get the cost from start to this state
                cost=problem.getCostOfActions(new_act)
                if cost==999999:  # if cost is 999999, no need to process this state
                    close_state.append(item[0])
                    continue
                #print cost                       
                
                # put the un-processed state into open queue, and update state cost if necessary
                open_state.update(item[0],cost)

                # update action queue with new cost if necessary
                if str(tuple(item[0])) in action_seq.keys():
                    if (action_seq[str(tuple(item[0]))][1]>cost):
                        action_seq[str(tuple(item[0]))]=(new_act,cost)
                else:
                    action_seq[str(tuple(item[0]))]=[]
                    action_seq[str(tuple(item[0]))]=(new_act,cost)
 
 
    if flag == 0:  # no path found
    	  return []    


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    # store the path 
    valid_path=[]
    
    # store the states that have been processed
    close_state = []
    
    # store the states that haven't been processed
    open_state = util.PriorityQueue()    
    

    # store action seqence by which can be reach the state from starting poing
    # the seqence is presentented like follows:
    # {'(5,5)':['w,s,e,w',10],'(10,8)':['w,w,e,w,s,n,e',30]}
    action_seq = {}
    
    start_state = problem.getStartState()
    #print type(start_state)
   
    # if the starting point is the goal , return 
    if (problem.isGoalState(start_state)):
        return []
   
    #put the first state into open_state queue with cost 0   
    open_state.push(start_state, 0) ;
    
    #assign the start start with empty action and cost 0      
    action_seq[str(tuple(start_state))] = []
    action_seq[str(tuple(start_state))]=([],0) 

    # set found flag = 0
    flag = 0 

    
    while (open_state.isEmpty()==False):
        state = open_state.pop()
        #print "---------Parent's state------------",state
        act=action_seq[str(tuple(state))][0]
        #print action_seq[str(tuple(state))][0][0]
        cst = action_seq[str(tuple(state))][1]

        if (problem.isGoalState(state)):
            return act       
        
        close_state.append(state)

        successors = problem.getSuccessors(state)
        
        for item in successors:				
            if item[0] not in close_state:
                #print "......process.suc.....", item[0],item[1]
                new_act=list(act)
                new_act.append(item[1])
                #print "child's action:",new_act
                cost=problem.getCostOfActions(new_act)+heuristic(item[0],problem)
              
                if cost==999999:
                    close_state.append(item[0])
                    continue
           
                open_state.update(item[0],cost)

                if str(tuple(item[0])) in action_seq.keys():
                    if (action_seq[str(tuple(item[0]))][1]>cost):
                        action_seq[str(tuple(item[0]))]=(new_act,cost)
                else:
                    action_seq[str(tuple(item[0]))]=[]
                    action_seq[str(tuple(item[0]))]=(new_act,cost)
                    #print action_seq
 
    if flag == 0:  # no path found
    	  return []    

    #util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
