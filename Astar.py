from SearchProblem import SearchProblem
from utils import PriorityQueue


def nullHeuristic(state, problem=None):
	"""
	A heuristic function estimates the cost from the current state to the nearest
	goal in the provided SearchProblem.  This heuristic is trivial.
	"""
	return 0

def manhattanHeuristic(position, problem, info={}):

    "The Manhattan distance heuristic for a PositionSearchProblem"

    xy1 = position
    xy2 = problem.getGoal()

    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem, info={}):

    "The Euclidean distance heuristic for a PositionSearchProblem"

    xy1 = position
    xy2 = problem.getGoal()
    
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5


def aStarSearch(problem, heuristic=manhattanHeuristic):
    
    """Search the node that has the lowest combined cost and heuristic first."""

    priority_q = PriorityQueue()
    visited = []
    node = {}

    root = problem.getStartState()
    
    node["parent"] = None
    node["action"] = None
    node["goal"] = 0
    node["heuristic"] = heuristic(root, problem)
    node["state"] = root

    priority_q.push(node, node["goal"] + node["heuristic"])     #push root

    while(not priority_q.isEmpty()):    

        node = priority_q.pop()
        state = node["state"]

        if(problem.isGoalState(state)):
            break

        if(state in visited):
            continue

        visited.append(state)
        children = problem.getSuccessors(state)
        
        if(children):
            for i in range(len(children)):

                if(children[i][0] not in visited):
                    sub_node = {}
                    sub_node["parent"] = node
                    sub_node["action"] = children[i][1]
                    sub_node["state"] = children[i][0]
                    sub_node["goal"] = children[i][2] + node["goal"]
                    sub_node["heuristic"] = heuristic(sub_node["state"], problem)
                    priority_q.push(sub_node, sub_node["goal"] + sub_node["heuristic"])
        
        
    path = []
    while(node["action"] != None):
        path.insert(0, node["action"])
        node = node["parent"]


    return path


if(__name__ == "__main__"):
    
    start = (0, 0)    
    obstacles = [(3, 1), (1, 2), (3, 3)]
    goal = (4, 4)
    
    problem = SearchProblem(start, goal, obstacles, 5)
    
    print(aStarSearch(problem))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    