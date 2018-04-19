import operator
from random import randint
from Astar import aStarSearch
from SearchProblem import SearchProblem


class localization:
    
    def __init__(self, start, obstacles, map_d, obstacles_num):
        
        self.start = start
        self.obstacles = obstacles
        self.dim = map_d
        self.num = obstacles_num

    def search_and_localize(self):
    
        goal = self.get_random_goal()
        
        problem = SearchProblem(self.start, goal, self.obstacles, self.dim)
            
        path = aStarSearch(problem)
            
        #send path to MC to execute
             
        #while(1):
                
            #position = car position
            #direction = car direction
            
            #distance = ultra sonic readings
                
            #if (distance < 10):
            
                #send stop to MC
                
                #increment = self.get_obstacle_pos(direction)
                #obstacle_pos = tuple(map(operator.add, position, increment))
                
                #self.add_obstacles(obstacle_pos)
                
                #self.start = position
                
                #break
                
            #if(goal sign from MC):
                
                #self.start = goal
                #break
            
    
    def get_obstacle_pos(self, direction):
        
        return{
                "N": (0, 1),
                "S": (0, -1),
                "E": (1, 0),
                "W": (-1, 0)}.get(direction, (0, 0))
        
                
    def add_obstacles(self, obstacle):
        
        self.obstacles.add(obstacle)
        
        
    def get_obstacles(self):
        
        return self.obstacles
        
        
    def get_random_goal(self):
        
        while(1):
            
            goal = tuple((randint(0, self.dim - 1), randint(0, self.dim - 1)))
            
            if(goal != self.start and goal not in self.obstacles):
                return goal
            
        
    def stop(self):
        
        if(self.num == len(self.obstacles)):
            return True
        
        return False
        
                
start = (0, 0)
obstacles = set(())
map_d = 4               #4 x 4
obstacles_num = 3

robot = localization(start, obstacles, map_d, obstacles_num)


while(1):
    
    robot.search_and_localize()
    
    if(robot.stop()):
        
        obstacles = robot.get_obstacles()
        break

























    
            