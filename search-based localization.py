import operator
import numpy as np
import pickle
#from ultraSonicThreads import *
from random import randint
from Astar import aStarSearch
from SearchProblem import SearchProblem
import serial
import time
ser=serial.Serial('/dev/ttyAMA0',baudrate=9600, bytesize=serial.EIGHTBITS,timeout=1,stopbits=1)

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


        i = 0

        position = self.start
        action = path[i]


        while(1):

            distance_west = distanceMeasurement(triggerList[0], echoList[0], 0)
            distance_north = distanceMeasurement(triggerList[1], echoList[1], 1)
            distance_east = distanceMeasurement(triggerList[2], echoList[2], 2)

            if (distance_west < 10 and action == "W"):

                 increment = self.get_obstacle_pos(action)
                 obstacle = tuple(map(operator.add, position, increment))

                 self.add_obstacles(obstacle)

                 self.start = position

                 #get image from hatem
                 #send to MC to rotate 90 to the left
                 ser.write('L')
                 images[image] = position
                 #send to MC to rotate 90 back
                 ser.write('R')

                 break

            elif(distance_north < 10 and action == "N"):

                 increment = self.get_obstacle_pos(action)
                 obstacle = tuple(map(operator.add, position, increment))

                 self.add_obstacles(obstacle)

                 self.start = position

                 #get image from hatem
                 images[image] = position

                 break


            elif(distance_east < 10 and action == "E"):

                 increment = self.get_obstacle_pos(action)
                 obstacle = tuple(map(operator.add, position, increment))

                 self.add_obstacles(obstacle)

                 self.start = position


                 #send to MC to rotate 90 to the right
                 ser.write('R')
                 #get image from hatem
                 images[image] = position
                 #send to MC to rotate 90 back
                 ser.write('L')
                 break


            else:

                #send action to MC
                print(action)
                ser.write(action)
                data=0
                while data !='1':
					data=ser.read(10)
                increment = self.get_obstacle_pos(action)
                position = tuple(map(operator.add, position, increment))
                i += 1

                self.start = position

                if(i == len(path)):
                    break

                action = path[i]





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

images = {}

#distance_west = 5
#distance_east = 10
#distance_north = 1

robot = localization(start, obstacles, map_d, obstacles_num)

iterations = 0

while(iterations < 10):

    robot.search_and_localize()

    iterations += 1

    #if(robot.stop()):

        #obstacles = robot.get_obstacles()
        #break


#obstacles = [(1, 2), (2, 3)]

obstacles = robot.get_obstacles()

grid = np.zeros((map_d, map_d))

for i in range(len(obstacles)):
    grid[obstacles[i]] = 1

print(grid)


with open("obstacles", "wb") as fb:
    pickle.dump(obstacles, fb)


with open("images", "wb") as fb:
    pickle.dump(images, fb)
