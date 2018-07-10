#
# === Introduction ===
#
# In this problem, you will again build a planner that helps a robot
#   find the best path through a warehouse filled with boxes
#   that it has to pick up and deliver to a dropzone. Unlike Part A,
#   however, in this problem the robot is moving in a continuous world
#   (albeit in discrete time steps) and has constraints on the amount
#   it can turn its wheels in a given time step.
# 
# Your file must be called `partB.py` and must have a class
#   called `DeliveryPlanner`.
# This class must have an `__init__` function that takes five 
#   arguments: `self`, `warehouse`, `todo`, `max_distance`, and
#   `max_steering`.
# The class must also have a function called `plan_delivery` that 
#   takes a single argument, `self`.
#
# === Input Specifications ===
# 
# `warehouse` will be a list of m strings, each with n characters,
#   corresponding to the layout of the warehouse. The warehouse is an
#   m x n grid. warehouse[i][j] corresponds to the spot in the ith row
#   and jth column of the warehouse, where the 0th row is the northern
#   end of the warehouse and the 0th column is the western end.
#
# The characters in each string will be one of the following:
#
# '.' (period) : traversable space.
# '#' (hash) : a wall. If the robot contacts a wall space, it will crash.
# '@' (dropzone): the space where all boxes must be delivered. The dropzone may be traversed like 
#   a '.' space.
#
# Each space is a 1 x 1 block. The upper-left corner of space warehouse[i][j] is at the point (j,-i) in
#   the plane. Spaces outside the warehouse are considered walls; if any part of the robot leaves the 
#   warehouse, it will be considered to have crashed into the exterior wall of the warehouse.
# 
# For example, 
#   warehouse = ['.#.',
#                '.#.',
#                '..@']
#   is a 3x3 warehouse. The dropzone is at space (2,-2) and there are walls at spaces (1,0) 
#   and (1,-1). The rest of the warehouse is empty space.
#
# The robot is a circle of radius 0.25. The robot begins centered in the dropzone space.
#   The robot's initial bearing is 0.
#
# The argument `todo` is a list of points representing the center point of each box.
#   todo[0] is the first box which must be delivered, followed by todo[1], and so on.
#   Each box is a square of size 0.2 x 0.2. If the robot contacts a box, it will crash.
#
# The arguments `max_distance` and `max_steering` are parameters constraining the movement
#   of the robot on a given time step. They are described more below.
#
# === Rules for Movement ===
#
# - The robot may move any distance between 0 and `max_distance` per time step.
# - The robot may set its steering angle anywhere between -`max_steering` and 
#   `max_steering` per time step. A steering angle of 0 means that the robot will
#   move according to its current bearing. A positive angle means the robot will 
#   turn counterclockwise by `steering_angle` radians; a negative steering_angle 
#   means the robot will turn clockwise by abs(steering_angle) radians.
# - Upon a movement, the robot will change its steering angle instantaneously to the 
#   amount indicated by the move, and then it will move a distance in a straight line in its
#   new bearing according to the amount indicated move.
# - The cost per move is 1 plus the amount of distance traversed by the robot on that move.
#
# - The robot may pick up a box whose center point is within 0.5 units of the robot's center point.
# - If the robot picks up a box, it incurs a total cost of 2 for that move (this already includes 
#   the 1-per-move cost incurred by the robot).
# - While holding a box, the robot may not pick up another box.
# - The robot may put a box down at a total cost of 1.5 for that move. The box must be placed so that:
#   - The box is not contacting any walls, the exterior of the warehouse, any other boxes, or the robot
#   - The box's center point is within 0.5 units of the robot's center point
# - A box is always oriented so that two of its edges are horizontal and the other two are vertical.
# - If a box is placed entirely within the '@' space, it is considered delivered and is removed from the 
#   warehouse.
# - The warehouse will be arranged so that it is always possible for the robot to move to the 
#   next box on the todo list without having to rearrange any other boxes.
#
# - If the robot crashes, it will stop moving and incur a cost of 100*distance, where distance
#   is the length it attempted to move that move. (The regular movement cost will not apply.)
# - If an illegal move is attempted, the robot will not move, but the standard cost will be incurred.
#   Illegal moves include (but are not necessarily limited to):
#     - picking up a box that doesn't exist or is too far away
#     - picking up a box while already holding one
#     - putting down a box too far away or so that it's touching a wall, the warehouse exterior, 
#       another box, or the robot
#     - putting down a box while not holding a box
#
# === Output Specifications ===
#
# `plan_delivery` should return a LIST of strings, each in one of the following formats.
#
# 'move {steering} {distance}', where '{steering}' is a floating-point number between
#   -`max_steering` and `max_steering` (inclusive) and '{distance}' is a floating-point
#   number between 0 and `max_distance`
# 
# 'lift {b}', where '{b}' is replaced by the index in the list `todo` of the box being picked up
#   (so if you intend to lift box 0, you would return the string 'lift 0')
#
# 'down {x} {y}', where '{x}' is replaced by the x-coordinate of the center point of where the box
#   will be placed and where '{y}' is replaced by the y-coordinate of that center point
#   (for example, 'down 1.5 -2.9' means to place the box held by the robot so that its center point
#   is (1.5,-2.9)).
#
# === Grading ===
# 
# - Your planner will be graded against a set of test cases, each equally weighted.
# - Each task will have a "baseline" cost. If your set of moves results in the task being completed
#   with a total cost of K times the baseline cost, you will receive 1/K of the credit for the
#   test case. (Note that if K < 1, this means you earn extra credit!)
# - Otherwise, you will receive no credit for that test case. This could happen for one of several 
#   reasons including (but not necessarily limited to):
#   - plan_delivery's moves do not deliver the boxes in the correct order.
#   - plan_delivery's output is not a list of strings in the prescribed format.
#   - plan_delivery does not return an output within the prescribed time limit.
#   - Your code raises an exception.
#
# === Additional Info ===
# 
# - You may add additional classes and functions as needed provided they are all in the file `partB.py`.
# - Your partB.py file must not execute any code when it is imported. 
# - Upload partB.py to Project 2 on T-Square in the Assignments section. Do not put it into an 
#   archive with other files.
# - Ask any questions about the directions or specifications on Piazza.
#

import copy

factor = 5

def expand_warehouse(warehouse, factor):
    expand = [[' ' for row in range(len(warehouse[0])*factor)] for col in range(len(warehouse)*factor)]
    for i in range(len(expand)):
        for j in range(len(expand[0])):
            expand[i][j] = warehouse[i/factor][j/factor]
    return expand


def add_wall(warehouse):
    row = len(warehouse[0])
    col = len(warehouse)
    warehouse_new = copy.deepcopy(warehouse)
    for i in range(col):
        for j in range(row):
            if warehouse[i][j] == '#' and j-1 >= 0 and j+1 < row and i-1 >= 0 and i+1 < col:
                warehouse_new[i][j-1] = '#'
                warehouse_new[i][j+1] = '#'
                warehouse_new[i-1][j] = '#'
                warehouse_new[i+1][j] = '#'
                warehouse_new[i-1][j-1] = '#'
                warehouse_new[i+1][j+1] = '#'
                warehouse_new[i-1][j+1] = '#'
                warehouse_new[i+1][j-1] = '#'  
    for j in range(row):
        warehouse_new[0][j] = '#'
        warehouse_new[-1][j] = '#'
    
    for i in range(col):
        warehouse_new[i][0] = '#'
        warehouse_new[i][-1] = '#'
    
    return warehouse_new

def add_box(warehouse, box):
    row = len(warehouse[0])
    col = len(warehouse)
    warehouse_new = copy.deepcopy(warehouse)
    for i in range(col):
        for j in range(row):
            if (box[0]-(2*j+1)*0.1)**2+(box[1]+(2*i+1)*0.1)**2 < 0.39**2:
                warehouse_new[i][j] = '#'
    return warehouse_new


def clean_box(warehouse, box):
    row = len(warehouse[0])
    col = len(warehouse)
    warehouse_new = copy.deepcopy(warehouse)
    for i in range(col):
        for j in range(row):
            if (box[0]-(2*j+1)*0.1)**2+(box[1]+(2*i+1)*0.1)**2 < 0.39**2:
                warehouse_new[i][j] = '.'
    return warehouse_new


def pos_calculation(path, max_steering, dropzone_coordinate):
    steering = 0
    bearing  = 0
    temp     = []
    pos      = []

    for i in range(len(path)-2):
        temp = []
        if path[i] != 'lift' and path[i] != 'down':
            j = i+1
            if path[j] == 'lift' or path[j] == 'down':
                j = i+2
            temp.append(path[i])
            temp.append(bearing)
            steering = compute_bearing(path[i], path[j])-bearing
            steering = truncate_angle(steering)
        
            if steering <= max_steering and steering >= -max_steering:
                temp.append(steering)
                pos.append(temp)
                temp = []
            else:
                if steering >= max_steering:
                    temp.append(max_steering)
                    pos.append(temp)
                    temp = []
                    temp.append(path[i])
                    temp.append(bearing+max_steering)
                    temp.append(steering-max_steering)
                    pos.append(temp)
                    temp = []
                if steering <= -max_steering:
                    temp.append(-max_steering)
                    pos.append(temp)
                    temp = []
                    temp.append(path[i])
                    temp.append(bearing-max_steering)
                    temp.append(steering+max_steering)
                    pos.append(temp)
                    temp = []
            bearing = compute_bearing(path[i], path[j])
       
        if path[i] == 'lift':
            pos.append('lift')
        if path[i] == 'down':
            pos.append('down')
            
    pos.append(copy.copy(pos[-1]))
    pos[-1][0] = dropzone_coordinate
    pos.append('down')
    
    return pos


def move_calculation(pos, factor, max_distance):
    moves = []
    iii   = 0
    while iii < len(pos)-2:
        k = 0
        for jjj in range(iii+1,len(pos)-1):
            if pos[jjj][0] == pos[iii][0]:
                dist = compute_distance(pos[iii][0], pos[jjj][0])
                temp = 'move '+str(pos[iii][2])+' '+str(dist/factor)
                moves.append(temp)
                iii = jjj

            elif pos[jjj][0] != pos[iii][0] and pos[jjj][1] != pos[jjj+1][1]:
                dist = compute_distance(pos[iii][0], pos[jjj][0])
                temp = 'move '+str(pos[iii][2])+' '+str(dist/factor)
                moves.append(temp)
                iii = jjj
            
            elif pos[jjj][0] != pos[iii][0] and pos[jjj][1] == pos[jjj+1][1]:
                k = 1
        if k == 1:
            dist = compute_distance(pos[iii][0], pos[jjj+1][0])
            if dist >= max_distance*factor:
                temp = 'move '+str(pos[iii][2])+' '+str((max_distance)/factor)
                moves.append(temp)
                temp = 'move '+str(pos[iii][2])+' '+str((dist-max_distance)/factor)
                moves.append(temp)
            else:
                temp = 'move '+str(pos[iii][2])+' '+str(dist/factor)
                moves.append(temp)
            iii = jjj
    return moves



class DeliveryPlanner:
    def __init__(self, warehouse, todo, max_distance, max_steering):
        self.warehouse = copy.deepcopy(warehouse)
        self.todo = todo
        self.dropzone = self.find_dropzone(warehouse)
        self.max_distance = max_distance
        self.max_steering = max_steering

    def find_dropzone(self, warehouse):
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                if self.warehouse[i][j] == '@':
                    return [i,j]
                        
    def find_path(self, start, end):
        path  = []
        [x,y] = start
        path.append([y, -x])
        
        while [x,y] != end:
            x2 = x + self.delta[self.action[x][y]][0]
            y2 = y + self.delta[self.action[x][y]][1]
            x  = x2
            y  = y2
            path.append([y2, -x2]) 
            
        return path
    
    
    def compute_value(self, start):
        self.value[start[0]][start[1]] = 0
        self.action = [[0 for row in range(len(self.warehouse[0]))] for col in range(len(self.warehouse))]
        self.value  = [[100 for row in range(len(self.warehouse[0]))] for col in range(len(self.warehouse))]
        self.delta  = [[-1, 0], # go up
                       [ 0,-1], # go left
                       [ 1, 0], # go down
                       [ 0, 1], # go right
                       [-1,-1], # go up-left
                       [ 1,-1], # go down-left
                       [ 1, 1], # go down-right
                       [-1, 1]] # go up-right
        cost_step   = [2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
        
        change = True
        while change:
            change = False
            for x in range(len(self.warehouse)):
                for y in range(len(self.warehouse[0])):
                    if [x,y] != start and self.warehouse[x][y] != '#':

                        for iii in range(len(self.delta)):
                            x2 = x + self.delta[iii][0]
                            y2 = y + self.delta[iii][1]

                            if x2 >= 0 and x2 < len(self.warehouse) and\
                            y2 >= 0 and y2 < len(self.warehouse[0]) and\
                            (self.warehouse[x2][y2] == '.' or [x2,y2] == start or self.warehouse[x2][y2] == '@'):
                                tmp = self.value[x2][y2] + cost_step[iii]
                                if tmp < self.value[x][y]:
                                    change = True
                                    self.value[x][y]  = tmp
                                    self.action[x][y] = iii
        return self.value
    
    
    def plan_delivery(self):
        dropzone = self.find_dropzone(self.warehouse)
        dropzone = [dropzone[0]+0.5,dropzone[1]+0.5]
        self.warehouse = expand_warehouse(self.warehouse, factor)
        self.warehouse = add_wall(self.warehouse)
        dropzone = [int(round(dropzone[0]*factor))-1, int(round(dropzone[1]*factor))-1]
        dropzone_coordinate = [dropzone[1],-dropzone[0]]

        for i in range(len(self.todo)):
            self.warehouse = add_box(self.warehouse, self.todo[i])
        
        all_moves = []
        box_coordinate = []

        for i in range(len(self.todo)):
            self.warehouse = clean_box(self.warehouse, self.todo[i])
            value = self.compute_value(dropzone) #get the cost map
            end = self.todo[i]
            m = int(-round(end[1]*factor))-1
            n = int(round(end[0]*factor))-1
            end = [m,n]
            box_coordinate.append([n,-m])

            # find the path_go from loadzone or grid adjacent to loadzone to the grid ajacenet to box
            path_go = self.find_path(end, dropzone)
            path_go.reverse()
            path_go.pop()
            path_go.pop()#
            path_go.append('lift')
            value = self.compute_value(end)
            
            # find the path_return from grid adjacent to box to the grid ajacenet to loadzone
            path_return = self.find_path(dropzone, end)
            path_return.pop()
            path_return.reverse()
            path_return.append('down')
            path_go.extend(path_return)
            all_moves.extend(path_go)
        
        pos = pos_calculation(all_moves, self.max_steering, dropzone_coordinate)
        
        #find index of lift or down, and minus 1
        seg_index = []
        for i in range(len(pos)):
            if pos[i] == 'lift' or pos[i] == 'down':
                seg_index.append(i-1)
                
        start = 0
        segment = []
        for i in range(len(seg_index)):
            segment.append(pos[start:seg_index[i]+1])
            start = seg_index[i]

        for each in segment:
            if 'lift' in each:
                each.remove('lift')
            if 'down' in each:
                each.remove('down')

        tmp_result = []
        for i in range(len(segment)):
            tmp_result.append(move_calculation(segment[i], factor, self.max_distance))
            
        k = 0
        for i in range(len(tmp_result)):
            if i%2 == 0:
                tmp_result[i].append('lift '+str(k))
            else:
                tmp_result[i].append('down '+str(dropzone[1]*1.0/factor)+' '+str(-dropzone[0]*1.0/factor))
                k=k+1
        
        results = []
        for i in range(len(tmp_result)):
            results = results + tmp_result[i]

        return results


##### copied from robot.py
import math
import random

PI = math.pi

def compute_distance(p, q):
    x1, y1 = p
    x2, y2 = q

    dx = x2 - x1
    dy = y2 - y1

    return math.sqrt(dx**2 + dy**2)

def compute_bearing(p, q):
    x1, y1 = p
    x2, y2 = q

    dx = x2 - x1
    dy = y2 - y1

    return math.atan2(dy, dx)

def truncate_angle(t):
    return ((t+PI) % (2*PI)) - PI

class Robot:
    def __init__(self, x=0.0, y=0.0, bearing=0.0, max_distance=1.0, max_steering=PI/2):
        self.x = x
        self.y = y
        self.bearing = bearing
        self.max_distance = max_distance
        self.max_steering = max_steering
        #self.set_position(x, y)
        #self.set_bearing(bearing)
        #self.set_max_distance(max_distance)

    def set_noise(self, steering_noise, distance_noise, measurement_noise):
        self.steering_noise = float(steering_noise)
        self.distance_noise = float(distance_noise)
        self.measurement_noise = float(measurement_noise)

    # move the robot
    def move(self, steering, distance, noise=False):
        if noise:
            steering += random.uniform(-0.1,0.1)
            distance *= random.uniform(0.9,1.1)

        steering = max(-self.max_steering, steering)
        steering = min(self.max_steering, steering)
        distance = max(0, distance)
        distance = min(self.max_distance, distance)

        self.bearing = truncate_angle(self.bearing + float(steering))
        self.x += distance * math.cos(self.bearing)
        self.y += distance * math.sin(self.bearing)

    def measure_distance_and_bearing_to(self, point, noise=False):
        
        current_position = (self.x, self.y)

        distance_to_point = compute_distance(current_position, point)
        bearing_to_point = compute_bearing(current_position, point)

        if noise:
            distance_sigma = 0.1*distance_to_point
            bearing_sigma = 0.05*distance_to_point

            distance_noise = random.gauss(0, distance_sigma)
            bearing_noise = random.gauss(0, bearing_sigma)
        else:
            distance_noise = 0
            bearing_noise = 0

        measured_distance = distance_to_point + distance_noise
        measured_bearing = truncate_angle(bearing_to_point + bearing_noise)

        return (measured_distance, measured_bearing)


    # find the next point without updating the robot's position
    def find_next_point(self, steering, distance, noise=False):

        if noise:
            steering += random.uniform(-0.1,0.1)
            distance *= random.uniform(0.9,1.1)

        steering = max(-self.max_steering, steering)
        steering = min(self.max_steering, steering)
        distance = max(0, distance)
        distance = min(self.max_distance, distance)

        bearing = truncate_angle(self.bearing + float(steering))
        x = self.x + (distance * math.cos(bearing))
        y = self.y + (distance * math.sin(bearing))

        return (x, y)


    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)





