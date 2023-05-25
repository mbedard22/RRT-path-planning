## COMP.4500- Mobile Robotics, SPR23
## Lab #4- Path Planning (Part 2)
## Danielle Le & Matthew Bedard 
import cozmo
from cmap import *
from gui import *
from utils import *
import asyncio
import os
import sys
import math
import time
import cv2
import random
from copy import deepcopy
from glob import glob
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from cozmo.util import degrees, Pose, distance_mm, speed_mmps, Angle
MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

cube_position = None
obs_position = None
obstacle_count = 0
def handle_object_appeared(evt, **kw):
    global cube_position
    global obstacle_count 
# This will be called whenever an EvtObjectAppeared is dispatched whenever an Object comes into view.
    if isinstance(evt.obj, cozmo.objects.LightCube): 
        cube_position = evt.obj.pose
    elif isinstance(evt.obj, CustomObject):
        obs_position = evt.obj.pose
        obstacle = [Node((obs_position.position.x + 85, obs_position.position.y + 70)),
        Node((obs_position.position.x +15, obs_position.position.y + 70)),
        Node((obs_position.position.x +85, obs_position.position.y + 0)), 
        Node((obs_position.position.x +15, obs_position.position.y + 0))]
        cmap.add_obstacle(obstacle)
        obstacle_count = obstacle_count + 1
        print("OBSTACLE ADDED!!!!!!")
    else:
        pass        

def handle_object_disappeared(evt, **kw):
    global cube_position

    if isinstance(evt.obj, cozmo.objects.LightCube):
        cube_position = None
    elif isinstance(evt.obj, CustomObject):
        obs_position = None
def coordinate_transform(robot_coords,cube_coords):
    robot_deg = robot_coords.rotation.angle_z.radians
    robot_cos = math.cos(robot_deg)
    robot_sin = math.sin(robot_deg)
    translation_matrix = np.matrix([[robot_cos, -robot_sin, robot_coords.position.x],
                                    [robot_sin, robot_cos, robot_coords.position.y],
                                    [0,     0,  1]])

    inverse = translation_matrix.I
    matrix = np.matrix([[cube_coords.position.x], [cube_coords.position.y],[1]])
    x = (inverse * matrix).A[0][0]
    y = (inverse * matrix).A[1][0]
    return x,y

def step_from_to(node0, node1, limit=75):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    distance = get_dist(node0, node1)


    if distance < limit:
        return node1
    else: 
        angle = np.arctan2(node1.y-node0.y, node1.x - node0.x)
        midNode = Node((node0.x + limit * np.cos(angle), node0.y + limit * np.sin(angle)))
        return midNode

    ############################################################################

def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    rand_node = Node([-5,-5])
    cmap_width = cmap.width
    cmap_height = cmap.height

    while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        new_x = random.randint(0, cmap_width)
        new_y = random.randint(0, cmap_height)
        rand_node = Node((new_x, new_y))
    return rand_node
 
    ############################################################################
def RRT(cmap, start):
    cmap.add_node(start)

    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #

        rand_node = cmap.get_random_valid_node() # Gets a random node to start
        nearest_node = None # No nearest node to begin with

        for map_node in cmap.get_nodes(): # iterates through the nodes on the cmap
            if nearest_node == None: # Sets the nearest_node to the first node on the cmap
                nearest_node = map_node
            elif get_dist(map_node, rand_node) < get_dist(nearest_node, rand_node): # Checks to see if the new node from the for loop is smaller distance
                nearest_node = map_node 

        rand_node = step_from_to(nearest_node, rand_node, 75) # Checks to make sure that the nearest node is within the radius of 75 otherwise returns a new node that is within 75

        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")

def find_angle(node0, node1):
    angle = np.arctan2(node1.y-node0.y, node1.x - node0.x)
    return angle
def distance(x,y):
    x_2 = x**2
    y_2 = y**2
    dist = math.sqrt(x_2 + y_2)
    return dist
async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent
    robot.set_head_angle(cozmo.util.Angle(0)) # Tilt head straight
    robot.move_lift(4) # Moves lift up
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    gain, exposure, mode = 390,3,1
    path_list = []
    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)
    await robot.world.define_custom_cube(CustomObjectTypes.CustomType00, CustomObjectMarkers.Diamonds2, 44, 30, 30, False)
    await robot.world.define_custom_cube(CustomObjectTypes.CustomType00, CustomObjectMarkers.Circles2, 44, 30, 30, False)

########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    
    mid_arena = Node([650 / 2, 450 / 2])
    start_vec= Node([50 + robot.pose.position.x, 35 + robot.pose.position.y])
    initial_obstacles = 0
    angle_to_center = find_angle(start_vec, mid_arena)
    mid_dist =  get_dist(start_vec, mid_arena)
    await  robot.turn_in_place(Angle(radians = angle_to_center), in_parallel= True).wait_for_completed()
    await robot.drive_straight(distance_mm(mid_dist), speed_mmps(50)).wait_for_completed()
    goal_position = cube_position
    while True:
        event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   # Get camera image
       
        print("%s" % goal_position.position.x, goal_position.position.y)
        if goal_position != None:
            robot.stop_all_motors()
            obstacles_list = deepcopy(cmap._obstacles)
            goal_node = Node((goal_position.position.x + 50, goal_position.position.y + 35)) # need to add method to handle cube in negative robot coords
            start_node = Node((robot.pose.position.x + 50, robot.pose.position.y + 35))
            cmap.set_start(start_node) 
            if (not cmap.is_inside_obstacles(goal_node)) and (cmap.is_inbound(goal_node)): #if cube position valid
                print("GOAL NODE VALID")
                cmap.reset()
                cmap.clear_goals()
                cmap.add_goal(goal_node) # add to goal
            RRT(cmap, cmap.get_start())
            if cmap.is_solved():
                for goal in cmap._goals:
                    cur = goal
                    while cur.parent is not None: #creates reversable list to get goal last and closest first
                        path_list.append(cur)
                        cur = cur.parent
            print("ROBOT POSITION: %s" % robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)  
            # print("CUBE POSITION: %s" % cube_position.position.x, cube_position.position.y)
            initial_angle = robot.pose.rotation.angle_z.radians
            for each in reversed(path_list): #goes through list of nodes on path starting at closest to start
                if (obstacle_count == initial_obstacles):
                    x1,y1 = coordinate_transform(robot.pose, goal_position)
                    if distance(x1,y1) < 50:
                        print("reached goal")
                        break 
                    robotX = 50 + robot.pose.position.x 
                    robotY = 35 + robot.pose.position.y
                    goalX = each.x 
                    goalY = each.y
                    drive_dist = cozmo.util.Distance(distance_mm=(math.sqrt((robotX - goalX)**2 + (robotY - goalY)**2)))
                    turn_angle = cozmo.util.Angle(radians=np.arctan2(goalY - robotY, goalX - robotX))
                    await robot.turn_in_place(angle = turn_angle, is_absolute = True).wait_for_completed(None)
                    await robot.drive_straight(distance = drive_dist, speed = cozmo.util.Speed(10)).wait_for_completed(None)
                else:
                    print("BREAKING MOVEMENT LOOP")
                    path_list.clear()
                    cmap.reset()
                    initial_obstacles += 1
                    break
            robot.stop_all_motors()
        else:
            await robot.drive_wheels(-10, 10) #spin
###############################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()

class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()

if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
