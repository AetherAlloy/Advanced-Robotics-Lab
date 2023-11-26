#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""
from inverse_geometry import computeqgrasppose
import pinocchio as pin
from pinocchio.utils import rotate
import numpy as np
from numpy.linalg import pinv
from tools import collision, cube_collision, setcubeplacement
from config import LEFT_HAND, RIGHT_HAND, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
import time

rng = np.random.default_rng()  

# Function to randomly generate potential pose 
def generate_configuration(robot, cube, viz):
    """
    Randomly generates potential cube locations. Takes as valid only those for which a valid grasping and
    collision-free robot configuration exists.
    """
    x1,y1,z1 = CUBE_PLACEMENT.translation
    x2,y2,z2 = CUBE_PLACEMENT_TARGET.translation
    
    y_low_bound = min(y1, y2) - 0.1
    y_high_bound = max(y1, y2) + 0.2
    y_diff = y_high_bound - y_low_bound

    x_low_bound = min(x1,x2)
    x_high_bound = max(x1,x2)
    x_diff = x_high_bound - x_low_bound

    z_low_bound = min(z1,z2)

    z_high_bound = max(z1,z2) + 0.3
    z_diff = z_high_bound - z_low_bound

    
    while True:
        x_rand = rng.random() * x_diff + x_low_bound
        y_rand = rng.random() * y_diff + y_low_bound
        z_rand = rng.random() * z_diff + z_low_bound

        random_cube_loc = pin.SE3(rotate('z', 0.),np.array([x_rand, y_rand, z_rand]))

        _, is_it_grasping_and_no_collision = computeqgrasppose(robot, robot.q0, cube, random_cube_loc, viz)

        # Only returns cube locations where a valid grasping and collision-free robot configuration exists
        if is_it_grasping_and_no_collision:
            break
    
    return random_cube_loc

def return_old_configuration(cs):
    if len(cs) >= 4:
        return cs[-4]
    elif len(cs) >= 3:
        return cs[-3]
    elif len(cs) >= 2:
        return cs[-2]
    else:
        return cs[-1]

def check_cube_perimeter(robot, cube, c):
    # check for no collision in vicinity of the cube
    c_down_ypos = c + np.array([0,0.05,-0.05])
    c_down_yneg = c + np.array([0,-0.05,-0.05])
    setcubeplacement(robot, cube, pin.SE3(rotate('z',0),c_down_ypos))
    cube_perimeter_collision_pos = pin.computeCollision(cube.collision_model, cube.collision_data, 1)
    setcubeplacement(robot, cube, pin.SE3(rotate('z',0),c_down_yneg))
    cube_perimeter_collision_neg = pin.computeCollision(cube.collision_model, cube.collision_data, 1)

    if cube_perimeter_collision_pos or cube_perimeter_collision_neg:
        return False
    else:
        return True

def compute_path_to_configuration(robot, cube, c0, c1, discretisation_steps, viz=None):
    """
    Given two cube location c0 and c1, interpolates between them and tries to find valid grasping and collision-free
    robot configurations.
    """
    ts = np.linspace(0,1,discretisation_steps)
    q = robot.q0.copy()
    c_old = [c0]

    for t in ts:
        c = c0*(1-t) + c1*t
        setcubeplacement(robot, cube, pin.SE3(rotate('z',0),c)) 
        
        q, is_it_grasping_and_no_collision = computeqgrasppose(robot, q, cube, pin.SE3(rotate('z',0), c), viz)
        
        is_it_grasping_and_no_collision = is_it_grasping_and_no_collision and check_cube_perimeter(robot, cube, c)
        
        if is_it_grasping_and_no_collision:
            c_old.append(c)
        else:
            # We don't return node just before collision, we return node a few steps back. This avoids having a node
            # too constricted by nearby obstacles.
            return pin.SE3(rotate('z',0), return_old_configuration(c_old))        

    return pin.SE3(rotate('z',0), c)

def is_valid_path_to_target(robot, cube, cnew, ct, discretisation_steps, viz):
    """
    For a new node that is found on the graph, checks if we can reach the target from it.
    """
    ts = np.linspace(0,1,discretisation_steps)
    q = robot.q0.copy()

    for t in ts:
        c = cnew*(1-t) + ct*t
        setcubeplacement(robot, cube, pin.SE3(rotate('z',0),c))
        
        q, is_it_grasping_and_no_collision = computeqgrasppose(robot, q, cube, pin.SE3(rotate('z',0),c), viz)
        
        is_it_grasping_and_no_collision = is_it_grasping_and_no_collision and check_cube_perimeter(robot, cube, c)

        if not(is_it_grasping_and_no_collision):
            return False

    return True

def distance(c1, c2):
    return np.sqrt(np.sum((c2.translation-c1.translation)**2))

def nearestvertex(G, cube_rand):
    dist_closest_vertex = np.inf
    
    for i, node in enumerate(G):
        _ , c = node
        dist = distance(cube_rand, c)
        if (dist < dist_closest_vertex):
            inear = i
            dist_closest_vertex = dist
    
    return inear

def ADD_EDGE_AND_VERTEX(G,parent,c):
    G += [(parent,c)]

def create_graph(robot, cube, cube_init,cube_goal,discretisationsteps,viz=None):
    """
    Creates graph of cube locations such that the robot can traverse the graph to successfully reach the
    target cube location starting from the initial cube location.
    """
    G = [(None, cube_init)]

    for i in range(500):
        cube_rand = generate_configuration(robot, cube, viz)
        cube_near_index = nearestvertex(G, cube_rand)
        cube_near = G[cube_near_index][1]
        cube_new = compute_path_to_configuration(robot, cube, cube_near.translation, cube_rand.translation, discretisationsteps, viz)    
        ADD_EDGE_AND_VERTEX(G,cube_near_index,cube_new)
        
        if is_valid_path_to_target(robot, cube, cube_new.translation, cube_goal.translation, discretisationsteps, viz):
            print ("Path to target found!")
            ADD_EDGE_AND_VERTEX(G,len(G)-1,cube_goal)
            return G, True
    
    return G, False

def get_path(G):
    path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[1]] + path
        node = G[node[0]]
    path = [G[0][1]] + path
    return path

def shortcut(robot, cube, path, discretisationsteps, viz):
    for i, c in enumerate(path):
        for j in reversed(range(i+2,len(path))):
            c2 = path[j]
            if is_valid_path_to_target(robot, cube, c.translation, c2.translation, discretisationsteps, viz):
                path = path[:i+1]+path[j:]
                return path
    return path

def shortcut_advanced(robot, cube, path, discretisationsteps, viz):
    """
    Applies the shortcut function until no more shortcuts can be found.
    """
    prev_path_len = 0
    while len(path) != prev_path_len:
        prev_path_len = len(path)
        path = shortcut(robot, cube, path, discretisationsteps, viz)
    
    return path

def displayedge(robot, cube, q0, c0, c1, discretisation_steps, viz):
    '''Displays the path obtained by linear interpolation of q0 to q1'''     
    ts = np.linspace(0,1,discretisation_steps)

    q = q0.copy()
    q_subpath = []

    for t in ts:
        c = c0.translation*(1-t) + c1.translation*t
        setcubeplacement(robot, cube, pin.SE3(rotate('z',0),c))
        q, _ = computeqgrasppose(robot, q, cube, pin.SE3(rotate('z',0), c), viz)
        q_subpath.append(q)
        
    return q_subpath
        
def displaypath(robot, cube, path, discretisation_steps, viz):
    q = robot.q0
    for c0, c1 in zip(path[:-1],path[1:]):
        q = displayedge(robot, cube, q, c0, c1, discretisation_steps, viz)[-1]

def generate_configuration_path(robot, cube, discretisation_steps, path, viz):
    """
    Given path of cube locations, interpolates between them to obtain dense set of robot configurations,
    which show the robot grasping a cube from the initial position and taking it to the target.
    """
    q = robot.q0
    q_path = []
    for c0, c1 in zip(path[:-1],path[1:]):
        q_path += displayedge(robot, cube, q, c0, c1, discretisation_steps, viz)
    return q_path

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(robot, cube, qinit,qgoal,cubeplacementq0, cubeplacementqgoal):
    """
    Returns two paths. One is the path of the CoM of the cube locations. The other is a dense path
    of grasping robot configurations guiding the robot from the initial position to the target.
    """
    print("Computing path.")
    discretisationsteps = 50
    
    graph, valid_path = create_graph(robot, cube, cubeplacementq0, cubeplacementqgoal, discretisationsteps, viz=None)
    if not(valid_path):
        print("Unable to find valid path")
        return [cubeplacementq0, cubeplacementqgoal], [qinit, qgoal]

    path = get_path(graph)
    path = shortcut_advanced(robot, cube, path, discretisationsteps, viz=None) 
    q_path = generate_configuration_path(robot, cube, discretisationsteps, path=path, viz=None)

    return path, q_path

if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    
    robot, cube, viz = setupwithmeshcat()

    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, None)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
  
    cube_path, dense_q_path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    displaypath(robot, cube, path=cube_path, discretisation_steps=100, viz=viz)
    
