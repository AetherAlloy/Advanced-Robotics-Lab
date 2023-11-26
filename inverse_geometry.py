#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""
import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, cube_collision, getcubeplacement, setcubeplacement, projecttojointlimits, distanceToObstacle
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
from tools import setcubeplacement

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
    DT = 0.05
    is_it_grasping = False
    
    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    
    IDX_LEFT_HAND = robot.model.getFrameId(LEFT_HAND)
    IDX_RIGHT_HAND = robot.model.getFrameId(RIGHT_HAND)
    
    q = qcurrent.copy()
    while True:
        # Run the algorithms that outputs values in robot.data
        pin.framesForwardKinematics(robot.model,robot.data,q)
        pin.computeJointJacobians(robot.model,robot.data,q)
        
        # We define the position and rotation of left and right hands
        oMlefthand = robot.data.oMf[IDX_LEFT_HAND]
        oMrighthand = robot.data.oMf[IDX_RIGHT_HAND]
        
        # We now define the toolMgoal equivalent
        lefthandMcubeL = oMlefthand.inverse() * oMcubeL     # Here matrix multiplication is given by * as we have Pinocchio objects
        righthandMcubeR = oMrighthand.inverse() * oMcubeR
        
        # 6D error between the two frames.
        lefthand_nuL = pin.log(lefthandMcubeL).vector
        o_nuL = oMlefthand.action @ lefthand_nuL            # Here matrix  multiplication is given by @ as oMf.action gives numpy matrix
        
        righthand_nuR = pin.log(righthandMcubeR).vector
        o_nuR = oMrighthand.action @ righthand_nuR
        
        # We stop once grasping configuration has been found
        if np.linalg.norm(o_nuL) < 2e-3 and np.linalg.norm(o_nuR) < 2e-3:
            # When hands get close enough to hook frames, we have a grasping configuration
            is_it_grasping = True
            break
        
        # Get corresponding jacobian for left hand and right hand
        lefthand_Jlefthand = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_LEFT_HAND)
        o_Jlefthand = oMlefthand.action @ lefthand_Jlefthand
        
        righthand_Jrighthand = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_RIGHT_HAND)
        o_Jrighthand = oMrighthand.action @ righthand_Jrighthand
        
        # We first calculate the optimal vq to approximate the left hand getting closer to the desired target. 
        # Using the null space of this first task, we then calculate the correction to vq necessary to also solve
        # the right hand task.
        vq = pinv(lefthand_Jlefthand) @ lefthand_nuL # [nv] 
        Plefthand = np.eye(robot.nv) - pinv(o_Jlefthand) @ o_Jlefthand
        vq += pinv(o_Jrighthand @ Plefthand) @ (o_nuR - o_Jrighthand @ vq)

        q = pin.integrate(robot.model, q, vq * DT)
        
        # If I want to display
        if viz is not None:
            viz.display(q)
    
    is_it_colliding = collision(robot, q) or cube_collision(cube)
    is_it_valid = is_it_grasping and not(is_it_colliding)
    
    return q, is_it_valid

if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()

    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, viz)
    setcubeplacement(robot, cube, CUBE_PLACEMENT)
    updatevisuals(viz, robot, cube, q0)
    