#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np

from bezier import Bezier
import pinocchio as pin
from config import LEFT_HAND, RIGHT_HAND

from ndcurves.optimization import problem_definition, setup_control_points
from nd_helper_functions import obtain_pts_time, genCost, quadprog_solve_qp

Kp = 1e5            # proportional gain (P of PD)
Kv = 2*np.sqrt(Kp)  # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent):
    q, vq = sim.getpybulletstate()
    q_of_t, vq_of_t, vvq_of_t = trajs

    aq = vvq_of_t(tcurrent) - Kp*(q - q_of_t(tcurrent)) - Kv*(vq - vq_of_t(tcurrent))
    
    # We now compute necessary torques to apply on left and right hand.
    IDX_LEFT_HAND = robot.model.getFrameId(LEFT_HAND)
    IDX_RIGHT_HAND = robot.model.getFrameId(RIGHT_HAND)

    pin.framesForwardKinematics(robot.model,robot.data,q)
    pin.computeJointJacobians(robot.model,robot.data,q)
    
    # Get corresponding jacobian for left hand and right hand
    lefthand_Jlefthand = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_LEFT_HAND)
    righthand_Jrighthand = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_RIGHT_HAND)
    
    abs_contact_force = 120
    contact_force = np.array([0,0,0,0,0,abs_contact_force])
    tau_right = righthand_Jrighthand.T @ (contact_force)
    tau_left = lefthand_Jlefthand.T @ (-contact_force)
    torques = pin.rnea(robot.model, robot.data, q, vq, aq) + tau_right + tau_left
    
    sim.step(torques)

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath

    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, None)

    _, dense_q_path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    #setting initial configuration
    sim.setqsim(q0)
    
    def maketraj(q0,q1,q_path,T): 
        # Having the first 3 control points equal means we start and end with 0 velocity and acceleration
        q_path = [q0]*3 + q_path + [q1]*3
        
        q_of_t = Bezier(q_path,t_max=T)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t
    
    total_time=5.
    
    # We first create a high-dimensional Bezier curve using all of our robot configurations as control point.
    # We then sample from it.
    trajs = maketraj(q0, qe, dense_q_path, T=1) 
    n_samples = 1000
    ptsTime = obtain_pts_time(n_samples, trajs[0])
    
    # The objective is then to define new lower-dimensional Bezier curve which approximates samples we took
    # from high-dimensional Bezier curve.
    dim = len(q0)
    degree = 9
    
    pD = problem_definition(dim)
    pD.degree = degree
    problem = setup_control_points(pD)
    variableBezier = problem.bezier()
    A, b = genCost(variableBezier, ptsTime)
    res = quadprog_solve_qp(A, b) 
    opt_control_points = np.array_split(res, degree+1) # optimal control points for low-dim Bezier curve

    # We now define a new lower-dimensional Bezier curve which tracks our previous path.
    trajs = maketraj(q0, qe, opt_control_points, total_time)
 
    tcur = 0.
    
    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, trajs, tcur)
        tcur += DT
    
    final_cube_loc = np.array(sim.p.getBasePositionAndOrientation(sim.cubeId)[0]) # final cube location
    distance = np.linalg.norm(final_cube_loc - CUBE_PLACEMENT_TARGET.translation)
    print(f"Distance to target of CoM of cube = {round(distance, 4)} meters.\n")
    

