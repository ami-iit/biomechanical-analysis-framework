import numpy as np
import idyntree.bindings as iDynTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualizeLegSkeleton(kinDynComp, KPI, DATA_LENGTH, joints_state, human_state, W_T_B):
    
    # Compute links origin position wrt world
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])

    linkOriginPosition = { 'Left': { 'UpperLeg': np.zeros((3, DATA_LENGTH)), 'LowerLeg': np.zeros((3, DATA_LENGTH)) , 'Foot': np.zeros((3, DATA_LENGTH)), 'Toe': np.zeros((3, DATA_LENGTH)), 'Heel': np.zeros((3, DATA_LENGTH))} , 'Right': { 'UpperLeg': np.zeros((3, DATA_LENGTH)), 'LowerLeg': np.zeros((3, DATA_LENGTH)) , 'Foot': np.zeros((3, DATA_LENGTH)), 'Toe': np.zeros((3, DATA_LENGTH)), 'Heel': np.zeros((3, DATA_LENGTH)) }}
    
    for lenIdx in range(DATA_LENGTH):
        s = joints_state['positions']['data'][lenIdx][0]
        ds = joints_state['velocities']['data'][lenIdx][0]
        lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
        ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
        base_vel = np.concatenate((lin_vel, ang_vel)) 
        
        kinDynComp.setRobotState(W_T_B[lenIdx], s, base_vel, ds, gravity)

        
        # Right
        link_transform = kinDynComp.getWorldTransform('RightUpperLeg').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Right']['UpperLeg'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('RightLowerLeg').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Right']['LowerLeg'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('RightFoot').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Right']['Foot'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('RightToe').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Right']['Toe'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('RightHeel').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Right']['Heel'][:, lenIdx] = link_transform[0:3,3]
        
        # Left
        link_transform = kinDynComp.getWorldTransform('LeftUpperLeg').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Left']['UpperLeg'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('LeftLowerLeg').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Left']['LowerLeg'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('LeftFoot').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Left']['Foot'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('LeftToe').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Left']['Toe'][:, lenIdx] = link_transform[0:3,3]
        link_transform = kinDynComp.getWorldTransform('LeftHeel').asHomogeneousTransform().toNumPy()
        linkOriginPosition['Left']['Heel'][:, lenIdx] = link_transform[0:3,3]
        
    # Left leg path visualizer
    fig,ax = plt.subplots(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    fig.suptitle('Left skeleton visualizer', fontsize=16)
    ax.grid(True)
    
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    
    ax.set_ylim(auto=True)
    
    # Skeleton: LeftUpperLeg-LeftLowerLeg-LeftFoot-LefToe
    for lenIndx in range(DATA_LENGTH):
        # Build skeleton
        p_LUL_x = linkOriginPosition['Left']['UpperLeg'][0, lenIndx]
        p_LUL_y = linkOriginPosition['Left']['UpperLeg'][1, lenIndx]
        p_LUL_z = linkOriginPosition['Left']['UpperLeg'][2, lenIndx]
        
        p_LLL_x = linkOriginPosition['Left']['LowerLeg'][0, lenIndx]
        p_LLL_y = linkOriginPosition['Left']['LowerLeg'][1, lenIndx]
        p_LLL_z = linkOriginPosition['Left']['LowerLeg'][2, lenIndx]
        
        p_LF_x = linkOriginPosition['Left']['Foot'][0, lenIndx]
        p_LF_y = linkOriginPosition['Left']['Foot'][1, lenIndx]
        p_LF_z = linkOriginPosition['Left']['Foot'][2, lenIndx]
        
        p_LT_x = linkOriginPosition['Left']['Toe'][0, lenIndx]
        p_LT_y = linkOriginPosition['Left']['Toe'][1, lenIndx]
        p_LT_z = linkOriginPosition['Left']['Toe'][2, lenIndx]
        
        ax.plot([p_LUL_x], [p_LUL_y], [p_LUL_z], 'or')
        ax.plot([p_LLL_x], [p_LLL_y], [p_LLL_z], 'or')
        ax.plot([p_LF_x], [p_LF_y], [p_LF_z], 'or')
        ax.plot([p_LT_x], [p_LT_y], [p_LT_z], 'or')
        
        
        ax.plot([p_LUL_x, p_LLL_x], [p_LUL_y, p_LLL_y], [p_LUL_z, p_LLL_z], 'k')
        ax.plot([p_LLL_x, p_LF_x], [p_LLL_y, p_LF_y], [p_LLL_z, p_LF_z], 'k')
        ax.plot([p_LF_x, p_LT_x], [p_LF_y, p_LT_y], [p_LF_z, p_LT_z], 'k')
        
        plt.draw()
        plt.pause(0.05)
        
    plt.show()
    
    
    # Right leg path visualizer
    fig,ax = plt.subplots(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    fig.suptitle('Right skeleton visualizer', fontsize=16)
    ax.grid(True)
    
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    
    ax.set_ylim(auto=True)
    
    # Skeleton: RightUpperLeg-RightLowerLeg-RightFoot-RightToe
    for lenIndx in range(DATA_LENGTH):
        # Build skeleton
        p_RUL_x = linkOriginPosition['Right']['UpperLeg'][0, lenIndx]
        p_RUL_y = linkOriginPosition['Right']['UpperLeg'][1, lenIndx]
        p_RUL_z = linkOriginPosition['Right']['UpperLeg'][2, lenIndx]
        
        p_RLL_x = linkOriginPosition['Right']['LowerLeg'][0, lenIndx]
        p_RLL_y = linkOriginPosition['Right']['LowerLeg'][1, lenIndx]
        p_RLL_z = linkOriginPosition['Right']['LowerLeg'][2, lenIndx]
        
        p_RF_x = linkOriginPosition['Right']['Foot'][0, lenIndx]
        p_RF_y = linkOriginPosition['Right']['Foot'][1, lenIndx]
        p_RF_z = linkOriginPosition['Right']['Foot'][2, lenIndx]
        
        p_RT_x = linkOriginPosition['Right']['Toe'][0, lenIndx]
        p_RT_y = linkOriginPosition['Right']['Toe'][1, lenIndx]
        p_RT_z = linkOriginPosition['Right']['Toe'][2, lenIndx]
        
        ax.plot([p_RUL_x], [p_RUL_y], [p_RUL_z], 'or')
        ax.plot([p_RLL_x], [p_RLL_y], [p_RLL_z], 'or')
        ax.plot([p_RF_x], [p_RF_y], [p_RF_z], 'or')
        
        ax.plot([p_RUL_x, p_RLL_x], [p_RUL_y, p_RLL_y], [p_RUL_z, p_RLL_z], 'k')
        ax.plot([p_RLL_x, p_RF_x], [p_RLL_y, p_RF_y], [p_RLL_z, p_RF_z], 'k')
        ax.plot([p_RF_x, p_RT_x], [p_RF_y, p_RT_y], [p_RF_z, p_RT_z], 'k')
        
        plt.draw()
        plt.pause(0.05)
    
    plt.show()
        
        
    return 0
        
        