import numpy as np
import idyntree.bindings as iDynTree

def computeStrideLength(kinDynComp, joints_state, human_state, W_T_base, KPI):
    """Compute stride length computes distance in meters of a gait cycle for both the feet. """
    
    # Compute the feet position in 
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])

    
    linkPosition_leftFoot = np.zeros((3, KPI['len']))
    linkPosition_rightFoot = np.zeros((3, KPI['len']))
    
    for lenIdx in range(KPI['len']):
        # Set the joint positions
        s = joints_state['positions']['data'][lenIdx][0]
        ds = joints_state['velocities']['data'][lenIdx][0]
        lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
        ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
        base_vel = np.concatenate((lin_vel, ang_vel)) 
        
        kinDynComp.setRobotState(W_T_base[lenIdx], s, base_vel, ds, gravity)
        
        # Left foot position wrt world
        W_T_link_LF = kinDynComp.getWorldTransform('LeftFoot')
        W_H_link_LF = W_T_link_LF.asHomogeneousTransform()
        linkPosition_leftFoot[:, lenIdx] = W_T_link_LF.getPosition().toNumPy()
        # Right foot position wrt world
        W_T_link_RF = kinDynComp.getWorldTransform('RightFoot')
        W_H_link_RF = W_T_link_RF.asHomogeneousTransform()
        linkPosition_rightFoot[:, lenIdx] = W_T_link_RF.getPosition().toNumPy()
    
    # Compute stride length
    strideLength = { 'Left': {}, 'Right': {} }
    # Left
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Left']['stride'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Left']['stride'][nrCycleIdx]['idxAtTheEnd']
        strideLength['Left'][nrCycleIdx] = {}
        strideLength['Left'][nrCycleIdx]['cycle'] = nrCycleIdx
        strideLength['Left'][nrCycleIdx]['distance'] = np.linalg.norm(linkPosition_leftFoot[:, final] - linkPosition_leftFoot[:, initial])
        
    # Right
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Right']['stride'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Right']['stride'][nrCycleIdx]['idxAtTheEnd']
        strideLength['Right'][nrCycleIdx] = {}
        strideLength['Right'][nrCycleIdx]['cycle'] = nrCycleIdx
        strideLength['Right'][nrCycleIdx]['distance'] = np.linalg.norm(linkPosition_rightFoot[:, final] - linkPosition_rightFoot[:, initial])
    
    
    return strideLength