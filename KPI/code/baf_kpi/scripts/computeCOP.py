import numpy as np
import idyntree.bindings as iDynTree

from baf_kpi.scripts.filterData import filterData

def computeCOP(kinDynComp, joints_state, human_state, W_T_base, node1, node2, KPI):
    """ ComputeCOP computes the Center of Pressure (COP) expressed wrt the world frame. """
    
    
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])
    
    # Compute COP wrt world
    COP = { 'total': { 'Right': {}, 'Left': {} } }
    # Define COP vector wrt Left foot
    Fz_left = node1['FT']['data'][:,:,2] - min(node1['FT']['data'][:,:,2])
    Fz_left = np.where(Fz_left == 0, 1e-10, Fz_left)
    Mx_left = node1['FT']['data'][:,:,3]
    My_left = node1['FT']['data'][:,:,4]
    COPLeft = np.zeros((4, KPI['len']))
    COPLeft[0,:] = (-My_left / Fz_left).squeeze()
    COPLeft[1,:] = (Mx_left / Fz_left).squeeze()
    COPLeft[2,:] = np.zeros(KPI['len'])
    COPLeft[3,:] = np.ones(KPI['len'])
    COP['total']['Left']['wrtFoot'] = COPLeft
    
    # Define COP vector wrt Right foot
    Fz_right = node2['FT']['data'][:,:,2] - min(node2['FT']['data'][:,:,2])
    Fz_right = np.where(Fz_right == 0, 1e-10, Fz_right)
    Mx_right = node2['FT']['data'][:,:,3]
    My_right = node2['FT']['data'][:,:,4]
    COPRight = np.zeros((4, KPI['len']))
    COPRight[0,:] = (-My_right / Fz_right).squeeze()
    COPRight[1,:] = (Mx_right / Fz_right).squeeze()
    COPRight[2,:] = np.zeros(KPI['len'])
    COPRight[3,:] = np.ones(KPI['len'])
    COP['total']['Right']['wrtFoot'] = COPRight
    
    # Compute COP wrt world per single cycle
    # Left
    COP['total']['Left']['wrtWorld'] = np.zeros((4, KPI['len']))
    COP['Left'] = {}
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        counter = 0
        initial = KPI['gaitCycleFeatures']['Left']['ss'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Left']['ss'][nrCycleIdx]['idxAtTheEnd']
        
        s = joints_state['positions']['data'][initial][0]
        ds = joints_state['velocities']['data'][initial][0]
        lin_vel = human_state['base_linear_velocity']['data'][initial][0]
        ang_vel = human_state['base_angular_velocity']['data'][initial][0]
        base_vel = np.concatenate((lin_vel, ang_vel)) 
        
        kinDynComp.setRobotState(W_T_base[initial], s, base_vel, ds, gravity)
        W_H_LF = kinDynComp.getWorldTransform('LeftFoot').asHomogeneousTransform().toNumPy()
        
        
        COP['Left'][nrCycleIdx] = {}
        COP['Left'][nrCycleIdx]['Cycle'] = nrCycleIdx
        COP['Left'][nrCycleIdx]['wrtFoot'] = np.zeros((4, final - initial))
        COP['Left'][nrCycleIdx]['wrtWorld'] = np.zeros((4, final - initial))
        for indx in range(initial, final):
            COP['total']['Left']['wrtWorld'][:,indx] = np.dot(W_H_LF, COPLeft[:,indx])
            COP['Left'][nrCycleIdx]['wrtFoot'][:,counter] = COPLeft[:,indx]
            COP['Left'][nrCycleIdx]['wrtWorld'][:,counter] = np.dot(W_H_LF, COPLeft[:,indx])
            counter += 1
    
        
        
        # Right
        COP['total']['Right']['wrtWorld'] = np.zeros((4, KPI['len']))
        COP['Right'] = {}
        for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
            counter = 0
            initial = KPI['gaitCycleFeatures']['Right']['ss'][nrCycleIdx]['idxAtTheBeginning']
            final = KPI['gaitCycleFeatures']['Right']['ss'][nrCycleIdx]['idxAtTheEnd']
            
            if initial > final:
                print("nrCycleIdx: ", nrCycleIdx)
            
            s = joints_state['positions']['data'][initial][0]
            ds = joints_state['velocities']['data'][initial][0]
            lin_vel = human_state['base_linear_velocity']['data'][initial][0]
            ang_vel = human_state['base_angular_velocity']['data'][initial][0]
            base_vel = np.concatenate((lin_vel, ang_vel)) 
            kinDynComp.setRobotState(W_T_base[initial], s, base_vel, ds, gravity)
            W_H_RF = kinDynComp.getWorldTransform('RightFoot').asHomogeneousTransform().toNumPy()
            
            COP['Right'][nrCycleIdx] = {}
            COP['Right'][nrCycleIdx]['Cycle'] = nrCycleIdx
            COP['Right'][nrCycleIdx]['wrtFoot'] = np.zeros((4, final - initial))
            COP['Right'][nrCycleIdx]['wrtWorld'] = np.zeros((4, final - initial))
            for indx in range(initial, final):
                COP['total']['Right']['wrtWorld'][:,indx] = np.dot(W_H_RF, COPRight[:,indx])
                COP['Right'][nrCycleIdx]['wrtFoot'][:,counter] = COPRight[:,indx]
                COP['Right'][nrCycleIdx]['wrtWorld'][:,counter] = np.dot(W_H_RF, COPRight[:,indx])
                counter += 1

    return COP
    
    