import idyntree.bindings as iDynTree
import numpy as np

def computeCOM(kinDynComp, joints_state, human_state, W_T_base, KPI):
    """ computeCOM computes quantity related to the Center of Mass (COM) expressed wrt the inertial frame. """
    
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])
    
    COM = { 'position': np.zeros((3, KPI['len'])), 'velocity': np.zeros((3, KPI['len'])) }
    
    
    for lenIdx in range(KPI['len']):
        # Set the joint positions
        s = joints_state['positions']['data'][lenIdx][0]
        ds = joints_state['velocities']['data'][lenIdx][0]
        lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
        ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
        base_vel = np.concatenate((lin_vel, ang_vel))        
        
        kinDynComp.setRobotState(W_T_base[lenIdx], s, base_vel, ds, gravity)
        
        # Compute COM position
        COM_position = kinDynComp.getCenterOfMassPosition()
        COM['position'][:,lenIdx] = COM_position.toNumPy()
        # Compute COM velocity
        COM_velocity = kinDynComp.getCenterOfMassVelocity()
        COM['velocity'][:,lenIdx] = COM_velocity.toNumPy()
        
    return COM