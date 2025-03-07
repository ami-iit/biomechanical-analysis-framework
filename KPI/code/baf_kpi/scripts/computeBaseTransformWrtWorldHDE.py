import idyntree.bindings as iDynTree
import numpy as np

from baf_kpi.scripts.quat2mat import quat2mat


def computeBaseTransformWrtWorldHDE(timestamps, human_state):
    """ Compute base transform wrt world computes the iDynTree transform of the base expressed w.r.t. the world frame. """
    
    W_T_base = []
    
    for lenIdx in range(len(timestamps)):
        # Rotation
        quaternion = human_state['base_orientation']['data'][:,lenIdx]
        W_T_B_Rot = quat2mat(quaternion)
        # Position
        W_T_B_Pos = human_state['base_position']['data'][lenIdx,:,:].squeeze()
        
        W_T_B = iDynTree.Transform()
        W_T_B.setRotation(W_T_B_Rot)
        W_T_B.setPosition(W_T_B_Pos)
        
        W_T_base.append(W_T_B)
        
    return W_T_base
        
        
        
        
        