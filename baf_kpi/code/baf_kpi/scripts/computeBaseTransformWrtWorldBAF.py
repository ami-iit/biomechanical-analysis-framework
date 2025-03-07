import idyntree.bindings as iDynTree
import numpy as np


def computeBaseTransformWrtWorldBAF(human_state):
    """ Compute base transform wrt world computes the iDynTree transform of the base expressed w.r.t. the world frame. """
    
    timestamps = human_state['base_position']['timestamps']
    W_T_base = []
    
    for lenIdx in range(len(timestamps)):
        # Rotation
        W_T_B_Rot = iDynTree.Rotation.RPY(human_state['base_orientation']['data'][lenIdx][0][0], human_state['base_orientation']['data'][lenIdx][0][1], human_state['base_orientation']['data'][lenIdx][0][2])
        # Position
        W_T_B_Pos = iDynTree.Position(human_state['base_position']['data'][lenIdx][0][0], human_state['base_position']['data'][lenIdx][0][1], human_state['base_position']['data'][lenIdx][0][2])
        # Transform
        W_T_base.append(iDynTree.Transform(W_T_B_Rot, W_T_B_Pos))
        
    return W_T_base
        
        
        
        
        