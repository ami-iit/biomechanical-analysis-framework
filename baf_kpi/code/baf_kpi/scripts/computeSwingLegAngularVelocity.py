import numpy as np

def computeSwingLegAngularVelocity(nodes, KPI):
    """computeSwingLegAngularVelocity computes the angular velocity of the gait cycle involved leg, for the swing phase, per each gait cycle."""
    
    # Compute swing leg angular velocity
    # Left
    for node in nodes:
        if node.get('attachedLink') == 'LeftUpperLeg':
            swingLegAngVelocity_left = node['angVel']['data']
            
    # Right
    for node in nodes:
        if node.get('attachedLink') == 'RightUpperLeg':
            swingLegAngVelocity_right = node['angVel']['data']
            
    # Compute angular velocity per single cycle
    swingLegAngVelocity = { 'Left': {}, 'Right': {} }
    # Left
    for nrCycleOfIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Left']['swing'][nrCycleOfIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Left']['swing'][nrCycleOfIdx]['idxAtTheEnd']
        swingLegAngVelocity['Left'][nrCycleOfIdx] = {}
        swingLegAngVelocity['Left'][nrCycleOfIdx]['cycle'] = nrCycleOfIdx
        swingLegAngVelocity['Left'][nrCycleOfIdx]['value'] = swingLegAngVelocity_left[initial:final+1].squeeze()
        swingLegAngVelocity['Left'][nrCycleOfIdx]['norm'] = np.linalg.norm(swingLegAngVelocity['Left'][nrCycleOfIdx]['value'], axis=1)
        
    # Right
    for nrCycleOfIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Right']['swing'][nrCycleOfIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Right']['swing'][nrCycleOfIdx]['idxAtTheEnd']
        swingLegAngVelocity['Right'][nrCycleOfIdx] = {}
        swingLegAngVelocity['Right'][nrCycleOfIdx]['cycle'] = nrCycleOfIdx
        swingLegAngVelocity['Right'][nrCycleOfIdx]['value'] = swingLegAngVelocity_right[initial:final+1].squeeze()
        swingLegAngVelocity['Right'][nrCycleOfIdx]['norm'] = np.linalg.norm(swingLegAngVelocity['Right'][nrCycleOfIdx]['value'], axis=1)
    
    return swingLegAngVelocity