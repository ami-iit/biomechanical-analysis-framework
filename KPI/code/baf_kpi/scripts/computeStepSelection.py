import numpy as np

def computeStepSelection(KPI):
    # Compute step selection computes the step phases from the vertical component of the ground reaction force.  In particular, the function  identifies the steady-state and the related HS (heel-strike) and TO (toe-off) events, excluding gait initiation and gait termination.

    # Initialization
    steadyStateEvents = {
            'legStart': None,
            'legEnd': None,
            'Left': {'TO': {}, 'HS': {}},
            'Right': {'TO': {}, 'HS': {}}
        }

    # Check false movement: False movements are removed on the basis of a time threshold three times the average temporal distance between events. Note: Events above this threshold are not considered.
    InitLeft = 0
    InitRight = 0
    
    if (np.max(np.diff(KPI['events']['Right']['HS']['idx'])) >= 3 * np.mean(np.diff(KPI['events']['Right']['HS']['idx'])) or
    np.max(np.diff(KPI['events']['Left']['HS']['idx'])) >= 3 * np.mean(np.diff(KPI['events']['Left']['HS']['idx']))):
    
        if np.max(np.diff(KPI['events']['Left']['HS']['idx'])) >= 3 * np.mean(np.diff(KPI['events']['Left']['HS']['idx'])):
            InitLeft = np.argmax(np.diff(KPI['events']['Left']['HS']['idx'])) + 1
        
        if np.max(np.diff(KPI['events']['Right']['HS']['idx'])) >= 3 * np.mean(np.diff(KPI['events']['Right']['HS']['idx'])):
            InitRight = np.argmax(np.diff(KPI['events']['Right']['HS']['idx'])) + 1
                
    # Find initial step
    firstTO = np.argmin([KPI['events']['Left']['TO']['idx'][InitLeft], KPI['events']['Right']['TO']['idx'][InitRight]])

    if firstTO == 0:  # Left
        steadyStateEvents['legStart'] = 'Left'
        print('[INFO] The gait starts with left leg.')
        LowerCut = KPI['events']['Right']['HS']['idx'][InitRight]

        # Extract data based on LowerCut
        for side in ['Left', 'Right']:
            steadyStateEvents[side]['TO']['idx'] = KPI['events'][side]['TO']['idx'][KPI['events'][side]['TO']['idx'] >= LowerCut]
            steadyStateEvents[side]['HS']['idx'] = KPI['events'][side]['HS']['idx'][KPI['events'][side]['HS']['idx'] >= LowerCut]
            steadyStateEvents[side]['TO']['timestamps'] = KPI['events'][side]['TO']['timestamps'][KPI['events'][side]['TO']['idx'] >= LowerCut]
            steadyStateEvents[side]['HS']['timestamps'] = KPI['events'][side]['HS']['timestamps'][KPI['events'][side]['HS']['idx'] >= LowerCut]
            steadyStateEvents[side]['TO']['time'] = KPI['events'][side]['TO']['time'][KPI['events'][side]['TO']['idx'] >= LowerCut]
            steadyStateEvents[side]['HS']['time'] = KPI['events'][side]['HS']['time'][KPI['events'][side]['HS']['idx'] >= LowerCut]

    else:  # Right
        steadyStateEvents['legStart'] = 'Right'
        print('[INFO] The gait starts with right leg.')
        LowerCut = KPI['events']['Left']['HS']['idx'][InitLeft]
        
        for side in ['Right', 'Left']:
            
            steadyStateEvents[side]['TO']['idx'] = KPI['events'][side]['TO']['idx'][KPI['events'][side]['TO']['idx'] >= LowerCut]
            steadyStateEvents[side]['HS']['idx'] = KPI['events'][side]['HS']['idx'][KPI['events'][side]['HS']['idx'] >= LowerCut]
            steadyStateEvents[side]['TO']['timestamps'] = KPI['events'][side]['TO']['timestamps'][KPI['events'][side]['TO']['idx'] >= LowerCut]
            steadyStateEvents[side]['HS']['timestamps'] = KPI['events'][side]['HS']['timestamps'][KPI['events'][side]['HS']['idx'] >= LowerCut]
            steadyStateEvents[side]['TO']['time'] = KPI['events'][side]['TO']['time'][KPI['events'][side]['TO']['idx'] >= LowerCut]
            steadyStateEvents[side]['HS']['time'] = KPI['events'][side]['HS']['time'][KPI['events'][side]['HS']['idx'] >= LowerCut]
            
    # Find final step
    lastHS = np.argmax([KPI['events']['Left']['HS']['idx'][-1], KPI['events']['Right']['HS']['idx'][-1]])
    
    if lastHS == 0:  # Left
        steadyStateEvents['legEnd'] = 'Left'
        print('[INFO] The gait ends with left leg.')
        UpperCut = KPI['events']['Right']['HS']['idx'][-1]
        
        for side in ['Left', 'Right']:
            
            valid_mask_TO = steadyStateEvents[side]['TO']['idx'] <= UpperCut
            valid_mask_HS = steadyStateEvents[side]['HS']['idx'] <= UpperCut
            
            steadyStateEvents[side]['TO']['idx'] = steadyStateEvents[side]['TO']['idx'][valid_mask_TO]
            steadyStateEvents[side]['HS']['idx'] = steadyStateEvents[side]['HS']['idx'][valid_mask_HS]
            steadyStateEvents[side]['TO']['timestamps'] = steadyStateEvents[side]['TO']['timestamps'][valid_mask_TO]
            steadyStateEvents[side]['HS']['timestamps'] = steadyStateEvents[side]['HS']['timestamps'][valid_mask_HS]
            steadyStateEvents[side]['TO']['time'] = steadyStateEvents[side]['TO']['time'][valid_mask_TO]
            steadyStateEvents[side]['HS']['time'] = steadyStateEvents[side]['HS']['time'][valid_mask_HS]
    
    else:  # Right
        steadyStateEvents['legEnd'] = 'Right'
        print('[INFO] The gait ends with right leg.')
        UpperCut = KPI['events']['Left']['HS']['idx'][-1]
        
        for side in ['Right', 'Left']:
            
            valid_mask_TO = steadyStateEvents[side]['TO']['idx'] <= UpperCut
            valid_mask_HS = steadyStateEvents[side]['HS']['idx'] <= UpperCut
            
            steadyStateEvents[side]['TO']['idx'] = steadyStateEvents[side]['TO']['idx'][valid_mask_TO]
            steadyStateEvents[side]['HS']['idx'] = steadyStateEvents[side]['HS']['idx'][valid_mask_HS]
            steadyStateEvents[side]['TO']['timestamps'] = steadyStateEvents[side]['TO']['timestamps'][valid_mask_TO]
            steadyStateEvents[side]['HS']['timestamps'] = steadyStateEvents[side]['HS']['timestamps'][valid_mask_HS]
            steadyStateEvents[side]['TO']['time'] = steadyStateEvents[side]['TO']['time'][valid_mask_TO]
            steadyStateEvents[side]['HS']['time'] = steadyStateEvents[side]['HS']['time'][valid_mask_HS]
            
            
    return steadyStateEvents
