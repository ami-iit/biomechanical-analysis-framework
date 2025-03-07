import numpy as np
from scipy.signal import find_peaks


def adaptiveDetectContactPattern(node1, node2, KPI, timestamps):
    # Contact pattern detection identifies the main gait events, HS (heel-strike) and TO (toe-off), from the vertical component of the ground reaction force using a double-threshold algorithm.
    

    
    threshold1 = KPI['thresholds']['percentageMassTH'] # first threshold (% of maximum force) to define the temporal instants of HS and TO events
    threshold2_percent = 0.5
    window_size = 5
    
    # Vertical force data 
    left_fz = node1['FT']['data'][:, 0, 2]
    # print("shape left_z", left_z.shape)
    right_fz = node2['FT']['data'][:, 0, 2]
    
    # Forces
    forces = {}
    forces['GTEventLeft1t'] = np.zeros((left_fz.shape[0], 1))
    forces['GTEventRight1t'] = np.zeros((right_fz.shape[0], 1))
    forces['GTEventLeft2t'] = np.zeros((left_fz.shape[0], 1))
    forces['GTEventRight2t'] = np.zeros((right_fz.shape[0], 1))
    
    forces['upperthreshold_Left1t'] = np.where(left_fz >= (np.max(left_fz) * threshold1))[0]
    forces['upperthreshold_Right1t'] = np.where(right_fz >= (np.max(right_fz) * threshold1))[0]
    
    forces['GTEventLeft1t'][forces['upperthreshold_Left1t']] = np.max(left_fz)
    forces['GTEventRight1t'][forces['upperthreshold_Right1t']] = np.max(right_fz)
    
    dynamic_threshold_left = []
    dynamic_threshold_right = []
    
    filtered_left_fz = left_fz[forces['upperthreshold_Left1t']]
    filtered_right_fz = right_fz[forces['upperthreshold_Right1t']]
    
    for i in range(0, len(filtered_left_fz) - window_size + 1, window_size):
        window_left = filtered_left_fz[i:i+window_size]
        
        max_left = np.max(window_left)
        
        dynamic_threshold_left.append(max_left)
        
        
    for i in range(0, len(filtered_right_fz) - window_size + 1, window_size):
        window_right = filtered_right_fz[i:i+window_size]
        
        max_right = np.max(window_right)
        dynamic_threshold_right.append(max_right)
        
    dynamic_threshold_left = np.array(dynamic_threshold_left)
    dynamic_threshold_right = np.array(dynamic_threshold_right)
    
    # Mean of the max values
    dynamic_threshold_left_mean = np.mean(dynamic_threshold_left) * threshold2_percent
    dynamic_threshold_right_mean = np.mean(dynamic_threshold_right) * threshold2_percent
    
    forces['upperthreshold_Left2t'] = np.where(left_fz[window_size:] >= dynamic_threshold_left_mean)[0]
    forces['upperthreshold_Right2t'] = np.where(right_fz[window_size:] >= dynamic_threshold_right_mean)[0]
    
    forces['GTEventLeft2t'][forces['upperthreshold_Left2t']] = np.max(left_fz)
    forces['GTEventRight2t'][forces['upperthreshold_Right2t']] = np.max(right_fz)
    
    
    forces['dGTEventLeft1t'] = computeGradient(forces['GTEventLeft1t'])
    forces['dGTEventRight1t'] = computeGradient(forces['GTEventRight1t'])
    
    forces['dGTEventLeft2t'] = computeGradient(forces['GTEventLeft2t'])
    forces['dGTEventRight2t'] = computeGradient(forces['GTEventRight2t'])
    
    forces['dGTEventLeft1t'] = checkSteps(forces['dGTEventLeft1t'], forces['dGTEventLeft2t'], 'HS')
    forces['dGTEventLeft1t'] = checkSteps(forces['dGTEventLeft1t'], forces['dGTEventLeft2t'], 'TO')
    
    forces['dGTEventRight1t'] = checkSteps(forces['dGTEventRight1t'], forces['dGTEventRight2t'], 'HS')
    forces['dGTEventRight1t'] = checkSteps(forces['dGTEventRight1t'], forces['dGTEventRight2t'], 'TO')
    
    # Gait Events
    events = { 'Left': {}, 'Right': {} }
    
    # Left
    events['Left']['HS'] = {}
    events['Left']['TO'] = {}
    events['Left']['HS']['idx'] = find_peaks(forces['dGTEventLeft1t'])[0]
    events['Left']['TO']['idx'] = find_peaks(-forces['dGTEventLeft1t'])[0]
    events['Left']['HS']['timestamps'] = timestamps[events['Left']['HS']['idx']]
    events['Left']['TO']['timestamps'] = timestamps[events['Left']['TO']['idx']]
    events['Left']['HS']['time'] = events['Left']['HS']['timestamps'] - timestamps[0]
    events['Left']['TO']['time'] = events['Left']['TO']['timestamps'] - timestamps[0]
    
    # Right
    events['Right']['HS'] = {}
    events['Right']['TO'] = {}
    events['Right']['HS']['idx'] = find_peaks(forces['dGTEventRight1t'])[0]
    events['Right']['TO']['idx'] = find_peaks(-forces['dGTEventRight1t'])[0]
    events['Right']['HS']['timestamps'] = timestamps[events['Right']['HS']['idx']]
    events['Right']['TO']['timestamps'] = timestamps[events['Right']['TO']['idx']]
    events['Right']['HS']['time'] = events['Right']['HS']['timestamps'] - timestamps[0]
    events['Right']['TO']['time'] = events['Right']['TO']['timestamps'] - timestamps[0] 
   
    
    return forces, events





def checkSteps(events, check_events, event_type):
    # Check the real shifting of the body mass
    
    events = np.squeeze(events)
    check_events = np.squeeze(check_events)
    events_correct = events.copy()
    
    if event_type == 'HS':
        local_max_events = np.where(np.r_[False, events[1:] > events[:-1]] & np.r_[events[:-1] > events[1:], False])[0]
        local_max_check = np.where(np.r_[False, check_events[1:] > check_events[:-1]] & np.r_[check_events[:-1] > check_events[1:], False])[0]
        
        i = 0
        while i < len(local_max_events) - 1:
            control = local_max_check[(local_max_check > local_max_events[i]) & (local_max_check < local_max_events[i+1])]
            
            if len(control) == 0:
                events_correct[local_max_events[i]] = 0
                local_max_events = np.delete(local_max_events, i)
                i -= 1
            i += 1
    
    elif event_type == 'TO':  # Toe Off events
        
        local_min_events = np.where(np.r_[False, events[1:] < events[:-1]] & np.r_[events[:-1] < events[1:], False])[0]
        local_min_check = np.where(np.r_[False, check_events[1:] < check_events[:-1]] & np.r_[check_events[:-1] < check_events[1:], False])[0]
        
        # Check if local_min_events is not empty
        if len(local_min_events) > 0:
            control = local_min_check[local_min_check < local_min_events[0]]
            if len(control) == 0:
                events_correct[local_min_events[0]] = 0
                local_min_events = np.delete(local_min_events, 0)

    return events_correct



def computeGradient(x):
    
    grad = np.zeros_like(x)

    grad[1:-1] = (x[2:] - x[:-2]) / 2  # Central difference
    
    grad[0] = x[1] - x[0]  # Forward difference
    
    grad[-1] = x[-1] - x[-2]  # Backward difference
    
    return grad