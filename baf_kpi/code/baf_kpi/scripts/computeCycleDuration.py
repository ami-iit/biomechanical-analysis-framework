import numpy as np

def computeCycleDuration(KPI):
    """
    Compute cycle duration computes the phases and sub-phases of the gait cycle from heel-strike and toe-off. 
    
    Legend:
    HS (heel-strike) : foot strike on the ground
    TO (toe-off)     : foot off the ground
    ds1              : first double support phase
    ds2              : second double support phase
    ss               : single support phase"""
    
    gaitCycleFeatures = {'Left': {}, 'Right': {}}
    
    
    # Left
    gaitCycleFeatures['Left']['stride'] = []
    for len_idx in range(len(KPI['steadyStateEvents']['Left']['HS']['timestamps']) - 1):
        # Stride
        stride = {
            'cycle': len_idx,
            'timeStride': KPI['steadyStateEvents']['Left']['HS']['timestamps'][len_idx + 1] - KPI['steadyStateEvents']['Left']['HS']['timestamps'][len_idx],
            'timeAtTheBeginning': KPI['steadyStateEvents']['Left']['HS']['timestamps'][len_idx],
            'timeAtTheEnd': KPI['steadyStateEvents']['Left']['HS']['timestamps'][len_idx + 1],
            'idxAtTheBeginning': KPI['steadyStateEvents']['Left']['HS']['idx'][len_idx],
            'idxAtTheEnd': KPI['steadyStateEvents']['Left']['HS']['idx'][len_idx + 1]
        }
    
        
        
        stance, swing, step, ds1, ds2, ss = findEvents(
            KPI['steadyStateEvents']['Left']['HS'],
            KPI['steadyStateEvents']['Left']['TO'],
            KPI['steadyStateEvents']['Right']['HS'],
            KPI['steadyStateEvents']['Right']['TO'],
            stride['timeStride'], len_idx
        )
        gaitCycleFeatures['Left'].setdefault('stance', []).append(stance)
        gaitCycleFeatures['Left'].setdefault('swing', []).append(swing)
        gaitCycleFeatures['Left'].setdefault('step', []).append(step)
        gaitCycleFeatures['Left'].setdefault('ds1', []).append(ds1)
        gaitCycleFeatures['Left'].setdefault('ds2', []).append(ds2)
        gaitCycleFeatures['Left'].setdefault('ss', []).append(ss)
        gaitCycleFeatures['Left']['stride'].append(stride)
    
    gaitCycleFeatures['Left']['nrOfCycles'] = len(gaitCycleFeatures['Left']['stance'])

    # Right side
    gaitCycleFeatures['Right']['stride'] = []
    for len_idx in range(len(KPI['steadyStateEvents']['Right']['HS']['timestamps']) - 1):
        stride = {
            'cycle': len_idx,
            'timeStride': KPI['steadyStateEvents']['Right']['HS']['timestamps'][len_idx + 1] - KPI['steadyStateEvents']['Right']['HS']['timestamps'][len_idx],
            'timeAtTheBeginning': KPI['steadyStateEvents']['Right']['HS']['timestamps'][len_idx],
            'timeAtTheEnd': KPI['steadyStateEvents']['Right']['HS']['timestamps'][len_idx + 1],
            'idxAtTheBeginning': KPI['steadyStateEvents']['Right']['HS']['idx'][len_idx],
            'idxAtTheEnd': KPI['steadyStateEvents']['Right']['HS']['idx'][len_idx + 1]
        }
        stance, swing, step, ds1, ds2, ss = findEvents(
            KPI['steadyStateEvents']['Right']['HS'],
            KPI['steadyStateEvents']['Right']['TO'],
            KPI['steadyStateEvents']['Left']['HS'],
            KPI['steadyStateEvents']['Left']['TO'],
            stride['timeStride'], len_idx
        )
        gaitCycleFeatures['Right'].setdefault('stance', []).append(stance)
        gaitCycleFeatures['Right'].setdefault('swing', []).append(swing)
        gaitCycleFeatures['Right'].setdefault('step', []).append(step)
        gaitCycleFeatures['Right'].setdefault('ds1', []).append(ds1)
        gaitCycleFeatures['Right'].setdefault('ds2', []).append(ds2)
        gaitCycleFeatures['Right'].setdefault('ss', []).append(ss)
        gaitCycleFeatures['Right']['stride'].append(stride)
    
    gaitCycleFeatures['Right']['nrOfCycles'] = len(gaitCycleFeatures['Right']['stance'])

    return gaitCycleFeatures


def findEvents(Hsleg, Toleg, HsLegop, ToLegop, timeStride, i):
    """
    Identifies step phases and sub-phases from given HS and TO events.
    """

    # Leg starting the gait
    HS1_leg_t = Hsleg['timestamps'][i]
    HS1_leg_i = Hsleg['idx'][i]
    HS2_leg_t = Hsleg['timestamps'][np.argmax(Hsleg['timestamps'] > HS1_leg_t)]
    HS2_leg_i = Hsleg['idx'][np.argmax(Hsleg['timestamps'] > HS1_leg_t)]
    TO_leg_t = Toleg['timestamps'][np.argmax((Toleg['timestamps'] > HS1_leg_t) & (Toleg['timestamps'] < HS2_leg_t))]
    TO_leg_i = Toleg['idx'][np.argmax((Toleg['timestamps'] > HS1_leg_t) & (Toleg['timestamps'] < HS2_leg_t))]

    # Leg opposite to the one starting the gait
    HS_legop_t = HsLegop['timestamps'][np.argmax((HsLegop['timestamps'] > HS1_leg_t))]
    HS_legop_i = HsLegop['idx'][np.argmax((HsLegop['timestamps'] > HS1_leg_t))]
    TO_legop_t = ToLegop['timestamps'][np.argmax((ToLegop['timestamps'] > HS1_leg_t))]
    TO_legop_i = ToLegop['idx'][np.argmax((ToLegop['timestamps'] > HS1_leg_t))]
    
    stance = {
        'timeStance': TO_leg_t - HS1_leg_t,
        'percStance': np.round((TO_leg_t - HS1_leg_t) / timeStride * 100).item(),
        'timeAtTheBeginning': HS1_leg_t,
        'timeAtTheEnd': TO_leg_t,
        'idxAtTheBeginning': HS1_leg_i,
        'idxAtTheEnd': TO_leg_i
    }
    swing = {
        'timeSwing': HS2_leg_t - TO_leg_t,
        'percSwing': np.round((HS2_leg_t - TO_leg_t) / timeStride * 100).item(),
        'timeAtTheBeginning': TO_leg_t,
        'timeAtTheEnd': HS2_leg_t,
        'idxAtTheBeginning': TO_leg_i,
        'idxAtTheEnd': HS2_leg_i
    }
    step = {
        'timeStep': HS_legop_t - HS1_leg_t,
        'percStep': np.round((HS_legop_t - HS1_leg_t) / timeStride * 100).item(),
        'timeAtTheBeginning': HS1_leg_t,
        'timeAtTheEnd': HS_legop_t,
        'idxAtTheBeginning': HS1_leg_i,
        'idxAtTheEnd': HS_legop_i
    }
    ds1 = {
        'timeds1': TO_legop_t - HS1_leg_t,
        'percds1': np.round((TO_legop_t - HS1_leg_t) / timeStride * 100).item(),
        'timeAtTheBeginning': HS1_leg_t,
        'timeAtTheEnd': TO_legop_t,
        'idxAtTheBeginning': HS1_leg_i,
        'idxAtTheEnd': TO_legop_i
    }
    ds2 = {
        'timeds2': TO_leg_t - HS_legop_t,
        'percds2': np.round((TO_leg_t - HS_legop_t) / timeStride * 100).item(),
        'timeAtTheBeginning': HS_legop_t,
        'timeAtTheEnd': TO_leg_t,
        'idxAtTheBeginning': HS_legop_i,
        'idxAtTheEnd': TO_leg_i
    }
    ss = {
        'timess': HS_legop_t - TO_legop_t,
        'percss': np.round((HS_legop_t - TO_legop_t) / timeStride * 100).item(),
        'timeAtTheBeginning': TO_legop_t,
        'timeAtTheEnd': HS_legop_t,
        'idxAtTheBeginning': TO_legop_i,
        'idxAtTheEnd': HS_legop_i
    }

    return stance, swing, step, ds1, ds2, ss