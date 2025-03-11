import numpy as np

def computeStanceAndSwingPercentage(KPI):
    """computeStanceAndSwingPercentage computes the percentage of time for the stance and swing phase wrt the time of each gait cycle."""
    
    phasePercentageWrtCycleDuration = {'Left': {}, 'Right': {}}
    # Compute left stance and swing percentage
    for nrCycleIdx in range (KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        phasePercentageWrtCycleDuration['Left'][nrCycleIdx] = {}
        phasePercentageWrtCycleDuration['Left'][nrCycleIdx]['cycle'] = nrCycleIdx
        # Stance
        phasePercentageWrtCycleDuration['Left'][nrCycleIdx]['percentageOfStance'] = np.round(100 * KPI['gaitCycleFeatures']['Left']['stance'][nrCycleIdx]['timeStance'] / KPI['gaitCycleFeatures']['Left']['stride'][nrCycleIdx]['timeStride']).item()
        # Swing
        phasePercentageWrtCycleDuration['Left'][nrCycleIdx]['percentageOfSwing'] = np.round(100 * KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['timeSwing'] / KPI['gaitCycleFeatures']['Left']['stride'][nrCycleIdx]['timeStride']).item()
    
    # Compute right stance and swing percentage
    for nrCycleIdx in range (KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
        phasePercentageWrtCycleDuration['Right'][nrCycleIdx] = {}
        phasePercentageWrtCycleDuration['Right'][nrCycleIdx]['cycle'] = nrCycleIdx
        # Stance
        phasePercentageWrtCycleDuration['Right'][nrCycleIdx]['percentageOfStance'] = np.round(100 * KPI['gaitCycleFeatures']['Right']['stance'][nrCycleIdx]['timeStance'] / KPI['gaitCycleFeatures']['Right']['stride'][nrCycleIdx]['timeStride']).item()
        # Swing
        phasePercentageWrtCycleDuration['Right'][nrCycleIdx]['percentageOfSwing'] = np.round(100 * KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['timeSwing'] / KPI['gaitCycleFeatures']['Right']['stride'][nrCycleIdx]['timeStride']).item()
        
    return phasePercentageWrtCycleDuration