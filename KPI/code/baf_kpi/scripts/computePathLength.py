import numpy as np

def computePathLength(KPI):
    """computePathLength computes the lateral lenght of the path during the swing. It is expressed as a percentage of the stride length. """
    
    pathLength = {'Left': {}, 'Right': {}}
    # Left
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        pathLength['Left'][nrCycleIdx] = {}
        pathLength['Left'][nrCycleIdx]['Cycle'] = nrCycleIdx
        pathLength['Left'][nrCycleIdx]['path'] = 0
        for samplesIdx in range(KPI['swingWidth']['Left'][nrCycleIdx]['positionInSwingRange'].shape[1] - 1): 
            pathLength['Left'][nrCycleIdx]['path'] += np.linalg.norm(KPI['swingWidth']['Left'][nrCycleIdx]['positionInSwingRange'][0:2,samplesIdx+1] - KPI['swingWidth']['Left'][nrCycleIdx]['positionInSwingRange'][0:2,samplesIdx])
        # Percentage of stride length
        pathLength['Left'][nrCycleIdx]['percentageOfStrideLength'] = np.round(100 * pathLength['Left'][nrCycleIdx]['path'] / KPI['strideLength']['Left'][nrCycleIdx]['distance'])
        
    # Right
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
        pathLength['Right'][nrCycleIdx] = {}
        pathLength['Right'][nrCycleIdx]['Cycle'] = nrCycleIdx
        pathLength['Right'][nrCycleIdx]['path'] = 0
        for samplesIdx in range(KPI['swingWidth']['Right'][nrCycleIdx]['positionInSwingRange'].shape[1] - 1):
            pathLength['Right'][nrCycleIdx]['path'] += np.linalg.norm(KPI['swingWidth']['Right'][nrCycleIdx]['positionInSwingRange'][0:2,samplesIdx+1] - KPI['swingWidth']['Right'][nrCycleIdx]['positionInSwingRange'][0:2,samplesIdx])
        # Percentage of stride length
        pathLength['Right'][nrCycleIdx]['percentageOfStrideLength'] = np.round(100 * pathLength['Right'][nrCycleIdx]['path'] / KPI['strideLength']['Right'][nrCycleIdx]['distance'])
    
    return pathLength