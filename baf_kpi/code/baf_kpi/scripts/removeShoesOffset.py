import numpy as np
from baf_kpi.scripts.filterData import filterData


def removeShoesOffset(node1, node2, KPI):
    """REMOVESHOESOFFSET removes the offset of the wrenches measured by the shoes."""
    
    # Offset removal for Fx, Fy, Mx, My, Mz
    node1NoOffset = {'FT': {'data': np.zeros(node1['FT']['data'].shape)}}
    node2NoOffset = {'FT': {'data': np.zeros(node2['FT']['data'].shape)}}
    
    # Find index to remove offset for Fx, Fy, Mx, My, Mz
    indexForLeftMean = KPI['gaitCycleFeatures']['Left']['stride'][0]['idxAtTheBeginning']
    indexForRightMean = KPI['gaitCycleFeatures']['Right']['stride'][0]['idxAtTheBeginning']
    
    # Compute mean of the first valuable samples Left
    meanFirstSamples_fx_left = np.mean(node1['FT']['data'][:indexForLeftMean, 0 , 0])
    meanFirstSamples_fy_left = np.mean(node1['FT']['data'][:indexForLeftMean, 0 , 1])
    meanFirstSamples_mx_left = np.mean(node1['FT']['data'][:indexForLeftMean, 0 , 3])
    meanFirstSamples_my_left = np.mean(node1['FT']['data'][:indexForLeftMean, 0 , 4])
    meanFirstSamples_mz_left = np.mean(node1['FT']['data'][:indexForLeftMean, 0 , 5])
    
    # Compute mean of the first valuable samples Right
    meanFirstSamples_fx_right = np.mean(node2['FT']['data'][:indexForRightMean, 0 , 0])
    meanFirstSamples_fy_right = np.mean(node2['FT']['data'][:indexForRightMean, 0 , 1])
    meanFirstSamples_mx_right = np.mean(node2['FT']['data'][:indexForRightMean, 0 , 3])
    meanFirstSamples_my_right = np.mean(node2['FT']['data'][:indexForRightMean, 0 , 4])
    meanFirstSamples_mz_right = np.mean(node2['FT']['data'][:indexForRightMean, 0 , 5])
    
    # Remove offset Left
    # node1NoOffset['FT']['data'] = np.zeros(node1['FT']['data'].shape)
    node1NoOffset['FT']['data'][:, 0, 0] = node1['FT']['data'][:, 0, 0] - meanFirstSamples_fx_left
    node1NoOffset['FT']['data'][:, 0, 1] = node1['FT']['data'][:, 0, 1] - meanFirstSamples_fy_left
    node1NoOffset['FT']['data'][:, 0, 3] = node1['FT']['data'][:, 0, 3] - meanFirstSamples_mx_left
    node1NoOffset['FT']['data'][:, 0, 4] = node1['FT']['data'][:, 0, 4] - meanFirstSamples_my_left
    node1NoOffset['FT']['data'][:, 0, 5] = node1['FT']['data'][:, 0, 5] - meanFirstSamples_mz_left
    
    # Remove offset Right
    # node2NoOffset['FT']['data'] = np.zeros(node2['FT']['data'].shape)
    node2NoOffset['FT']['data'][:, 0, 0] = node2['FT']['data'][:, 0, 0] - meanFirstSamples_fx_right
    node2NoOffset['FT']['data'][:, 0, 1] = node2['FT']['data'][:, 0, 1] - meanFirstSamples_fy_right
    node2NoOffset['FT']['data'][:, 0, 3] = node2['FT']['data'][:, 0, 3] - meanFirstSamples_mx_right
    node2NoOffset['FT']['data'][:, 0, 4] = node2['FT']['data'][:, 0, 4] - meanFirstSamples_my_right
    node2NoOffset['FT']['data'][:, 0, 5] = node2['FT']['data'][:, 0, 5] - meanFirstSamples_mz_right
    
    # Offset removal for Fz
    # Compute left values for offset removal when Right single support
    meanLeftCollector = []
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Right']['ss'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Right']['ss'][nrCycleIdx]['idxAtTheEnd']
        meanLeftCollector.append(np.nanmean(node1['FT']['data'][initial:final+1, 0, 2]))
        
    meanValueOffsetLeft = np.nanmean(meanLeftCollector)
    
    # Compute right values for offset removal when Left single support
    meanRightCollector = []
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Left']['ss'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Left']['ss'][nrCycleIdx]['idxAtTheEnd']
        meanRightCollector.append(np.mean(node2['FT']['data'][initial:final+1, 0, 2]))
        
    meanValueOffsetRight = np.nanmean(meanRightCollector)
    
    # Remove offset 
    node1NoOffset['FT']['data'][:, 0, 2] = node1['FT']['data'][:, 0, 2] - meanValueOffsetLeft
    node2NoOffset['FT']['data'][:, 0, 2] = node2['FT']['data'][:, 0, 2] - meanValueOffsetRight
    
    return node1NoOffset, node2NoOffset
    