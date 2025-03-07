import numpy as np
from scipy.signal import butter,filtfilt, lfilter

def filterData(node1,node2,filterIntensity):
    # Filter data
    baseCutFrequency = 5
    cutFrequency = baseCutFrequency * filterIntensity
    
    filterOrder = 3
    resamplingFrequency = 65
    
    B,A = butter(filterOrder, cutFrequency/(resamplingFrequency/2))
    
    node1Filt = {'FT': {'data': np.zeros(node1['FT']['data'].shape)}}
    node2Filt = {'FT': {'data': np.zeros(node2['FT']['data'].shape)}}
    
    node1Filt['FT']['data'] = filtfilt(B,A,node1['FT']['data'], axis=0)
    node2Filt['FT']['data'] = filtfilt(B,A,node2['FT']['data'], axis=0)
    
    return node1Filt,node2Filt