import numpy as np
import pandas as pd

def getKPIreferenceParameters(soa, EDSS):
    EDSS = np.unique(EDSS)
    
    kpi_params = {}

    for EDSS_ID in EDSS:
        tmpEDSS = soa[f'EDSS{EDSS_ID}']  
        
        cadence_mean = []
        strideTime_mean = []
        velocity_mean = []
        strideLength_mean = []
        stancePercentage_mean = []
        swingPercentage_mean = []
        cadence_std = []
        strideTime_std = []
        velocity_std = []
        strideLength_std = []
        stancePercentage_std = []
        swingPercentage_std = []

        for idx in range(len(tmpEDSS['EDSS'])):
            cadence_mean.append(tmpEDSS['cadence'][idx][0])  
            strideTime_mean.append(tmpEDSS['strideTime'][idx][0])
            velocity_mean.append(tmpEDSS['velocity'][idx][0])
            strideLength_mean.append(tmpEDSS['strideLength'][idx][0])
            stancePercentage_mean.append(tmpEDSS['stancePercentage'][idx][0])
            swingPercentage_mean.append(tmpEDSS['swingPercentage'][idx][0])
            cadence_std.append(tmpEDSS['cadence'][idx][1])  
            strideTime_std.append(tmpEDSS['strideTime'][idx][1])
            velocity_std.append(tmpEDSS['velocity'][idx][1])
            strideLength_std.append(tmpEDSS['strideLength'][idx][1])
            stancePercentage_std.append(tmpEDSS['stancePercentage'][idx][1])
            swingPercentage_std.append(tmpEDSS['swingPercentage'][idx][1])

        cadence_mean = [float(x) if x != 'nan' else np.nan for x in cadence_mean]  
        cadence_mean = [x for x in cadence_mean if not np.isnan(x)]
        
        strideTime_mean = [float(x) if x != 'nan' else np.nan for x in strideTime_mean]
        strideTime_mean = [x for x in strideTime_mean if not np.isnan(x)]

        velocity_mean = [float(x) if x != 'nan' else np.nan for x in velocity_mean]
        velocity_mean = [x for x in velocity_mean if not np.isnan(x)]

        strideLength_mean = [float(x) if x != 'nan' else np.nan for x in strideLength_mean]
        strideLength_mean = [x for x in strideLength_mean if not np.isnan(x)]

        stancePercentage_mean = [float(x) if x != 'nan' else np.nan for x in stancePercentage_mean]
        stancePercentage_mean = [x for x in stancePercentage_mean if not np.isnan(x)]

        swingPercentage_mean = [float(x) if x != 'nan' else np.nan for x in swingPercentage_mean]
        swingPercentage_mean = [x for x in swingPercentage_mean if not np.isnan(x)]

        cadence_std = [float(x) if x != 'nan' else np.nan for x in cadence_std]
        cadence_std = [x for x in cadence_std if not np.isnan(x)]

        strideTime_std = [float(x) if x != 'nan' else np.nan for x in strideTime_std]
        strideTime_std = [x for x in strideTime_std if not np.isnan(x)]

        velocity_std = [float(x) if x != 'nan' else np.nan for x in velocity_std]
        velocity_std = [x for x in velocity_std if not np.isnan(x)]

        strideLength_std = [float(x) if x != 'nan' else np.nan for x in strideLength_std]
        strideLength_std = [x for x in strideLength_std if not np.isnan(x)]

        stancePercentage_std = [float(x) if x != 'nan' else np.nan for x in stancePercentage_std]
        stancePercentage_std = [x for x in stancePercentage_std if not np.isnan(x)]

        swingPercentage_std = [float(x) if x != 'nan' else np.nan for x in swingPercentage_std]
        swingPercentage_std = [x for x in swingPercentage_std if not np.isnan(x)]


        if f'EDSS{EDSS_ID}' not in kpi_params:
            kpi_params[f'EDSS{EDSS_ID}'] = {}

        kpi_params[f'EDSS{EDSS_ID}']['cadence'] = {
            'mean': np.mean(cadence_mean),
            'std': np.mean(cadence_std)
        }
        kpi_params[f'EDSS{EDSS_ID}']['strideTime'] = {
            'mean': np.mean(strideTime_mean),
            'std': np.mean(strideTime_std)
        }
        kpi_params[f'EDSS{EDSS_ID}']['velocity'] = {
            'mean': np.mean(velocity_mean),
            'std': np.mean(velocity_std)
        }
        kpi_params[f'EDSS{EDSS_ID}']['strideLength'] = {
            'mean': np.mean(strideLength_mean),
            'std': np.mean(strideLength_std)
        }
        kpi_params[f'EDSS{EDSS_ID}']['stancePercentage'] = {
            'mean': np.mean(stancePercentage_mean),
            'std': np.mean(stancePercentage_std)
        }
        kpi_params[f'EDSS{EDSS_ID}']['swingPercentage'] = {
            'mean': np.mean(swingPercentage_mean),
            'std': np.mean(swingPercentage_std)
        }
        
    
    # Define vectors for KPI wrt EDSS
    cadenceWrtEDSS = {'mean': [], 'std': []}
    strideTimeWrtEDSS = {'mean': [], 'std': []}
    velocityWrtEDSS = {'mean': [], 'std': []}
    strideLengthWrtEDSS = {'mean': [], 'std': []}
    stancePercentageWrtEDSS = {'mean': [], 'std': []}
    swingPercentageWrtEDSS = {'mean': [], 'std': []}

    for EDSS_ID in EDSS:
        cadenceWrtEDSS['mean'].append(kpi_params[f'EDSS{EDSS_ID}']['cadence']['mean'])
        cadenceWrtEDSS['std'].append(kpi_params[f'EDSS{EDSS_ID}']['cadence']['std'])

        strideTimeWrtEDSS['mean'].append(kpi_params[f'EDSS{EDSS_ID}']['strideTime']['mean'])
        strideTimeWrtEDSS['std'].append(kpi_params[f'EDSS{EDSS_ID}']['strideTime']['std'])

        velocityWrtEDSS['mean'].append(kpi_params[f'EDSS{EDSS_ID}']['velocity']['mean'])
        velocityWrtEDSS['std'].append(kpi_params[f'EDSS{EDSS_ID}']['velocity']['std'])

        strideLengthWrtEDSS['mean'].append(kpi_params[f'EDSS{EDSS_ID}']['strideLength']['mean'])
        strideLengthWrtEDSS['std'].append(kpi_params[f'EDSS{EDSS_ID}']['strideLength']['std'])

        stancePercentageWrtEDSS['mean'].append(kpi_params[f'EDSS{EDSS_ID}']['stancePercentage']['mean'])
        stancePercentageWrtEDSS['std'].append(kpi_params[f'EDSS{EDSS_ID}']['stancePercentage']['std'])

        swingPercentageWrtEDSS['mean'].append(kpi_params[f'EDSS{EDSS_ID}']['swingPercentage']['mean'])
        swingPercentageWrtEDSS['std'].append(kpi_params[f'EDSS{EDSS_ID}']['swingPercentage']['std'])
    
    kpi_params['cadenceWrtEDSS'] = cadenceWrtEDSS
    kpi_params['strideTimeWrtEDSS'] = strideTimeWrtEDSS
    kpi_params['velocityWrtEDSS'] = velocityWrtEDSS
    kpi_params['strideLengthWrtEDSS'] = strideLengthWrtEDSS
    kpi_params['stancePercentageWrtEDSS'] = stancePercentageWrtEDSS
    kpi_params['swingPercentageWrtEDSS'] = swingPercentageWrtEDSS    

    return kpi_params