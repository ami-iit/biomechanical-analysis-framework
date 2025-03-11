import numpy as np
import pandas as pd
import os


def saveKPI(KPI, paths, SUBJECT_ID, TRIAL_ID, node1Filt, node2Filt):
    all_metrics = {}
    cycleDurationLeft = [KPI['gaitCycleFeatures']['Left']['stride'][i]['timeStride'] for i in range(len(KPI['gaitCycleFeatures']['Left']['stride']))]
    cycleDurationRight = [KPI['gaitCycleFeatures']['Right']['stride'][i]['timeStride'] for i in range(len(KPI['gaitCycleFeatures']['Right']['stride']))] 
    all_metrics['CycleDuration'] = calculate_metrics(cycleDurationLeft, cycleDurationRight, 'Cycle Duration')
    
    strideLengthLeft = [KPI['strideLength']['Left'][i]['distance'] for i in range(len(KPI['strideLength']['Left']))]
    strideLengthRight = [KPI['strideLength']['Right'][i]['distance'] for i in range(len(KPI['strideLength']['Right']))]
    all_metrics['strideLength'] = calculate_metrics(strideLengthLeft, strideLengthRight, 'Stride Length')
    
    angLeft = [KPI['swingLegAngVelocity']['Left'][i]['norm'] for i in range(len(KPI['swingLegAngVelocity']['Left']))]
    angVelLeft = list(np.concatenate(angLeft, axis=0))
    angRight = [KPI['swingLegAngVelocity']['Right'][i]['norm'] for i in range(len(KPI['swingLegAngVelocity']['Right']))]
    angVelRight = list(np.concatenate(angRight, axis=0))
    all_metrics['angVel'] = calculate_metrics(angVelLeft, angVelRight, 'Angular Velocity')
    
    swingWidthLeft = [KPI['swingWidth']['Left'][i]['percentageOfStrideLength'] for i in range(len(KPI['swingWidth']['Left']))]
    swingWidthRight = [KPI['swingWidth']['Right'][i]['percentageOfStrideLength'] for i in range(len(KPI['swingWidth']['Right']))]
    all_metrics['swingWidth'] = calculate_metrics(swingWidthLeft, swingWidthRight, 'Swing Width')
    
    pathLengthLeft = [KPI['pathLength']['Left'][i]['percentageOfStrideLength'] for i in range(len(KPI['pathLength']['Left']))]
    pathLengthRight = [KPI['pathLength']['Right'][i]['percentageOfStrideLength'] for i in range(len(KPI['pathLength']['Right']))]
    all_metrics['pathLength'] = calculate_metrics(pathLengthLeft, pathLengthRight, 'Path Length')
    
    stancePhaseLeft = [KPI['phasePercentageWrtCycleDuration']['Left'][i]['percentageOfStance'] for i in range(len(KPI['phasePercentageWrtCycleDuration']['Left']))]
    stancePhaseRight = [KPI['phasePercentageWrtCycleDuration']['Right'][i]['percentageOfStance'] for i in range(len(KPI['phasePercentageWrtCycleDuration']['Right']))]
    all_metrics['stancePhase'] = calculate_metrics(stancePhaseLeft, stancePhaseRight, 'Stance Phase')
    
    swingTimeLeft = [KPI['phasePercentageWrtCycleDuration']['Left'][i]['percentageOfSwing'] for i in range(len(KPI['phasePercentageWrtCycleDuration']['Left']))]
    swingTimeRight = [KPI['phasePercentageWrtCycleDuration']['Right'][i]['percentageOfSwing'] for i in range(len(KPI['phasePercentageWrtCycleDuration']['Right']))]
    all_metrics['swingTime'] = calculate_metrics(swingTimeLeft, swingTimeRight, 'Swing Time')
    
    
    all_metrics['FT_x'] = calculate_metrics(node1Filt['FT']['data'][:, 0, 0], node2Filt['FT']['data'][:, 0, 0], 'FT_x')
    all_metrics['FT_y'] = calculate_metrics(node1Filt['FT']['data'][:, 0, 1], node2Filt['FT']['data'][:, 0, 1], 'FT_y')
    all_metrics['FT_z'] = calculate_metrics(node1Filt['FT']['data'][:, 0, 2], node2Filt['FT']['data'][:, 0, 2], 'FT_z')
    
    
    rows = []
    for metric_name, values in all_metrics.items():
        rows.append({
            'Parameter': f'{metric_name}_Left',
            'Mean': values['mean'][0],
            'Std': values['std'][0],
            'CV': values['cv'][0],
        })
        rows.append({
            'Parameter': f'{metric_name}_Right',
            'Mean': values['mean'][1],
            'Std': values['std'][1],
            'CV': values['cv'][1],
        })
        
    rows.insert(0, {
    'Parameter': 'Cadence',
    'Mean': KPI['cadence'],
    'Std': None,  
    'CV': None    
    })

    df = pd.DataFrame(rows)
    file_name = f"S{SUBJECT_ID:02d}_T{TRIAL_ID}.xlsx"
    file_path = os.path.join(paths['pathToProcessedData'], file_name)
    df.to_excel(file_path, index=False)
    
    return 0
    


def calculate_metrics(left_values, right_values, metric_name):
    mean_left = np.mean(left_values)
    mean_right = np.mean(right_values)
    std_left = np.std(left_values)
    std_right = np.std(right_values)
    cv_left = std_left / mean_left if mean_left != 0 else 0
    cv_right = std_right / mean_right if mean_right != 0 else 0

    return {
        'mean': [mean_left, mean_right],
        'std': [std_left, std_right],
        'cv': [cv_left, cv_right]
    }

    