import idyntree.bindings as iDynTree
import numpy as np
from pathlib import Path
from joblib import load
import matplotlib.pyplot as plt
from scipy.stats import ttest_ind
from scipy import stats
from scipy.interpolate import CubicSpline
from baf_kpi.scripts.loadKPIreferenceParameters import loadKPIreferenceParameters
from baf_kpi.scripts.getKPIreferenceParameters import getKPIreferenceParameters


def main():

    ttest = True

    # Colors
    rightStanceColor = (0, 0.4470, 0.7410)
    rightSwingColor = (0.3010, 0.7450, 0.9330)
    leftStanceColor = (1, 0, 0)
    leftSwingColor = (0.97, 0.55, 0.55)
    doubleSupportColor = (0.901960784313726, 0.901960784313726, 0.901960784313726)
    singleSupportColor_Left = (1, 0.800000011920929, 0.800000011920929)
    singleSupportColor_Right = (0.87058824300766, 0.921568632125854, 0.980392158031464)
    soaKPIstdColor = (0.501960784313725 ,  0.501960784313725,   0.501960784313725)
    transparencyStd = 0.3  # Transparency for shaded std area

    EXPERIMENT_DIR = 'C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/KPI_Python/dataset'
    datasetRoot = Path(EXPERIMENT_DIR).resolve()
    pathToGlobalPlots = datasetRoot / 'plots'
    paths = {}
    paths['datasetRoot'] = datasetRoot.as_posix()
    paths['pathToPlotsGlobal'] = pathToGlobalPlots.as_posix()

    if not Path(paths['pathToPlotsGlobal']).exists():
        Path(paths['pathToPlotsGlobal']).mkdir(parents=True, exist_ok=True)

    """Extract data"""
    subjectList = [1,2,3,4,5,9,10,11,12]
    # subjectList = [1,2,3,4,5]
    nrOfSubjects = len(subjectList)
    trialList = [19,20,21,22,23,24,25,26]
    # trialList = [19,23,26]
    nrOfTrials = len(trialList)
    EDSS = [4,2,6,2,4,4,2,2,4] # Expanded Disability Status Scale level
    # EDSS = [4,2,6,2,4]
    gender = ['M','F','M','M','F']
    dominance = ['Left','Right','Right','Right','Right']


    subjectsLabel = []
    subjectsLabelWithEDSS = []
    analysisResults = {}

    for subjIdx in range(nrOfSubjects):
        SUBJECT_ID = subjectList[subjIdx]
        pathToSubject = datasetRoot / f'S{SUBJECT_ID:02d}'
        paths['pathToSubject'] = pathToSubject.as_posix()
        subjectsLabel.append(f'S{SUBJECT_ID:02d}')
        subjectsLabelWithEDSS.append(f'S{SUBJECT_ID:02d} (EDSS {EDSS[subjIdx]})')
        
        analysisResults[SUBJECT_ID] = {
            'label': f'S{SUBJECT_ID:02d}',
            'EDSS': EDSS[subjIdx]
        }
        
        for trialIdx in range(nrOfTrials):
            TRIAL_ID = trialList[trialIdx]
            pathToTrial = pathToSubject / f'trial{TRIAL_ID:02d}'
            paths['pathToTrial'] = pathToTrial.as_posix()
            analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}'] = {
                'KPI': load(pathToTrial / 'processed' / 'KPI.joblib'),
            }
        
    # Extract KPIs references from the state-of-the-art (soa)
    soa = loadKPIreferenceParameters()
    KPIparams = getKPIreferenceParameters(soa,EDSS)


    """Swing and stance phases"""
    fig, axs = plt.subplots(nrOfTrials, 1, figsize=(10, 8))
    fig.suptitle('Intersubject phases', fontsize=12)
    fig.patch.set_facecolor('white')

    xTick_vector_1S = np.arange(1.6, 41, 2)
    xTick_vector_2S = np.arange(1.9, 41, 2)
    xTick_vector_3S = np.arange(2.2, 41, 2)
    xTick_vector_4S = np.arange(2.5, 41, 2)

    xTick_positions = [(xTick_vector_1S[i] + xTick_vector_2S[i] + xTick_vector_3S[i] + xTick_vector_4S[i]) / 4 for i in range(nrOfSubjects)]


    for trialIdx in range(nrOfTrials):
        TRIAL_ID = trialList[trialIdx]
        ax = axs[trialIdx] if nrOfTrials > 1 else axs  

        for subjIdx in range(nrOfSubjects):
            SUBJECT_ID = subjectList[subjIdx]
            
            totalPercentageL_sum = []
            totalPercentageR_sum = []
            
            percStance_L = np.mean([analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Left']['stance'][i]['percStance'] for i in range(len(analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Left']['stance']))])
            percStance_L = np.round(percStance_L)
            percSwing_L = np.mean([analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Left']['swing'][i]['percSwing'] for i in range(len(analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Left']['swing']))])
            percSwing_L = np.round(percSwing_L)
            totalPercentageL_sum.append(np.round([percStance_L, percSwing_L]))

            # Calcola e arrotonda le medie delle percentuali di stance e swing per il lato destro
            percStance_R = np.mean([analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Right']['stance'][i]['percStance'] for i in range(len(analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Right']['stance']))])
            percStance_R  = np.round(percStance_R)
            percSwing_R = np.mean([analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Right']['swing'][i]['percSwing'] for i in range(len(analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['gaitCycleFeatures']['Right']['swing']))])
            percSwing_R = np.round(percSwing_R)
            totalPercentageR_sum.append(np.round([percStance_R, percSwing_R]))

            bar1 = ax.bar(xTick_vector_1S[subjIdx], totalPercentageL_sum[0][0], width=0.27, color=leftStanceColor, linewidth=2)

            bar2 = ax.bar(xTick_vector_2S[subjIdx], totalPercentageL_sum[0][1], width=0.27, color=leftSwingColor, linewidth=2)

            bar3 = ax.bar(xTick_vector_3S[subjIdx], totalPercentageR_sum[0][0], width=0.27, color=rightStanceColor, linewidth=2)

            bar4 = ax.bar(xTick_vector_4S[subjIdx], totalPercentageR_sum[0][1], width=0.27, color=rightSwingColor, linewidth=2)

        # ax.autoscale(enable=True, axis='x', tight=True)
        plot_refStance = ax.plot(ax.get_xlim(), [soa['stancePercentageRefValue']] * 2, '--k', linewidth=2)
        plot_refSwing = ax.plot(ax.get_xlim(), [soa['swingPercentageRefValue']] * 2, 'k', linewidth=1.5)

        ax.set_ylim([0, 100])
        # ax.set_ylabel(f'Trial {TRIAL_ID:02d}', fontsize=10, fontweight='bold')
        if trialIdx == 0:
            ax.set_title('Phase %', fontsize=12)
        ax.set_xticks(xTick_positions)
        ax.set_xticklabels(subjectsLabelWithEDSS, fontsize=10, rotation=45, ha='right')
        ax.grid(True)
        ax.tick_params(axis='both', labelsize=10)

        if trialIdx == 0:
            leg = ax.legend(['ref stance','ref swing','left stance','left swing','right stance','right swing'],
                        fontsize=10, ncol=3, loc='upper right', bbox_to_anchor=(1.0, 2.1))
            
        if trialIdx == 0 or trialIdx == 1:
            ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n TUG', fontsize=10, color='black', fontweight='bold')
        elif trialIdx == 2 or trialIdx == 3:
            ax.set_ylabel(f'Trial{TRIAL_ID:02d} \n T25FWT', fontsize=10, color='black', fontweight='bold')
        elif trialIdx == 4 or trialIdx == 5:
            ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n F8WT', fontsize=10, color='black', fontweight='bold')
        elif trialIdx == 6:
            ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n CMI', fontsize=10, color='black', fontweight='bold')
        elif trialIdx == 7:
            ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n 6MWT', fontsize=10, color='black', fontweight='bold')
            
        
    # plt.tight_layout(rect=[0, 0, 1, 0.95]) 
    



    """Forces percentage wrt vertical force"""

    fig, axes = plt.subplots(nrOfTrials, 1, figsize=(10, 6 * nrOfTrials))
    # fig.suptitle('Forces Percentage', fontsize=14)
    fig.patch.set_facecolor('white')

    xTick_vector_1S = np.arange(1.6, 40.6, 2)
    xTick_vector_2S = np.arange(1.9, 40.9, 2)
    xTick_vector_3S = np.arange(2.2, 40.2, 2)
    xTick_vector_4S = np.arange(2.5, 40.5, 2)

    xTick_positions = [(xTick_vector_1S[i] + xTick_vector_2S[i] + xTick_vector_3S[i] + xTick_vector_4S[i]) / 4 for i in range(nrOfSubjects)]
    

    for trialIdx in range(nrOfTrials):
        TRIAL_ID = trialList[trialIdx]
        ax = axes[trialIdx] if nrOfTrials > 1 else axes  
        
        for subjIdx in range(nrOfSubjects):
            SUBJECT_ID = subjectList[subjIdx]
            
            bar1 = ax.bar(xTick_vector_1S[subjIdx], 
                        analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['percentageWrtVerticalForce']['percentage_AP_left'],
                        width=0.27, color='white', edgecolor='red', linewidth=2)
            
            bar2 = ax.bar(xTick_vector_2S[subjIdx], 
                        analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['percentageWrtVerticalForce']['percentage_ML_left'],
                        width=0.27, color=[0.8, 0.8, 0.8], edgecolor='red', linewidth=2, linestyle=':')
            
            bar3 = ax.bar(xTick_vector_3S[subjIdx], 
                        analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['percentageWrtVerticalForce']['percentage_AP_right'],
                        width=0.27, color='white', edgecolor='blue', linewidth=2)
            
            bar4 = ax.bar(xTick_vector_4S[subjIdx], 
                        analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']['KPI']['percentageWrtVerticalForce']['percentage_ML_right'],
                        width=0.27, color=[0.8, 0.8, 0.8], edgecolor='blue', linewidth=2, linestyle=':')

        ref_AP_value = soa['APpercentageRefValue']
        plot_refAP = ax.plot(ax.get_xlim(), [ref_AP_value] * 2, '--k', linewidth=2)[0]

        ax.set_ylim([0, 50])
        ax.set_ylabel(f'Trial {TRIAL_ID:02d}', fontsize=10, fontweight='bold')
        ax.set_xticks(xTick_positions)
        ax.set_xticklabels(subjectsLabel, rotation=40, ha='right', fontsize=8)
        ax.grid(True)
        if trialIdx == 0:
            ax.set_title('Force %', fontsize=12)
        ax.tick_params(axis='both', which='major', labelsize=14)

        if trialIdx == 0:
            legend = ax.legend([bar1, bar2, bar3, bar4, plot_refAP], 
                        ['AP left', 'ML left', 'AP right', 'ML right', 'ref AP'], 
                        fontsize=10, ncol=3, loc='upper right', bbox_to_anchor=(1.0, 2.1))
        
    plt.tight_layout(rect=[0, 0, 0.9, 0.9]) 
    
    
    # """STATISTICS"""

    # """Variables correlation"""
    # nrOfKPIs = 5 # Number of KPIs to be plotted
    # fig, axes = plt.subplots(nrOfKPIs, nrOfTrials, figsize=(15, 8), squeeze=False)
    # # fig.suptitle('KPI correlation w.r.t. EDSS', fontsize=16)
    # fig.patch.set_facecolor('white')
    
    # fig.subplots_adjust(wspace=0.5, hspace=0.5)


    # for trialIdx in range(nrOfTrials):
    #     TRIAL_ID = trialList[trialIdx]

    #     # Initialize interSubject data
    #     interSubject = {
    #         'cadence': [],
    #         'cycleDurationLeft': [],
    #         'cycleDurationRight': [],
    #         'nrOfCycleLeft': [],
    #         'nrOfCycleRight': [],
    #         'COMvelocity': [],
    #         'stanceLeft': [],
    #         'stanceRight': [],
    #         'swingLeft': [],
    #         'swingRight': [],
    #         'swingWidthLeft': [],
    #         'swingWidthRight': []
    #     }

    #     # Gather data for each subject
    #     for subjIdx in range(nrOfSubjects):
    #         SUBJECT_ID = subjectList[subjIdx]
    #         tmpStruct = analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']
            
    #         # Cadence
    #         interSubject['cadence'].append(tmpStruct['KPI']['cadence'])
            
    #         # Cycle duration (or stride time)
    #         interSubject['cycleDurationLeft'].append(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Left']['stride'][i]['timeStride'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Left']['stride']))]))
    #         interSubject['cycleDurationRight'].append(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Right']['stride'][i]['timeStride'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Right']['stride']))]))
            
    #         # Number of cycles
    #         interSubject['nrOfCycleLeft'].append(tmpStruct['KPI']['gaitCycleFeatures']['Left']['nrOfCycles'])
    #         interSubject['nrOfCycleRight'].append(tmpStruct['KPI']['gaitCycleFeatures']['Right']['nrOfCycles'])
            
    #         # COM velocity
    #         interSubject['COMvelocity'].append(np.mean(np.linalg.norm(tmpStruct['KPI']['COM']['velocity'], axis=0)))
            
    #         # Swing/Stance phases
    #         interSubject['stanceLeft'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Left']['stance'][i]['percStance'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Left']['stance']))])))
    #         interSubject['stanceRight'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Right']['stance'][i]['percStance'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Right']['stance']))])))
    #         interSubject['swingLeft'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Left']['swing'][i]['percSwing'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Left']['swing']))])))
    #         interSubject['swingRight'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Right']['swing'][i]['percSwing'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Right']['swing']))])))
            
    #         # Swing width as percentage of stride length
    #         interSubject['swingWidthLeft'].append(round(np.mean([tmpStruct['KPI']['swingWidth']['Left'][i]['percentageOfStrideLength'] for i in range(len(tmpStruct['KPI']['swingWidth']['Left']))])))
    #         interSubject['swingWidthRight'].append(round(np.mean([tmpStruct['KPI']['swingWidth']['Right'][i]['percentageOfStrideLength'] for i in range(len(tmpStruct['KPI']['swingWidth']['Right']))])))

    #     # Plotting for each KPI
    #     for KPIidx in range(nrOfKPIs):
    #         ax = axes[KPIidx][trialIdx] 
    #         # Scatter plot and reference
    #         if KPIidx == 0:  # CADENCE
    #             xVect = np.unique(EDSS)
    #             ax.scatter(EDSS, interSubject['cadence'], s=30, facecolor='none', edgecolor=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=2)
                
    #             slope, intercept, r_value, p_value, std_err = stats.linregress(EDSS, interSubject['cadence'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_line = slope * EDSS_dense + intercept
    #             trend = ax.plot(EDSS_dense, regression_line, color=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
        
    #             xFlipVect = np.concatenate([xVect, np.flip(xVect)])
    #             patchVect = np.concatenate([np.array(KPIparams['cadenceWrtEDSS']['mean']) + np.array(KPIparams['cadenceWrtEDSS']['std']),
    #                                         np.flip(np.array(KPIparams['cadenceWrtEDSS']['mean']) - np.array(KPIparams['cadenceWrtEDSS']['std']))])
                
    #             ax.plot(xVect, KPIparams['cadenceWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
    #             patchX = ax.fill_between(xFlipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
    #             if trialIdx == 0:
    #                 ax.set_title("TUG", fontsize = 15, fontweight='bold')
    #             elif trialIdx == 1:
    #                 ax.set_title("F8WT", fontsize = 15, fontweight='bold')
    #             elif trialIdx == 2:
    #                 ax.set_title("6MWT", fontsize = 15, fontweight='bold')
    #             ax.set_ylim([50, 120])
    #             # ax.set_xlabel('EDSS',fontsize=10)
    #             ax.set_ylabel('Cadence \n [step/min]', fontsize=10)
                
    #             if trialIdx == nrOfTrials - 1:
    #                 ax.legend([trend[0], patchX],['Trend', 'Ref Std'], loc = 'center right', bbox_to_anchor = (1.5,0.5))
            
    #         elif KPIidx == 1:  # COM VELOCITY
    #             xVect = np.unique(EDSS)
    #             ax.scatter(EDSS, interSubject['COMvelocity'], s=30, facecolor='none', edgecolor=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=2)
                
    #             slope, intercept, r_value, p_value, std_err = stats.linregress(EDSS, interSubject['COMvelocity'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_line = slope * EDSS_dense + intercept
    #             trend = ax.plot(EDSS_dense, regression_line, color=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
    #             xFLipVect = np.concatenate([xVect, np.flip(xVect)])
    #             patchVect = np.concatenate([np.array(KPIparams['velocityWrtEDSS']['mean']) + np.array(KPIparams['velocityWrtEDSS']['std']),
    #                                         np.flip(np.array(KPIparams['velocityWrtEDSS']['mean']) - np.array(KPIparams['velocityWrtEDSS']['std']))])
                
    #             ax.plot(xVect, KPIparams['velocityWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
    #             patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
    #             ax.set_ylim([0.4, 1.6])
    #             # ax.set_xlabel('EDSS',fontsize=10)
    #             ax.set_ylabel('Velocity [m/s]',fontsize=10)
    #             if trialIdx == nrOfTrials - 1:
    #                 ax.legend([trend[0], patchX],['Trend', 'Ref Std'], loc = 'center right', bbox_to_anchor = (1.5,0.5))

    #         elif KPIidx == 2: # CYCLE DURATION (STRIDE TIME)
    #             xVect = np.unique(EDSS)
    #             ax.scatter(EDSS, interSubject['cycleDurationLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=2)
    #             ax.scatter(EDSS, interSubject['cycleDurationRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=2)
                
    #             slopeRight, interceptRight, r_valueRight, p_valueRight, std_errRight = stats.linregress(EDSS, interSubject['cycleDurationRight'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_lineRight = slopeRight * EDSS_dense + interceptRight
    #             trendRight = ax.plot(EDSS_dense, regression_lineRight, color=rightStanceColor, linewidth=1.2)
                
    #             slopeLeft, interceptLeft, r_valueLeft, p_valueLeft, std_errLeft = stats.linregress(EDSS, interSubject['cycleDurationLeft'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_lineLeft = slopeLeft * EDSS_dense + interceptLeft
    #             trendLeft = ax.plot(EDSS_dense, regression_lineLeft, color=leftStanceColor, linewidth=1.2)
                
                
    #             xFLipVect = np.concatenate([xVect, np.flip(xVect)])
    #             patchVect = np.concatenate([np.array(KPIparams['strideTimeWrtEDSS']['mean']) + np.array(KPIparams['strideTimeWrtEDSS']['std']),
    #                                         np.flip(np.array(KPIparams['strideTimeWrtEDSS']['mean']) - np.array(KPIparams['strideTimeWrtEDSS']['std']))])
                
    #             ax.plot(xVect, KPIparams['strideTimeWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
    #             patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
    #             ax.set_ylim([0.9, 2.1])
    #             # ax.set_xlabel('EDSS',fontsize=10)
    #             ax.set_ylabel('Stride \n Time [s]',fontsize=10)
    #             if trialIdx == nrOfTrials - 1:
    #                 ax.legend([trendRight[0], trendLeft[0], patchX],['trend right', 'trend left','Ref Std'], loc = 'center right', bbox_to_anchor=(1.5, 0.5))
                
    #         elif KPIidx == 3: # STANCE PERCENTAGE
    #             xVect = np.unique(EDSS)
    #             ax.scatter(EDSS, interSubject['stanceLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=2)
    #             ax.scatter(EDSS, interSubject['stanceRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=2)
                
                
    #             slopeRight, interceptRight, r_valueRight, p_valueRight, std_errRight = stats.linregress(EDSS, interSubject['stanceRight'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_lineRight = slopeRight * EDSS_dense + interceptRight
    #             ax.plot(EDSS_dense, regression_lineRight, color=rightStanceColor, linewidth=1.2)
                
    #             slopeLeft, interceptLeft, r_valueLeft, p_valueLeft, std_errLeft = stats.linregress(EDSS, interSubject['stanceLeft'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_lineLeft = slopeLeft * EDSS_dense + interceptLeft
    #             ax.plot(EDSS_dense, regression_lineLeft, color=leftStanceColor, linewidth=1.2)
                
                
    #             xFLipVect = np.concatenate([xVect, np.flip(xVect)])
    #             patchVect = np.concatenate([np.array(KPIparams['stancePercentageWrtEDSS']['mean']) + np.array(KPIparams['stancePercentageWrtEDSS']['std']),
    #                                         np.flip(np.array(KPIparams['stancePercentageWrtEDSS']['mean']) - np.array(KPIparams['stancePercentageWrtEDSS']['std']))])
                
    #             ax.plot(xVect, KPIparams['stancePercentageWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
    #             patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
    #             plot_refSTance = ax.plot([0,7], [soa['stancePercentageRefValue']] * 2, '--k', linewidth=1)
                
    #             ax.set_ylim([60, 75])
    #             # ax.set_xlabel('EDSS',fontsize=10)
    #             ax.set_ylabel('Stance \n [% stride]',fontsize=10)
    #             if trialIdx == nrOfTrials - 1:
    #                 ax.legend([trendRight[0], trendLeft[0], plot_refSTance[0], patchX],['trend right','trend left','Ref Control','Ref Std'], loc = 'center right', bbox_to_anchor=(1.5, 0.5))
                
    #         elif KPIidx == 4: # SWING 
    #             xVect = np.unique(EDSS)
    #             ax.scatter(EDSS, interSubject['swingLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=2)
    #             ax.scatter(EDSS, interSubject['swingRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=2)
                
    #             slopeRight, interceptRight, r_valueRight, p_valueRight, std_errRight = stats.linregress(EDSS, interSubject['swingRight'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_lineRight = slopeRight * EDSS_dense + interceptRight
    #             ax.plot(EDSS_dense, regression_lineRight, color=rightStanceColor, linewidth=1.2)
                
    #             slopeLeft, interceptLeft, r_valueLeft, p_valueLeft, std_errLeft = stats.linregress(EDSS, interSubject['swingLeft'])
    #             EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
    #             regression_lineLeft = slopeLeft * EDSS_dense + interceptLeft
    #             ax.plot(EDSS_dense, regression_lineLeft, color=leftStanceColor, linewidth=1.2)
                
                
    #             xFLipVect = np.concatenate([xVect, np.flip(xVect)])
    #             patchVect = np.concatenate([np.array(KPIparams['swingPercentageWrtEDSS']['mean']) + np.array(KPIparams['swingPercentageWrtEDSS']['std']),
    #                                         np.flip(np.array(KPIparams['swingPercentageWrtEDSS']['mean']) - np.array(KPIparams['swingPercentageWrtEDSS']['std']))])
                
    #             ax.plot(xVect, KPIparams['swingPercentageWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
    #             patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
    #             plot_refSwing = ax.plot([0,7], [soa['swingPercentageRefValue']] * 2, '--k', linewidth=1)
                
    #             ax.set_ylim([25, 41])
    #             ax.set_xlabel('EDSS',fontsize=10)
    #             ax.set_ylabel('Swing \n [% stride]',fontsize=10)
    #             if trialIdx == nrOfTrials - 1:
    #                 ax.legend([trendRight[0], trendLeft[0], plot_refSwing[0], patchX],['trend right','trend left','Ref Control','Ref Std'], loc = 'center right', bbox_to_anchor=(1.5, 0.5))
            
    #         ax.grid(True)
    #         ax.set_xlim([1, 7])

    """STATISTICS"""

    """Variables correlation"""
    nrOfKPIs = 5 # Number of KPIs to be plotted
    fig, axes = plt.subplots(nrOfTrials, nrOfKPIs, figsize=(30, 30), squeeze=False)
    # fig, axes = plt.subplots(nrOfKPIs, nrOfTrials, figsize=(30, 15), squeeze=False)
    # fig.suptitle('KPI correlation w.r.t. EDSS', fontsize=16)
    fig.patch.set_facecolor('white')
    
    fig.subplots_adjust(wspace=0.2, hspace=0.4)
    # fig.subplots_adjust(wspace=0.4, hspace=0.2)


    for trialIdx in range(nrOfTrials):
        TRIAL_ID = trialList[trialIdx]

        # Initialize interSubject data
        interSubject = {
            'cadence': [],
            'cycleDurationLeft': [],
            'cycleDurationRight': [],
            'nrOfCycleLeft': [],
            'nrOfCycleRight': [],
            'COMvelocity': [],
            'stanceLeft': [],
            'stanceRight': [],
            'swingLeft': [],
            'swingRight': [],
            'strideLengthLeft': [],
            'strideLengthRight': [],
            'swingWidthLeft': [],
            'swingWidthRight': []
        }

        # Gather data for each subject
        for subjIdx in range(nrOfSubjects):
            SUBJECT_ID = subjectList[subjIdx]
            tmpStruct = analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']
            
            # Cadence
            interSubject['cadence'].append(tmpStruct['KPI']['cadence'])
            
            # Cycle duration (or stride time)
            interSubject['cycleDurationLeft'].append(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Left']['stride'][i]['timeStride'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Left']['stride']))]))
            interSubject['cycleDurationRight'].append(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Right']['stride'][i]['timeStride'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Right']['stride']))]))
            
            # Number of cycles
            interSubject['nrOfCycleLeft'].append(tmpStruct['KPI']['gaitCycleFeatures']['Left']['nrOfCycles'])
            interSubject['nrOfCycleRight'].append(tmpStruct['KPI']['gaitCycleFeatures']['Right']['nrOfCycles'])
            
            # COM velocity
            interSubject['COMvelocity'].append(np.mean(np.linalg.norm(tmpStruct['KPI']['COM']['velocity'], axis=0)))
            
            # Swing/Stance phases
            interSubject['stanceLeft'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Left']['stance'][i]['percStance'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Left']['stance']))])))
            interSubject['stanceRight'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Right']['stance'][i]['percStance'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Right']['stance']))])))
            interSubject['swingLeft'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Left']['swing'][i]['percSwing'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Left']['swing']))])))
            interSubject['swingRight'].append(round(np.mean([tmpStruct['KPI']['gaitCycleFeatures']['Right']['swing'][i]['percSwing'] for i in range(len(tmpStruct['KPI']['gaitCycleFeatures']['Right']['swing']))])))
            
            # Stride length
            # interSubject['strideLengthLeft'].append(np.mean([tmpStruct['KPI']['strideLength']['Left'][i]['distance'] for i in range(len(tmpStruct['KPI']['strideLength']['Left']))]))
            # interSubject['strideLengthRight'].append(np.mean([tmpStruct['KPI']['strideLength']['Right'][i]['distance'] for i in range(len(tmpStruct['KPI']['strideLength']['Right']))]))
            
            # Swing width as percentage of stride length
            interSubject['swingWidthLeft'].append(round(np.mean([tmpStruct['KPI']['swingWidth']['Left'][i]['percentageOfStrideLength'] for i in range(len(tmpStruct['KPI']['swingWidth']['Left']))])))
            interSubject['swingWidthRight'].append(round(np.mean([tmpStruct['KPI']['swingWidth']['Right'][i]['percentageOfStrideLength'] for i in range(len(tmpStruct['KPI']['swingWidth']['Right']))])))
            
        interSubject['cadence'] = np.array(interSubject['cadence'])
        EDSS = np.array(EDSS)
        
        # Plotting for each KPI
        for KPIidx in range(nrOfKPIs):
            ax.grid(True)
            ax = axes[trialIdx, KPIidx]
            # ax = axes[KPIidx,trialIdx]
            # Scatter plot and reference
            if KPIidx == 0:  # CADENCE
                xVect = np.unique(EDSS)
                ax.scatter(EDSS, interSubject['cadence'], s=30, facecolor='none', edgecolor=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
                # slope, intercept = np.polyfit(EDSS, interSubject['cadence'], 1)  
                # trend_line = np.poly1d([slope, intercept])
                # trend = ax.plot(EDSS, trend_line(EDSS), color=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
                
                slope, intercept, r_value, p_value, std_err = stats.linregress(EDSS, interSubject['cadence'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_line = slope * EDSS_dense + intercept
                trend = ax.plot(EDSS_dense, regression_line, color=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                

                xFlipVect = np.concatenate([xVect, np.flip(xVect)])
                patchVect = np.concatenate([np.array(KPIparams['cadenceWrtEDSS']['mean']) + np.array(KPIparams['cadenceWrtEDSS']['std']),
                                            np.flip(np.array(KPIparams['cadenceWrtEDSS']['mean']) - np.array(KPIparams['cadenceWrtEDSS']['std']))])
                
                ax.plot(xVect, KPIparams['cadenceWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
                patchX = ax.fill_between(xFlipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
                
                if trialIdx == 0 or trialIdx == 1:
                    ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n TUG', fontsize=10, color='black', fontweight='bold')
                elif trialIdx == 2 or trialIdx == 3:
                    ax.set_ylabel(f'Trial{TRIAL_ID:02d} \n T25FWT', fontsize=10, color='black', fontweight='bold')
                elif trialIdx == 4 or trialIdx == 5:
                    ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n F8WT', fontsize=10, color='black', fontweight='bold')
                elif trialIdx == 6:
                    ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n CMI', fontsize=10, color='black', fontweight='bold')
                elif trialIdx == 7:
                    ax.set_ylabel(f'Trial {TRIAL_ID:02d} \n 6MWT', fontsize=10, color='black', fontweight='bold')
                   
                if trialIdx == 0: 
                    ax.set_title('Cadence \n [steps/min]', fontsize=10)
                    ax.legend([trend[0], patchX],['Trend', 'Ref Std'], loc = 'upper right', bbox_to_anchor=(1.35, 2.0))
                    
                    
                ax.set_ylim([60, 125])
                # ax.set_xlabel('EDSS',fontsize=10)
                
                
                if trialIdx == nrOfTrials - 1:
                    ax.set_xlabel('EDSS',fontsize=10)
            
            elif KPIidx == 1:  # COM VELOCITY
                xVect = np.unique(EDSS)
                ax.scatter(EDSS, interSubject['COMvelocity'], s=30, facecolor='none', edgecolor=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
                # slope,intercept = np.polyfit(EDSS, interSubject['COMvelocity'], 1)
                # trend_line = np.poly1d([slope,intercept])
                # trend = ax.plot(EDSS, trend_line(EDSS), color=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
                slope, intercept, r_value, p_value, std_err = stats.linregress(EDSS, interSubject['COMvelocity'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_line = slope * EDSS_dense + intercept
                trend = ax.plot(EDSS_dense, regression_line, color=[0.149019607843137, 0.149019607843137, 0.149019607843137], linewidth=1.2)
                
                
                xFLipVect = np.concatenate([xVect, np.flip(xVect)])
                patchVect = np.concatenate([np.array(KPIparams['velocityWrtEDSS']['mean']) + np.array(KPIparams['velocityWrtEDSS']['std']),
                                            np.flip(np.array(KPIparams['velocityWrtEDSS']['mean']) - np.array(KPIparams['velocityWrtEDSS']['std']))])
                
                ax.plot(xVect, KPIparams['velocityWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
                patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
                ax.set_ylim([0.4, 1.6])
                
                if trialIdx == 0:
                    ax.set_title('Velocity [m/s]',fontsize=10)
                
                if trialIdx == nrOfTrials - 1:
                    ax.set_xlabel('EDSS',fontsize=10)

            elif KPIidx == 2: # CYCLE DURATION (STRIDE TIME)
                xVect = np.unique(EDSS)
                ax.scatter(EDSS, interSubject['cycleDurationLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=1.2)
                ax.scatter(EDSS, interSubject['cycleDurationRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=1.2)
                
                # slopeRight = np.polyfit(EDSS, interSubject['cycleDurationRight'], 1)
                # trendRight = ax.plot(EDSS, np.poly1d(slopeRight)(EDSS), color=rightStanceColor, linewidth=1.2)
                
                # slopeLeft = np.polyfit(EDSS, interSubject['cycleDurationLeft'], 1)
                # trendLeft = ax.plot(EDSS, np.poly1d(slopeLeft)(EDSS), color=leftStanceColor, linewidth=1.2)
                
                slopeRight, interceptRight, r_valueRight, p_valueRight, std_errRight = stats.linregress(EDSS, interSubject['cycleDurationRight'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_lineRight = slopeRight * EDSS_dense + interceptRight
                trendRight = ax.plot(EDSS_dense, regression_lineRight, color=rightStanceColor, linewidth=1.2)
                
                slopeLeft, interceptLeft, r_valueLeft, p_valueLeft, std_errLeft = stats.linregress(EDSS, interSubject['cycleDurationLeft'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_lineLeft = slopeLeft * EDSS_dense + interceptLeft
                trendLeft = ax.plot(EDSS_dense, regression_lineLeft, color=leftStanceColor, linewidth=1.2)
                
                
                xFLipVect = np.concatenate([xVect, np.flip(xVect)])
                patchVect = np.concatenate([np.array(KPIparams['strideTimeWrtEDSS']['mean']) + np.array(KPIparams['strideTimeWrtEDSS']['std']),
                                            np.flip(np.array(KPIparams['strideTimeWrtEDSS']['mean']) - np.array(KPIparams['strideTimeWrtEDSS']['std']))])
                
                ax.plot(xVect, KPIparams['strideTimeWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
                patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
                ax.set_ylim([0.9, 2.1])
                
                if trialIdx == 0:
                    ax.set_title('Cycle \n Duration [s]',fontsize=10)
                    ax.legend([trendRight[0], trendLeft[0], patchX],['trend right', 'trend left','Ref Std'], loc = 'upper right', bbox_to_anchor=(1.35, 2.3))
                    
                if trialIdx == nrOfTrials - 1:
                    ax.set_xlabel('EDSS',fontsize=10)
                
                
            # elif KPIidx == 3: # STRIDE LENGTH
            #     xVect = np.unique(EDSS)
            #     ax.scatter(EDSS, interSubject['strideLengthLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=2)
            #     ax.scatter(EDSS, interSubject['strideLengthRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=2)
                
            #     slopeLeft = np.polyfit(EDSS, interSubject['strideLengthLeft'], 1)
            #     trendLeft = ax.plot(EDSS, np.poly1d(slopeLeft)(EDSS), color=leftStanceColor, linewidth=2)
                
            #     slopeRight = np.polyfit(EDSS, interSubject['strideLengthRight'], 1)
            #     trendRight = ax.plot(EDSS, np.poly1d(slopeRight)(EDSS), color=rightStanceColor, linewidth=2)
                
            #     xFLipVect = np.concatenate([xVect, np.flip(xVect)])
            #     patchVect = np.concatenate([np.array(KPIparams['strideLengthWrtEDSS']['mean']) + np.array(KPIparams['strideLengthWrtEDSS']['std']),
            #                                 np.flip(np.array(KPIparams['strideLengthWrtEDSS']['mean']) - np.array(KPIparams['strideLengthWrtEDSS']['std']))])
                
            #     ax.plot(xVect, KPIparams['strideLengthWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
            #     patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
            #     ax.set_ylim([0.4, 1.4])
            #     # ax.set_xlabel('EDSS',fontsize=10)
            #     ax.set_ylabel('Stride \n Length [m]',fontsize=10)
                
            #     if trialIdx == nrOfTrials - 1:
            #         ax.legend([trendRight[0], trendLeft[0], patchX],['trend right','trend left','Ref Std'], loc = 'center right', bbox_to_anchor=(1.3, 0.5))

                
                
            elif KPIidx == 3: # STANCE PERCENTAGE
                xVect = np.unique(EDSS)
                ax.scatter(EDSS, interSubject['stanceLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=1.2)
                ax.scatter(EDSS, interSubject['stanceRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=1.2)
                
                
                # slopeRight = np.polyfit(EDSS, interSubject['stanceRight'], 1)
                # trendRight = ax.plot(EDSS, np.poly1d(slopeRight)(EDSS), color=rightStanceColor, linewidth=1.2)
                
                # slopeLeft = np.polyfit(EDSS, interSubject['stanceLeft'], 1)
                # trendLeft = ax.plot(EDSS, np.poly1d(slopeLeft)(EDSS), color=leftStanceColor, linewidth=1.2)
                
                slopeRight, interceptRight, r_valueRight, p_valueRight, std_errRight = stats.linregress(EDSS, interSubject['stanceRight'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_lineRight = slopeRight * EDSS_dense + interceptRight
                ax.plot(EDSS_dense, regression_lineRight, color=rightStanceColor, linewidth=1.2)
                
                slopeLeft, interceptLeft, r_valueLeft, p_valueLeft, std_errLeft = stats.linregress(EDSS, interSubject['stanceLeft'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_lineLeft = slopeLeft * EDSS_dense + interceptLeft
                ax.plot(EDSS_dense, regression_lineLeft, color=leftStanceColor, linewidth=1.2)
                
                
                xFLipVect = np.concatenate([xVect, np.flip(xVect)])
                patchVect = np.concatenate([np.array(KPIparams['stancePercentageWrtEDSS']['mean']) + np.array(KPIparams['stancePercentageWrtEDSS']['std']),
                                            np.flip(np.array(KPIparams['stancePercentageWrtEDSS']['mean']) - np.array(KPIparams['stancePercentageWrtEDSS']['std']))])
                
                ax.plot(xVect, KPIparams['stancePercentageWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
                patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
                plot_refSTance = ax.plot([0,7], [soa['stancePercentageRefValue']] * 2, '--k', linewidth=1)
                
                ax.set_ylim([58, 75])
                
                if trialIdx == 0:
                    ax.set_title('Stance phase \n [% duration]',fontsize=10)
                    ax.legend([trendRight[0], trendLeft[0], plot_refSTance[0], patchX],['trend right','trend left','Ref Control','Ref Std'], loc = 'upper right', bbox_to_anchor=(1.5, 2.6))
                    
                if trialIdx == nrOfTrials - 1:
                    ax.set_xlabel('EDSS',fontsize=10)
                
                
            elif KPIidx == 4: # SWING PERCENTAGE
                xVect = np.unique(EDSS)
                ax.scatter(EDSS, interSubject['swingLeft'], s=30, facecolor='none', edgecolor=leftStanceColor, linewidth=1.2)
                ax.scatter(EDSS, interSubject['swingRight'], s=30, facecolor='none', edgecolor=rightStanceColor, linewidth=1.2)
                
                # slopeRight = np.polyfit(EDSS, interSubject['swingRight'], 1)
                # trendRight = ax.plot(EDSS, np.poly1d(slopeRight)(EDSS), color=rightStanceColor, linewidth=1.2)
                
                # slopeLeft = np.polyfit(EDSS, interSubject['swingLeft'], 1)
                # trendLeft = ax.plot(EDSS, np.poly1d(slopeLeft)(EDSS), color=leftStanceColor, linewidth=1.2)
                
                slopeRight, interceptRight, r_valueRight, p_valueRight, std_errRight = stats.linregress(EDSS, interSubject['swingRight'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_lineRight = slopeRight * EDSS_dense + interceptRight
                ax.plot(EDSS_dense, regression_lineRight, color=rightStanceColor, linewidth=1.2)
                
                slopeLeft, interceptLeft, r_valueLeft, p_valueLeft, std_errLeft = stats.linregress(EDSS, interSubject['swingLeft'])
                EDSS_dense = np.linspace(np.min(EDSS), np.max(EDSS), 100)
                regression_lineLeft = slopeLeft * EDSS_dense + interceptLeft
                ax.plot(EDSS_dense, regression_lineLeft, color=leftStanceColor, linewidth=1.2)
                
                
                xFLipVect = np.concatenate([xVect, np.flip(xVect)])
                patchVect = np.concatenate([np.array(KPIparams['swingPercentageWrtEDSS']['mean']) + np.array(KPIparams['swingPercentageWrtEDSS']['std']),
                                            np.flip(np.array(KPIparams['swingPercentageWrtEDSS']['mean']) - np.array(KPIparams['swingPercentageWrtEDSS']['std']))])
                
                ax.plot(xVect, KPIparams['swingPercentageWrtEDSS']['mean'], color=soaKPIstdColor, linewidth=0.3)
                
                patchX = ax.fill_between(xFLipVect, patchVect, color=soaKPIstdColor, alpha=transparencyStd)
                
                plot_refSwing = ax.plot([0,7], [soa['swingPercentageRefValue']] * 2, '--k', linewidth=1)
                
                ax.set_ylim([25, 41])
                
                if trialIdx == nrOfTrials - 1:
                    ax.set_xlabel('EDSS',fontsize=10)
                
                if trialIdx == 0:
                    ax.set_title('Swing phase\n [% duration]',fontsize=10)
                
            
            
            ax.set_xlim([1, 7])
        
    

    """t-test"""
    if ttest:
        # Define values for stats analysis
        FgenderIdx = np.where(np.array(gender) == 'F')[0]
        MgenderIdx = np.where(np.array(gender) == 'M')[0]
        lefDominanceIdx = np.where(np.array(dominance) == 'Left')[0]
        rightDominanceIdx = np.where(np.array(dominance) == 'Right')[0]
        EDSS2Idx = np.where(np.array(EDSS) == 2)[0]
        EDSS4Idx = np.where(np.array(EDSS) == 4)[0]
        EDSS6Idx = np.where(np.array(EDSS) == 6)[0]
        
        nrOfStats = 3
        fig,axs = plt.subplots(nrows=nrOfTrials, ncols=nrOfStats, figsize=(15, 20))
        # fig.suptitle('Statistics', fontsize=10)
        fig.patch.set_facecolor('white')
        
        fig.subplots_adjust(wspace=0.5, hspace=1.4)
        
        interSubjAnalysis = {}
        
        for trialIdx in range(nrOfTrials):
            TRIAL_ID = trialList[trialIdx]
            interSubjectTotalGaitSteps = []
            
            for subjIdx in range(nrOfSubjects):
                SUBJECT_ID = subjectList[subjIdx]
                tmpStruct = analysisResults[SUBJECT_ID][f'Trial{TRIAL_ID:02d}']
                interSubjectTotalGaitSteps.append(tmpStruct['KPI']['totalGaitSteps'])
            
            interSubjectTotalGaitSteps = np.array(interSubjectTotalGaitSteps)
            
            # Steps vs gender
            genderF = interSubjectTotalGaitSteps[FgenderIdx]
            genderM = interSubjectTotalGaitSteps[MgenderIdx]
            h_gender, p_gender = ttest2(genderF, genderM)
            # h_gender, p_gender = ttest_ind(genderF, genderM, equal_var=False)
            
            
            # Steps vs dominance
            dominanceLeft = interSubjectTotalGaitSteps[lefDominanceIdx]
            dominanceRight = interSubjectTotalGaitSteps[rightDominanceIdx]
            h_dominance, p_dominance = ttest2(dominanceLeft, dominanceRight)
            # h_dominance , p_dominance = ttest_ind(dominanceLeft, dominanceRight, equal_var=False)
            
            # Steps vs EDSS
            EDSS2 = interSubjectTotalGaitSteps[EDSS2Idx]
            EDSS4 = interSubjectTotalGaitSteps[EDSS4Idx]
            EDSS6 = interSubjectTotalGaitSteps[EDSS6Idx]
            h_EDSS, p_EDSS = ttest2(EDSS2, EDSS4)
            # h_EDSS, p_EDSS = ttest_ind(EDSS2, EDSS4, equal_var=False)

            
            interSubjAnalysis[f'Trial{TRIAL_ID:02d}'] = {
                'stats' : {
                    'stepsVsGender': { 'h': h_gender, 'p': p_gender },
                    'stepsVsDominance': { 'h': h_dominance, 'p': p_dominance },
                    'stepsVsEDSS': { 'h': h_EDSS, 'p': p_EDSS }
                }
            }
            
            # Plot boxplots
            for statIdx in range(nrOfStats):
                ax = axs[trialIdx, statIdx] if nrOfTrials > 1 else axs[statIdx]
                ax.grid(True)  
                
                if statIdx == 0: # STEPS ALL
                    ax.boxplot(interSubjectTotalGaitSteps)
                    ax.set_xticks([])
                    if trialIdx == 0:
                        ax.set_title(f'Steps', fontsize = 10)
                    ax.set_ylabel(f'Trial {TRIAL_ID:02d}', fontsize=10, color='black', fontweight='bold')
                elif statIdx == 1: # STEPS vs GENDER
                    ax.boxplot([genderF, genderM], tick_labels=['F', 'M'])
                    if trialIdx == 0:
                        ax.set_title(f'Steps vs Gender', fontsize = 10)
                elif statIdx == 2: # STEPS vs EDSS
                    ax.boxplot([EDSS2, EDSS4, EDSS6], tick_labels=['2', '4', '6'])
                    if trialIdx == 0:
                        ax.set_title(f'Steps vs EDSS', fontsize = 10)
                             

    plt.tight_layout() 

    plt.show()


def ttest2(group1, group2):
    """
    Perform a t-test similar to MATLAB's `ttest2` with 'equal' variance assumption.
    """
    # Sample sizes
    n1, n2 = len(group1), len(group2)
    
    # Means and variances
    mean1, mean2 = np.mean(group1), np.mean(group2)
    if n1 > 1 and n2 > 1:
        var1, var2 = np.var(group1, ddof=1), np.var(group2, ddof=1)
    elif n1 == 1:
        var1 = 0
        if n2 == 1:
            var2 = 0
        else:
            var2 = np.var(group2, ddof=1)

        
    
    # Pooled variance
    pooled_var = ((n1 - 1) * var1 + (n2 - 1) * var2) / (n1 + n2 - 2)
    pooled_std = np.sqrt(pooled_var * (1/n1 + 1/n2))
    
    # T statistic
    t_stat = (mean1 - mean2) / pooled_std
    
    # Degrees of freedom
    df = n1 + n2 - 2
    
    # p-value
    p_value = 2 * stats.t.sf(np.abs(t_stat), df)
    
    return t_stat, p_value
    
    
if __name__ == '__main__':
    main()