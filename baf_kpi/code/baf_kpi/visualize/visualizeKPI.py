import numpy as np
from pathlib import Path
import os
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import idyntree.bindings as iDynTree
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

from baf_kpi.scripts.computeVectorNormalization import computeVectorNormalization
from baf_kpi.visualize.visualizeLegSkeleton import visualizeLegSkeleton

def visualizeKPI(KPI, unifiedTimestamp, DATA_LENGTH, plotKPIs, paths, saveOn, legSkeletonViz, kinDynComp, joints_state, human_state, W_T_B, node1, node2):
    
    
    pathToPlots = Path(paths["pathToPlots"])

    if not os.path.exists(pathToPlots):
        os.makedirs(pathToPlots)

    # Colors
    rightStanceColor = (0, 0.4470, 0.7410)
    rightSwingColor = (0.3010, 0.7450, 0.9330)
    leftStanceColor = (1, 0, 0)
    leftSwingColor = (0.97, 0.55, 0.55)
    doubleSupportColor = (0.901960784313726, 0.901960784313726, 0.901960784313726)
    singleSupportColor_Left = (1, 0.800000011920929, 0.800000011920929)
    singleSupportColor_Right = (0.87058824300766, 0.921568632125854, 0.980392158031464)

    transparencyStd = 0.3  # Transparency for shaded std area

    # Define variables 
    ABS_UNIFIED_TIMESTAMP = unifiedTimestamp - unifiedTimestamp[0]
    TIME_VECT = np.arange(1,102)
    TIME_FLIP_VECT = np.concatenate((TIME_VECT, np.flip(TIME_VECT)))
    SUBJECT_WEIGHT = np.mean(node1['FT']['data'][:,:,2] + node2['FT']['data'][:,:,2])/9.81 # [kg]
    
    tmp = {}
    shoes = {}
    if plotKPIs == True:
        """PLOT: CONTACT GAIT PATTERN"""
        # Mean force computation
        shoes['meanValue'] = np.mean(node1['FT']['data'][:,:,2] + node2['FT']['data'][:,:,2])/2
        tmp['meanVect'] = np.zeros((1, KPI['len']))
        tmp['meanVect'][0,:] = shoes['meanValue']
        
        # Range limits for plot
        tmp['maxLeftShoe'] = np.max(node1['FT']['data'][:,:,2])
        tmp['minLeftShoe'] = np.min(node1['FT']['data'][:,:,2])
        tmp['maxRightShoe'] = np.max(node2['FT']['data'][:,:,2])
        tmp['minRightShoe'] = np.min(node2['FT']['data'][:,:,2])
        
        # Force percentage wrt vertical force
        tmp['fx_min_left'] = np.min(node1['FT']['data'][:,:,0])
        tmp['fx_max_left'] = np.max(node1['FT']['data'][:,:,0])
        tmp['fy_min_left'] = np.min(node1['FT']['data'][:,:,1])
        tmp['fy_max_left'] = np.max(node1['FT']['data'][:,:,1])
        tmp['fz_min_left'] = np.min(node1['FT']['data'][:,:,2])
        tmp['fz_max_left'] = np.max(node1['FT']['data'][:,:,2])
        tmp['fx_magnitude_left'] = np.linalg.norm(tmp['fx_min_left']- tmp['fx_max_left'])
        tmp['fy_magnitude_left'] = np.linalg.norm(tmp['fy_min_left']- tmp['fy_max_left'])  
        tmp['fz_magnitude_left'] = np.linalg.norm(tmp['fz_min_left']- tmp['fz_max_left'])
        
        tmp['fx_min_right'] = np.min(node2['FT']['data'][:,:,0])
        tmp['fx_max_right'] = np.max(node2['FT']['data'][:,:,0])
        tmp['fy_min_right'] = np.min(node2['FT']['data'][:,:,1])
        tmp['fy_max_right'] = np.max(node2['FT']['data'][:,:,1])
        tmp['fz_min_right'] = np.min(node2['FT']['data'][:,:,2])
        tmp['fz_max_right'] = np.max(node2['FT']['data'][:,:,2])
        tmp['fx_magnitude_right'] = np.linalg.norm(tmp['fx_min_right']- tmp['fx_max_right'])
        tmp['fy_magnitude_right'] = np.linalg.norm(tmp['fy_min_right']- tmp['fy_max_right'])
        tmp['fz_magnitude_right'] = np.linalg.norm(tmp['fz_min_right']- tmp['fz_max_right'])
        
        
        percentage_AP_left = round(100 * tmp['fx_magnitude_left'] / tmp['fz_magnitude_left'])
        percentage_ML_left = round(100 * tmp['fy_magnitude_left'] / tmp['fz_magnitude_left'])
        percentage_AP_right = round(100 * tmp['fx_magnitude_right'] / tmp['fz_magnitude_right'])
        percentage_ML_right = round(100 * tmp['fy_magnitude_right'] / tmp['fz_magnitude_right'])
        percentageForces = [[percentage_AP_left, percentage_AP_right],[ percentage_ML_left, percentage_ML_right]]
        
        fig, axes = plt.subplots(3, 1, figsize=(8, 8))
        fig.suptitle(f'Contact Gait Pattern', fontsize=16)
        fig.patch.set_facecolor('white')
        plt.subplots_adjust(hspace=0.5)
        titleLabels = ['Antero-posterior (AP)', 'Medial-Lateral (ML)', 'Vertical (V)']
        yLabels = ['Fx', 'Fy', 'Fz']
        
        for plotIdx in range(3):
            ax = axes[plotIdx]
            
            y1 = min(np.min(node1['FT']['data'][:,:,plotIdx]), 
                  np.min(node2['FT']['data'][:,:,plotIdx]))
            y2 = max(np.max(node1['FT']['data'][:,:,plotIdx]), 
                  np.max(node2['FT']['data'][:,:,plotIdx]))
            
            for shadedIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
                # DS1Left
                x1 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['ds1'][shadedIdx]['idxAtTheBeginning']]
                x2 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['ds1'][shadedIdx]['idxAtTheEnd']]
                shadedPatch1 = ax.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=doubleSupportColor)
                # DS2Left
                x1 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['ds2'][shadedIdx]['idxAtTheBeginning']]
                x2 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['ds2'][shadedIdx]['idxAtTheEnd']]
                shadedPatch1 = ax.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=doubleSupportColor)
                # SSLeft
                flag_SSleft = True
                x1 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['ss'][shadedIdx]['idxAtTheBeginning']]
                x2 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['ss'][shadedIdx]['idxAtTheEnd']]
                shadedPatch2 = ax.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=singleSupportColor_Left)
            
            # SSRight
            for shadedIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
                flag_SSright = True
                x1 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Right']['ss'][shadedIdx]['idxAtTheBeginning']]
                x2 = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Right']['ss'][shadedIdx]['idxAtTheEnd']]
                shadedPatch3 = ax.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=singleSupportColor_Right)
                
            # PLOT
            plot1 = ax.plot(ABS_UNIFIED_TIMESTAMP, node1['FT']['data'][:,:,plotIdx], 'r')
            plot2 = ax.plot(ABS_UNIFIED_TIMESTAMP, node2['FT']['data'][:,:,plotIdx], 'b')
            
            if plotIdx == 2:
                plotMean = ax.plot(ABS_UNIFIED_TIMESTAMP.reshape(-1), tmp['meanVect'].reshape(-1), 'k--')
                
            ax.set_title(titleLabels[plotIdx], fontsize=16)
            ax.set_ylabel(f'{yLabels[plotIdx]} [N]', fontsize=14)
            
            if plotIdx == 2:
                ax.set_xlabel('Time [s]', fontsize=16)
                
            ax.grid(True)            
            
            # LEGEND
            if plotIdx != 2:
                if flag_SSleft:  # DS + leftSS
                    leg = ax.legend([plot1[0], plot2[0], shadedPatch1[0], shadedPatch2[0]],
                                    [f'Left, {int(percentageForces[plotIdx][0])}% of V', 
                                    f'Right, {int(percentageForces[plotIdx][1])}% of V', 
                                    'doubleSupport', 'LeftSupport'], 
                                    loc='upper right', fontsize=10)
                if flag_SSright:  # DS + rightSS
                    leg = ax.legend([plot1[0], plot2[0], shadedPatch1[0], shadedPatch3[0]],
                                    [f'Left, {int(percentageForces[plotIdx][0])}% of V', 
                                    f'Right, {int(percentageForces[plotIdx][1])}% of V', 
                                    'doubleSupport', 'RightSupport'], 
                                    loc='upper right', fontsize=10)
                if flag_SSleft and flag_SSright:  # DS + leftSS + rightSS
                    leg = ax.legend([plot1[0], plot2[0], shadedPatch1[0], shadedPatch2[0], shadedPatch3[0]],
                                    [f'Left, {int(percentageForces[plotIdx][0])}% of V', 
                                    f'Right, {int(percentageForces[plotIdx][1])}% of V', 
                                    'doubleSupport', 'LeftSupport', 'RightSupport'], 
                                    loc='upper right', fontsize=10)
            else:
                if flag_SSleft:  # DS + leftSS
                    leg = ax.legend([plot1[0], plot2[0], plotMean[0], shadedPatch1[0], shadedPatch2[0]],
                                    ['Left', 'Right', 'Mean', 'doubleSupport', 'LeftSupport'], 
                                    loc='upper right', fontsize=10)
                if flag_SSright:  # DS + rightSS
                    leg = ax.legend([plot1[0], plot2[0], plotMean[0], shadedPatch1[0], shadedPatch3[0]],
                                    ['Left', 'Right', 'Mean', 'doubleSupport', 'RightSupport'], 
                                    loc='upper right', fontsize=10)
                if flag_SSleft and flag_SSright:  # DS + leftSS + rightSS
                    leg = ax.legend([plot1[0], plot2[0], plotMean[0], shadedPatch1[0], shadedPatch2[0], shadedPatch3[0]],
                                    ['Left', 'Right', 'Mean', 'doubleSupport', 'LeftSupport', 'RightSupport'], 
                                    loc='upper right', fontsize=10)
            plt.axis('tight')
            ax.set_xlim(0, ABS_UNIFIED_TIMESTAMP[-1])

        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'contactPatternForces.pdf'), dpi=600, format='pdf')
        
        
        
        """PLOT: NORMALIZED VERTICAL FORCE IN STANCE"""
        KPI['contactPattern']['forceInStance'] = {}
        # Left
        totalSamples = KPI['gaitCycleFeatures']['Left']['stance'][-1]['idxAtTheEnd']
        KPI['contactPattern']['forceInStance']['effectiveforces_Left'] = np.zeros(len(node1['FT']['data'][:, 0, 2]), totalSamples)
        KPI['contactPattern']['forceInStance']['Left'] = []
        
        for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
            initial = KPI['gaitCycleFeatures']['Left']['stance'][nrCycleIdx]['idxAtTheBeginning']
            final = KPI['gaitCycleFeatures']['Left']['stance'][nrCycleIdx]['idxAtTheEnd']
            # Whole vector
            KPI['contactPattern']['forceInStance']['effectiveforces_Left'][initial:final+1] = node1['FT']['data'][initial:final+1, 0, 2]
            # Expressed per each cycle
            KPI['contactPattern']['forceInStance']['Left'].append(node1['FT']['data'][initial:final+1, 0, 2])
            
        # Right
        totalSamples = KPI['gaitCycleFeatures']['Right']['stance'][-1]['idxAtTheEnd']
        KPI['contactPattern']['forceInStance']['effectiveforces_Right'] = np.zeros(len(node2['FT']['data'][:, 0, 2]), totalSamples)
        KPI['contactPattern']['forceInStance']['Right'] = []
        
        for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
            initial = KPI['gaitCycleFeatures']['Right']['stance'][nrCycleIdx]['idxAtTheBeginning']
            final = KPI['gaitCycleFeatures']['Right']['stance'][nrCycleIdx]['idxAtTheEnd']
            # Whole vector
            KPI['contactPattern']['forceInStance']['effectiveforces_Right'][initial:final+1] = node2['FT']['data'][initial:final+1, 0, 2]
            # Expressed per each cycle
            KPI['contactPattern']['forceInStance']['Right'].append(node2['FT']['data'][initial:final+1, 0, 2])
            
        # Normalize all the cycles
        # Left
        LeftNormalizedFz = computeVectorNormalization(KPI['contactPattern']['forceInStance']['Left'],101) # 101 samples for being in 100% in stance phase
        LeftNormalizedFz = ((LeftNormalizedFz/9.81)/SUBJECT_WEIGHT)*100
        
        # Right
        RightNormalizedFz = computeVectorNormalization(KPI['contactPattern']['forceInStance']['Right'],101)
        RightNormalizedFz = ((RightNormalizedFz/9.81)/SUBJECT_WEIGHT)*100
        
        # Compute mean and standard deviation
        # Left
        LeftMean = np.mean(LeftNormalizedFz, axis=0)
        LeftStd = np.std(LeftNormalizedFz, axis=0)
        # Right
        RightMean = np.mean(RightNormalizedFz, axis=0)
        RightStd = np.std(RightNormalizedFz, axis=0)
        
        # Plot
        fig, (ax1,ax2) = plt.subplots(1,2,figsize=(8,8))
        fig.suptitle('Normalized Vertical Force in Stance stance w.r.t. bodyweight', fontsize=16)
        
        # LEFT
        errorPlot = 'std'
        if errorPlot == 'std':
            errorPlotValues = LeftStd
        elif errorPlot == 'sem':
            errorPlotValues = LeftStd/np.sqrt(len(LeftNormalizedFz))
            
        # PLOT
        patchVect = np.concatenate((LeftMean+errorPlotValues, np.flip(LeftMean-errorPlotValues)))
        ax1.plot(TIME_VECT, LeftMean, color=leftStanceColor, linewidth=2)
        ax1.fill(TIME_FLIP_VECT, patchVect,color=leftStanceColor, alpha=transparencyStd)
        ax1.grid(True)
        ax1.set_ylim([0, 130])
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.set_aspect('equal', 'box')
        # TITLE
        ax1.set_title('Left')
        # LABELS
        ax1.set_ylabel('Body weight [%]')
        ax1.set_xlabel('Stance [%]')
        ax1.tick_params(labelsize=15)
        # LEGEND
        ax1.legend(['Mean','std'], loc='upper right', fontsize=10)
        
        # RIGHT
        errorPlot = 'std'
        if errorPlot == 'std':
            errorPlotValues = RightStd
        elif errorPlot == 'sem':
            errorPlotValues = RightStd/np.sqrt(len(RightNormalizedFz))
        
        patchVect = np.concatenate((RightMean+errorPlotValues, np.flip(RightMean-errorPlotValues)))
        
        ax2.plot(TIME_VECT, RightMean, color=rightStanceColor, linewidth=2)
        ax2.fill(TIME_FLIP_VECT, patchVect,color=rightStanceColor,alpha=transparencyStd)
        ax2.grid(True)
        ax2.set_ylim([0, 130])
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.set_aspect('equal', 'box')
        # TITLE
        ax2.set_title('Right')
        # LABELS
        ax2.set_ylabel('Body weight [%]')
        ax2.set_xlabel('Stance [%]')
        ax2.tick_params(labelsize=15)
        # LEGEND
        ax2.legend(['Mean','std'], loc='upper right', fontsize=10)
        
        
        
        """PLOT: COM"""
        fig = plt.figure(figsize=(8,8))
        ax = fig.add_subplot(111, projection='3d')
        fig.suptitle('COM displacement w.r.t. feet and hands', fontsize=16)
        ax.grid(True)
        
        # COM position wrt feet and hands positions
        s = iDynTree.JointPosDoubleArray(kinDynComp.model())
        ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
        base_vel = iDynTree.Twist()
        gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])

        position = {'RF': np.zeros((3,KPI['len'])), 'LF': np.zeros((3,KPI['len'])), 'RH': np.zeros((3,KPI['len'])), 'LH': np.zeros((3,KPI['len']))}
        
        for lenIdx in range(KPI['len']):
            # Set the joint positions
            s = joints_state['positions']['data'][lenIdx][0]
            ds = joints_state['velocities']['data'][lenIdx][0]
            lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
            ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
            base_vel = np.concatenate((lin_vel, ang_vel)) 
            kinDynComp.setRobotState(W_T_B[lenIdx], s, base_vel, ds, gravity)
            
            # Computation of feet position
            position['RF'][:,lenIdx] = kinDynComp.getWorldTransform('RightFoot').getPosition().toNumPy()
            position['LF'][:,lenIdx] = kinDynComp.getWorldTransform('LeftFoot').getPosition().toNumPy()
            # Computation of hands position
            position['RH'][:,lenIdx] = kinDynComp.getWorldTransform('RightHand').getPosition().toNumPy()
            position['LH'][:,lenIdx] = kinDynComp.getWorldTransform('LeftHand').getPosition().toNumPy()            
        
        # PLOT
        plotCoMpos, = ax.plot(KPI['COM']['position'][0, 1:], KPI['COM']['position'][1, 1:], KPI['COM']['position'][2, 1:], 'k', linewidth=1.5, label='COM')
        plot_rf_pos, = ax.plot(position['RF'][0, 1:], position['RF'][1, 1:], position['RF'][2, 1:], 'b', linewidth=1.5, label='Right foot')
        plot_lf_pos, = ax.plot(position['LF'][0, 1:], position['LF'][1, 1:], position['LF'][2, 1:], 'r', linewidth=1.5, label='Left foot')
        plot_rh_pos, = ax.plot(position['RH'][0, 1:], position['RH'][1, 1:], position['RH'][2, 1:], 'm', linewidth=1.5, label='Right hand')
        plot_lh_pos, = ax.plot(position['LH'][0, 1:], position['LH'][1, 1:], position['LH'][2, 1:], color=[1.0, 0.533, 0.0], linewidth=1.5, label='Left hand')

        # LABELS
        ax.set_xlabel('x [m]', fontsize=14)
        ax.set_ylabel('y [m]', fontsize=14)
        ax.set_zlabel('z [m]', fontsize=14)
        
        # LEGEND
        ax.legend(loc='upper right', fontsize=12)
        
        
        
        """COM velocity"""
        vel_yLabels = ['Vx', 'Vy', 'Vz']
        
        fig, axs = plt.subplots(1, 3, figsize=(8, 8))

        for plotIdx in range(3):  # 3 componenti di velocità
            ax = axs[plotIdx]
            ax.plot(ABS_UNIFIED_TIMESTAMP, KPI['COM']['velocity'][plotIdx, :], 'k', linewidth=2)
            ax.grid(True)
            if plotIdx == 1:
                ax.set_title('COM velocity', fontsize=16)
                
            # LABELS
            ax.set_xlabel('Time [s]', fontsize=14)
            ax.set_ylabel(f'{vel_yLabels[plotIdx]} [m/s]', fontsize=14)
            ax.tick_params(axis='both', labelsize=15)

            ax.autoscale(enable=True, axis='x', tight=True)

        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'COM_velocity.pdf'), dpi=600, format='pdf')
        
        
        """PLOT: COP vs COM"""
        fig,ax = plt.subplots(figsize=(8, 8))

        fig.suptitle('COP vs COM expressed w.r.t. world', fontsize=16)
        ax.grid(True)
        
        # PLOT
        plotCOMpos = ax.plot(KPI['COM']['position'][0, 1:], KPI['COM']['position'][1, 1:], 'k', linewidth=1.5, label='COM')
        plotCOPleft = ax.plot(KPI['COP']['total']['Left']['wrtWorld'][0, :], KPI['COP']['total']['Left']['wrtWorld'][1, :], 'or', linewidth=1.5, label='COP Left')
        plotCOPright = ax.plot(KPI['COP']['total']['Right']['wrtWorld'][0, :], KPI['COP']['total']['Right']['wrtWorld'][1, :], 'ob', linewidth=1.5, label='COP Right')
        
        # LABELS
        ax.set_xlabel('x [m]', fontsize=14)
        ax.set_ylabel('y [m]', fontsize=14)
        
        ax.legend(loc='upper right', fontsize=10)
        
        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'COPvsCOM.pdf'), dpi=600, format='pdf')
        
        
        
        """PLOT: STRIDE LENGTH"""
        fig, axs = plt.subplots(1, 3, figsize=(8, 8))
        fig.suptitle('Stride length', fontsize=16)
        ax = axs[0]
        # Left PLOT
        distanceValuesL = [KPI['strideLength']['Left'][i]['distance'] for i in range(len(KPI['strideLength']['Left']))]
        distancePlotL = ax.bar(range(len(distanceValuesL)), distanceValuesL, color=leftStanceColor, width=0.5, edgecolor='none')
                
        ax.set_ylim([0, 1.5])
        ax.grid(True)
        ax.set_title('Left')
        ax.set_xlabel('Cycle')
        ax.set_ylabel('Stride length [m]')
        ax.autoscale(enable=True, axis='x', tight=True)
        
        ax = axs[1]
        # Right PLOT
        distanceValuesR = [KPI['strideLength']['Right'][i]['distance'] for i in range(len(KPI['strideLength']['Right']))]
        distancePlotR = ax.bar(range(len(distanceValuesR)), distanceValuesR, color=rightStanceColor, width=0.5, edgecolor='none')

        ax.set_ylim([0, 1.5])
        ax.grid(True)
        ax.set_title('Right')
        ax.set_xlabel('Cycle')
        ax.set_ylabel('Stride length [m]')
        ax.autoscale(enable=True, axis='x', tight=True)
        
        ax = axs[2]
        # PLOT
        totalDistanceL = np.sum(distanceValuesL)
        totalDistanceR = np.sum(distanceValuesR)
        totalDistance = np.array([totalDistanceL, totalDistanceR])
        totalDistanceChart = plt.pie(totalDistance, labels=['Left', 'Right'], autopct='%1.1f%%', colors=[leftStanceColor, rightStanceColor])
        ax.set_title('Stride length %')
        ax.autoscale(enable=True, axis='x', tight=True)
        # LABELS
        labels = ['Left', 'Right']
        ax.legend(totalDistanceChart[0], labels, loc='upper right', fontsize=10)
        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'strideLength.pdf'), dpi=600, format='pdf')
        
        
        
        """PLOT: GAIT CYCLE PHASES"""
        fig, axs = plt.subplots(1, 4, figsize=(8, 8))
        
        bar_width = 0.4  
        
        # PLOT Left
        ax = axs[0]
        percStanceValuesL = [KPI['gaitCycleFeatures']['Left']['stance'][i]['percStance'] for i in range(len(KPI['gaitCycleFeatures']['Left']['stance']))]
        percSwingValuesL = [KPI['gaitCycleFeatures']['Left']['swing'][i]['percSwing'] for i in range(len(KPI['gaitCycleFeatures']['Left']['swing']))]
        # percentagePlotL1 = ax.bar(range(len(percStanceValuesL)), percStanceValuesL, color=leftStanceColor, width=0.5, edgecolor='none')
        # percentagePlotL2 = ax.bar(range(len(percSwingValuesL)), percSwingValuesL, bottom=percStanceValuesL, color=leftSwingColor, width=0.5, edgecolor='none')
        x_positionsL = range(len(percStanceValuesL))
        percentagePlotL1 = ax.bar([x - bar_width/2 for x in x_positionsL], percStanceValuesL, width=bar_width, color=leftStanceColor, edgecolor='none', label='Stance')
        percentagePlotL2 = ax.bar([x + bar_width/2 for x in x_positionsL], percSwingValuesL, width=bar_width, color=leftSwingColor, edgecolor='none', label='Swing')

        ax.set_ylim([0, 100])
        ax.grid(True)
        ax.set_title('Left')
        ax.set_xlabel('Cycle')
        ax.set_ylabel('Stride %')
        ax.legend(['Stance', 'Swing'], loc='upper right', fontsize=10)
        ax.autoscale(enable=True, axis='x', tight=True)
        
        # PLOT Right
        ax = axs[1]
        percStanceValuesR = [KPI['gaitCycleFeatures']['Right']['stance'][i]['percStance'] for i in range(len(KPI['gaitCycleFeatures']['Right']['stance']))]
        percSwingValuesR = [KPI['gaitCycleFeatures']['Right']['swing'][i]['percSwing'] for i in range(len(KPI['gaitCycleFeatures']['Right']['swing']))]
        # percentagePlotR1 = ax.bar(range(len(percStanceValuesR)), percStanceValuesR, color=rightStanceColor, width=0.5, edgecolor='none')
        # percentagePlotR2 = ax.bar(range(len(percSwingValuesR)), percSwingValuesR, bottom=percStanceValuesR, color=rightSwingColor, width=0.5, edgecolor='none')
        x_positionsR = range(len(percStanceValuesR))
        percentagePlotR1 = ax.bar([x - bar_width/2 for x in x_positionsR], percStanceValuesR, width=bar_width, color=rightStanceColor, edgecolor='none', label='Stance')
        percentagePlotR2 = ax.bar([x + bar_width/2 for x in x_positionsR], percSwingValuesR, width=bar_width, color=rightSwingColor, edgecolor='none', label='Swing')       
        ax.set_ylim([0, 100])
        ax.grid(True)
        ax.set_title('Right')
        ax.set_xlabel('Cycle')
        ax.set_ylabel('Stride %')
        ax.legend(['Stance', 'Swing'], loc='upper right', fontsize=10)
        ax.autoscale(enable=True, axis='x', tight=True)
        
        # PLOT
        ax = axs[2]
        percStanceL = np.sum(KPI['gaitCycleFeatures']['Left']['stance'][i]['percStance'] for i in range(len(KPI['gaitCycleFeatures']['Left']['stance'])))
        percSwingL = np.sum(KPI['gaitCycleFeatures']['Left']['swing'][i]['percSwing'] for i in range(len(KPI['gaitCycleFeatures']['Left']['swing'])))
        totalPercentageL = np.array([percStanceL, percSwingL])
        totalPercentagePlotL = ax.pie(totalPercentageL, labels=['Stance', 'Swing'], autopct='%1.1f%%', colors=[leftStanceColor, leftSwingColor])
        ax.set_title('Left Stride %')
        ax.grid(True)
        # LABELS
        labels = ['Stance', 'Swing']
        ax.legend(totalPercentagePlotL[0], labels, loc='upper right', fontsize=10)
        ax.autoscale(enable=True, axis='x', tight=True)
        
        # PLOT
        ax = axs[3]
        percStanceR = np.sum(KPI['gaitCycleFeatures']['Right']['stance'][i]['percStance'] for i in range(len(KPI['gaitCycleFeatures']['Right']['stance'])))
        percSwingR = np.sum(KPI['gaitCycleFeatures']['Right']['swing'][i]['percSwing'] for i in range(len(KPI['gaitCycleFeatures']['Right']['swing'])))
        totalPercentageR = np.array([percStanceR, percSwingR])
        totalPercentagePlotR = ax.pie(totalPercentageR, labels=['Stance', 'Swing'], autopct='%1.1f%%', colors=[rightStanceColor, rightSwingColor])
        ax.set_title('Right Stride %')
        ax.grid(True)
        # LABELS
        labels = ['Stance', 'Swing']
        ax.legend(totalPercentagePlotR[0], labels, loc='upper right', fontsize=10)   
        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'gaitCyclePhases.pdf'), dpi=600, format='pdf')
        
        
        
        """PLOT: SWING WIDTH"""
        fig,ax = plt.subplots(figsize=(8, 8))
        fig.suptitle('Swing width', fontsize=16)
        ax.grid(True)
        ax.autoscale(enable=True, axis='x', tight=True)
        
        # PLOT
        for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
            y1 = 0
            y2 = KPI['swingWidth']['Left'][nrCycleIdx]['maxWidth']
            x1 = KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['timeAtTheBeginning'] - unifiedTimestamp[0]
            x2 = KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['timeAtTheEnd'] - unifiedTimestamp[0]
            shadedPatch1 = ax.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=singleSupportColor_Left)
            
            tmp['x'] = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['idxAtTheBeginning']:KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['idxAtTheEnd']+1].flatten()
            y_data = np.abs(KPI['swingWidth']['Left'][nrCycleIdx]['distance']).flatten()
            markerline, stemlines, baseline = ax.stem(tmp['x'], y_data, linefmt='-', markerfmt='o', basefmt='', label='Left width')
            plt.setp(stemlines, 'color', leftStanceColor)
            markerline.set_markerfacecolor(leftStanceColor)
            plt.setp(markerline, 'color', leftStanceColor)
            plt.setp(baseline, 'color', leftStanceColor)
            
            
        for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
            y1 = 0
            y2 = -KPI['swingWidth']['Right'][nrCycleIdx]['maxWidth']
            x1 = KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['timeAtTheBeginning'] - unifiedTimestamp[0]
            x2 = KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['timeAtTheEnd'] - unifiedTimestamp[0]
            shadedPatch2 = ax.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=singleSupportColor_Right)
            
            tmp['x'] = ABS_UNIFIED_TIMESTAMP[KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['idxAtTheBeginning']:KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['idxAtTheEnd']+1].flatten()
            y_data = np.abs(KPI['swingWidth']['Right'][nrCycleIdx]['distance']).flatten()
            markerline2, stemlines2, baseline2 = ax.stem(tmp['x'], -y_data, linefmt='-', markerfmt='o', basefmt='', label='Right width')
            plt.setp(stemlines2, 'color', rightStanceColor)
            markerline2.set_markerfacecolor(rightStanceColor)
            plt.setp(markerline2, 'color', rightStanceColor)
            plt.setp(baseline2, 'color', rightStanceColor)
            
        # LABELS
        ax.set_xlabel('Time [s]', fontsize=14)
        ax.set_ylabel('Width [m]', fontsize=14)
        # LEGEND
        # handles, labels = ax.get_legend_handles_labels()
        # ax.legend(handles, ['Right width', 'Left width', 'Left swing', 'Right swing'], loc='upper right', fontsize=18)
        legend_elements = [
            Line2D([0], [0], color=rightStanceColor, marker='o', markersize=5, linestyle='-', label='Right width'),
            Line2D([0], [0], color=leftStanceColor, marker='o', markersize=5, linestyle='-', label='Left width'),
            Patch(facecolor=singleSupportColor_Left, edgecolor='none', label='Left swing'),
            Patch(facecolor=singleSupportColor_Right, edgecolor='none', label='Right swing')
        ]

        # Aggiunta della legenda con colori corretti
        ax.legend(handles=legend_elements, loc='upper right', fontsize=18)
        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'swingWidth.pdf'), dpi=600, format='pdf')
        
        
        """PLOT: PATH LENGTH"""
        fig, axs = plt.subplots(1, 2, figsize=(8, 8))
        fig.suptitle('Path length', fontsize=16)
        
        # PLOT: Left
        ax = axs[0]
        # percentagePlotL = ax.bar(range(len(KPI['pathLength']['Left'])), (KPI['pathLength']['Left'][i]['percentageOfStrideLength'] for i in range(len(KPI['pathLength']['Left']))), color=leftStanceColor, width=0.5, edgecolor='none')
        x_positions = range(len(KPI['pathLength']['Left']))
        heights_left = [KPI['pathLength']['Left'][i]['percentageOfStrideLength'] for i in x_positions]
        percentagePlotL = ax.bar(x_positions, heights_left, color=leftStanceColor, width=0.5, edgecolor='none')

        ax.axhline(y=100, color='k', linestyle='-', linewidth=1.0)
        ax.set_ylim([0, 180])
        ax.grid(True)
        ax.set_title('Left')
        ax.set_xlabel('Cycle')
        ax.set_ylabel('Stride length %')
        ax.autoscale(enable=True, axis='x', tight=True)
        
        # PLOT: Right
        ax = axs[1]
        # percentagePlotR = ax.bar(range(len(KPI['pathLength']['Right'])),(KPI['pathLength']['Right'][i]['percentageOfStrideLength'] for i in range(len(KPI['pathLength']['Right']))), color=rightStanceColor, width=0.5, edgecolor='none')
        x_positions = range(len(KPI['pathLength']['Right']))
        heights_right = [KPI['pathLength']['Right'][i]['percentageOfStrideLength'] for i in x_positions]
        percentagePlotR = ax.bar(x_positions, heights_right, color=rightStanceColor, width=0.5, edgecolor='none')
        
        ax.axhline(y=100, color='k', linestyle='-', linewidth=1.0)
        ax.set_ylim([0, 180])
        ax.grid(True)
        ax.set_title('Right')
        ax.set_xlabel('Cycle')
        ax.set_ylabel('Stride length %')
        ax.autoscale(enable=True, axis='x', tight=True)
        
        legend_elements = [
            Patch(color=leftStanceColor, label='Left Path Length'),
            Patch(color=rightStanceColor, label='Right Path Length')
        ]
        fig.legend(handles=legend_elements, loc='upper right', fontsize=10)
        
        
        plt.show()
        
        if saveOn:
            fig.savefig(os.path.join(pathToPlots, 'pathLength.pdf'), dpi=600, format='pdf')
        
        
        
        """VISUALIZATION: LEG SKELETON"""
        if legSkeletonViz:
            visualizeLegSkeleton(kinDynComp,KPI,DATA_LENGTH,joints_state,human_state,W_T_B)
            
        
        
        
        """PLOT: COM + COP"""            
        fig, ax = plt.subplots(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.set_ylim([-2, 1])
        ax.set_xlim([-2, 4])
        ax.set_zlim([-0.3, 2])
        
        metadata = {'title': 'COM + COP', 'xlabel': 'x [m]', 'ylabel': 'y [m]', 'zlabel': 'z [m]'}
        writer = FFMpegWriter(fps=10, metadata=metadata)
        
        if not os.path.exists(Path(paths["pathToVideos"])):
            os.makedirs(Path(paths["pathToVideos"]))
            
        pathVideo = os.path.join(paths["pathToVideos"], 'COM_COP.mp4')
        
        if saveOn:
            with writer.saving(fig, pathVideo, 100):
                for lenIdx in range(KPI['len']-1):
                    ax.plot(KPI['COM']['position'][0, lenIdx], KPI['COM']['position'][1, lenIdx], KPI['COM']['position'][2, lenIdx], '.k')
                    ax.plot(KPI['COP']['total']['Left']['wrtWorld'][0, lenIdx], KPI['COP']['total']['Left']['wrtWorld'][1, lenIdx], 'o', markersize=5, markeredgecolor='none', markerfacecolor=[1, 0, 0])
                    ax.plot(KPI['COP']['total']['Right']['wrtWorld'][0, lenIdx], KPI['COP']['total']['Right']['wrtWorld'][1, lenIdx], 'o', markersize=5, markeredgecolor='none', markerfacecolor=[0, 0, 1])
                
                    writer.grab_frame()
                    plt.draw()
                    plt.pause(0.1)
        else:
            for lenIdx in range(KPI['len'] - 1):
                ax.plot(KPI['COM']['position'][0, lenIdx], KPI['COM']['position'][1, lenIdx], KPI['COM']['position'][2, lenIdx], '.k')
                ax.plot(KPI['COP']['total']['Left']['wrtWorld'][0, lenIdx], KPI['COP']['total']['Left']['wrtWorld'][1, lenIdx], 'o', markersize=5, markeredgecolor='none', markerfacecolor=[1, 0, 0])
                ax.plot(KPI['COP']['total']['Right']['wrtWorld'][0, lenIdx], KPI['COP']['total']['Right']['wrtWorld'][1, lenIdx], 'o', markersize=5, markeredgecolor='none', markerfacecolor=[0, 0, 1])
                
                plt.draw()
                plt.pause(0.1)

        plt.show()            
 
    return 0