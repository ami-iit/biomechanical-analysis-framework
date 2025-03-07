import numpy as np
import os
import re
import idyntree.bindings as iDynTree
from joblib import dump
from pathlib import Path
import argparse
import json
import tkinter as tk
from tkinter import filedialog
from idyntree.visualize import MeshcatVisualizer
import resolve_robotics_uri_py
import pandas as pd




from baf_kpi.visualize.visualizeModel import visualizeModel
from baf_kpi.scripts.setPaths import setPaths
from baf_kpi.scripts.filterData import filterData
from baf_kpi.scripts.getSensorToLinkMap import getSensorToLinkMap
from baf_kpi.scripts.cell2iDynTreeStringVector import cell2iDynTreeStringVector
from baf_kpi.dataLoader.dataLoaderBAF import loadDataBAF
from baf_kpi.dataLoader.dataLoaderFilteredBAF import loadDataFilteredBAF
from baf_kpi.dataLoader.dataLoaderHDE import loadDataHDE
from baf_kpi.scripts.contactPatternDetection import detectContactPattern
from baf_kpi.scripts.adaptiveContactPatternDetection import adaptiveDetectContactPattern
from baf_kpi.scripts.computeStepSelection import computeStepSelection
from baf_kpi.scripts.computeCycleDuration import computeCycleDuration
from baf_kpi.scripts.computeStrideLength import computeStrideLength
from baf_kpi.scripts.computeCOM import computeCOM
from baf_kpi.scripts.computeCOP import computeCOP
from baf_kpi.scripts.computeSwingLegAngularVelocity import computeSwingLegAngularVelocity
from baf_kpi.scripts.computeStanceAndSwingPercentage import computeStanceAndSwingPercentage
from baf_kpi.scripts.computeSwingWidth import computeSwingWidth
from baf_kpi.scripts.computePathLength import computePathLength
from baf_kpi.scripts.removeShoesOffset import removeShoesOffset
from baf_kpi.scripts.computeNioshKPI import computeNioshKPI
from baf_kpi.scripts.computeLiftingEnergyConsumption import computeLiftingEnergyConsumption
from baf_kpi.scripts.saveKPI import saveKPI
from baf_kpi.visualize.visualizeKPI import visualizeKPI

""" INITIALIZATION"""
JOINT_NAMES = [
    'jT9T8_rotx',
    'jT9T8_rotz',
    'jRightShoulder_rotx',
    'jRightShoulder_roty',
    'jRightShoulder_rotz',
    'jRightElbow_roty',
    'jRightElbow_rotz',
    'jLeftShoulder_rotx',
    'jLeftShoulder_roty',
    'jLeftShoulder_rotz',
    'jLeftElbow_roty',
    'jLeftElbow_rotz',
    'jLeftHip_rotx',
    'jLeftHip_roty',
    'jLeftHip_rotz',
    'jLeftKnee_roty',
    'jLeftKnee_rotz',
    'jLeftAnkle_rotx',
    'jLeftAnkle_roty',
    'jLeftAnkle_rotz',
    'jLeftBallFoot_roty',
    'jRightHip_rotx',
    'jRightHip_roty',
    'jRightHip_rotz',
    'jRightKnee_roty',
    'jRightKnee_rotz',
    'jRightAnkle_rotx',
    'jRightAnkle_roty',
    'jRightAnkle_rotz',
    'jRightBallFoot_roty',
    'jL5S1_roty',
    # extra manually added joints for wrists
    'jLeftWrist_rotx',
    'jLeftWrist_rotz',
    'jRightWrist_rotx',
    'jRightWrist_rotz'
]


NODES_ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

ATTACHED_LINKS = [
    "LeftFoot",        # node 1
    "RightFoot",       # node 2
    "Pelvis",          # node 3
    "LeftLowerArm",    # node 4
    "LeftUpperArm",    # node 5
    "T8",              # node 6
    "RightUpperArm",   # node 7
    "RightLowerArm",   # node 8
    "LeftUpperLeg",    # node 9
    "LeftLowerLeg",    # node 10
    "RightUpperLeg",   # node 11
    "RightLowerLeg"    # node 12
]


def main():
    
    config = getConfig()
    
    global SUBJECT_ID, trialList, subjectMass, EXPERIMENT_DIR, optVisualizeModel, plotKPIs, saveOn, legSkeletonViz, URDF_NAME, dataBAF
    
    # Load values from the configuration file
    SUBJECT_ID = config["subject_id"]
    URDF_NAME = f"S{SUBJECT_ID:02d}"
    trialList = config["trial_list"]
    subjectMass = config["subject_mass"]
    EXPERIMENT_DIR = config["experiment_dir"]
    optVisualizeModel = config["visualize_model"]
    plotKPIs = config["plot_kpis"]
    saveOn = config["save_on"]
    legSkeletonViz = config["leg_skeleton_viz"]
    dataBAF = config["data_BAF"]
    
    
    
    for trialIdx in range(len(trialList)):
        TRIAL_ID = trialList[trialIdx]
        
        paths = setPaths(EXPERIMENT_DIR, SUBJECT_ID, TRIAL_ID)            
        
        
        print("ANALYSIS SUBJECT: ", SUBJECT_ID, " TRIAL: ", TRIAL_ID)
        computeKPI(paths, URDF_NAME, TRIAL_ID, subjectMass, plotKPIs, saveOn, legSkeletonViz, JOINT_NAMES, NODES_ID, ATTACHED_LINKS, dataBAF)
        
        
        
def getConfig():
    
    default_config_path = os.path.join(os.path.dirname(__file__), 'configs', 'config.json')
    
    root = tk.Tk()
    root.withdraw()
    
    print("Select the configuration file or press 'Cancel' to use the default configuration file.")
    file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
    
    if file_path:
        print(f"Selected configuration file: {file_path}")
        with open(file_path , 'r') as f:
            config = json.load(f)
    else:
        print(f"Using default configuration file: {default_config_path}")
        with open(default_config_path , 'r') as f:
            config = json.load(f)
    
    return config

    
        

def computeKPI(paths, URDF_NAME, TRIAL_ID, subjectMass, plotKPIs, saveOn, legSkeletonViz, JOINT_NAMES, NODES_ID, ATTACHED_LINKS, dataBAF):
    """
    Computes key performance indicators (KPI) for gait analysis.
    
    Parameters:
    - paths (dict): Dictionary with paths to experimental data.
    - URDF_NAME (str): Name of the URDF model file.
    - TRIAL_ID (int): Trial ID.
    - subjectMass (float): Mass of the subject.
    - plotKPIs (bool): Whether to plot KPIs.
    - saveOn (bool): Whether to save results.
    - legSkeletonViz (bool): Whether to visualize leg skeleton.
    - JOINT_NAMES (list): Names of joints in the model.
    - NODES_ID (list): IDs of the nodes.
    - ATTACHED_LINKS (list): Links attached to the nodes.
    - dataBAF (bool): True if the data is from BAF, False if the data is from HDE.
    """    
    
    # Initialize iDynTree model
    modelFilePath = Path(paths["pathToSubject"]) / URDF_NAME 
    modelFilePath = modelFilePath.as_posix() + '.urdf'
    
    
    """ If you use a model with meshes, you need to modify the URDF file to include the path to the meshes. 
        UNCOMMENT THE FOLLOWING LINES IF YOU ARE USING A MODEL WITH "package" AS ENVIRONMENT VARIABLE, IF YOU HAVE THE MODEL LIKE: "package:///meshes/Pelvis.stl"  """
    # local_meshes_path = os.getenv('package')

    # if not local_meshes_path:
    #     raise EnvironmentError("The environment variable 'package' is not set")
    
    # directory_meshes_path = resolve_robotics_uri_py.resolve_robotics_uri("package://meshes/Head.stl")
    # local_meshes_path = os.path.dirname(directory_meshes_path)  

    # with open(modelFilePath, "r") as file:
    #     urdf_content = file.read()
        
    # def modify_mesh_line(match):
        
    #     filename = match.group(1)
    #     scale_values = match.group(2)
        
    #     new_filename = f"{local_meshes_path}/" + filename.split("\\")[-1]
        
    #     return f'<mesh filename="{new_filename}" scale="{scale_values}"/>'

    # modified_urdf_content = re.sub(r'<mesh filename=".*?//meshes/(.*?)" scale="(.*?)"/>', modify_mesh_line, urdf_content)

    # with open(modelFilePath, "w") as file:
    #     file.write(modified_urdf_content)

    # print("[OUTPUT] Model with path successfully created. \u2713")
    

    
    humanModelLoader = iDynTree.ModelLoader()
    if not humanModelLoader.loadReducedModelFromFile(modelFilePath, cell2iDynTreeStringVector(JOINT_NAMES)):
        print("Error loading model")
        exit(1)
    humanModel = humanModelLoader.model()

    # Build iDynTree object for kinematics
    kinDynComp = iDynTree.KinDynComputations()
    kinDynComp.loadRobotModel(humanModel)

    # Load data
    if dataBAF:
        human_state, joints_state, human_wrench, human_dynamics, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataBAF(paths)
    else:
        human_state, joints_state, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataHDE(paths, NODES_ID, ATTACHED_LINKS)
        

    iFeelParams, nodes = getSensorToLinkMap(NODES_ID,ATTACHED_LINKS, nodes)
    
    node1 = nodes[0]
    node2 = nodes[1]
    
    # Filter data
    node1Filt, node2Filt = filterData(node1, node2, filterIntensity=1.0)
    
                    
    """  GAIT ANALYSIS   """ 
    print("Start: Gait KPI Computation")
    KPI = {}
    KPI['len'] = DATA_LENGTH
    
    # Add zeros to the end of the data for considering the last 4 additional joints
    add_zeros = np.zeros((KPI['len'], 1, 4))
    joints_state['positions']['data'] = np.concatenate((joints_state['positions']['data'], add_zeros), axis=2)
    joints_state['velocities']['data'] = np.concatenate((joints_state['velocities']['data'], add_zeros), axis=2)
    

    """ Contact pattern detection and steady state events """
    KPI['thresholds'] = {}
    KPI['thresholds']['percentageMassTH'] = 0.05 # Events threshold %percentage of total mass
    KPI['thresholds']['percentageMassTHCK'] = 0.25 # Check threshold %percentage of total mass


    KPI['contactPattern'], KPI['events'] = detectContactPattern(node1Filt, node2Filt, KPI, TIMESTAMPS)


    #  Compute steady state identification
    KPI['steadyStateEvents'] = computeStepSelection(KPI)
    
    if KPI['steadyStateEvents']['legStart'] == 'Left':
        timeStartHS = KPI['events']['Left']['HS']['timestamps'][0]
        timeStartTO = KPI['events']['Left']['TO']['timestamps'][0]
    else:
        timeStartHS = KPI['events']['Right']['HS']['timestamps'][0]
        timeStartTO = KPI['events']['Right']['TO']['timestamps'][0]
        
    timeStart = min(timeStartHS, timeStartTO)
        
    if KPI['steadyStateEvents']['legEnd'] == 'Left':
        timeEndHS = KPI['events']['Left']['HS']['timestamps'][-1]
        timeEndTO = KPI['events']['Left']['TO']['timestamps'][-1]
    else:
        timeEndHS = KPI['events']['Right']['HS']['timestamps'][-1]
        timeEndTO = KPI['events']['Right']['TO']['timestamps'][-1]
    
    timeEnd = max(timeEndHS, timeEndTO)
        
    KPI['totalGaitDuration'] = timeEnd - timeStart
    print("Total gait duration: ", KPI['totalGaitDuration'])
    
    
    """ If the trial is 26, it is the 6MWT trial and we need to split the data into 4 segments of 60 seconds each """
    if TRIAL_ID == 26:
        segment_duration = 60
        num_segments = 4
        
        for segment_idx in range(num_segments):
            KPI = {}
            segment_start = timeStart + segment_idx * segment_duration
            segment_end = segment_start + segment_duration
            
            print("Start segment: ", segment_idx+1)
            
            mask = (TIMESTAMPS >= segment_start) & (TIMESTAMPS <= segment_end)
            segment_timestamps = TIMESTAMPS[mask]
            
            segment_human_state = {'base_position': {}, 'base_orientation': {}, 'base_angular_velocity': {}, 'base_linear_velocity': {}}
            segment_joints_state = {'positions': {}, 'velocities': {}}
            segment_node1Filt = {'FT': {}}
            segment_node2Filt = {'FT': {}}
            
            segment_human_state['base_position']['data'] = human_state['base_position']['data'][mask,:,:]
            segment_human_state['base_orientation']['data'] = human_state['base_orientation']['data'][:,mask]
            segment_human_state['base_angular_velocity']['data'] = human_state['base_angular_velocity']['data'][mask,:,:]
            segment_human_state['base_linear_velocity']['data'] = human_state['base_linear_velocity']['data'][mask,:,:]
            
            segment_joints_state['positions']['data'] = joints_state['positions']['data'][mask,:,:]
            segment_joints_state['velocities']['data'] = joints_state['velocities']['data'][mask,:,:]
            segment_joints_state['positions']['timestamps'] = joints_state['positions']['timestamps'][mask]
            
            
            segment_node1Filt['FT']['data'] = node1Filt['FT']['data'][mask,:,:]
            segment_node2Filt['FT']['data'] = node2Filt['FT']['data'][mask,:,:]
                
            
            segment_trial_id = f"{TRIAL_ID}-{segment_idx+1}"
            
            KPI['len'] = len(segment_timestamps)
            KPI['thresholds'] = {}
            KPI['thresholds']['percentageMassTH'] = 0.05 # Events threshold %percentage of total mass
            KPI['thresholds']['percentageMassTHCK'] = 0.25 # Check threshold %percentage of total mass
            KPI['contactPattern'], KPI['events'] = detectContactPattern(segment_node1Filt, segment_node2Filt, KPI, segment_timestamps)
            KPI['steadyStateEvents'] = computeStepSelection(KPI)
            KPI['totalGaitDuration'] = segment_end - segment_start
            KPI['totalGaitSteps'] = len(KPI['events']['Left']['HS']['idx']) + len(KPI['events']['Right']['HS']['idx']) - 1
            KPI['cadence'] = np.round(60 * KPI['totalGaitSteps'] / KPI['totalGaitDuration'])
            KPI['gaitCycleFeatures'] = computeCycleDuration(KPI)
            KPI['strideLength'] = computeStrideLength(kinDynComp, segment_joints_state, segment_human_state, W_T_B, KPI)
            KPI['COM'] = computeCOM(kinDynComp, segment_joints_state, segment_human_state, W_T_B, KPI)
            
            KPI['swingLegAngVelocity'] = computeSwingLegAngularVelocity(nodes, KPI)
            KPI['phasePercentageWrtCycleDuration'] = computeStanceAndSwingPercentage(KPI)
            KPI['swingWidth'] = computeSwingWidth(kinDynComp, segment_joints_state, segment_human_state, W_T_B, KPI)
            KPI['pathLength'] = computePathLength(KPI)
            
            print("End: Gait KPI Computation for segment ", segment_idx+1)
            
            if saveOn:
                if not os.path.exists(Path(paths['pathToProcessedData'])):
                        os.makedirs(Path(paths['pathToProcessedData']))
                        
                # Save KPIs
                saveKPI(KPI, paths, SUBJECT_ID, segment_trial_id, segment_node1Filt, segment_node2Filt)
                
            if optVisualizeModel:
                visualizeModel(modelFilePath, JOINT_NAMES, segment_timestamps, KPI, segment_joints_state, segment_human_state, dataBAF)
            
            
            
    else:
            
        
        # Compute number of steps and cadence (steps/min)
        KPI['totalGaitSteps'] = len(KPI['events']['Left']['HS']['idx']) + len(KPI['events']['Right']['HS']['idx']) - 1
        KPI['cadence'] = np.round(60 * KPI['totalGaitSteps'] / KPI['totalGaitDuration'])
        print("Total number of steps: ", KPI['totalGaitSteps'])

        # Cycle duration
        KPI['gaitCycleFeatures'] = computeCycleDuration(KPI)   

        # Stride length
        KPI['strideLength'] = computeStrideLength(kinDynComp, joints_state, human_state, W_T_B, KPI)

        # Center of mass (COM)
        KPI['COM'] = computeCOM(kinDynComp, joints_state, human_state, W_T_B, KPI)


        if dataBAF == False:
            # Shoes offset removal
            node1NoOffset, node2NoOffset = removeShoesOffset(node1Filt, node2Filt,KPI)

            # Center of pressure (COP)
            KPI['COP'] = computeCOP(kinDynComp, joints_state, human_state, W_T_B, node1NoOffset, node2NoOffset, KPI)
            
        else:
            # Center of pressure (COP)
            KPI['COP'] = computeCOP(kinDynComp, joints_state, human_state, W_T_B, node1Filt, node2Filt, KPI)


        # Compute Fx and Fy forces percentage wrt vertical force Fz: required for InterSubjectAnalysis
        tmp = {}
        if dataBAF == True:
            node1NoOffset = node1Filt
            node2NoOffset = node2Filt
            
        tmp['fx_min_left'] = np.min(node1NoOffset['FT']['data'][:,:,0])
        tmp['fx_max_left'] = np.max(node1NoOffset['FT']['data'][:,:,0])
        tmp['fy_min_left'] = np.min(node1NoOffset['FT']['data'][:,:,1])
        tmp['fy_max_left'] = np.max(node1NoOffset['FT']['data'][:,:,1])
        tmp['fz_min_left'] = np.min(node1NoOffset['FT']['data'][:,:,2])
        tmp['fz_max_left'] = np.max(node1NoOffset['FT']['data'][:,:,2])
        tmp['fx_magnitude_left'] = np.linalg.norm(tmp['fx_min_left'] - tmp['fx_max_left'])
        tmp['fy_magnitude_left'] = np.linalg.norm(tmp['fy_min_left'] - tmp['fy_max_left'])  
        tmp['fz_magnitude_left'] = np.linalg.norm(tmp['fz_min_left'] - tmp['fz_max_left'])
        
        tmp['fx_min_right'] = np.min(node2NoOffset['FT']['data'][:,:,0])
        tmp['fx_max_right'] = np.max(node2NoOffset['FT']['data'][:,:,0])
        tmp['fy_min_right'] = np.min(node2NoOffset['FT']['data'][:,:,1])
        tmp['fy_max_right'] = np.max(node2NoOffset['FT']['data'][:,:,1])
        tmp['fz_min_right'] = np.min(node2NoOffset['FT']['data'][:,:,2])
        tmp['fz_max_right'] = np.max(node2NoOffset['FT']['data'][:,:,2])
        tmp['fx_magnitude_right'] = np.linalg.norm(tmp['fx_min_right'] - tmp['fx_max_right'])
        tmp['fy_magnitude_right'] = np.linalg.norm(tmp['fy_min_right'] - tmp['fy_max_right'])
        tmp['fz_magnitude_right'] = np.linalg.norm(tmp['fz_min_right'] - tmp['fz_max_right'])
        
        KPI['percentageWrtVerticalForce'] = {}
        KPI['percentageWrtVerticalForce']['percentage_AP_left'] = round(100 * tmp['fx_magnitude_left'] / tmp['fz_magnitude_left'])
        KPI['percentageWrtVerticalForce']['percentage_ML_left'] = round(100 * tmp['fy_magnitude_left'] / tmp['fz_magnitude_left'])
        KPI['percentageWrtVerticalForce']['percentage_AP_right'] = round(100 * tmp['fx_magnitude_right'] / tmp['fz_magnitude_right'])
        KPI['percentageWrtVerticalForce']['percentage_ML_right'] = round(100 * tmp['fy_magnitude_right'] / tmp['fz_magnitude_right'])
        percentageForces = [[KPI['percentageWrtVerticalForce']['percentage_AP_left'], KPI['percentageWrtVerticalForce']['percentage_ML_left']],
                            [KPI['percentageWrtVerticalForce']['percentage_AP_right'], KPI['percentageWrtVerticalForce']['percentage_ML_right']]]
        

        
        # Swing leg angular velocity
        KPI['swingLegAngVelocity'] = computeSwingLegAngularVelocity(nodes, KPI)

        # Percentage of stance and swing 
        KPI['phasePercentageWrtCycleDuration'] = computeStanceAndSwingPercentage(KPI)

        # Swing width
        KPI['swingWidth'] = computeSwingWidth(kinDynComp, joints_state, human_state, W_T_B, KPI)

        # Path length
        KPI['pathLength'] = computePathLength(KPI)
        
        # NIOSH KPI
        KPI['Niosh'] = computeNioshKPI(kinDynComp, joints_state, human_state, W_T_B, KPI, TIMESTAMPS)
        
        # KPI['LEC'] = computeLiftingEnergyConsumption(KPI, subjectMass)

        print("End: Gait KPI Computation")
        
        if plotKPIs:
            visualizeKPI(KPI,TIMESTAMPS,DATA_LENGTH,plotKPIs,paths,saveOn,legSkeletonViz,kinDynComp,joints_state,human_state,W_T_B,node1NoOffset,node2NoOffset)

        print("End: Gait KPI Visualization")
        
        if saveOn:
            if not os.path.exists(Path(paths['pathToProcessedData'])):
                    os.makedirs(Path(paths['pathToProcessedData']))
                    
            # Save KPIs
            saveKPI(KPI, paths, SUBJECT_ID, TRIAL_ID, node1Filt, node2Filt)

            

            dump(KPI, paths['pathToProcessedData'] + '/KPI.joblib')
            dump(joints_state, paths['pathToProcessedData'] + '/joints_state.joblib')
            dump(human_state, paths['pathToProcessedData'] + '/human_state.joblib')


            W_T_B_serializable = []

            for transform in W_T_B:
                try:
                    # Convert `transform` into matrix 4x4 numpy
                    matrix = transform.asHomogeneousTransform().toNumPy()  
                    W_T_B_serializable.append(matrix)
                except AttributeError as e:
                    print(f"Error in conversion: {e}")

            # Convert into a numpy array
            W_T_B_serializable = np.array(W_T_B_serializable)
                    
            dump(W_T_B_serializable, paths['pathToProcessedData'] + '/W_T_B.joblib')
        
        
        
        """ VISUALIZATION  OF THE MODEL """
        if optVisualizeModel:
            visualizeModel(modelFilePath, JOINT_NAMES, TIMESTAMPS, KPI, joints_state, human_state, dataBAF)

    
    print("END")
    
    
if __name__ == "__main__":
    
    main()
    
    



    



