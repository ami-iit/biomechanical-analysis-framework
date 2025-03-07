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
from rich.progress import track
import resolve_robotics_uri_py
import pandas as pd



from baf_kpi.visualize.visualizeModel import visualizeModel
from baf_kpi.scripts.setPaths import setPaths
from baf_kpi.scripts.filterData import filterData
from baf_kpi.scripts.getSensorToLinkMap import getSensorToLinkMap
from baf_kpi.scripts.cell2iDynTreeStringVector import cell2iDynTreeStringVector
from baf_kpi.dataLoader.dataLoaderBAF2 import loadDataBAF2
from baf_kpi.dataLoader.dataLoaderFilteredBAF import loadDataFilteredBAF
from baf_kpi.dataLoader.dataLoaderHDE import loadDataHDE
from baf_kpi.dataLoader.dataLoaderHDE2 import loadDataHDE2
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
    'jL5S1_roty'
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
    
    global SUBJECT_ID, trialList, subjectMass, EXPERIMENT_DIR, optVisualizeModel, plotKPIs, saveOn, videoRecording, legSkeletonViz, URDF_NAME, dataBAF
    
    # Load values from the configuration file
    SUBJECT_ID = config["subject_id"]
    URDF_NAME = f"S{SUBJECT_ID:02d}"
    trialList = config["trial_list"]
    subjectMass = config["subject_mass"]
    EXPERIMENT_DIR = config["experiment_dir"]
    optVisualizeModel = config["visualize_model"]
    plotKPIs = config["plot_kpis"]
    saveOn = config["save_on"]
    videoRecording = config["video_recording"]
    legSkeletonViz = config["leg_skeleton_viz"]
    dataBAF = config["data_BAF"]
    
    
    
    for trialIdx in range(len(trialList)):
        TRIAL_ID = trialList[trialIdx]
        
        paths = setPaths(EXPERIMENT_DIR, SUBJECT_ID, TRIAL_ID)
        print("ANALYSIS SUBJECT: ", SUBJECT_ID, " TRIAL: ", TRIAL_ID)
        computeKPI(paths, URDF_NAME, TRIAL_ID, subjectMass, plotKPIs, saveOn, videoRecording, legSkeletonViz, JOINT_NAMES, NODES_ID, ATTACHED_LINKS, dataBAF)
        
        
        
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

    
        

def computeKPI(paths, URDF_NAME, TRIAL_ID, subjectMass, plotKPIs, saveOn, videoRecording, legSkeletonViz, JOINT_NAMES, NODES_ID, ATTACHED_LINKS, dataBAF):
    """
    Computes key performance indicators (KPI) for gait analysis.
    
    Parameters:
    - paths (dict): Dictionary with paths to experimental data.
    - URDF_NAME (str): Name of the URDF model file.
    - plotKPIs (bool): Whether to plot KPIs.
    - saveOn (bool): Whether to save results.
    - videoRecording (bool): Whether to record animations.
    - legSkeletonViz (bool): Whether to visualize leg skeleton.
    - JOINT_NAMES (list): Names of joints in the model.
    - NODES_ID (list): IDs of the nodes.
    - ATTACHED_LINKS (list): Links attached to the nodes.
    """    
    
    # Initialize iDynTree model
    modelFilePath = Path(paths["pathToSubject"]) / URDF_NAME 
    modelFilePath = modelFilePath.as_posix() + '.urdf'
    
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
        human_state, joints_state, human_wrench, human_dynamics, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataBAF2(paths)
    else:
        # human_state, joints_state, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataHDE(paths, NODES_ID, ATTACHED_LINKS)
        human_state, joints_state, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataHDE2(paths, NODES_ID, ATTACHED_LINKS)
        
        

    iFeelParams, nodes = getSensorToLinkMap(NODES_ID,ATTACHED_LINKS, nodes)
    
    node1 = nodes[0]
    node2 = nodes[1]
    
    # Filter data
    node1Filt, node2Filt = filterData(node1, node2, filterIntensity=1.0)
    
                    
    """  GAIT ANALYSIS   """ 
    print("Start: Gait KPI Computation")

    KPI = {}
    KPI['len'] = DATA_LENGTH
    KPI['totalGaitDuration'] = durationInSec
    

    """ Contact pattern detection and steady state events """
    KPI['thresholds'] = {}
    KPI['thresholds']['percentageMassTH'] = 0.05 # Events threshold %percentage of total mass
    KPI['thresholds']['percentageMassTHCK'] = 0.25 # Check threshold %percentage of total mass


    # Calcola Niosh dal momento in cui inizia il lifting
    # NIOSH KPI
    KPI['Niosh'] = computeNioshKPI(kinDynComp, joints_state, human_state, W_T_B, KPI, TIMESTAMPS)
    
    # KPI['LEC'] = computeLiftingEnergyConsumption(KPI, subjectMass)


if __name__ == "__main__":
    
    main()