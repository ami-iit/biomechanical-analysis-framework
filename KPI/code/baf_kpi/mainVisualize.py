import numpy as np
import os
import re
import json
import idyntree.bindings as iDynTree
from pathlib import Path
import tkinter as tk
from tkinter import filedialog
from idyntree.visualize import MeshcatVisualizer
from rich.progress import track
import resolve_robotics_uri_py
import time
import matplotlib.pyplot as plt

from baf_kpi.scripts.cell2iDynTreeStringVector import cell2iDynTreeStringVector
from baf_kpi.scripts.setPaths import setPaths
from baf_kpi.dataLoader.dataLoaderBAF import loadDataBAF
from baf_kpi.dataLoader.dataLoaderHDE import loadDataHDE
from baf_kpi.scripts.filterData import filterData
from baf_kpi.scripts.contactPatternDetection import detectContactPattern
from baf_kpi.scripts.computeStepSelection import computeStepSelection
from baf_kpi.scripts.computeCycleDuration import computeCycleDuration
from baf_kpi.scripts.computeCOM import computeCOM
from baf_kpi.scripts.computeCOP import computeCOP
from baf_kpi.scripts.removeShoesOffset import removeShoesOffset
from baf_kpi.scripts.updateCOM import updateCOM
from baf_kpi.scripts.updateCOP import updateCOP
from baf_kpi.scripts.updateEffort import updateEffort




JOINT_NAMES = [
    'jC7RightShoulder_rotx',
    'jC7RightShoulder_roty',
    'jC7RightShoulder_rotz',
    'jC7LeftShoulder_rotx',
    'jC7LeftShoulder_roty',
    'jC7LeftShoulder_rotz',
    'jL5S1_roty',
    'jL5S1_rotx',
    'jL4L3_roty',
    'jL4L3_rotx',
    'jL1T12_roty',
    'jL1T12_rotx',
    'jT9T8_rotx',
    'jT9T8_roty',
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


parentLinkNames = [
    "LeftFoot", "LeftLowerLeg", "LeftUpperLeg", "LeftHand", "LeftForeArm", "LeftUpperArm",
    "RightFoot", "RightLowerLeg", "RightUpperLeg", "RightHand", "RightForeArm", "RightUpperArm", "LowerTrunk"
]

sphericalJointNames = [
    "jLeftAnkle", "jLeftKnee", "jLeftHip", "jLeftWrist", "jLeftElbow", "jLeftShoulder",
    "jRightAnkle", "jRightKnee", "jRightHip", "jRightWrist", "jRightElbow", "jRightShoulder", "jL5S1"
]

maxEffort = [
    90.0, 40.0, 30.0, 6.0, 20.0, 30.0, 90.0, 40.0, 30.0, 6.0, 20.0, 30.0, 30.0
]

colorBackground = [
    0.4, 0.7, 0.9, 1.0
]

forceScalingFactor = 0.005

class JointEffortData:
    def __init__(self, parentLinkName, sphericalJointName,  effortMax):
        self.parentLinkName = parentLinkName # Name of the parent link
        self.sphericalJointName = sphericalJointName # Name of the spherical joint
        self.fakeJointsIndices = [] # Indices of the fake joints 
        self.effortMax = effortMax # Maximum effort
        self.effort = 0.0 # Current effort

def main():
    
    
    config = getConfig()
    
    global SUBJECT_ID, trialList, EXPERIMENT_DIR, URDF_NAME, dataBAF
    
    # Load values from the configuration file
    SUBJECT_ID = config["subject_id"]
    URDF_NAME = f"S{SUBJECT_ID:02d}"
    trialList = config["trial_list"]
    EXPERIMENT_DIR = config["experiment_dir"]
    dataBAF = config["data_BAF"]
    
    
    
    for trialIdx in range(len(trialList)):
        TRIAL_ID = trialList[trialIdx]
        
        paths = setPaths(EXPERIMENT_DIR, SUBJECT_ID, TRIAL_ID)
        print("ANALYSIS SUBJECT: ", SUBJECT_ID, " TRIAL: ", TRIAL_ID)
        visualizeModel(paths, URDF_NAME, TRIAL_ID, JOINT_NAMES, NODES_ID, ATTACHED_LINKS, dataBAF)
              
        
def getConfig():
    
    default_config_path = os.path.join(os.path.dirname(__file__), 'configs', 'configVisualize.json')
    
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


def visualizeModel(paths, URDF_NAME, TRIAL_ID, JOINT_NAMES, NODES_ID, ATTACHED_LINKS, dataBAF):
    modelFilePath = Path(paths["pathToSubject"]) / URDF_NAME 
    modelFilePath = modelFilePath.as_posix() + '.urdf'
    
    
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
        

    humanModelLoader = iDynTree.ModelLoader()
    if not humanModelLoader.loadReducedModelFromFile(modelFilePath, cell2iDynTreeStringVector(JOINT_NAMES)):
        print("Error loading model")
        exit(1)
    humanModel = humanModelLoader.model()
    
    # print("Human model to string: ", humanModel.toString())

    # Build iDynTree object for kinematics
    kinDynComp = iDynTree.KinDynComputations()
    kinDynComp.loadRobotModel(humanModel)
    
    # Load data
    if dataBAF:
        human_state, joints_state, human_wrench, human_dynamics, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataBAF(paths)
    else:
        human_state, joints_state, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B = loadDataHDE(paths, NODES_ID, ATTACHED_LINKS)


    node1 = nodes[0]
    node2 = nodes[1]
    
    # Filter data
    node1Filt, node2Filt = filterData(node1, node2, filterIntensity=1.0)
    
    # Compute KPI
    KPI = {}
    KPI['len'] = DATA_LENGTH
    
    add_zeros = np.zeros((KPI['len'], 1, 4))
    joints_state['positions']['data'] = np.concatenate((joints_state['positions']['data'], add_zeros), axis=2)
    joints_state['velocities']['data'] = np.concatenate((joints_state['velocities']['data'], add_zeros), axis=2)
    
    KPI['thresholds'] = {}
    KPI['thresholds']['percentageMassTH'] = 0.05 # Events threshold %percentage of total mass
    KPI['thresholds']['percentageMassTHCK'] = 0.25 # Check threshold %percentage of total mass


    # KPI['contactPattern'], KPI['events'] = detectContactPattern(node1Filt, node2Filt, KPI, TIMESTAMPS)
    # KPI['steadyStateEvents'] = computeStepSelection(KPI)
    # KPI['gaitCycleFeatures'] = computeCycleDuration(KPI)   
    
    
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])
    
    
    links_lhand_COM, links_rhand_COM = [], []    
    links_pelvis = []
    links_lfoot, links_rfoot = [], []
    links_ltoe, links_rtoe = [], []
    links_llower_leg, links_rlower_leg = [], []
    links_lupper_leg, links_rupper_leg = [], []
    links_lower_trunk = []
    links_upper_trunk = []
    links_neck = []
    links_head = []
    links_lupper_arm, links_rupper_arm = [], []
    links_lfore_arm, links_rfore_arm = [], []
    
    zOffset = 0.045
    
    
    # Compute minimum height of the hands
    for lenIdx in range(KPI['len']-1):
        # Set the joint positions
        s = joints_state['positions']['data'][lenIdx][0]
        ds = joints_state['velocities']['data'][lenIdx][0]
        lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
        ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
        base_vel = np.concatenate((lin_vel, ang_vel))  
        
        kinDynComp.setRobotState(W_T_B[lenIdx], s, base_vel, ds, gravity)
        
        # Compute the position of the hands
        H_link_lhand_COM = kinDynComp.getWorldTransform("LeftHandCOM")
        H_link_lhand_np_COM = H_link_lhand_COM.asHomogeneousTransform().toNumPy()
        link_lhand_pos_COM = H_link_lhand_np_COM[0:3,3]
        links_lhand_COM.append(link_lhand_pos_COM)
        
        H_link_rhand_COM = kinDynComp.getWorldTransform("RightHandCOM")
        H_link_rhand_np_COM = H_link_rhand_COM.asHomogeneousTransform().toNumPy()
        link_rhand_pos_COM = H_link_rhand_np_COM[0:3,3]
        links_rhand_COM.append(link_rhand_pos_COM)
        
        # Compute the position of the pelvis
        H_link_pelvis = kinDynComp.getWorldTransform("Pelvis")
        H_link_pelvis_np = H_link_pelvis.asHomogeneousTransform().toNumPy()
        link_pelvis_pos = H_link_pelvis_np[0:3,3]
        links_pelvis.append(link_pelvis_pos)
        
        # Compute position of the feet
        H_link_lfoot = kinDynComp.getWorldTransform("LeftFoot")
        H_link_lfoot_np = H_link_lfoot.asHomogeneousTransform().toNumPy()
        link_lfoot_pos = H_link_lfoot_np[0:3,3]
        links_lfoot.append(link_lfoot_pos)
        
        H_link_rfoot = kinDynComp.getWorldTransform("RightFoot")
        H_link_rfoot_np = H_link_rfoot.asHomogeneousTransform().toNumPy()
        link_rfoot_pos = H_link_rfoot_np[0:3,3]
        links_rfoot.append(link_rfoot_pos)
        
        # Compute the position of the toes
        H_link_ltoe = kinDynComp.getWorldTransform("LeftToe")
        H_link_ltoe_np = H_link_ltoe.asHomogeneousTransform().toNumPy()
        link_ltoe_pos = H_link_ltoe_np[0:3,3]
        links_ltoe.append(link_ltoe_pos)
        
        H_link_rtoe = kinDynComp.getWorldTransform("RightToe")
        H_link_rtoe_np = H_link_rtoe.asHomogeneousTransform().toNumPy()
        link_rtoe_pos = H_link_rtoe_np[0:3,3]
        links_rtoe.append(link_rtoe_pos)
        
        # Compute position of the lower legs
        H_link_llower_leg = kinDynComp.getWorldTransform("LeftLowerLeg")
        H_link_llower_leg_np = H_link_llower_leg.asHomogeneousTransform().toNumPy()
        link_llower_leg_pos = H_link_llower_leg_np[0:3,3]
        links_llower_leg.append(link_llower_leg_pos)
        
        H_link_rlower_leg = kinDynComp.getWorldTransform("RightLowerLeg")
        H_link_rlower_leg_np = H_link_rlower_leg.asHomogeneousTransform().toNumPy()
        link_rlower_leg_pos = H_link_rlower_leg_np[0:3,3]
        links_rlower_leg.append(link_rlower_leg_pos)
        
        # Compute position of the upper legs
        H_link_lupper_leg = kinDynComp.getWorldTransform("LeftUpperLeg")
        H_link_lupper_leg_np = H_link_lupper_leg.asHomogeneousTransform().toNumPy()
        link_lupper_leg_pos = H_link_lupper_leg_np[0:3,3]
        links_lupper_leg.append(link_lupper_leg_pos)
        
        H_link_rupper_leg = kinDynComp.getWorldTransform("RightUpperLeg")
        H_link_rupper_leg_np = H_link_rupper_leg.asHomogeneousTransform().toNumPy()
        link_rupper_leg_pos = H_link_rupper_leg_np[0:3,3]
        links_rupper_leg.append(link_rupper_leg_pos)
        
        # # Compute the position of the lower trunk
        # H_link_lower_trunk = kinDynComp.getWorldTransform("LowerTrunk")
        # H_link_lower_trunk_np = H_link_lower_trunk.asHomogeneousTransform().toNumPy()
        # link_lower_trunk_pos = H_link_lower_trunk_np[0:3,3]
        # links_lower_trunk.append(link_lower_trunk_pos)
        
        # # Compute the position of the upper trunk
        # H_link_upper_trunk = kinDynComp.getWorldTransform("UpperTrunk")
        # H_link_upper_trunk_np = H_link_upper_trunk.asHomogeneousTransform().toNumPy()
        # link_upper_trunk_pos = H_link_upper_trunk_np[0:3,3]
        # links_upper_trunk.append(link_upper_trunk_pos)
        
        # Compute the position of the neck
        H_link_neck = kinDynComp.getWorldTransform("Neck")
        H_link_neck_np = H_link_neck.asHomogeneousTransform().toNumPy()
        link_neck_pos = H_link_neck_np[0:3,3]
        links_neck.append(link_neck_pos)
        
        # Compute the position of the head
        H_link_head = kinDynComp.getWorldTransform("Head")
        H_link_head_np = H_link_head.asHomogeneousTransform().toNumPy()
        link_head_pos = H_link_head_np[0:3,3]
        links_head.append(link_head_pos)
        
        # Compute the position of the upper arms
        H_link_lupper_arm = kinDynComp.getWorldTransform("LeftUpperArm")
        H_link_lupper_arm_np = H_link_lupper_arm.asHomogeneousTransform().toNumPy()
        link_lupper_arm_pos = H_link_lupper_arm_np[0:3,3]
        links_lupper_arm.append(link_lupper_arm_pos)
        
        H_link_rupper_arm = kinDynComp.getWorldTransform("RightUpperArm")
        H_link_rupper_arm_np = H_link_rupper_arm.asHomogeneousTransform().toNumPy()
        link_rupper_arm_pos = H_link_rupper_arm_np[0:3,3]
        links_rupper_arm.append(link_rupper_arm_pos)
        
        # Compute the position of the lower arms
        H_link_lfore_arm = kinDynComp.getWorldTransform("LeftForeArm")
        H_link_lfore_arm_np = H_link_lfore_arm.asHomogeneousTransform().toNumPy()
        link_lfore_arm_pos = H_link_lfore_arm_np[0:3,3]
        links_lfore_arm.append(link_lfore_arm_pos)
        
        H_link_rfore_arm = kinDynComp.getWorldTransform("RightForeArm")
        H_link_rfore_arm_np = H_link_rfore_arm.asHomogeneousTransform().toNumPy()
        link_rfore_arm_pos = H_link_rfore_arm_np[0:3,3]
        links_rfore_arm.append(link_rfore_arm_pos)
        
        
        
    start_idx = 0
    end_idx = 1000

    
    links_lhand_COM = np.array(links_lhand_COM)
    links_rhand_COM = np.array(links_rhand_COM)
    z_lhand_COM = links_lhand_COM[start_idx:end_idx,2] 
    z_rhand_COM = links_rhand_COM[start_idx:end_idx,2] 
    
    links_pelvis = np.array(links_pelvis)
    links_pelvis = links_pelvis[start_idx:end_idx,2] 
    
    links_lfoot = np.array(links_lfoot)
    links_rfoot = np.array(links_rfoot)
    z_lfoot = links_lfoot[start_idx:end_idx,2] 
    z_rfoot = links_rfoot[start_idx:end_idx,2] 
    
    links_ltoes = np.array(links_ltoe)
    links_rtoes = np.array(links_rtoe)
    z_ltoe = links_ltoes[start_idx:end_idx,2] 
    z_rtoe = links_rtoes[start_idx:end_idx,2] 
    
    links_llower_leg = np.array(links_llower_leg)
    links_rlower_leg = np.array(links_rlower_leg)
    z_llower_leg = links_llower_leg[start_idx:end_idx,2] 
    z_rlower_leg = links_rlower_leg[start_idx:end_idx,2] 
    
    links_lupper_leg = np.array(links_lupper_leg)
    links_rupper_leg = np.array(links_rupper_leg)
    z_lupper_leg = links_lupper_leg[start_idx:end_idx,2] 
    z_rupper_leg = links_rupper_leg[start_idx:end_idx,2] 
    
    # links_lower_trunk = np.array(links_lower_trunk)
    # z_lower_trunk = links_lower_trunk[start_idx:end_idx,2] 
    
    # links_upper_trunk = np.array(links_upper_trunk)
    # z_upper_trunk = links_upper_trunk[start_idx:end_idx,2] 
    
    links_neck = np.array(links_neck)
    z_neck = links_neck[start_idx:end_idx,2] 
    
    links_head = np.array(links_head)
    z_head = links_head[start_idx:end_idx,2] 
    
    links_lupper_arm = np.array(links_lupper_arm)
    links_rupper_arm = np.array(links_rupper_arm)
    z_lupper_arm = links_lupper_arm[start_idx:end_idx,2] 
    z_rupper_arm = links_rupper_arm[start_idx:end_idx,2] 
    
    links_lfore_arm = np.array(links_lfore_arm)
    links_rfore_arm = np.array(links_rfore_arm)
    z_lfore_arm = links_lfore_arm[start_idx:end_idx,2] 
    z_rfore_arm = links_rfore_arm[start_idx:end_idx,2] 
    
    # # Find minimum height of the hands
    # min_z_lhand_COM = np.min(z_lhand_COM)
    # min_lhand_COM_idx = np.argmin(z_lhand_COM) + start_idx
    # time_min_lhand_COM = TIMESTAMPS[min_lhand_COM_idx] - TIMESTAMPS[0]
    # print("Min left hand COM: ", min_z_lhand_COM)
    # print("Time min left hand COM: ", time_min_lhand_COM)
    
    
    # min_z_rhand_COM = np.min(z_rhand_COM)
    # min_rhand_COM_idx = np.argmin(z_rhand_COM) + start_idx
    # time_min_rhand_COM = TIMESTAMPS[min_rhand_COM_idx] - TIMESTAMPS[0]
    # print("Min right hand COM: ", min_z_rhand_COM)
    # print("Time min right hand COM: ", time_min_rhand_COM)
    
    # # Find maximum height of the hands
    # max_z_lhand_COM = np.max(z_lhand_COM)
    # max_lhand_COM_idx = np.argmax(z_lhand_COM) + start_idx
    # time_max_lhand_COM = TIMESTAMPS[max_lhand_COM_idx] - TIMESTAMPS[0]
    # print("Max left hand COM: ", max_z_lhand_COM)
    # print("Time max left hand COM: ", time_max_lhand_COM)
    
    # max_z_rhand_COM = np.max(z_rhand_COM)
    # max_rhand_COM_idx = np.argmax(z_rhand_COM) + start_idx
    # time_max_rhand_COM = TIMESTAMPS[max_rhand_COM_idx] - TIMESTAMPS[0]
    # print("Max right hand COM: ", max_z_rhand_COM)
    # print("Time max right hand COM: ", time_max_rhand_COM)
    
    
    # # PLot the height of the upper legs
    # plt.figure()
    # plt.plot(TIMESTAMPS[start_idx:end_idx]-TIMESTAMPS[0], z_lfoot, label="Left", color='red')
    # plt.plot(TIMESTAMPS[start_idx:end_idx]-TIMESTAMPS[0], z_rfoot, label="Right", color='blue')
    # plt.xlim([12.5, 28])
    # # plt.ylim([0.9150, 0.9151])
    # plt.xlabel("Time [s]")
    # plt.ylabel("Height [m]")
    # plt.legend()
    # # plt.show()
    
    # zhand_COM = (z_lhand_COM + z_rhand_COM)/2
    # plt.figure()
    # # plt.plot(TIMESTAMPS[start_idx:end_idx]-TIMESTAMPS[0], z_lhand_COM, label="Left", color='red')
    # # plt.plot(TIMESTAMPS[start_idx:end_idx]-TIMESTAMPS[0], z_rhand_COM, label="Right", color='blue')
    # plt.plot(TIMESTAMPS[start_idx:end_idx]-TIMESTAMPS[0], zhand_COM, label="COM", color='green')
    # plt.xlim([6, 22])
    # plt.ylim([0.42, 0.75])
    # plt.xlabel("Time [s]")
    # plt.ylabel("Height [m]")
    # # plt.legend(loc='upper right')
    # plt.show()

    
    
    
    # Center of mass (COM)
    KPI['COM'] = computeCOM(kinDynComp, joints_state, human_state, W_T_B, KPI)

    # # Center of pressure (COP)
    # if dataBAF == False:
    #     # Shoes offset removal
    #     node1NoOffset, node2NoOffset = removeShoesOffset(node1Filt, node2Filt,KPI)

    #     # Center of pressure (COP)
    #     KPI['COP'] = computeCOP(kinDynComp, joints_state, human_state, W_T_B, node1NoOffset, node2NoOffset, KPI)
        
    # else:
    #     # Center of pressure (COP)
    #     KPI['COP'] = computeCOP(kinDynComp, joints_state, human_state, W_T_B, node1Filt, node2Filt, KPI)



    viz = MeshcatVisualizer()
    viz.load_model_from_file(modelFilePath, JOINT_NAMES, "Human Model",color=[0.0,0.0,0.0,0.3])
    viz.open()

    viz.start_recording_animation()
    viz.set_animation_frame(0)

    timeScaling = 5
    dt_base = np.average(np.diff(TIMESTAMPS))
    index = 0

    
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])

    # # Create and configure spheres for visualization
    radius = 0.05
    color_sphere = [0.0, 0.0, 0.0, 1.0]
    
    diameter = 0.25
    thickness = 0.01
    
    # for parentLinkName in parentLinkNames:
    #     viz.load_sphere(radius, parentLinkName, color=color_sphere)
        
    # for com in range(len(TIMESTAMPS)):
    #     viz.load_cylinder(diameter/2, thickness, shape_name=f"COM_{com}", color=[0,0,0,0.5])
    #     viz.load_cylinder(diameter/2, thickness, shape_name=f"COP_Left_{com}", color=[1,0,0,0.5])
    #     viz.load_cylinder(diameter/2, thickness, shape_name=f"COP_Right_{com}", color=[0,0,1,0.5])
    
    # for i in range(0, len(TIMESTAMPS)):
    for i in track(range(0, len(TIMESTAMPS), int(1 / (dt_base * (30 / timeScaling)))), description="Visualizing model"):

        time_init = time.time()
        
        t = TIMESTAMPS[i]
        
        viz.set_animation_frame(index)
        

        if dataBAF == True:
            rpy = human_state['base_orientation']['data'][i,0,:]
        else:
            rotation = iDynTree.Rotation()
            rotation.fromQuaternion(human_state['base_orientation']['data'][:,i])
            rpy = rotation.asRPY()

        orientation = iDynTree.Rotation.RPY(rpy[0], rpy[1], rpy[2]).toNumPy()

        # Update the model
        viz.set_multibody_system_state(base_position = human_state['base_position']['data'][i,0,:], base_rotation = orientation, joint_value = joints_state['positions']['data'][i,0,:], model_name="Human Model")

        ''' Update COM ''' 
        # updateCOM(viz, KPI, i, diameter, thickness)
        
        ''' Update COP '''
        # updateCOP(viz, KPI, i, diameter, thickness)
        
        
        ''' Update visualization of efforts and external wrenches''' 
        # updateEffort(joints_state, human_state, kinDynComp, W_T_B, parentLinkNames, sphericalJointNames, maxEffort, JOINT_NAMES, viz, i, human_dynamics, s, ds, base_vel, gravity) 
        
            
        # time_end = time.time()
        
        # timer = time_end - time_init
        
        # if timer < 0.033:
        #     time.sleep(0.033-timer)
        
        index += timeScaling
    
    viz.publish_animation()

if __name__ == "__main__":
    main()