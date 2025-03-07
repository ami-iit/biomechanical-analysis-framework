import sys
import os

project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_path not in sys.path:
    sys.path.append(project_path)
    
import matplotlib.pyplot as plt
import numpy as np
from joblib import load
from scipy.io import loadmat
import idyntree.bindings as iDynTree
from scripts.cell2iDynTreeStringVector import cell2iDynTreeStringVector


file_path_MATLAB = 'C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/MAPest/core/dataset/S01/trial23/processed/KPI.mat'
file_path_PYTHON = 'C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/KPI_Python/dataKPI/KPI.joblib'

"""LOAD DATA: PYTHON"""
KPI_Python = load(file_path_PYTHON)
joints_state = load('C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/KPI_Python/dataKPI/joints_state.joblib')
human_state = load('C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/KPI_Python/dataKPI/human_state.joblib')
W_T_B = load('C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/KPI_Python/dataKPI/W_T_B.joblib')

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

modelFilePath = 'C:/Users/ldanovaro/Documents/GitHub/element_ifeel-aism/KPI_Python/dataset/S01/S01.urdf' 
humanModelLoader = iDynTree.ModelLoader()
if not humanModelLoader.loadReducedModelFromFile(modelFilePath, cell2iDynTreeStringVector(JOINT_NAMES)):
    print("Error loading model")
    exit(1)
humanModel = humanModelLoader.model()

# Build iDynTree object for kinematics
kinDynComp = iDynTree.KinDynComputations()
kinDynComp.loadRobotModel(humanModel)



"""LOAD DATA: MATLAB"""
data = loadmat(file_path_MATLAB)
KPI_data = data['KPI']

KPI_Matlab = {'thresholds': {}, 'contactPattern': {'forcesInStance': {}}, 'events': {'Left': {'HS': {}, 'TO':{}}, 'Right': {'HS':{}, 'TO':{}}}, 'steadyStateEvents': { 'Left': {'HS': {}, 'TO':{}}, 'Right': {'HS':{}, 'TO':{}}}, 'gaitCycleFeatures': {'Left':{}, 'Right':{}}, 'strideLength': {}, 'COM': {}, 'COP': {}, 'swingLegAngVelocity': {}, 'phasePercentageWrtCyclceDuration': {}, 'swingWidth': {}, 'pathLength': {}}

KPI_Matlab['len'] = KPI_data['len'][0][0][0][0]
KPI_Matlab['totalGaitDuration'] = KPI_data['totalGaitDuration'][0][0][0][0]
KPI_Matlab['totalGaitSteps'] = KPI_data['totalGaitSteps'][0][0][0][0]
KPI_Matlab['cadence'] = KPI_data['cadence'][0][0][0][0]


"""COMPARISON: CADENCE"""
errorCadence = KPI_Python['cadence'] - KPI_Matlab['cadence']

"""COMPARISON: CYCLE DURATION"""
timeStrideLeft_MATLAB = KPI_data['gaitCycleFeatures'][0][0][0]['Left'][0][0]['stride'][0][0]['timeStride']
timeStrideLeft_Python = [KPI_Python['gaitCycleFeatures']['Left']['stride'][i]['timeStride'] for i in range(len(KPI_Python['gaitCycleFeatures']['Left']['stride']))]
errorTimeStride_left = timeStrideLeft_Python - timeStrideLeft_MATLAB

timeStrideRight_MATLAB = KPI_data['gaitCycleFeatures'][0][0][0]['Right'][0][0]['stride'][0][0]['timeStride']
timeStrideRight_Python = [KPI_Python['gaitCycleFeatures']['Right']['stride'][i]['timeStride'] for i in range(len(KPI_Python['gaitCycleFeatures']['Right']['stride']))]
errorTimeStride_right = timeStrideRight_Python - timeStrideRight_MATLAB

# ERROR PLOT
# fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# axs[0].plot(errorTimeStride_left, color='blue', label='Left TimeStride Error')
# axs[0].set_title('Left TimeStride Error')
# axs[0].set_xlabel('Index of Gait Cycle')
# axs[0].set_ylabel('Error (s)')
# axs[0].grid(True)
# axs[0].legend()
# axs[0].autoscale(enable=True, axis='x', tight=True)

# axs[1].plot(errorTimeStride_right, color='red', label='Right TimeStride Error')
# axs[1].set_title('Right TimeStride Error')
# axs[1].set_xlabel('Index of Gait Cycle')
# axs[1].set_ylabel('Error (s)')
# axs[1].grid(True)
# axs[1].legend()
# axs[1].autoscale(enable=True, axis='x', tight=True)

# fig.suptitle('TimeStride Error', fontsize=16)

# plt.tight_layout()

fig, ax = plt.subplots(figsize=(10, 6))

ax.plot(errorTimeStride_left, color='blue', label='Left TimeStride Error')
ax.plot(errorTimeStride_right, color='red', label='Right TimeStride Error')

ax.set_title('TimeStride Error')
ax.set_xlabel('Index of Gait Cycle')
ax.set_ylabel('Error (s)')
ax.grid(True)
ax.legend()
ax.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()

# COMPARISON PLOT
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].plot(timeStrideLeft_MATLAB, marker='o', linestyle='-', color='blue', label='TimeStride Left MATLAB')
axs[0].plot(timeStrideLeft_Python, marker='x', linestyle='--', color='red', label='TimeStride Left Python')
axs[0].set_title('Left TimeStride')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Time (s)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

axs[1].plot(timeStrideRight_MATLAB, marker='o', linestyle='-', color='green', label='TimeStride Right MATLAB')
axs[1].plot(timeStrideRight_Python, marker='x', linestyle='--', color='orange', label='TimeStride Right Python')
axs[1].set_title('Right TimeStride')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Time (s)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

fig.suptitle('Comparison of TimeStride', fontsize=16)

plt.tight_layout()


    
"""COMPARISON: STRIDE LENGTH"""
strideLengthLeftDistances_MATLAB = KPI_data['strideLength'][0][0][0]['Left'][0][0]['distance']
strideLengthLeftDistances_Python = [KPI_Python['strideLength']['Left'][i]['distance'] for i in range(len(KPI_Python['strideLength']['Left']))]
errorStrideLength_left = strideLengthLeftDistances_Python - strideLengthLeftDistances_MATLAB

strideLengthRightDistances_MATLAB = KPI_data['strideLength'][0][0][0]['Right'][0][0]['distance']
strideLengthRightDistances_Python = [KPI_Python['strideLength']['Right'][i]['distance'] for i in range(len(KPI_Python['strideLength']['Right']))]
errorStrideLength_right = strideLengthRightDistances_Python - strideLengthRightDistances_MATLAB

# ERROR PLOT
fig, ax = plt.subplots(figsize=(10, 6))

ax.plot(errorStrideLength_left, color='blue', marker='o', linestyle='-', label='Left Stride Length Error')
ax.plot(errorStrideLength_right, color='red', marker='x', linestyle='--', label='Right Stride Length Error')

ax.set_title('Stride Length Error')
ax.set_xlabel('Index of Gait Cycle')
ax.set_ylabel('Error (m)')
ax.grid(True)
ax.legend()
ax.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()


# COMPARISON PLOT
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].plot(strideLengthLeftDistances_MATLAB, marker='o', linestyle='-', color='blue', label='Stride Left MATLAB')
axs[0].plot(strideLengthLeftDistances_Python, marker='x', linestyle='--', color='red', label='Stride Left Python')
axs[0].set_title('Left Stride Length')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Distance (m)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

axs[1].plot(strideLengthRightDistances_MATLAB, marker='o', linestyle='-', color='green', label='Stride Right MATLAB')
axs[1].plot(strideLengthRightDistances_Python, marker='x', linestyle='--', color='orange', label='Stride Right Python')
axs[1].set_title('Right Stride Length')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Distance (m)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

fig.suptitle('Comparison of Stride Length', fontsize=16)

plt.tight_layout()



"""COMPARISON: MAX ANGULAR VELOCITY"""


"""COMPARISON: SWING WIDTH"""
percentageSwingWidthLeft_MATLAB = KPI_data['swingWidth'][0][0][0]['Left'][0][0]['percentageOfstrideLength']
percentageSwingWidthLeft_Python = [KPI_Python['swingWidth']['Left'][i]['percentageOfStrideLength'] for i in range(len(KPI_Python['swingWidth']['Left']))]
errorSwingWidth_left = percentageSwingWidthLeft_Python - percentageSwingWidthLeft_MATLAB

percentageSwingWidthRight_MATLAB = KPI_data['swingWidth'][0][0][0]['Right'][0][0]['percentageOfstrideLength']
percentageSwingWidthRight_Python = [KPI_Python['swingWidth']['Right'][i]['percentageOfStrideLength'] for i in range(len(KPI_Python['swingWidth']['Right']))]
errorSwingWidth_right = percentageSwingWidthRight_Python - percentageSwingWidthRight_MATLAB

# ERROR PLOT
fig, ax = plt.subplots(figsize=(10, 6))

ax.plot(errorSwingWidth_left, marker='o', linestyle='-', color='blue', label='Left Swing Width Error')
ax.plot(errorSwingWidth_right, marker='x', linestyle='--', color='red', label='Right Swing Width Error')

ax.set_title('Swing Width Error')
ax.set_xlabel('Index of Gait Cycle')
ax.set_ylabel('Error (%)')
ax.grid(True)
ax.legend()
ax.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()


# COMPARISON PLOT
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].plot(percentageSwingWidthLeft_MATLAB, marker='o', linestyle='-', color='blue', label='Swing Width Left MATLAB')
axs[0].plot(percentageSwingWidthLeft_Python, marker='x', linestyle='--', color='red', label='Swing Width Left Python')
axs[0].set_title('Left Swing Width')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Percentage of Stride Length (%)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

axs[1].plot(percentageSwingWidthRight_MATLAB, marker='o', linestyle='-', color='green', label='Swing Width Right MATLAB')
axs[1].plot(percentageSwingWidthRight_Python, marker='x', linestyle='--', color='orange', label='Swing Width Right Python')
axs[1].set_title('Right Swing Width')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Percentage of Stride Length (%)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

fig.suptitle('Comparison of Swing Width', fontsize=16)

plt.tight_layout()


# COMPARISON BAR PLOT
matlab_left = [val[0][0] for val in percentageSwingWidthLeft_MATLAB]
matlab_right = [val[0][0] for val in percentageSwingWidthRight_MATLAB]
python_left = percentageSwingWidthLeft_Python
python_right = percentageSwingWidthRight_Python

indices = np.arange(len(matlab_left)) 

bar_width = 0.45

fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].bar(indices, matlab_left, bar_width, color='blue', label='Swing Width Left MATLAB')
axs[0].bar(indices + bar_width, python_left, bar_width, color='red', label='Swing Width Left Python')
axs[0].set_title('Left Swing Width')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Percentage of Stride Length (%)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

for i in range(len(matlab_left)):
    axs[0].text(indices[i], matlab_left[i] + 0.2, f'{matlab_left[i]:.0f}', ha='center', color='blue')
    axs[0].text(indices[i] + bar_width, python_left[i] + 1.8, f'{python_left[i]:.0f}', ha='center', color='red')

axs[1].bar(indices, matlab_right, bar_width, color='green', label='Swing Width Right MATLAB')
axs[1].bar(indices + bar_width, python_right, bar_width, color='orange', label='Swing Width Right Python')
axs[1].set_title('Right Swing Width')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Percentage of Stride Length (%)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

for i in range(len(matlab_right)):
    axs[1].text(indices[i], matlab_right[i] + 0.2, f'{matlab_right[i]:.0f}', ha='center', color='green')
    axs[1].text(indices[i] + bar_width, python_right[i] + 1.8, f'{python_right[i]:.0f}', ha='center', color='orange')

fig.suptitle('Comparison of Swing Width (Left and Right)', fontsize=16)

plt.tight_layout()



"""COMPARISON: PATH LENGTH"""
pathLengthLeft_MATLAB = KPI_data['pathLength'][0][0][0]['Left'][0][0]['percentageOfstrideLength']
pathLengthLeft_Python = [KPI_Python['pathLength']['Left'][i]['percentageOfStrideLength'] for i in range(len(KPI_Python['pathLength']['Left']))]
errorPathLength_left = pathLengthLeft_Python - pathLengthLeft_MATLAB

pathLengthRight_MATLAB = KPI_data['pathLength'][0][0][0]['Right'][0][0]['percentageOfstrideLength']
pathLengthRight_Python = [KPI_Python['pathLength']['Right'][i]['percentageOfStrideLength'] for i in range(len(KPI_Python['pathLength']['Right']))]
errorPathLength_right = pathLengthRight_Python - pathLengthRight_MATLAB

# ERROR PLOT
fig, ax = plt.subplots(figsize=(10, 6))

ax.plot(errorPathLength_left, color='blue', marker='o', linestyle='-', label='Left Path Length Error')
ax.plot(errorPathLength_right, color='red', marker='x', linestyle='--', label='Right Path Length Error')

ax.set_title('Path Length Error')
ax.set_xlabel('Index of Gait Cycle')
ax.set_ylabel('Error (%)')
ax.grid(True)
ax.legend()
ax.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()


# COMPARISON PLOT
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].plot(pathLengthLeft_MATLAB, color='blue', marker='o', linestyle='-', label='Path Length Left MATLAB')
axs[0].plot(pathLengthLeft_Python, color='red', marker='x', linestyle='--', label='Path Length Left Python')
axs[0].set_title('Left Path Length')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Percentage of Stride Length (%)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

axs[1].plot(pathLengthRight_MATLAB, color='green', marker='o', linestyle='-', label='Path Length Right MATLAB')
axs[1].plot(pathLengthRight_Python, color='orange', marker='x', linestyle='--', label='Path Length Right Python')
axs[1].set_title('Right Path Length')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Percentage of Stride Length of Stride Length (%)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

fig.suptitle('Comparison of Path Length', fontsize=16)

plt.tight_layout()


# COMPARISON BAR PLOT
matlab_left = [val[0][0] for val in pathLengthLeft_MATLAB]
matlab_right = [val[0][0] for val in pathLengthRight_MATLAB]
python_left = pathLengthLeft_Python
python_right = pathLengthRight_Python

indices = np.arange(len(matlab_left)) 

bar_width = 0.45

fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].bar(indices, matlab_left, bar_width, color='blue', label='Path Length Left MATLAB')
axs[0].bar(indices + bar_width, python_left, bar_width, color='red', label='Path Length Left Python')
axs[0].set_title('Left Path Length')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Percentage of Stride Length (%)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

for i in range(len(matlab_left)):
    axs[0].text(indices[i], matlab_left[i] + 0.2, f'{matlab_left[i]:.0f}', ha='center', color='blue')
    axs[0].text(indices[i] + bar_width, python_left[i] + 4.8, f'{python_left[i]:.0f}', ha='center', color='red')
    
axs[1].bar(indices, matlab_right, bar_width, color='green', label='Path Length Right MATLAB')
axs[1].bar(indices + bar_width, python_right, bar_width, color='orange', label='Path Length Right Python')
axs[1].set_title('Right Path Length')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Percentage of Stride Length (%)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

for i in range(len(matlab_right)):
    axs[1].text(indices[i], matlab_right[i] + 0.2, f'{matlab_right[i]:.0f}', ha='center', color='green')
    axs[1].text(indices[i] + bar_width, python_right[i] + 4.8, f'{python_right[i]:.0f}', ha='center', color='orange')
    
fig.suptitle('Comparison of Path Length', fontsize=16)

plt.tight_layout()



"""COMPARISON: STANCE PHASE"""
stancePercentageLeft_MATLAB = KPI_data['phasePercentageWrtCycleDuration'][0][0][0]['Left'][0][0]['percentageOfstance']
stancePercentageLeft_Python = [KPI_Python['phasePercentageWrtCycleDuration']['Left'][i]['percentageOfStance'] for i in range(len(KPI_Python['phasePercentageWrtCycleDuration']['Left']))]
errorStancePercentage_left = stancePercentageLeft_Python - stancePercentageLeft_MATLAB

stancePercentageRight_MATLAB = KPI_data['phasePercentageWrtCycleDuration'][0][0][0]['Right'][0][0]['percentageOfstance']
stancePercentageRight_Python = [KPI_Python['phasePercentageWrtCycleDuration']['Right'][i]['percentageOfStance'] for i in range(len(KPI_Python['phasePercentageWrtCycleDuration']['Right']))]
errorStancePercentage_right = stancePercentageRight_Python - stancePercentageRight_MATLAB

# ERROR PLOT
fig, ax = plt.subplots(figsize=(10, 6))

ax.plot(errorStancePercentage_left, color='blue', marker='o', linestyle='-', label='Left Stance Percentage Error')
ax.plot(errorStancePercentage_right, color='red', marker='x', linestyle='--', label='Right Stance Percentage Error')

ax.set_title('Stance Percentage Error')
ax.set_xlabel('Index of Gait Cycle')
ax.set_ylabel('Error (%)')
ax.grid(True)
ax.legend()
ax.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()

# COMPARISON PLOT
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].plot(stancePercentageLeft_MATLAB, marker='o', linestyle='-', color='blue', label='Stance Percentage Left MATLAB')
axs[0].plot(stancePercentageLeft_Python, marker='x', linestyle='--', color='red', label='Stance Percentage Left Python')

axs[0].set_title('Left Stance Percentage')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Percentage of Stance (%)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

axs[1].plot(stancePercentageRight_MATLAB, marker='o', linestyle='-', color='green', label='Stance Percentage Right MATLAB')
axs[1].plot(stancePercentageRight_Python, marker='x', linestyle='--', color='orange', label='Stance Percentage Right Python')

axs[1].set_title('Right Stance Percentage')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Percentage of Stance (%)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

fig.suptitle('Comparison of Stance Percentage', fontsize=16)

plt.tight_layout()


"""COMPARISON: SWING PHASE"""
swingPercentageLeft_MATLAB = KPI_data['phasePercentageWrtCycleDuration'][0][0][0]['Left'][0][0]['percentageOfswing']
swingPercentageLeft_Python = [KPI_Python['phasePercentageWrtCycleDuration']['Left'][i]['percentageOfSwing'] for i in range(len(KPI_Python['phasePercentageWrtCycleDuration']['Left']))]
errorSwingPercentage_left = swingPercentageLeft_Python - swingPercentageLeft_MATLAB

swingPercentageRight_MATLAB = KPI_data['phasePercentageWrtCycleDuration'][0][0][0]['Right'][0][0]['percentageOfswing']
swingPercentageRight_Python = [KPI_Python['phasePercentageWrtCycleDuration']['Right'][i]['percentageOfSwing'] for i in range(len(KPI_Python['phasePercentageWrtCycleDuration']['Right']))]
errorSwingPercentage_right = swingPercentageRight_Python - swingPercentageRight_MATLAB

# ERROR PLOT
fig, ax = plt.subplots(figsize=(10, 6))

ax.plot(errorSwingPercentage_left, color='blue', marker='o', linestyle='-', label='Left Swing Percentage Error')
ax.plot(errorSwingPercentage_right, color='red', marker='x', linestyle='--', label='Right Swing Percentage Error')

ax.set_title('Swing Percentage Error')
ax.set_xlabel('Index of Gait Cycle')
ax.set_ylabel('Error (%)')
ax.grid(True)
ax.legend()
ax.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()

# COMPARISON PLOT
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

axs[0].plot(swingPercentageLeft_MATLAB, marker='o', linestyle='-', color='blue', label='Swing Percentage Left MATLAB')
axs[0].plot(swingPercentageLeft_Python, marker='x', linestyle='--', color='red', label='Swing Percentage Left Python')

axs[0].set_title('Left Swing Percentage')
axs[0].set_xlabel('Index of Gait Cycle')
axs[0].set_ylabel('Percentage of Swing (%)')
axs[0].grid(True)
axs[0].legend()
axs[0].autoscale(enable=True, axis='x', tight=True)

axs[1].plot(swingPercentageRight_MATLAB, marker='o', linestyle='-', color='green', label='Swing Percentage Right MATLAB')
axs[1].plot(swingPercentageRight_Python, marker='x', linestyle='--', color='orange', label='Swing Percentage Right Python')

axs[1].set_title('Right Swing Percentage')
axs[1].set_xlabel('Index of Gait Cycle')
axs[1].set_ylabel('Percentage of Swing (%)')
axs[1].grid(True)
axs[1].legend()
axs[1].autoscale(enable=True, axis='x', tight=True)

fig.suptitle('Comparison of Swing Percentage', fontsize=16)

plt.tight_layout()


"""COMPARISON: CENTER OF MASS (COM)"""
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')
fig.suptitle('COM displacement w.r.t. feet and hands', fontsize=16)
ax.grid(True)

# COM position wrt feet and hands positions
s = iDynTree.JointPosDoubleArray(kinDynComp.model())
ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
base_vel = iDynTree.Twist()
gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])

position = {'RF': np.zeros((3,KPI_Python['len'])), 'LF': np.zeros((3,KPI_Python['len'])), 'RH': np.zeros((3,KPI_Python['len'])), 'LH': np.zeros((3,KPI_Python['len']))}

for lenIdx in range(KPI_Python['len']):
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
plotCoMpos_Python, = ax.plot(KPI_Python['COM']['position'][0, 1:], KPI_Python['COM']['position'][1, 1:], KPI_Python['COM']['position'][2, 1:], 'k', linewidth=1.5, label='COM Python')
plotCoMpos_Matlab, = ax.plot(KPI_data['COM'][0][0][0]['position'][0][0, 1:], KPI_data['COM'][0][0][0]['position'][0][1, 1:], KPI_data['COM'][0][0][0]['position'][0][2, 1:], 'g', linewidth=1.5, linestyle='--', label='COM Matlab')
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

plt.tight_layout()


"""COMPARISON: COM VELOCITY"""
vel_yLabels = ['Vx', 'Vy', 'Vz']

fig, axs = plt.subplots(1, 3, figsize=(8, 8))

for plotIdx in range(3):  
    ax = axs[plotIdx]
    ax.plot(human_state['base_position']['timestamps'], KPI_Python['COM']['velocity'][plotIdx, :], 'b', linewidth=2, marker='o', linestyle='-', label='COM Python')
    ax.plot(human_state['base_position']['timestamps'], KPI_data['COM'][0][0][0]['velocity'][0][plotIdx, :], 'r', linewidth=2, marker='x', linestyle='--', label='COM Matlab')
    ax.grid(True)
    if plotIdx == 1:
        ax.set_title('COM velocity', fontsize=16)
        
    # LABELS
    ax.set_xlabel('Time [s]', fontsize=14)
    ax.set_ylabel(f'{vel_yLabels[plotIdx]} [m/s]', fontsize=14)
    ax.tick_params(axis='both', labelsize=15)

    ax.autoscale(enable=True, axis='x', tight=True)
    
# ERROR PLOT
fig, axs = plt.subplots(1, 3, figsize=(14, 8))

for plotIdx in range(3):  
    ax = axs[plotIdx]
    ax.plot(human_state['base_position']['timestamps'], KPI_Python['COM']['velocity'][plotIdx, :] - KPI_data['COM'][0][0][0]['velocity'][0][plotIdx, :], 'k', linewidth=2, linestyle='-', label='COM Python')
    ax.grid(True)
    if plotIdx == 1:
        ax.set_title('COM velocity error', fontsize=16)
        
    # LABELS
    ax.set_xlabel('Time [s]', fontsize=10)
    ax.set_ylabel(f'{vel_yLabels[plotIdx]} [m/s]', fontsize=10)
    ax.tick_params(axis='both', labelsize=10)

    ax.autoscale(enable=True, axis='x', tight=True)

    plt.tight_layout()
    
"""COMPARISON: COP vs COM"""
fig,ax = plt.subplots(figsize=(8, 8))

fig.suptitle('COP vs COM expressed w.r.t. world', fontsize=16)
ax.grid(True)

# PLOT
plotCOMpos_Python = ax.plot(KPI_Python['COM']['position'][0, 1:], KPI_Python['COM']['position'][1, 1:], 'k', linewidth=1.5, label='COM')
plotCoMpos_Matlab = ax.plot(KPI_data['COM'][0][0][0]['position'][0][0, 1:], KPI_data['COM'][0][0][0]['position'][0][1, 1:], 'm', linewidth=1.5, linestyle='--', label='COM Matlab')

plotCOPleft_Python = ax.plot(KPI_Python['COP']['total']['Left']['wrtWorld'][0, :], KPI_Python['COP']['total']['Left']['wrtWorld'][1, :], 'or', linewidth=1.5, label='COP Left Python')
plotCOPleft_Matlab = ax.plot(KPI_data['COP'][0][0][0]['total'][0]['Left'][0][0]['wrtWorld'][0][0][0,1:], KPI_data['COP'][0][0][0]['total'][0]['Left'][0][0]['wrtWorld'][0][0][1,1:], 'og', markersize=2,  label='COP Left Matlab')
plotCOPright_Python = ax.plot(KPI_Python['COP']['total']['Right']['wrtWorld'][0, :], KPI_Python['COP']['total']['Right']['wrtWorld'][1, :], 'ob', linewidth=1.5, label='COP Right Python')
plotCOPright_Matlab = ax.plot(KPI_data['COP'][0][0][0]['total'][0]['Right'][0][0]['wrtWorld'][0][0][0,1:], KPI_data['COP'][0][0][0]['total'][0]['Right'][0][0]['wrtWorld'][0][0][1,1:], 'oy', markersize=2, label='COP Right Matlab')

# LABELS
ax.set_xlabel('x [m]', fontsize=14)
ax.set_ylabel('y [m]', fontsize=14)

ax.legend(loc='upper right', fontsize=10)


"""COMPARISON: CONTACT PATTERN"""




    
    
        


plt.show()





