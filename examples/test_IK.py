import biomechanical_analysis_framework as baf
import bipedal_locomotion_framework.bindings as blf
import idyntree.swig as idyn
import icub_models
import numpy as np
import resolve_robotics_uri_py
import pathlib
import h5py
import manifpy as manif

def get_kindyn():

    joints_list = ["neck_pitch", "neck_roll", "neck_yaw",
                   "torso_pitch", "torso_roll", "torso_yaw",
                   "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
                   "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
                   "l_hip_pitch", "l_hip_roll", "l_hip_yaw","l_knee", "l_ankle_pitch", "l_ankle_roll",
                   "r_hip_pitch", "r_hip_roll", "r_hip_yaw","r_knee", "r_ankle_pitch", "r_ankle_roll"]

    model_loader = idyn.ModelLoader()
    # assert model_loader.loadReducedModelFromFile(str(icub_models.get_model_file("ergoCubSN001")), joints_list)

    urdf_path = str(resolve_robotics_uri_py.resolve_robotics_uri("package://ergoCub/robots/ergoCubSN001/model.urdf"))
    assert model_loader.loadReducedModelFromFile(str(urdf_path), joints_list)

    # create KinDynComputationsDescriptor
    kindyn = idyn.KinDynComputations()
    assert kindyn.loadRobotModel(model_loader.model())

    return joints_list, kindyn

def normalize_quat(quat):
    norm = np.linalg.norm(quat)
    return [quat[1] / norm, quat[2] / norm, quat[3] / norm, quat[0] / norm]

print("test_IK")

joints_list, kindyn = get_kindyn()

# create KinDynComputationsDescriptor
assert kindyn.getNrOfDegreesOfFreedom() == len(joints_list)

# Set the joint positions to random values
joint_values = [np.random.uniform(-0.5, 0.5) for _ in range(kindyn.getNrOfDegreesOfFreedom())]
assert kindyn.setJointPos(joint_values)

# Set the robot state
updated_world_T_base = np.array([[1., 0., 0., 0.],[0., 0., -1., 0.],[0., 1., 0., 0.],[0., 0., 0., 1.]])
updated_s = [np.random.uniform(-0.5,0.5) for _ in range(kindyn.getNrOfDegreesOfFreedom())]
updated_base_velocity = [np.random.uniform(-0.5,0.5) for _ in range(6)]
updated_s_dot = [np.random.uniform(-0.5,0.5) for _ in range(kindyn.getNrOfDegreesOfFreedom())]
updated_world_gravity = [np.random.uniform(-0.5,0.5) for _ in range(3)]
assert kindyn.setRobotState(updated_world_T_base,updated_s,updated_base_velocity,
                            updated_s_dot,updated_world_gravity)

humanIK = baf.ik.HumanIK()
# empty_handler = blf.parameters_handler.TomlParametersHandler()

# Set the parameters from toml file
qp_ik_params = blf.parameters_handler.TomlParametersHandler()
toml = pathlib.Path("test_IK.toml").expanduser()
assert toml.is_file()
ok = qp_ik_params.set_from_file(str(toml))
# qp_ik_params.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")

humanIK.initialize(qp_ik_params, kindyn)

# Create the node struct (dict)
# Read in the data
mocap_filename = "/home/evelyd/pi_learning_for_collaborative_carrying/datasets/collaborative_payload_carrying/leader_backward/follower.mat"
mocap_data = h5py.File(mocap_filename, 'r')
mocap_data_cleaned = mocap_data['robot_logger_device']

# Cut the data at the start time
# Get the index of the timestamp closest to the start time
start_time = 12.59
zeroed_timestamps = np.squeeze(mocap_data_cleaned['shoe1']['FT']['timestamps'][:] - mocap_data_cleaned['shoe1']['FT']['timestamps'][0])
start_time_index = np.argmin(np.abs(zeroed_timestamps - start_time))

# Assign the data for the SO3 and Gravity tasks into a struct
# Create the list of nodes in order of the toml file
pose_nodes = [3]
orientation_nodes = [6, 7, 8, 5, 4, 11, 12, 9, 10]
floor_contact_nodes = [1, 2]

node_struct = {}
for node in orientation_nodes + floor_contact_nodes:
    # Define time series of rotations for this node
    I_R_IMU = [manif.SO3(quaternion=normalize_quat([quat[1], quat[2], quat[3], quat[0]])) for quat in np.squeeze(mocap_data_cleaned['node' + str(node)]['orientation']['data'][start_time_index:])]
    # Define time series of angular velocities for this node
    I_omega_IMU = [manif.SO3Tangent(omega) for omega in np.squeeze(mocap_data_cleaned['node' + str(node)]['angVel']['data'][start_time_index:])]
    # Assign these values to the node struct
    # node_struct[node] = {'I_R_IMU': I_R_IMU, 'I_omega_IMU': I_omega_IMU}
    nodeData = baf.ik.nodeData()
    nodeData.I_R_IMU = I_R_IMU[0]
    nodeData.I_omega_IMU = I_omega_IMU[0]
    nodeData.I_position = [0., 0., 0.]
    nodeData.I_linearVelocity = [0., 0., 0.]
    node_struct[node] = nodeData

pose_node_struct = {}
for node in pose_nodes:
    # Define time series of rotations for this node
    I_R_IMU = [manif.SO3(quaternion=normalize_quat([quat[1], quat[2], quat[3], quat[0]])) for quat in np.squeeze(mocap_data_cleaned['node' + str(node)]['orientation']['data'][start_time_index:])]
    # Define time series of angular velocities for this node
    I_omega_IMU = [manif.SO3Tangent(omega) for omega in np.squeeze(mocap_data_cleaned['node' + str(node)]['angVel']['data'][start_time_index:])]
    # Assign these values to the node struct
    # node_struct[node] = {'I_R_IMU': I_R_IMU, 'I_omega_IMU': I_omega_IMU}
    nodeData = baf.ik.nodeData()
    nodeData.I_R_IMU = I_R_IMU[0]
    nodeData.I_omega_IMU = I_omega_IMU[0]
    nodeData.I_position = [0., 0., 0.]
    nodeData.I_linearVelocity = [0., 0., 0.]
    pose_node_struct[node] = nodeData

# Put robot in T pose
calib_joint_positions = np.array([0.]*len(joints_list))
calib_joint_positions[joints_list.index("l_shoulder_roll")] = np.pi/2
calib_joint_positions[joints_list.index("r_shoulder_roll")] = np.pi/2

humanIK.calibrateWorldYaw(node_struct)
humanIK.calibrateAllWithWorld(node_struct, "l_sole")

humanIK.updatePoseTask(3, pose_node_struct[3].I_position, pose_node_struct[3].I_R_IMU, pose_node_struct[3].I_linearVelocity, pose_node_struct[3].I_omega_IMU)
humanIK.updatePoseTasks(pose_node_struct)