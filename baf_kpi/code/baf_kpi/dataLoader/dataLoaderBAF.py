import h5py
import numpy as np
import os

from baf_kpi.scripts.computeBaseTransformWrtWorldBAF import computeBaseTransformWrtWorldBAF



# Global variables
DATA_LENGTH = None
TIMESTAMPS = None

human_state = {}
joints_state = {}
human_wrench = {}
human_dynamics = {}
node1 = {}
node2 = {}
node3 = {}
node4 = {}
node5 = {}
node6 = {}
node7 = {}
node8 = {}
node9 = {}
node10 = {}
node11 = {}
node12 = {}

resamplingFrequency = float(30)

def loadDataBAF(paths):
    
    file_path = readPath(paths)

    with h5py.File(file_path, 'r') as f:
        
        robot_logger_device = f['robot_logger_device']
        
        # Human state data
        if 'human_state' in robot_logger_device:
            human_state = loadHumanState(robot_logger_device)
        else:
            print("'human_state' not found in 'robot_logger_device'.")

        # Joints state data
        if 'joints_state' in robot_logger_device:
            joints_state = loadJointsState(robot_logger_device)
        else:
            print("'joints_state' not found in 'robot_logger_device'.")

        # Human wrench data
        if 'human_wrench' in robot_logger_device:
            human_wrench = loadHumanWrench(robot_logger_device)
        else:
            print("'human_wrench' not found in 'robot_logger_device'.")

        # Human dynamics data
        if 'human_dynamics' in robot_logger_device:
            human_dynamics = loadHumanDynamics(robot_logger_device)        
        else:
            print("'human_dynamics' not found in 'robot_logger_device'.")


        nodes = loadNodes(robot_logger_device)
        
        human_state, joints_state, nodes, human_dynamics, TIMESTAMPS, DATA_LENGTH = dataSynchronization(human_state, joints_state, nodes, human_dynamics)
        
        
        W_T_B = computeBaseTransformWrtWorldBAF(human_state)
        
        
        DATA_LENGTH = len(human_state['base_position']['data']) 
        TIMESTAMPS = np.array(human_state['base_position']['timestamps'])[:-1].squeeze()
        durationInSec = TIMESTAMPS[-1] - TIMESTAMPS[0]
        
        
            
    return human_state, joints_state, human_wrench, human_dynamics, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B
      
def loadNodes(robot_logger_device):
    # Node data
    if 'node1' in robot_logger_device:
        node1 = robot_logger_device['node1']

        node1 = {
            'FT': {
                'data': np.array(node1['FT']['data']),
                'dimensions': np.array(node1['FT']['dimensions']),
                # 'elements_names': np.array(node1['FT']['elements_names']),
                # 'units_of_measure': np.array(node1['FT']['units_of_measure']),
                # 'name': np.array(node1['FT']['name']),
                'timestamps': np.array(node1['FT']['timestamps'])
            },
            'angVel': {
                'data': np.array(node1['angVel']['data']),
                'dimensions': np.array(node1['angVel']['dimensions']),
                # 'elements_names': np.array(node1['angVel']['elements_names']),
                # 'units_of_measure': np.array(node1['angVel']['units_of_measure']),
                # 'name': np.array(node1['angVel']['name']),
                'timestamps': np.array(node1['angVel']['timestamps'])
            },
            'linAcc': {
                'data': np.array(node1['linAcc']['data']),
                'dimensions': np.array(node1['linAcc']['dimensions']),
                # 'elements_names': np.array(node1['linAcc']['elements_names']),
                # 'units_of_measure': np.array(node1['linAcc']['units_of_measure']),
                # 'name': np.array(node1['linAcc']['name']),
                'timestamps': np.array(node1['linAcc']['timestamps'])
            },
            'orientation': {
                'data': np.array(node1['orientation']['data']),
                'dimensions': np.array(node1['orientation']['dimensions']),
                # 'elements_names': np.array(node1['orientation']['elements_names']),
                # 'units_of_measure': np.array(node1['orientation']['units_of_measure']),
                # 'name': np.array(node1['orientation']['name']),
                'timestamps': np.array(node1['orientation']['timestamps'])
            }
        }
    else:
        print("'node1' not found in 'robot_logger_device'.")
        
    if 'node2' in robot_logger_device:
        node2 = robot_logger_device['node2']

        node2 = {
            'FT': {
                'data': np.array(node2['FT']['data']),
                'dimensions': np.array(node2['FT']['dimensions']),
                # 'elements_names': np.array(node2['FT']['elements_names']),
                # 'units_of_measure': np.array(node2['FT']['units_of_measure']),
                # 'name': np.array(node2['FT']['name']),
                'timestamps': np.array(node2['FT']['timestamps'])
            },
            'angVel': {
                'data': np.array(node2['angVel']['data']),
                'dimensions': np.array(node2['angVel']['dimensions']),
                # 'elements_names': np.array(node2['angVel']['elements_names']),
                # 'units_of_measure': np.array(node2['angVel']['units_of_measure']),
                # 'name': np.array(node2['angVel']['name']),
                'timestamps': np.array(node2['angVel']['timestamps'])
            },
            'linAcc': {
                'data': np.array(node2['linAcc']['data']),
                'dimensions': np.array(node2['linAcc']['dimensions']),
                # 'elements_names': np.array(node2['linAcc']['elements_names']),
                # 'units_of_measure': np.array(node2['linAcc']['units_of_measure']),
                # 'name': np.array(node2['linAcc']['name']),
                'timestamps': np.array(node2['linAcc']['timestamps'])
            },
            'orientation': {
                'data': np.array(node2['orientation']['data']),
                'dimensions': np.array(node2['orientation']['dimensions']),
                # 'elements_names': np.array(node2['orientation']['elements_names']),
                # 'units_of_measure': np.array(node2['orientation']['units_of_measure']),
                # 'name': np.array(node2['orientation']['name']),
                'timestamps': np.array(node2['orientation']['timestamps'])
            }
        }
    else:
        print("'node2' not found in 'robot_logger_device'.")
        
    
    nodes = [] 

    for i in range(2, 12):
        node_name = 'node' + str(i + 1)  

        if node_name in robot_logger_device:
            node_data = robot_logger_device[node_name]

            node_data = {
                'angVel': {
                    'data': np.array(node_data['angVel']['data']),
                    'dimensions': np.array(node_data['angVel']['dimensions']),
                    # 'elements_names': np.array(node_data['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node_data['angVel']['units_of_measure']),
                    # 'name': np.array(node_data['angVel']['name']),
                    'timestamps': np.array(node_data['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node_data['linAcc']['data']),
                    'dimensions': np.array(node_data['linAcc']['dimensions']),
                    # 'elements_names': np.array(node_data['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node_data['linAcc']['units_of_measure']),
                    # 'name': np.array(node_data['linAcc']['name']),
                    'timestamps': np.array(node_data['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node_data['orientation']['data']),
                    'dimensions': np.array(node_data['orientation']['dimensions']),
                    # 'elements_names': np.array(node_data['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node_data['orientation']['units_of_measure']),
                    # 'name': np.array(node_data['orientation']['name']),
                    'timestamps': np.array(node_data['orientation']['timestamps'])
                }
            }

            nodes.append(node_data)
        else:
            print(f"'{node_name}' not found in 'robot_logger_device'.")
                
        
    nodes = [node1, node2] + nodes    
    
    return nodes  


def loadHumanState(robot_logger_device):
    human_state = robot_logger_device['human_state']
    human_state = {
        'base_position': {
            'data': np.array(human_state['base_position']['data']),
            'dimensions': np.array(human_state['base_position']['dimensions']),
            # 'elements_names': np.array(human_state['base_position']['elements_names']),
            # 'units_of_measure': np.array(human_state['base_position']['units_of_measure']),
            # 'name': np.array(human_state['base_position']['name']),
            'timestamps': np.array(human_state['base_position']['timestamps'])
        },
        'base_orientation': {
            'data': np.array(human_state['base_orientation']['data']),
            'dimensions': np.array(human_state['base_orientation']['dimensions']),
            # 'elements_names': np.array(human_state['base_orientation']['elements_names']),
            # 'units_of_measure': np.array(human_state['base_orientation']['units_of_measure']),
            # 'name': np.array(human_state['base_orientation']['name']),
            'timestamps': np.array(human_state['base_orientation']['timestamps'])
        },
        'base_angular_velocity': {
            'data': np.array(human_state['base_angular_velocity']['data']),
            'dimensions': np.array(human_state['base_angular_velocity']['dimensions']),
            # 'elements_names': np.array(human_state['base_angular_velocity']['elements_names']),
            # 'units_of_measure': np.array(human_state['base_angular_velocity']['units_of_measure']),
            # 'name': np.array(human_state['base_angular_velocity']['name']),
            'timestamps': np.array(human_state['base_angular_velocity']['timestamps'])
        },
        'base_linear_velocity': {
            'data': np.array(human_state['base_linear_velocity']['data']),
            'dimensions': np.array(human_state['base_linear_velocity']['dimensions']),
            # 'elements_names': np.array(human_state['base_linear_velocity']['elements_names']),
            # 'units_of_measure': np.array(human_state['base_linear_velocity']['units_of_measure']),
            # 'name': np.array(human_state['base_linear_velocity']['name']),
            'timestamps': np.array(human_state['base_linear_velocity']['timestamps'])
        }
    }

            
    return human_state


def loadJointsState(robot_logger_device):
    joints_state = robot_logger_device['joints_state']
    joints_state = {
        'positions': {
            'data': np.array(joints_state['positions']['data']),
            'dimensions': np.array(joints_state['positions']['dimensions']),
            # 'elements_names': np.array(joints_state['positions']['elements_names']),
            # 'units_of_measure': np.array(joints_state['positions']['units_of_measure']),
            # 'name': np.array(joints_state['positions']['name']),
            'timestamps': np.array(joints_state['positions']['timestamps'])
        },
        'velocities': {
            'data': np.array(joints_state['velocities']['data']),
            'dimensions': np.array(joints_state['velocities']['dimensions']),
            # 'elements_names': np.array(joints_state['velocities']['elements_names']),
            # 'units_of_measure': np.array(joints_state['velocities']['units_of_measure']),
            # 'name': np.array(joints_state['velocities']['name']),
            'timestamps': np.array(joints_state['velocities']['timestamps'])
        }
    }
    return joints_state


def loadHumanWrench(robot_logger_device):
    human_wrench = robot_logger_device['human_wrench']
    human_wrench = {
        'RightHandCOM': {
            'data': np.array(human_wrench['wrenches']['RightHandCOM']['data']),
            'dimensions': np.array(human_wrench['wrenches']['RightHandCOM']['dimensions']),
            # 'elements_names': np.array(human_wrench['wrenches']['RightHandCOM']['elements_names']),
            # 'units_of_measure': np.array(human_wrench['wrenches']['RightHandCOM']['units_of_measure']),
            # 'name': np.array(human_wrench['wrenches']['RightHandCOM']['name']),
            'timestamps': np.array(human_wrench['wrenches']['RightHandCOM']['timestamps'])
        },
        'LeftHandCOM': {
            'data': np.array(human_wrench['wrenches']['LeftHandCOM']['data']),
            'dimensions': np.array(human_wrench['wrenches']['LeftHandCOM']['dimensions']),
            # 'elements_names': np.array(human_wrench['wrenches']['LeftHandCOM']['elements_names']),
            # 'units_of_measure': np.array(human_wrench['wrenches']['LeftHandCOM']['units_of_measure']),
            # 'name': np.array(human_wrench['wrenches']['LeftHandCOM']['name']),
            'timestamps': np.array(human_wrench['wrenches']['LeftHandCOM']['timestamps'])
        },
        'RightFoot' : {
            'data': np.array(human_wrench['wrenches']['RightFoot']['data']),
            'dimensions': np.array(human_wrench['wrenches']['RightFoot']['dimensions']),
            # 'elements_names': np.array(human_wrench['wrenches']['RightFoot']['elements_names']),
            # 'units_of_measure': np.array(human_wrench['wrenches']['RightFoot']['units_of_measure']),
            # 'name': np.array(human_wrench['wrenches']['RightFoot']['name']),
            'timestamps': np.array(human_wrench['wrenches']['RightFoot']['timestamps'])
        },
        'LeftFoot' : {
            'data': np.array(human_wrench['wrenches']['LeftFoot']['data']),
            'dimensions': np.array(human_wrench['wrenches']['LeftFoot']['dimensions']),
            # 'elements_names': np.array(human_wrench['wrenches']['LeftFoot']['elements_names']),
            # 'units_of_measure': np.array(human_wrench['wrenches']['LeftFoot']['units_of_measure']),
            # 'name': np.array(human_wrench['wrenches']['LeftFoot']['name']),
            'timestamps': np.array(human_wrench['wrenches']['LeftFoot']['timestamps'])
        }
    }
    return human_wrench


def loadHumanDynamics(robot_logger_device):
    human_dynamics = robot_logger_device['human_dynamics']
    human_dynamics = {
        'joint_torques': {
            'data': np.array(human_dynamics['joint_torques']['data']),
            'dimensions': np.array(human_dynamics['joint_torques']['dimensions']),
            # 'elements_names': np.array(human_dynamics['joint_torques']['elements_names']),
            # 'units_of_measure': np.array(human_dynamics['joint_torques']['units_of_measure']),
            # 'name': np.array(human_dynamics['joint_torques']['name']),
            'timestamps': np.array(human_dynamics['joint_torques']['timestamps'])
        }
    }
    return human_dynamics




def dataSynchronization(human_state, joints_state, nodes, human_dynamics):
    """   DATA SYNCHRONIZATION   """
    # The data are not perfectly synchronized, so we need to synchronize them
    
    # STEP 1 : Replace NaN (if) with existing valid samples
    # human_state - base_position
    nan_indices_basepos = np.argwhere(np.isnan(human_state['base_position']['data']))
    if nan_indices_basepos.size > 0:
        for row_idx, col_idx in nan_indices_basepos:
            if col_idx == 0:
                human_state['base_position']['data'][row_idx, col_idx] = human_state['base_position']['data'][row_idx, col_idx + 1]
            else:
                human_state['base_position']['data'][row_idx, col_idx] = human_state['base_position']['data'][row_idx, col_idx - 1]
                
    # human_state - base_orientation
    nan_indices_baseori = np.argwhere(np.isnan(human_state['base_orientation']['data']))
    if nan_indices_baseori.size > 0:
        for row_idx, col_idx in nan_indices_baseori:
            if col_idx == 0:
                human_state['base_orientation']['data'][row_idx, col_idx] = human_state['base_orientation']['data'][row_idx, col_idx + 1]
            else:
                human_state['base_orientation']['data'][row_idx, col_idx] = human_state['base_orientation']['data'][row_idx, col_idx - 1]
    
    # human_state - base_angular_velocity
    nan_indices_baseangvel = np.argwhere(np.isnan(human_state['base_angular_velocity']['data']))
    if nan_indices_baseangvel.size > 0:
        for row_idx, col_idx in nan_indices_baseangvel:
            if col_idx == 0:
                human_state['base_angular_velocity']['data'][row_idx, col_idx] = human_state['base_angular_velocity']['data'][row_idx, col_idx + 1]
            else:
                human_state['base_angular_velocity']['data'][row_idx, col_idx] = human_state['base_angular_velocity']['data'][row_idx, col_idx - 1]
    
    # human_state - base_linear_velocity
    nan_indices_baselinvel = np.argwhere(np.isnan(human_state['base_linear_velocity']['data']))
    if nan_indices_baselinvel.size > 0:
        for row_idx, col_idx in nan_indices_baselinvel:
            if col_idx == 0:
                human_state['base_linear_velocity']['data'][row_idx, col_idx] = human_state['base_linear_velocity']['data'][row_idx, col_idx + 1]
            else:
                human_state['base_linear_velocity']['data'][row_idx, col_idx] = human_state['base_linear_velocity']['data'][row_idx, col_idx - 1]
                
    # joints_state - positions
    nan_indices_jointspos = np.argwhere(np.isnan(joints_state['positions']['data']))
    if nan_indices_jointspos.size > 0:
        for row_idx, col_idx in nan_indices_jointspos:
            if col_idx == 0:
                joints_state['positions']['data'][row_idx, col_idx] = joints_state['positions']['data'][row_idx, col_idx + 1]
            else:
                joints_state['positions']['data'][row_idx, col_idx] = joints_state['positions']['data'][row_idx, col_idx - 1]
    
    # joints_state - velocities
    nan_indices_jointsvel = np.argwhere(np.isnan(joints_state['velocities']['data']))
    if nan_indices_jointsvel.size > 0:
        for row_idx, col_idx in nan_indices_jointsvel:
            if col_idx == 0:
                joints_state['velocities']['data'][row_idx, col_idx] = joints_state['velocities']['data'][row_idx, col_idx + 1]
            else:
                joints_state['velocities']['data'][row_idx, col_idx] = joints_state['velocities']['data'][row_idx, col_idx - 1]
                
    # nodes - FT
    for node in range(0,2):
        nan_indices_FT = np.argwhere(np.isnan(nodes[node]['FT']['data']))
        if nan_indices_FT.size > 0:
            for row_idx, col_idx in nan_indices_FT:
                if col_idx == 0:
                    nodes[node]['FT']['data'][row_idx, col_idx] = nodes[node]['FT']['data'][row_idx, col_idx + 1]
                else:
                    nodes[node]['FT']['data'][row_idx, col_idx] = nodes[node]['FT']['data'][row_idx, col_idx - 1]
        
    # nodes - angVel
    for node in nodes:
        nan_indices_angvel = np.argwhere(np.isnan(node['angVel']['data']))
        if nan_indices_angvel.size > 0:
            for row_idx, col_idx in nan_indices_angvel:
                if col_idx == 0:
                    node['angVel']['data'][row_idx, col_idx] = node['angVel']['data'][row_idx, col_idx + 1]
                else:
                    node['angVel']['data'][row_idx, col_idx] = node['angVel']['data'][row_idx, col_idx - 1]
    
    # nodes - linAcc
    for node in nodes:
        nan_indices_linacc = np.argwhere(np.isnan(node['linAcc']['data']))
        if nan_indices_linacc.size > 0:
            for row_idx, col_idx in nan_indices_linacc:
                if col_idx == 0:
                    node['linAcc']['data'][row_idx, col_idx] = node['linAcc']['data'][row_idx, col_idx + 1]
                else:
                    node['linAcc']['data'][row_idx, col_idx] = node['linAcc']['data'][row_idx, col_idx - 1]
    
    # nodes - orientation
    for node in nodes:
        nan_indices_orientation = np.argwhere(np.isnan(node['orientation']['data']))
        if nan_indices_orientation.size > 0:
            for row_idx, col_idx in nan_indices_orientation:
                if col_idx == 0:
                    node['orientation']['data'][row_idx, col_idx] = node['orientation']['data'][row_idx, col_idx + 1]
                else:
                    node['orientation']['data'][row_idx, col_idx] = node['orientation']['data'][row_idx, col_idx - 1]
                    
    # human_dynamics - joint_torques
    nan_indices_jointtorques = np.argwhere(np.isnan(human_dynamics['joint_torques']['data']))
    if nan_indices_jointtorques.size > 0:
        for row_idx, col_idx in nan_indices_jointtorques:
            if col_idx == 0:
                human_dynamics['joint_torques']['data'][row_idx, col_idx] = human_dynamics['joint_torques']['data'][row_idx, col_idx + 1]
            else:
                human_dynamics['joint_torques']['data'][row_idx, col_idx] = human_dynamics['joint_torques']['data'][row_idx, col_idx - 1]
                    
                    
    # STEP 2 : Signals re-alignment and interpolation
    timestamp_human_state, idxTS_human_state = np.unique(human_state['base_position']['timestamps'].squeeze(), return_index=True)
    timestamp_joints_state, idxTS_joints_state = np.unique(joints_state['positions']['timestamps'].squeeze(), return_index=True)
    timestamp_nodes, idxTS_nodes = np.unique(nodes[0]['FT']['timestamps'].squeeze(), return_index=True)
    timestamp_human_dyn, idxTS_human_dyn = np.unique(human_dynamics['joint_torques']['timestamps'].squeeze(), return_index=True)
    
    timestamp_resyncro = np.arange(max(timestamp_human_state[0], timestamp_joints_state[0], timestamp_nodes[0], timestamp_human_dyn[0]), min(timestamp_human_state[-1], timestamp_joints_state[-1], timestamp_nodes[-1], timestamp_human_dyn[-1]), round(1 / resamplingFrequency, 15))
    
    unifiedTimeStamps = timestamp_resyncro
    
    TIMESTAMPS = unifiedTimeStamps
    DATA_LENGTH = len(unifiedTimeStamps)
    
    # Update human state data
    human_state['base_position']['data'] = interpolate(timestamp_human_state, human_state['base_position']['data'].squeeze(), timestamp_resyncro, idxTS_human_state)
    human_state['base_position']['timestamps'] = timestamp_resyncro
    human_state['base_orientation']['data'] = interpolate(timestamp_human_state, human_state['base_orientation']['data'].squeeze(), timestamp_resyncro, idxTS_human_state)
    human_state['base_orientation']['timestamps'] = timestamp_resyncro
    human_state['base_angular_velocity']['data'] = interpolate(timestamp_human_state, human_state['base_angular_velocity']['data'].squeeze(), timestamp_resyncro, idxTS_human_state)
    human_state['base_angular_velocity']['timestamps'] = timestamp_resyncro
    human_state['base_linear_velocity']['data'] = interpolate(timestamp_human_state, human_state['base_linear_velocity']['data'].squeeze(), timestamp_resyncro, idxTS_human_state)
    human_state['base_linear_velocity']['timestamps'] = timestamp_resyncro
    
    # Update joints state data
    joints_state['positions']['data'] = interpolate(timestamp_joints_state, joints_state['positions']['data'].squeeze(), timestamp_resyncro, idxTS_joints_state)
    joints_state['positions']['timestamps'] = timestamp_resyncro
    joints_state['velocities']['data'] = interpolate(timestamp_joints_state, joints_state['velocities']['data'].squeeze(), timestamp_resyncro, idxTS_joints_state)
    joints_state['velocities']['timestamps'] = timestamp_resyncro
    
    # Update nodes data
    nodes[0]['FT']['data'] = interpolate(timestamp_nodes, nodes[0]['FT']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[0]['FT']['timestamps'] = timestamp_resyncro
    nodes[0]['angVel']['data'] = interpolate(timestamp_nodes, nodes[0]['angVel']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[0]['angVel']['timestamps'] = timestamp_resyncro
    nodes[0]['linAcc']['data'] = interpolate(timestamp_nodes, nodes[0]['linAcc']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[0]['linAcc']['timestamps'] = timestamp_resyncro
    nodes[0]['orientation']['data'] = interpolate(timestamp_nodes, nodes[0]['orientation']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[0]['orientation']['timestamps'] = timestamp_resyncro
    
    nodes[1]['FT']['data'] = interpolate(timestamp_nodes, nodes[1]['FT']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[1]['FT']['timestamps'] = timestamp_resyncro
    nodes[1]['angVel']['data'] = interpolate(timestamp_nodes, nodes[1]['angVel']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[1]['linAcc']['timestamps'] = timestamp_resyncro
    nodes[1]['linAcc']['data'] = interpolate(timestamp_nodes, nodes[1]['linAcc']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[1]['angVel']['timestamps'] = timestamp_resyncro
    nodes[1]['orientation']['data'] = interpolate(timestamp_nodes, nodes[1]['orientation']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
    nodes[1]['orientation']['timestamps'] = timestamp_resyncro
    
    for node_idx in range(2, len(nodes)):
        nodes[node_idx]['angVel']['data'] = interpolate(timestamp_nodes, nodes[node_idx]['angVel']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
        nodes[node_idx]['angVel']['timestamps'] = timestamp_resyncro
        nodes[node_idx]['linAcc']['data'] = interpolate(timestamp_nodes, nodes[node_idx]['linAcc']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
        nodes[node_idx]['linAcc']['timestamps'] = timestamp_resyncro
        nodes[node_idx]['orientation']['data'] = interpolate(timestamp_nodes, nodes[node_idx]['orientation']['data'].squeeze(), timestamp_resyncro, idxTS_nodes)
        nodes[node_idx]['orientation']['timestamps'] = timestamp_resyncro
        
        
        
    human_state['base_position']['data'] = human_state['base_position']['data'].T[:,np.newaxis,:]
    human_state['base_orientation']['data'] = human_state['base_orientation']['data'].T[:,np.newaxis,:]
    human_state['base_angular_velocity']['data'] = human_state['base_angular_velocity']['data'].T[:,np.newaxis,:]
    human_state['base_linear_velocity']['data'] = human_state['base_linear_velocity']['data'].T[:,np.newaxis,:]
    
    joints_state['positions']['data'] = joints_state['positions']['data'].T[:,np.newaxis,:]
    joints_state['velocities']['data'] = joints_state['velocities']['data'].T[:,np.newaxis,:]
    
    for node in range(0,2):
        nodes[node]['FT']['data'] = nodes[node]['FT']['data'].T[:,np.newaxis,:]
        nodes[node]['angVel']['data'] = nodes[node]['angVel']['data'].T[:,np.newaxis,:]
        nodes[node]['linAcc']['data'] = nodes[node]['linAcc']['data'].T[:,np.newaxis,:]
        nodes[node]['orientation']['data'] = nodes[node]['orientation']['data'].T[:,np.newaxis,:]
        
    for node in range(2, 12):
        nodes[node]['angVel']['data'] = nodes[node]['angVel']['data'].T[:,np.newaxis,:]
        nodes[node]['linAcc']['data'] = nodes[node]['linAcc']['data'].T[:,np.newaxis,:]
        nodes[node]['orientation']['data'] = nodes[node]['orientation']['data'].T[:,np.newaxis,:]
    
    # Update human dynamics data
    human_dynamics['joint_torques']['data'] = interpolate(timestamp_human_dyn, human_dynamics['joint_torques']['data'].squeeze(), timestamp_resyncro, idxTS_human_dyn)
    human_dynamics['joint_torques']['timestamps'] = timestamp_resyncro
    
    return human_state, joints_state, nodes, human_dynamics, TIMESTAMPS, DATA_LENGTH


def interpolate(t_origin, data_set, t_resync, indx):
    
    dataSetR = np.zeros((data_set.shape[1], len(t_resync)))
    
    for i in range(data_set.shape[1]):
        dataSetR[i, :] = np.interp(t_resync, t_origin, data_set[indx, i])
        
    return dataSetR

def readPath(paths):
    target_substring = 'robot_logger_device'
    
    for file_name in os.listdir(paths['pathToRawData']):
        if target_substring in file_name:
            return os.path.join(paths['pathToRawData'], file_name).replace("\\", "/")