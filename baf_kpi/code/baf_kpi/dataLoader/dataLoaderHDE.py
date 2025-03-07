import numpy as np
import h5py
from scipy.io import loadmat
from scipy.io.matlab import matfile_version
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

import pandas as pd
import re
import os

from baf_kpi.scripts.computeBaseTransformWrtWorldHDE import computeBaseTransformWrtWorldHDE


# Global variables
DATA_LENGTH = None
TIMESTAMPS = None

human_state = {}
joints_state = {}
human_wrench = {}
human_dynamics = {}
node1 = {}
node2 = {}

resamplingFrequency = float(65)

def loadDataHDE(paths, nodesId, atatchedLinks):
    
    iFeelMap = getSensor2LinkMap(nodesId, atatchedLinks)
    
    suit, shoes = loadIfeelData(paths, iFeelMap, nodesId, atatchedLinks)
    
    kinematics = loadHumanData(paths)
    
    durationInSec = suit['properties']['durationInSec']
    
    suit, shoes, kinematics = dataSynchronization(suit, shoes, kinematics)
    
    
    human_state = {
        'base_position': {
            'data': kinematics['base']['position'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'base_orientation': {
            # 'data': base_or_rpy,
            'data': kinematics['base']['orientation'],
            'timestamps': kinematics['timestamp']
        },
        'base_angular_velocity': {
            'data': kinematics['base']['velocity6D'][3:, :].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'base_linear_velocity': {
            'data': kinematics['base']['velocity6D'][0:3, :].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        }
    }
    
    
    TIMESTAMPS = kinematics['timestamp']
    DATA_LENGTH = len(kinematics['timestamp']) 
    
    W_T_B = computeBaseTransformWrtWorldHDE(kinematics['timestamp'], human_state)
    
    
    
    joints_state = {
        'positions' : {
            'data': kinematics['s'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'velocities' : {
            'data': kinematics['ds'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        }
    }
    
    node1 = {
        'FT': {
            'data' : shoes['shoes_raw']['Left'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'angVel': {
            'data': suit['data'][0]['meas']['angularVelocity'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'linAcc': {
            'data': suit['data'][0]['meas']['freeBodyAcceleration'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'orientation': {
            'data': suit['data'][0]['meas']['orientation'],
            'timestamps': kinematics['timestamp']
        }        
    }
    
    
    node2 = {
        'FT': {
            'data' : shoes['shoes_raw']['Right'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'angVel': {
            'data': suit['data'][1]['meas']['angularVelocity'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'linAcc': {
            'data': suit['data'][1]['meas']['freeBodyAcceleration'].T[:, np.newaxis, :],
            'timestamps': kinematics['timestamp']
        },
        'orientation': {
            'data': suit['data'][1]['meas']['orientation'],
            'timestamps': kinematics['timestamp']
        }
    }
    
    nodes = []
    
    for i in range(2,12):
        node_name = 'node' + str(i+1)
        
        node_data = {
            'magnetometer': {
                'data': suit['data'][i]['meas']['magneticMoment'],
                'timestamps': kinematics['timestamp']
            },
            'angVel': {
                'data': suit['data'][i]['meas']['angularVelocity'].T[:, np.newaxis, :],
                'timestamps': kinematics['timestamp']
            },
            'linAcc': {
                'data': suit['data'][i]['meas']['linearAcceleration'].T[:, np.newaxis, :],
                'timestamps': kinematics['timestamp']
            },
            'orientation': {
                'data': suit['data'][i]['meas']['orientation'],
                'timestamps': kinematics['timestamp']
            }
        }
        
        nodes.append(node_data)
        
    
    nodes = [ node1, node2 ] + nodes
            
    
    return human_state, joints_state, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B


def dataSynchronization(suit, shoes, kinematics):
    """   DATA SYNCHRONIZATION   """
    # The data are not perfectly synchronized, so we need to synchronize them
    
    # STEP 1 : Replace NaN (if) with existing valid samples
    # Kinematics - s
    nan_indices_s = np.argwhere(np.isnan(kinematics['s']))
    if nan_indices_s.size > 0:
        for row_idx, col_idx in nan_indices_s:
            if col_idx == 0:  # only for the first sample
                kinematics['s'][row_idx, col_idx] = kinematics['s'][row_idx, col_idx + 1]
            else:
                kinematics['s'][row_idx, col_idx] = kinematics['s'][row_idx, col_idx - 1]

    # Kinematics - ds
    nan_indices_ds = np.argwhere(np.isnan(kinematics['ds']))
    if nan_indices_ds.size > 0:
        for row_idx, col_idx in nan_indices_ds:
            if col_idx == 0:  # only for the first sample
                kinematics['ds'][row_idx, col_idx] = kinematics['ds'][row_idx, col_idx + 1]
            else:
                kinematics['ds'][row_idx, col_idx] = kinematics['ds'][row_idx, col_idx - 1]

    # Base fields
    for base_field in kinematics['base']:
        nan_indices_base = np.argwhere(np.isnan(kinematics['base'][base_field]))
        if nan_indices_base.size > 0:
            for row_idx, col_idx in nan_indices_base:
                if col_idx == 0:  # only for the first sample
                    kinematics['base'][base_field][row_idx, col_idx] = kinematics['base'][base_field][row_idx, col_idx + 1]
                else:
                    kinematics['base'][base_field][row_idx, col_idx] = kinematics['base'][base_field][row_idx, col_idx - 1]

    
    # Suit data
    for node_idx in range(suit['properties']['nrOfNodes']):
        for suit_field in suit['data'][node_idx]['meas']:
            nan_indices_suit = np.argwhere(np.isnan(suit['data'][node_idx]['meas'][suit_field]))
            if nan_indices_suit.size > 0:
                for row_idx, col_idx in nan_indices_suit:
                    if col_idx == 0:  # only for the first sample
                        suit['data'][node_idx]['meas'][suit_field][row_idx, col_idx] = suit['data'][node_idx]['meas'][suit_field][row_idx, col_idx + 1]
                    else:
                        suit['data'][node_idx]['meas'][suit_field][row_idx, col_idx] = suit['data'][node_idx]['meas'][suit_field][row_idx, col_idx - 1]

    # Shoes data
    for shoes_field in shoes['shoes_raw']:
        nan_indices_shoes = np.argwhere(np.isnan(shoes['shoes_raw'][shoes_field]))
        if nan_indices_shoes.size > 0:
            for row_idx, col_idx in nan_indices_shoes:
                if col_idx == 0:  # only for the first sample
                    shoes['shoes_raw'][shoes_field][row_idx, col_idx] = shoes['shoes_raw'][shoes_field][row_idx, col_idx + 1]
                else:
                    shoes['shoes_raw'][shoes_field][row_idx, col_idx] = shoes['shoes_raw'][shoes_field][row_idx, col_idx - 1]
    
    
    # STEP 2 : Signals re-alignment and interpolation
    timestamp_suit, indxTSSuit = np.unique(suit['properties']['timestamp'], return_index=True)
    timestamp_kinematics, indxTSKin = np.unique(kinematics['timestamp'], return_index=True)
    
    timestamp_resyncro = np.arange(max(timestamp_suit[0], timestamp_kinematics[0]),
                               min(timestamp_suit[-1], timestamp_kinematics[-1]),
                               round(1 / resamplingFrequency, 15))
    
    unifiedTimestamp = timestamp_resyncro
    
    kinematics['timestamp'] = timestamp_resyncro
    kinematics['lenData'] = len(kinematics['timestamp'])
    kinematics['base']['velocity6D'] = interpolate(timestamp_kinematics, kinematics['base']['velocity6D'].squeeze(), timestamp_resyncro, indxTSKin)
    kinematics['base']['orientation'] = interpolate(timestamp_kinematics, kinematics['base']['orientation'].squeeze(), timestamp_resyncro, indxTSKin)
    kinematics['base']['position'] = interpolate(timestamp_kinematics, kinematics['base']['position'].squeeze(), timestamp_resyncro, indxTSKin)
    kinematics['s'] = interpolate(timestamp_kinematics, kinematics['s'].squeeze(), timestamp_resyncro, indxTSKin)
    kinematics['ds'] = interpolate(timestamp_kinematics, kinematics['ds'].squeeze(), timestamp_resyncro, indxTSKin)
    kinematics['dds'] = interpolate(timestamp_kinematics, kinematics['dds'].squeeze(), timestamp_resyncro, indxTSKin)

    # Aggiornamento dei dati della tuta (suit)
    suit['properties']['timestamp'] = timestamp_resyncro
    suit['properties']['lenData'] = len(suit['properties']['timestamp'])
    for node_idx in range(suit['properties']['nrOfNodes']):
        suit_fields = suit['data'][node_idx]['meas'].keys()
        for suit_field in suit_fields:
            suit['data'][node_idx]['meas'][suit_field] = interpolate(timestamp_suit, suit['data'][node_idx]['meas'][suit_field], timestamp_resyncro, indxTSSuit)

    # Aggiornamento dei dati delle scarpe (shoes)
    shoes['shoes_raw']['Left'] = interpolate(timestamp_suit, shoes['shoes_raw']['Left'].squeeze(), timestamp_resyncro, indxTSSuit)
    shoes['shoes_raw']['Left_back'] = interpolate(timestamp_suit, shoes['shoes_raw']['Left_back'].squeeze(), timestamp_resyncro, indxTSSuit)
    shoes['shoes_raw']['Left_front'] = interpolate(timestamp_suit, shoes['shoes_raw']['Left_front'].squeeze(), timestamp_resyncro, indxTSSuit)
    shoes['shoes_raw']['Right'] = interpolate(timestamp_suit, shoes['shoes_raw']['Right'].squeeze(), timestamp_resyncro, indxTSSuit)
    shoes['shoes_raw']['Right_back'] = interpolate(timestamp_suit, shoes['shoes_raw']['Right_back'].squeeze(), timestamp_resyncro, indxTSSuit)
    shoes['shoes_raw']['Right_front'] = interpolate(timestamp_suit, shoes['shoes_raw']['Right_front'].squeeze(), timestamp_resyncro, indxTSSuit)

    return suit, shoes, kinematics

def interpolate(t_origin, data_set, t_resync, indx):
    
    dataSetR = np.zeros((data_set.shape[0], len(t_resync)))
    
    for i in range(data_set.shape[0]):
        dataSetR[i, :] = np.interp(t_resync, t_origin, data_set[i, indx])
        
    return dataSetR

def getSensor2LinkMap(nodeID, attachedLink):
    """ getSensorToLinkMap creates a table for mapping nodes ID and links where the nodes are attached to."""
    sensor2linkMap = pd.DataFrame({
        'nodeID': nodeID,
        'attachedLink': attachedLink
    })
    
    return sensor2linkMap

def loadIfeelData(paths, iFeelMap, nodesId, attachedLinks):
    """  LOAD WEARABLE DATA   """    
    target_substring = "ifeel_data"
    counterIfeelData = 0
    # Find files with the target substring
    path_ifeel = []
    for file_name in os.listdir(paths['pathToRawData']):
        if target_substring in file_name:
            counterIfeelData += 1
            full_path = os.path.join(paths['pathToRawData'], file_name).replace("\\", "/")
            path_ifeel.append(full_path)
            
            
    # Concatenate elements from the collector to a unique masterFile_wearable_data
    masterFile_wearable_dataCollector = []  

    for file_path in path_ifeel:
        wearable_data = loadmat(file_path) 
        masterFile_wearable_dataCollector.append(wearable_data)

    masterFile_wearable_data = { "wearable_data": {} }
    first_file_data = masterFile_wearable_dataCollector[0]["ifeel_data"][0]

    wearable_list = [key for key in first_file_data.dtype.names if key not in ["description_list", "yarp_robot_name"]]

    for field in wearable_list:
        tmp_data_vector = []
        tmp_timestamps = []
        
        for wearable_data in masterFile_wearable_dataCollector:
            current_data = wearable_data["ifeel_data"][0]
            
            if current_data[field][0][0][0] is not None:
                field_data = np.squeeze(current_data[field][0][0]["data"][0])  
                field_timestamps = current_data[field][0][0]["timestamps"][0]
            
                if field_data.ndim == 1:
                    field_data = np.expand_dims(field_data, axis=-1)

                tmp_data_vector.append(field_data)
                tmp_timestamps.append(field_timestamps)
        
        concatenated_data_vector = np.concatenate(tmp_data_vector, axis=1)
        concatenated_timestamps = np.concatenate(tmp_timestamps, axis=1)

        masterFile_wearable_data["wearable_data"][field] = {
            "data": concatenated_data_vector,
            "dimensions": [4, 1, concatenated_data_vector.shape[1]],
            "timestamps": concatenated_timestamps,
        }
        
        
    suit, shoes = wearable2suit(masterFile_wearable_data["wearable_data"], iFeelMap, wearable_list)

    return suit, shoes


def wearable2suit(masterFile, iFeelMap, wearableList):
    
    names = wearableList
    
    wearableData = {'properties': {}, 'data': {}}
    wearableData['properties']['nrOfNodes'] = iFeelMap.shape[0]
    
    for nodeIdx in range(wearableData['properties']['nrOfNodes']):
        if 'Node' in names[nodeIdx]:
            wearableData['properties']['lenData'] = masterFile[names[nodeIdx]]['dimensions'][2]
            wearableData['properties']['timestamp'] = masterFile[names[nodeIdx]]['timestamps'].squeeze()
            break
    
    wearableData['properties']['nrOfLinks'] = iFeelMap.shape[0]
    time = wearableData['properties']['timestamp'] - wearableData['properties']['timestamp'][0]
    wearableData['properties']['durationInSec'] = time[-1]
    diff_time = np.diff(time)
    mean_time = np.mean(diff_time)
    wearableData['properties']['samplingTime'] = mean_time
    wearableData['properties']['samplingFreq'] = np.floor(1/mean_time) 
    
    
    
    """ SUIT"""
    # Create clustered data
    for nodeIdx in range(wearableData['properties']['nrOfNodes']):
        wearableData['data'][nodeIdx] = {}
        wearableData['data'][nodeIdx]['node'] = str(iFeelMap.iloc[nodeIdx, 0])
        wearableData['data'][nodeIdx]['attachedLink'] = str(iFeelMap.iloc[nodeIdx, 1])
        
        wearableData['data'][nodeIdx]['meas'] = {}
        
        for listIdx in range(len(names)):
            if 'ft6D' not in names[listIdx]: # condition to discard elements for shoes
                if re.search(r'\d+', names[listIdx]): # condition to discard elements without numbers
                    
                    name = names[listIdx]
                    # Free body acceleration
                    if (int(re.search(r'\d+', names[listIdx]).group()) -1 == iFeelMap.iloc[nodeIdx, 0]) and 'fbAcc' in names[listIdx]:
                        wearableData['data'][nodeIdx]['meas']['freeBodyAcceleration'] = np.zeros((3, wearableData['properties']['lenData']))
                        wearableData['data'][nodeIdx]['meas_tmp'] = {}
                        wearableData['data'][nodeIdx]['meas_tmp']['fbAcc'] = masterFile[name]["data"]

                        for lenIdx in range(wearableData['properties']['lenData']):
                            wearableData['data'][nodeIdx]['meas']['freeBodyAcceleration'][:, lenIdx] = wearableData['data'][nodeIdx]['meas_tmp']['fbAcc'][1:4, lenIdx]
            
                    # Linear acceleration
                    if (int(re.search(r'\d+', names[listIdx]).group()) -1 == iFeelMap.iloc[nodeIdx, 0]) and 'acc' in names[listIdx]:
                        wearableData['data'][nodeIdx]['meas']['linearAcceleration'] = np.zeros((3, wearableData['properties']['lenData']))
                        wearableData['data'][nodeIdx]['meas_tmp'] = {}
                        wearableData['data'][nodeIdx]['meas_tmp']['linAcc'] = masterFile[name]["data"]
                        
                        for lenIdx in range(wearableData['properties']['lenData']):
                            wearableData['data'][nodeIdx]['meas']['linearAcceleration'][:, lenIdx] = wearableData['data'][nodeIdx]['meas_tmp']['linAcc'][1:4, lenIdx]
            
            
                    # Magnetic field
                    if (int(re.search(r'\d+', names[listIdx]).group()) -1 == iFeelMap.iloc[nodeIdx, 0]) and 'mag' in names[listIdx]:
                        wearableData['data'][nodeIdx]['meas']['magneticMoment'] = np.zeros((3, wearableData['properties']['lenData']))
                        wearableData['data'][nodeIdx]['meas_tmp'] = {}
                        wearableData['data'][nodeIdx]['meas_tmp']['mag'] = masterFile[name]["data"]
                        
                        for lenIdx in range(wearableData['properties']['lenData']):
                            wearableData['data'][nodeIdx]['meas']['magneticMoment'][:, lenIdx] = wearableData['data'][nodeIdx]['meas_tmp']['mag'][1:4, lenIdx]
                    
                    
                    # Angular velocity
                    if (int(re.search(r'\d+', names[listIdx]).group()) -1 == iFeelMap.iloc[nodeIdx, 0]) and 'gyro' in names[listIdx]:
                        wearableData['data'][nodeIdx]['meas']['angularVelocity'] = np.zeros((3, wearableData['properties']['lenData']))
                        wearableData['data'][nodeIdx]['meas_tmp'] = {}
                        wearableData['data'][nodeIdx]['meas_tmp']['gyro'] = masterFile[name]["data"]
                        
                        for lenIdx in range(wearableData['properties']['lenData']):
                            wearableData['data'][nodeIdx]['meas']['angularVelocity'][:, lenIdx] = wearableData['data'][nodeIdx]['meas_tmp']['gyro'][1:4, lenIdx]
                            
                    
                    # Orientation
                    if (int(re.search(r'\d+', names[listIdx]).group()) - 1 == iFeelMap.iloc[nodeIdx, 0]) and 'orient' in names[listIdx]:
                        wearableData['data'][nodeIdx]['meas']['orientation'] = np.zeros((4, wearableData['properties']['lenData']))
                        wearableData['data'][nodeIdx]['meas_tmp'] = {}
                        wearableData['data'][nodeIdx]['meas_tmp']['orient'] = masterFile[name]["data"]
                        
                        for lenIdx in range(wearableData['properties']['lenData']):
                            wearableData['data'][nodeIdx]['meas']['orientation'][:, lenIdx] = wearableData['data'][nodeIdx]['meas_tmp']['orient'][1:5, lenIdx]
                    
                    
                    # Virtual link
                    if (int(re.search(r'\d+', names[listIdx]).group()) - 1 == iFeelMap.iloc[nodeIdx, 0]) and 'vLink' in names[listIdx]:
                        
                        wearableData['data'][nodeIdx]['meas']['virtualLink'] = np.zeros((19, wearableData['properties']['lenData']))
                        wearableData['data'][nodeIdx]['meas_tmp'] = {}
                        wearableData['data'][nodeIdx]['meas_tmp']['vLink'] = masterFile[name]["data"]
                        
                        for lenIdx in range(wearableData['properties']['lenData']):
                            wearableData['data'][nodeIdx]['meas']['virtualLink'][:, lenIdx] = wearableData['data'][nodeIdx]['meas_tmp']['vLink'][1:20, lenIdx]
        
    for nodeIdx in wearableData['data']:
        wearableData['data'][nodeIdx].pop('meas_tmp', None)

    suit = wearableData

    """SHOES"""
    tmp = {
    'shoesName': [
        'iFeelSuit_ft6D_Node_1',
        'iFeelSuit_ft6D_Node_1_Back',
        'iFeelSuit_ft6D_Node_1_Front',
        'iFeelSuit_ft6D_Node_2',
        'iFeelSuit_ft6D_Node_2_Back',
        'iFeelSuit_ft6D_Node_2_Front']
    }
    
    shoes = {'nodes': []}
    
    for nrShoesNodeIdx in range (len(tmp['shoesName'])):
        shoes['nodes'].append({
        'label': tmp['shoesName'][nrShoesNodeIdx],
        'meas': np.zeros((6, wearableData['properties']['lenData']))
    })
        
        
    # Cluster data from wearable
    tmp['shoes'] = []
    for nrShoesNodeIdx in range(len(tmp['shoesName'])):
        shoe = {
            'labels': tmp['shoesName'][nrShoesNodeIdx],
            'nodes': []
        }
        for listIdx in range(len(names)):
            wearableName = names[listIdx]
            if wearableName == tmp['shoesName'][nrShoesNodeIdx] and 'Node' in wearableName:
             shoe['nodes'].append(listIdx)
        tmp['shoes'].append(shoe)   
        
    # Fill with wearable data
    for nrShoesNodeIdx in range(len(tmp['shoes'])):
        shoes['nodes'][nrShoesNodeIdx]['meas'] = np.zeros((6, wearableData['properties']['lenData']))
        if shoes['nodes'][nrShoesNodeIdx]['label'] == tmp['shoes'][nrShoesNodeIdx]['labels']:
            tmp['indexValueInWearableData_nodes'] = tmp['shoes'][nrShoesNodeIdx]['nodes']
            tmp['wearableField_nodes'] = masterFile[tmp['shoes'][nrShoesNodeIdx]['labels']]["data"]
            for lenIdx in range(wearableData['properties']['lenData']):
                shoes['nodes'][nrShoesNodeIdx]['meas'][:, lenIdx] = tmp['wearableField_nodes'][1:,lenIdx]
    
    
    # Map nodes number and feet
    shoes_data = {}
    
    shoes['nodes'][0]['label'] = 'leftFoot'
    shoes_data['Left'] = shoes['nodes'][0]['meas']
    shoes['nodes'][1]['label'] = 'RightFoot'
    shoes_data['Right'] = shoes['nodes'][3]['meas']
    
    shoes['nodes'][2]['label'] = 'leftFootBack'
    shoes_data['Left_back'] = shoes['nodes'][1]['meas']
    shoes['nodes'][3]['label'] = 'leftFootFront'
    shoes_data['Left_front'] = shoes['nodes'][2]['meas']
    
    shoes['nodes'][4]['label'] = 'rightFootBack'
    shoes_data['Right_back'] = shoes['nodes'][4]['meas']
    shoes['nodes'][5]['label'] = 'rightFootFront'
    shoes_data['Right_front'] = shoes['nodes'][5]['meas']
    
    shoes['shoes_raw'] = shoes_data
    
    return suit, shoes
           
           
           
def loadHumanData(paths):
    """   LOAD HUMAN DATA   """
    """LOAD MULTIPLE HUMAN DATA FILES"""
    target_substring = "human_data"
    path_human = []
    
    for file_name in os.listdir(paths['pathToRawData']):
        if target_substring in file_name:
            full_path = os.path.join(paths['pathToRawData'], file_name).replace("\\", "/")
            path_human.append(full_path)
            
    timestamps_all = []
    base_position_all = []
    base_velocity_all = []
    base_orientation_all = []
    joint_positions_all = []
    joint_velocities_all = []
    
    for file_path in path_human:
        version = matfile_version(file_path)
    
        if version[0] == 1:
            data = loadmat(file_path)
            human_data = data['human_data']
            nodes = human_data[0]
            dtype = nodes.dtype
            names = dtype.names
            
            if 'human_state' in human_data.dtype.names:
                human_state = human_data['human_state']
                nodes_human = human_state[0][0][0][0]
                dtype_human = nodes_human.dtype
                names_human = dtype_human.names
                if 'joint_velocities' in names_human:
                    joint_velocities = human_state[0][0][0][0]['joint_velocities'][0][0][0]
                    joint_velocities_all.append(joint_velocities)
                if 'joint_positions' in names_human:
                    joint_positions = human_state[0][0][0][0]['joint_positions'][0][0][0]
                    joint_positions_all.append(joint_positions)
                if 'base_velocity' in names_human:
                    base_velocity = human_state[0][0][0][0]['base_velocity'][0][0][0]
                    base_velocity_all.append(base_velocity)
                if 'base_orientation' in names_human:
                    base_orientation = human_state[0][0][0][0]['base_orientation'][0][0][0]
                    base_orientation_all.append(base_orientation)
                if 'base_position' in names_human:
                    base_position = human_state[0][0][0][0]['base_position'][0][0][0]
                    base_position_all.append(base_position)
                    
                nodes_pos = nodes_human[0][0][0]
                dtype_pos = nodes_pos.dtype
                names_pos = dtype_pos.names
                if 'timestamps' in names_pos:
                    timestamps = human_state[0][0][0][0]['base_position']['timestamps'][0][0].squeeze()
                    timestamps_all.append(timestamps)

        
        
        elif version[0] == 2:
            with h5py.File(file_path, 'r') as f:
                human_data = f['human_data']
                if 'human_state' in human_data:
                    human_state = human_data['human_state']
                    if 'base_position' in human_state:
                        base_position = np.array(human_state['base_position']['data']).T
                        base_position_all.append(base_position)
                        timestamps = np.array(human_state['base_position']['timestamps']).squeeze()
                        timestamps_all.append(timestamps)
                    if 'base_orientation' in human_state:
                        base_orientation = np.array(human_state['base_orientation']['data']).T
                        base_orientation_all.append(base_orientation)
                    if 'base_velocity' in human_state:
                        base_velocity = np.array(human_state['base_velocity']['data']).T 
                        base_velocity_all.append(base_velocity)
                else:
                    print("human_state not found in human_data")
                    
                if 'joints_state' in human_data:
                    joints_state = human_data['joints_state']
                    if 'positions' in joints_state:
                        joint_positions = np.array(joints_state['positions']['data']).T
                        joint_positions_all.append(joint_positions)
                    if 'velocities' in joints_state:
                        joint_velocities = np.array(joints_state['velocities']['data']).T
                        joint_velocities_all.append(joint_velocities)
                else:
                    print("joints_state not found in human_data")
                
                
    kinematics = {}
    kinematics['timestamp'] = np.concatenate(timestamps_all, axis=0)
    kinematics['base'] = {}
    kinematics['base']['position'] = np.concatenate(base_position_all, axis=2)
    kinematics['base']['velocity6D'] = np.concatenate(base_velocity_all, axis=2)
    kinematics['base']['orientation'] = np.concatenate(base_orientation_all, axis=2)
    kinematics['s'] = np.concatenate(joint_positions_all, axis=2)
    kinematics['ds'] = np.concatenate(joint_velocities_all, axis=2)
    kinematics['dds'] = np.zeros_like(kinematics['s'])
    
    return kinematics