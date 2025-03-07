import h5py
import numpy as np

from baf_kpi.scripts.computeBaseTransformWrtWorldBAF import computeBaseTransformWrtWorldBAF

# Global variables
DATA_LENGTH = None
TIMESTAMPS = None

THRESHOLD_LENGTH = 1000

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

def loadDataFilteredBAF(file_path):

    with h5py.File(file_path, 'r') as f:
        
        robot_logger_device = f['robot_logger_device']
        
        # Human state data
        if 'human_state' in robot_logger_device:
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

            human_state = limit_data_length(human_state)
            
            DATA_LENGTH = len(human_state['base_position']['data'])
            TIMESTAMPS = np.array(human_state['base_position']['timestamps']).squeeze()
            durationInSec = TIMESTAMPS[-1] - TIMESTAMPS[0]
            

        else:
            print("'human_state' not found in 'robot_logger_device'.")

        # Joints state data
        if 'joints_state' in robot_logger_device:
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
            
            joints_state = limit_data_length(joints_state)


        else:
            print("'joints_state' not found in 'robot_logger_device'.")


        W_T_B = computeBaseTransformWrtWorldBAF(joints_state, human_state)
        
        # Human wrench data
        if 'human_wrench' in robot_logger_device:
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
            
            human_wrench = limit_data_length(human_wrench)

        else:
            print("'human_wrench' not found in 'robot_logger_device'.")

        # Human dynamics data
        if 'human_dynamics' in robot_logger_device:
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
            
            human_dynamics = limit_data_length(human_dynamics)

        else:
            print("'human_dynamics' not found in 'robot_logger_device'.")

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
            
            node1 = limit_data_length(node1)
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
            
            node2 = limit_data_length(node2)
        else:
            print("'node2' not found in 'robot_logger_device'.")
            
        if 'node3' in robot_logger_device:
            node3 = robot_logger_device['node3']

            node3 = {
                'magnetometer': {
                    'data': np.array(node3['magnetometer']['data']),
                    'dimensions': np.array(node3['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node3['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node3['magnetometer']['units_of_measure']),
                    # 'name': np.array(node3['magnetometer']['name']),
                    'timestamps': np.array(node3['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node3['angVel']['data']),
                    'dimensions': np.array(node3['angVel']['dimensions']),
                    # 'elements_names': np.array(node3['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node3['angVel']['units_of_measure']),
                    # 'name': np.array(node3['angVel']['name']),
                    'timestamps': np.array(node3['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node3['linAcc']['data']),
                    'dimensions': np.array(node3['linAcc']['dimensions']),
                    # 'elements_names': np.array(node3['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node3['linAcc']['units_of_measure']),
                    # 'name': np.array(node3['linAcc']['name']),
                    'timestamps': np.array(node3['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node3['orientation']['data']),
                    'dimensions': np.array(node3['orientation']['dimensions']),
                    # 'elements_names': np.array(node3['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node3['orientation']['units_of_measure']),
                    # 'name': np.array(node3['orientation']['name']),
                    'timestamps': np.array(node3['orientation']['timestamps'])
                }
            }
            
            node3 = limit_data_length(node3)
        else:
            print("'node3' not found in 'robot_logger_device'.")
            
        if 'node4' in robot_logger_device:
            node4 = robot_logger_device['node4']

            node4 = {
                'magnetometer': {
                    'data': np.array(node4['magnetometer']['data']),
                    'dimensions': np.array(node4['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node4['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node4['magnetometer']['units_of_measure']),
                    # 'name': np.array(node4['magnetometer']['name']),
                    'timestamps': np.array(node4['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node4['angVel']['data']),
                    'dimensions': np.array(node4['angVel']['dimensions']),
                    # 'elements_names': np.array(node4['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node4['angVel']['units_of_measure']),
                    # 'name': np.array(node4['angVel']['name']),
                    'timestamps': np.array(node4['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node4['linAcc']['data']),
                    'dimensions': np.array(node4['linAcc']['dimensions']),
                    # 'elements_names': np.array(node4['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node4['linAcc']['units_of_measure']),
                    # 'name': np.array(node4['linAcc']['name']),
                    'timestamps': np.array(node4['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node4['orientation']['data']),
                    'dimensions': np.array(node4['orientation']['dimensions']),
                    # 'elements_names': np.array(node4['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node4['orientation']['units_of_measure']),
                    # 'name': np.array(node4['orientation']['name']),
                    'timestamps': np.array(node4['orientation']['timestamps'])
                }
            }
            
            node4 = limit_data_length(node4)
        else:
            print("'node4' not found in 'robot_logger_device'.")
            
        if 'node5' in robot_logger_device:
            node5 = robot_logger_device['node5']

            node5 = {
                'magnetometer': {
                    'data': np.array(node5['magnetometer']['data']),
                    'dimensions': np.array(node5['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node5['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node5['magnetometer']['units_of_measure']),
                    # 'name': np.array(node5['magnetometer']['name']),
                    'timestamps': np.array(node5['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node5['angVel']['data']),
                    'dimensions': np.array(node5['angVel']['dimensions']),
                    # 'elements_names': np.array(node5['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node5['angVel']['units_of_measure']),
                    # 'name': np.array(node5['angVel']['name']),
                    'timestamps': np.array(node5['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node5['linAcc']['data']),
                    'dimensions': np.array(node5['linAcc']['dimensions']),
                    # 'elements_names': np.array(node5['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node5['linAcc']['units_of_measure']),
                    # 'name': np.array(node5['linAcc']['name']),
                    'timestamps': np.array(node5['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node5['orientation']['data']),
                    'dimensions': np.array(node5['orientation']['dimensions']),
                    # 'elements_names': np.array(node5['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node5['orientation']['units_of_measure']),
                    # 'name': np.array(node5['orientation']['name']),
                    'timestamps': np.array(node5['orientation']['timestamps'])
                }
            }
            
            node5 = limit_data_length(node5)
        else:
            print("'node5' not found in 'robot_logger_device'.")
            
        if 'node6' in robot_logger_device:
            node6 = robot_logger_device['node6']

            node6 = {
                'magnetometer': {
                    'data': np.array(node6['magnetometer']['data']),
                    'dimensions': np.array(node6['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node6['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node6['magnetometer']['units_of_measure']),
                    # 'name': np.array(node6['magnetometer']['name']),
                    'timestamps': np.array(node6['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node6['angVel']['data']),
                    'dimensions': np.array(node6['angVel']['dimensions']),
                    # 'elements_names': np.array(node6['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node6['angVel']['units_of_measure']),
                    # 'name': np.array(node6['angVel']['name']),
                    'timestamps': np.array(node6['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node6['linAcc']['data']),
                    'dimensions': np.array(node6['linAcc']['dimensions']),
                    # 'elements_names': np.array(node6['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node6['linAcc']['units_of_measure']),
                    # 'name': np.array(node6['linAcc']['name']),
                    'timestamps': np.array(node6['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node6['orientation']['data']),
                    'dimensions': np.array(node6['orientation']['dimensions']),
                    # 'elements_names': np.array(node6['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node6['orientation']['units_of_measure']),
                    # 'name': np.array(node6['orientation']['name']),
                    'timestamps': np.array(node6['orientation']['timestamps'])
                }
            }
            
            node6 = limit_data_length(node6)
        else:
            print("'node6' not found in 'robot_logger_device'.")
            
        if 'node7' in robot_logger_device:
            node7 = robot_logger_device['node7']

            node7 = {
                'magnetometer': {
                    'data': np.array(node7['magnetometer']['data']),
                    'dimensions': np.array(node7['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node7['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node7['magnetometer']['units_of_measure']),
                    # 'name': np.array(node7['magnetometer']['name']),
                    'timestamps': np.array(node7['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node7['angVel']['data']),
                    'dimensions': np.array(node7['angVel']['dimensions']),
                    # 'elements_names': np.array(node7['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node7['angVel']['units_of_measure']),
                    # 'name': np.array(node7['angVel']['name']),
                    'timestamps': np.array(node7['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node7['linAcc']['data']),
                    'dimensions': np.array(node7['linAcc']['dimensions']),
                    # 'elements_names': np.array(node7['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node7['linAcc']['units_of_measure']),
                    # 'name': np.array(node7['linAcc']['name']),
                    'timestamps': np.array(node7['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node7['orientation']['data']),
                    'dimensions': np.array(node7['orientation']['dimensions']),
                    # 'elements_names': np.array(node7['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node7['orientation']['units_of_measure']),
                    # 'name': np.array(node7['orientation']['name']),
                    'timestamps': np.array(node7['orientation']['timestamps'])
                }
            }
            
            node7 = limit_data_length(node7)
        else:
            print("'node7' not found in 'robot_logger_device'.")
            
        if 'node8' in robot_logger_device:
            node8 = robot_logger_device['node8']

            node8 = {
                'magnetometer': {
                    'data': np.array(node8['magnetometer']['data']),
                    'dimensions': np.array(node8['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node8['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node8['magnetometer']['units_of_measure']),
                    # 'name': np.array(node8['magnetometer']['name']),
                    'timestamps': np.array(node8['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node8['angVel']['data']),
                    'dimensions': np.array(node8['angVel']['dimensions']),
                    # 'elements_names': np.array(node8['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node8['angVel']['units_of_measure']),
                    # 'name': np.array(node8['angVel']['name']),
                    'timestamps': np.array(node8['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node8['linAcc']['data']),
                    'dimensions': np.array(node8['linAcc']['dimensions']),
                    # 'elements_names': np.array(node8['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node8['linAcc']['units_of_measure']),
                    # 'name': np.array(node8['linAcc']['name']),
                    'timestamps': np.array(node8['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node8['orientation']['data']),
                    'dimensions': np.array(node8['orientation']['dimensions']),
                    # 'elements_names': np.array(node8['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node8['orientation']['units_of_measure']),
                    # 'name': np.array(node8['orientation']['name']),
                    'timestamps': np.array(node8['orientation']['timestamps'])
                }
            }
            
            node8 = limit_data_length(node8)
        else:
            print("'node8' not found in 'robot_logger_device'.")
            
        if 'node9' in robot_logger_device:
            node9 = robot_logger_device['node9']

            node9 = {
                'magnetometer': {
                    'data': np.array(node9['magnetometer']['data']),
                    'dimensions': np.array(node9['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node9['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node9['magnetometer']['units_of_measure']),
                    # 'name': np.array(node9['magnetometer']['name']),
                    'timestamps': np.array(node9['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node9['angVel']['data']),
                    'dimensions': np.array(node9['angVel']['dimensions']),
                    # 'elements_names': np.array(node9['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node9['angVel']['units_of_measure']),
                    # 'name': np.array(node9['angVel']['name']),
                    'timestamps': np.array(node9['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node9['linAcc']['data']),
                    'dimensions': np.array(node9['linAcc']['dimensions']),
                    # 'elements_names': np.array(node9['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node9['linAcc']['units_of_measure']),
                    # 'name': np.array(node9['linAcc']['name']),
                    'timestamps': np.array(node9['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node9['orientation']['data']),
                    'dimensions': np.array(node9['orientation']['dimensions']),
                    # 'elements_names': np.array(node9['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node9['orientation']['units_of_measure']),
                    # 'name': np.array(node9['orientation']['name']),
                    'timestamps': np.array(node9['orientation']['timestamps'])
                }
            }
            
            node9 = limit_data_length(node9)
        else:
            print("'node9' not found in 'robot_logger_device'.")
            
        if 'node10' in robot_logger_device:
            node10 = robot_logger_device['node10']

            node10 = {
                'magnetometer': {
                    'data': np.array(node10['magnetometer']['data']),
                    'dimensions': np.array(node10['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node10['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node10['magnetometer']['units_of_measure']),
                    # 'name': np.array(node10['magnetometer']['name']),
                    'timestamps': np.array(node10['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node10['angVel']['data']),
                    'dimensions': np.array(node10['angVel']['dimensions']),
                    # 'elements_names': np.array(node10['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node10['angVel']['units_of_measure']),
                    # 'name': np.array(node10['angVel']['name']),
                    'timestamps': np.array(node10['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node10['linAcc']['data']),
                    'dimensions': np.array(node10['linAcc']['dimensions']),
                    # 'elements_names': np.array(node10['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node10['linAcc']['units_of_measure']),
                    # 'name': np.array(node10['linAcc']['name']),
                    'timestamps': np.array(node10['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node10['orientation']['data']),
                    'dimensions': np.array(node10['orientation']['dimensions']),
                    # 'elements_names': np.array(node10['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node10['orientation']['units_of_measure']),
                    # 'name': np.array(node10['orientation']['name']),
                    'timestamps': np.array(node10['orientation']['timestamps'])
                }
            }
            
            node10 = limit_data_length(node10)
        else:
            print("'node10' not found in 'robot_logger_device'.")
            
        if 'node11' in robot_logger_device:
            node11 = robot_logger_device['node11']

            node11 = {
                'magnetometer': {
                    'data': np.array(node11['magnetometer']['data']),
                    'dimensions': np.array(node11['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node11['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node11['magnetometer']['units_of_measure']),
                    # 'name': np.array(node11['magnetometer']['name']),
                    'timestamps': np.array(node11['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node11['angVel']['data']),
                    'dimensions': np.array(node11['angVel']['dimensions']),
                    # 'elements_names': np.array(node11['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node11['angVel']['units_of_measure']),
                    # 'name': np.array(node11['angVel']['name']),
                    'timestamps': np.array(node11['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node11['linAcc']['data']),
                    'dimensions': np.array(node11['linAcc']['dimensions']),
                    # 'elements_names': np.array(node11['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node11['linAcc']['units_of_measure']),
                    # 'name': np.array(node11['linAcc']['name']),
                    'timestamps': np.array(node11['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node11['orientation']['data']),
                    'dimensions': np.array(node11['orientation']['dimensions']),
                    # 'elements_names': np.array(node11['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node11['orientation']['units_of_measure']),
                    # 'name': np.array(node11['orientation']['name']),
                    'timestamps': np.array(node11['orientation']['timestamps'])
                }
            }
            
            node11 = limit_data_length(node11)
        else:
            print("'node11' not found in 'robot_logger_device'.")
            
        if 'node12' in robot_logger_device:
            node12 = robot_logger_device['node12']

            node12 = {
                'magnetometer': {
                    'data': np.array(node12['magnetometer']['data']),
                    'dimensions': np.array(node12['magnetometer']['dimensions']),
                    # 'elements_names': np.array(node12['magnetometer']['elements_names']),
                    # 'units_of_measure': np.array(node12['magnetometer']['units_of_measure']),
                    # 'name': np.array(node12['magnetometer']['name']),
                    'timestamps': np.array(node12['magnetometer']['timestamps'])
                },
                'angVel': {
                    'data': np.array(node12['angVel']['data']),
                    'dimensions': np.array(node12['angVel']['dimensions']),
                    # 'elements_names': np.array(node12['angVel']['elements_names']),
                    # 'units_of_measure': np.array(node12['angVel']['units_of_measure']),
                    # 'name': np.array(node12['angVel']['name']),
                    'timestamps': np.array(node12['angVel']['timestamps'])
                },
                'linAcc': {
                    'data': np.array(node12['linAcc']['data']),
                    'dimensions': np.array(node12['linAcc']['dimensions']),
                    # 'elements_names': np.array(node12['linAcc']['elements_names']),
                    # 'units_of_measure': np.array(node12['linAcc']['units_of_measure']),
                    # 'name': np.array(node12['linAcc']['name']),
                    'timestamps': np.array(node12['linAcc']['timestamps'])
                },
                'orientation': {
                    'data': np.array(node12['orientation']['data']),
                    'dimensions': np.array(node12['orientation']['dimensions']),
                    # 'elements_names': np.array(node12['orientation']['elements_names']),
                    # 'units_of_measure': np.array(node12['orientation']['units_of_measure']),
                    # 'name': np.array(node12['orientation']['name']),
                    'timestamps': np.array(node12['orientation']['timestamps'])
                }
            }
            
            node12 = limit_data_length(node12)
        else:
            print("'node12' not found in 'robot_logger_device'.") 
            
    nodes = [node1, node2, node3, node4, node5, node6, node7, node8, node9, node10, node11, node12]
            
    # return human_state, joints_state, human_wrench, human_dynamics, node1, node2, node3, node4, node5, node6, node7, node8, node9, node10, node11, node12, nodes, DATA_LENGTH, TIMESTAMPS  
    return human_state, joints_state, human_wrench, human_dynamics, nodes, DATA_LENGTH, TIMESTAMPS, durationInSec, W_T_B
            
            
def limit_data_length(data_dict):
    for key in data_dict:
        if isinstance(data_dict[key], dict):
            for sub_key in data_dict[key]:
                if isinstance(data_dict[key][sub_key], np.ndarray):
                    data_dict[key][sub_key] = data_dict[key][sub_key][:THRESHOLD_LENGTH]
    return data_dict
