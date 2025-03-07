import numpy as np
import idyntree.bindings as iDynTree
from idyntree.visualize import MeshcatVisualizer
from rich.progress import track
import time

from baf_kpi.scripts.updateCOM import updateCOM
from baf_kpi.scripts.updateCOP import updateCOP



def visualizeModel(modelFilePath, JOINT_NAMES, TIMESTAMPS, KPI, joints_state, human_state, dataBAF):
    
    viz = MeshcatVisualizer()
    viz.load_model_from_file(modelFilePath, JOINT_NAMES, "Human Model")
    viz.open()
    
    viz.start_recording_animation()
    viz.set_animation_frame(0)
    

    timeScaling = 5
    dt_base = np.average(np.diff(TIMESTAMPS))
    index = 0
    
    for i in track(range(0, len(TIMESTAMPS), int(1 / (dt_base * (30 / timeScaling)))), description="Visualizing model"):
        t = TIMESTAMPS[i]
        
        time_init = time.time()
        
        viz.set_animation_frame(index)
        
        # Find the closest timestamp in the joints state
        idx = np.argmin(np.abs(joints_state['positions']['timestamps'] - t))


        if dataBAF == True:
            rpy = human_state['base_orientation']['data'][i,0,:]
        else:
            rotation = iDynTree.Rotation()
            rotation.fromQuaternion(human_state['base_orientation']['data'][:,i])
            rpy = rotation.asRPY()

        orientation = iDynTree.Rotation.RPY(rpy[0], rpy[1], rpy[2]).toNumPy()

        # Update the model
        viz.set_multibody_system_state(base_position = human_state['base_position']['data'][i,0,:], base_rotation = orientation, joint_value = joints_state['positions']['data'][idx,0,:], model_name="Human Model")

        index += timeScaling
        
    viz.publish_animation()
