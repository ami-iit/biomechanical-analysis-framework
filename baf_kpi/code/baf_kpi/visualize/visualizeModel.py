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
    

    timeScaling = 5
    dt_base = np.average(np.diff(TIMESTAMPS))
    index = 0
    
    diameter = 0.25
    thickness = 0.01
    
    for com in range(len(TIMESTAMPS)):
        viz.load_cylinder(diameter/2, thickness, shape_name=f"COM_{com}", color=[0,0,0,0.5])
        viz.load_cylinder(diameter/2, thickness, shape_name=f"COP_Left_{com}", color=[1,0,0,0.5])
        viz.load_cylinder(diameter/2, thickness, shape_name=f"COP_Right_{com}", color=[0,0,1,0.5])

    
    for i in range(0, len(TIMESTAMPS)):
        t = TIMESTAMPS[i]
        
        time_init = time.time()
        
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
        
        # # Update COM and COP
        updateCOM(viz, KPI, i, diameter, thickness)
        updateCOP(viz, KPI, i, diameter, thickness)
        

        
        time_end = time.time()
        
        timer = time_end - time_init
        
        if timer < 0.033:
            time.sleep(0.033-timer)
            
        index += timeScaling