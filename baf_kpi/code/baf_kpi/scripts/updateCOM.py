import numpy as np

def updateCOM(viz, KPI, i, diameter, thickness):
    
    com_i = KPI['COM']['position'][:,i]
    viz.load_cylinder(diameter/2, thickness, shape_name=f"COM", color=[0,0,0,1.0])
    viz.set_primitive_geometry_transform(position=com_i, rotation=np.eye(3), shape_name=f"COM")
    
    return 0