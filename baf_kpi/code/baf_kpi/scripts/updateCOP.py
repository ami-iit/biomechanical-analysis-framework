import numpy as np

def updateCOP(viz, KPI, i, diameter, thickness):
    
    cop_i_left = KPI['COP']['total']['Left']['wrtWorld'][0:3,i]
    cop_i_right = KPI['COP']['total']['Right']['wrtWorld'][0:3,i]
    viz.load_cylinder(diameter/2, thickness, shape_name=f"COP_Left_{i}", color=[1,0,0,1.0])
    viz.load_cylinder(diameter/2, thickness, shape_name=f"COP_Right_{i}", color=[0,0,1,1.0])
    viz.set_primitive_geometry_transform(position=cop_i_left, rotation=np.eye(3), shape_name=f"COP_Left_{i}")
    viz.set_primitive_geometry_transform(position=cop_i_right, rotation=np.eye(3), shape_name=f"COP_Right_{i}")
                                         
    return 0