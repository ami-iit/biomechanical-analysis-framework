import idyntree.bindings as iDynTree
import numpy as np

def cell2iDynTreeStringVector(cell):
    """cell2iDynTreeStringVector transforms an iDynTree StringVector object into cell array."""
    
    selectedJoints = iDynTree.StringVector()
    for i in range(len(cell)):
        selectedJoints.push_back(cell[i])
        
    return selectedJoints