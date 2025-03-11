import numpy as np
import idyntree.bindings as iDynTree

def computeSwingWidth(kinDynComp, joints_state, human_state, W_T_base, KPI):
    """computeSwingWidth computes the lateral displacemnet of the feet during the swing. It is expressed as a percentage of the stride length. """
    
    # Compute feet link origin wrt world
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])
    
    linkOriginPosition = {'LeftFoot': np.zeros((3, KPI['len'])), 'RightFoot': np.zeros((3, KPI['len']))}
    for lenIdx in range(KPI['len']):
        # Set the joint positions
        s = joints_state['positions']['data'][lenIdx][0]
        ds = joints_state['velocities']['data'][lenIdx][0]
        lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
        ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
        base_vel = np.concatenate((lin_vel, ang_vel)) 
        
        kinDynComp.setRobotState(W_T_base[lenIdx], s, base_vel, ds, gravity)
        
        # Left
        link_transform = kinDynComp.getWorldTransform('LeftFoot').asHomogeneousTransform().toNumPy()
        linkOriginPosition['LeftFoot'][:,lenIdx] = link_transform[0:3,3]
        # Right
        link_transform = kinDynComp.getWorldTransform('RightFoot').asHomogeneousTransform().toNumPy()
        linkOriginPosition['RightFoot'][:,lenIdx] = link_transform[0:3,3]
        
    # Compute swing width per single cycle
    swingWidth = {'Left': {}, 'Right': {}}
    # Left
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Left']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Left']['swing'][nrCycleIdx]['idxAtTheEnd']
        
        swingWidth['Left'][nrCycleIdx] = {}
        swingWidth['Left'][nrCycleIdx]['Cycle'] = nrCycleIdx
        swingWidth['Left'][nrCycleIdx]['positionInSwingRange'] = linkOriginPosition['LeftFoot'][:,initial:final+1]
        
        x = np.linspace(linkOriginPosition['LeftFoot'][0,initial], linkOriginPosition['LeftFoot'][0,final], (final-initial+1))
        y = np.linspace(linkOriginPosition['LeftFoot'][1,initial], linkOriginPosition['LeftFoot'][1,final], (final-initial+1))
        z = np.linspace(linkOriginPosition['LeftFoot'][2,initial], linkOriginPosition['LeftFoot'][2,final], (final-initial+1))
        swingWidth['Left'][nrCycleIdx]['gaitDirectionLine'] = np.array([x,y,z])
        
        
        # Distance
        swingWidth['Left'][nrCycleIdx]['distance'] = np.zeros((1,swingWidth['Left'][nrCycleIdx]['positionInSwingRange'].shape[1]))
        for lenIdxSwing in range(swingWidth['Left'][nrCycleIdx]['positionInSwingRange'].shape[1]):
            swingWidth['Left'][nrCycleIdx]['distance'][:,lenIdxSwing] = np.linalg.norm(swingWidth['Left'][nrCycleIdx]['gaitDirectionLine'][:,lenIdxSwing] - swingWidth['Left'][nrCycleIdx]['positionInSwingRange'][:,lenIdxSwing])
        swingWidth['Left'][nrCycleIdx]['maxWidth'] = np.max(swingWidth['Left'][nrCycleIdx]['distance'])
        
        # Percentage of stride length
        swingWidth['Left'][nrCycleIdx]['percentageOfStrideLength'] = np.round(100 * swingWidth['Left'][nrCycleIdx]['maxWidth'] / KPI['strideLength']['Left'][nrCycleIdx]['distance'])
        
    # Right
    for nrCycleIdx in range(KPI['gaitCycleFeatures']['Right']['nrOfCycles']):
        initial = KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['idxAtTheBeginning']
        final = KPI['gaitCycleFeatures']['Right']['swing'][nrCycleIdx]['idxAtTheEnd']
        
        swingWidth['Right'][nrCycleIdx] = {}
        swingWidth['Right'][nrCycleIdx]['Cycle'] = nrCycleIdx
        swingWidth['Right'][nrCycleIdx]['positionInSwingRange'] = linkOriginPosition['RightFoot'][:,initial:final+1]
        
        x = np.linspace(linkOriginPosition['RightFoot'][0,initial], linkOriginPosition['RightFoot'][0,final], (final-initial+1))
        y = np.linspace(linkOriginPosition['RightFoot'][1,initial], linkOriginPosition['RightFoot'][1,final], (final-initial+1))
        z = np.linspace(linkOriginPosition['RightFoot'][2,initial], linkOriginPosition['RightFoot'][2,final], (final-initial+1))
        swingWidth['Right'][nrCycleIdx]['gaitDirectionLine'] = np.array([x,y,z])
        
        
        # Distance
        swingWidth['Right'][nrCycleIdx]['distance'] = np.zeros((1,swingWidth['Right'][nrCycleIdx]['positionInSwingRange'].shape[1]))
        for lenIdxSwing in range(swingWidth['Right'][nrCycleIdx]['positionInSwingRange'].shape[1]):
            swingWidth['Right'][nrCycleIdx]['distance'][:,lenIdxSwing] = np.linalg.norm(swingWidth['Right'][nrCycleIdx]['gaitDirectionLine'][:,lenIdxSwing] - swingWidth['Right'][nrCycleIdx]['positionInSwingRange'][:,lenIdxSwing])
        swingWidth['Right'][nrCycleIdx]['maxWidth'] = np.max(swingWidth['Right'][nrCycleIdx]['distance'])
        
        # Percentage of stride length
        swingWidth['Right'][nrCycleIdx]['percentageOfStrideLength'] = np.round(100 * swingWidth['Right'][nrCycleIdx]['maxWidth'] / KPI['strideLength']['Right'][nrCycleIdx]['distance'])
    
    
    
    return swingWidth