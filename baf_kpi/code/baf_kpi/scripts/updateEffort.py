import numpy as np

class JointEffortData:
    def __init__(self, parentLinkName, sphericalJointName,  effortMax):
        self.parentLinkName = parentLinkName # Name of the parent link
        self.sphericalJointName = sphericalJointName # Name of the spherical joint
        self.fakeJointsIndices = [] # Indices of the fake joints 
        self.effortMax = effortMax # Maximum effort
        self.effort = 0.0 # Current effort
        

def updateEffort(joints_state, human_state, kinDynComp, W_T_B, parentLinkNames, sphericalJointNames, maxEffort, JOINT_NAMES, viz, i, human_dynamics, s, ds, base_vel, gravity):
    
    s = joints_state['positions']['data'][i][0]
    ds = joints_state['velocities']['data'][i][0]
    lin_vel = human_state['base_linear_velocity']['data'][i][0]
    ang_vel = human_state['base_angular_velocity']['data'][i][0]
    base_vel = np.concatenate((lin_vel, ang_vel)) 
    
    kinDynComp.setRobotState(W_T_B[i], s, base_vel, ds, gravity)
    
    
    minR = 0.0
    minG = 1.0
    minB = 0.0
    maxR = 1.0
    maxG = 0.0
    maxB = 0.0
    
    # Store the effort data for each joint
    modelEffortData = []
    
    for k in range(len(parentLinkNames)):
        data = JointEffortData(
            parentLinkNames[k],
            sphericalJointNames[k],
            maxEffort[k])
        
        modelEffortData.append(data)
        
    # Associate URDF joints with effort data
    for jointIdx in range(len(JOINT_NAMES)):
        urdfJointName = JOINT_NAMES[jointIdx]
        
        for effortData in modelEffortData:
            if effortData.sphericalJointName in urdfJointName:
                effortData.fakeJointsIndices.append(jointIdx)
                break
        
        
    # Create and configure spheres for visualization
    radius = 0.05
    color = [0.0, 0.0, 0.0, 1.0]
        
    
    # # Prepare for visualizing external wrenches
    # wrenchesList = list(human_wrench.keys())
    # wrenchSourceLinkIndices = []
    # for wrenchSourceLink in wrenchesList:
    #     frameIndex = humanModelLoader.model().getLinkIndex(wrenchSourceLink)
    #     wrenchSourceLinkIndices.append(frameIndex)
        
    # for vectorIndex in range(len(wrenchesList)):
    #     linkTransform = viz.modelViz.getWorldLinkTransform(wrenchSourceLinkIndices[vectorIndex])
    #     force = [0.0, 0.0, 0.0]  # Sostituire con il formato corretto, se necessario
    #     viz.vectors().addVector(linkTransform.getPosition(), force)
    
    # Update the visualization of the efforts
    for j in range(len(modelEffortData)):
        jointEffortData = modelEffortData[j]
        
        effortTmp = 0.0
        
        # Compute the magnitude of the joint torque and position it in the correct link
        for modelFakeJointIdx in jointEffortData.fakeJointsIndices:
            effortTmp += np.pow(human_dynamics['joint_torques']['data'][modelFakeJointIdx,i], 2)
    
        effortTmp = np.sqrt(effortTmp)
        jointEffortData.effort = effortTmp
        
        # Calculate the weight of the effort for visualization
        effortWeight = jointEffortData.effort / jointEffortData.effortMax
        if effortWeight > 1.0:
            effortWeight = 1.0
        
        # Interpolate the color based on the effort weight
        r = minR + (maxR - minR) * effortWeight
        g = minG + (maxG - minG) * effortWeight
        b = minB + (maxB - minB) * effortWeight
        
        color = [r, g, b, 1.0]
        
        
        H = kinDynComp.getWorldTransform(jointEffortData.parentLinkName)
        H_np = H.asHomogeneousTransform().toNumPy()
        pos = H_np[0:3,3]
        
        # viz.load_sphere(radius, jointEffortData.parentLinkName, color)
        viz.set_primitive_geometry_transform(pos, np.eye(3), jointEffortData.parentLinkName)
        viz.set_primitive_geometry_property('color', color, jointEffortData.parentLinkName)
        
    return 0
    