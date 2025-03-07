
import idyntree.bindings as iDynTree

URDF_FILE_PATH = "C:/Users/ldanovaro/Documents/GitHub/element_human-body-estimation/human-model-generator/code/models/humanModels/S_template_testMeshes.urdf"

def main():
    
    dynComp = iDynTree.KinDynComputations()
    mdlLoader = iDynTree.ModelLoader()
    # Possibility of inserting a reduced list of joints
    JOINTLIST = []
    mdlLoader.loadModelFromFile(URDF_FILE_PATH)
    dynComp.loadRobotModel(mdlLoader.model())
    ndofs = dynComp.model().getNrOfDOFs()
    NrOfJoints = dynComp.model().getNrOfJoints()
    for indxJoint in range(NrOfJoints):
        JOINTLIST.append(dynComp.model().getJointName(indxJoint))
    
    print("/n[INFO] Visualization :/n")
    viz = iDynTree.Visualizer()
    vizOpt = iDynTree.VisualizerOptions()
    vizOpt.winWidth = 1500
    vizOpt.winHeight = 1000
    viz.init(vizOpt)

    env = viz.enviroment()
    env.setElementVisibility("floor_grid", True)
    env.setElementVisibility("world_frame", True)
    viz.setColorPalette("meshcat")
    # frames = viz.frames()
    cam = viz.camera()
    cam.setPosition(iDynTree.Position(2, 1, 2.5))
    viz.camera().animator().enableMouseControl(True)

    viz.addModel(mdlLoader.model(), "ModelVisualizer")

    gravity = [0.0, 0.0, -9.81]
    quaternion_idyn = iDynTree.Vector4([1, 0, 0, 0])
    G_T_b_rot = iDynTree.Rotation()
    G_T_b_rot.fromQuaternion(quaternion_idyn)
    G_T_b_pos = iDynTree.Position([0, 0, 0])
    G_T_base = iDynTree.Transform(G_T_b_rot, G_T_b_pos)
    s = [0] * ndofs

    viz.modelViz("ModelVisualizer").setPositions(G_T_base, s)

    while viz.run():
        viz.draw()
        
        
        
if __name__ == "__main__":
    main()