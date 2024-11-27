from softrobots.parts.finger import Finger
import Sofa.ImGui as MyGui
import Sofa.SofaConstraintSolver  # Needed
from math import pi


def createScene(root):
    root.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    root.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]
    root.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    root.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [LinearSolverConstraintCorrection]
    root.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI]
    root.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    root.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    root.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]
    root.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    root.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    root.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [RestShapeSpringsForceField]
    root.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    root.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    root.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetTopologyContainer]
    root.addObject('RequiredPlugin', name='SoftRobots')
    root.addObject('RequiredPlugin', name='SoftRobots.Inverse')

    root.addObject('VisualStyle', displayFlags='showInteractionForceFields')

    root.addObject('QPInverseProblemSolver')
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('ViewerSetting', resolution=[3840, 2160])
    root.gravity.value = [0, -9180, 0]

    finger = Finger(root, youngModulus=600)
    finger.integration.rayleighMass.value=0.1
    finger.integration.rayleighStiffness.value=0.1
    finger.getMass().totalMass.value = 0.075

    cable = finger.PullingCable
    cable.removeObject(cable.CableConstraint)
    cable.addObject('CableActuator', indices=list(range(0, 14)), pullPoint=[0.0, 12.5, 2.5], maxDispVariation=10, minForce=0)

    target = root.addChild('Target')
    target.addObject('MechanicalObject', template='Rigid3', position=[-103, 30, 7, 0, 0, 0, 1], showObject=True, showObjectScale=10)

    effector = finger.addChild('Effector')
    effector.addObject('MechanicalObject', template='Rigid3', position=([-103, 7, 7, 0, 0, 0, 1]))
    effector.addObject('PositionEffector', template='Rigid3', indices=0, effectorGoal=target.getMechanicalState().position.linkpath, maxSpeed=200)
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # To be able to control your robot with the GUI, you need to provide some key components:
    # 1. The node of your effector's target (templated Rigid3)
    # 2. The node of the effector (templated Rigid3)
    # 3. The QPInverseProblemSolver
    MyGui.setIPController(target, effector, root.QPInverseProblemSolver)

    # __My Robot__ Window
    # In this window you can display information and settings
    # 1. MyGui.MyRobotWindow.addInformation(string description, SofaData (scalar) value)
    #    This will simply display a read only value, with your description
    MyGui.MyRobotWindow.addInformation('Motors max displacement', effector.PositionEffector.maxSpeed)
    # 2. MyGui.MyRobotWindow.addSetting(string description, SofaData (scalar) value, float min, float, max)
    #    This will expose your data as an editable setting
    MyGui.MyRobotWindow.addSetting('Max TCP speed (mm/s)', effector.PositionEffector.maxSpeed, 0, 2000)

    # __Move__ Window
    # 1. MyGui.MoveWindow.setTCPLimits(float minDisplacement, float maxDisplacement,
    #                                  float minAngle, float maxAngle) # only if we control the effector in orientation
    #    This will set the limits for the TCP, starting from the initial position of the target.
    MyGui.MoveWindow.setTCPLimits(-100, 100, -pi, pi)
    # 2. MyGui.MoveWindow.setActuatorsDescription(string description)
    #    This allows you to custom the header of the actuators (default is 'Motors Position (rad)')
    MyGui.MoveWindow.setActuatorsDescription('Motor Displacement (mm)')
    # 3. MyGui.MoveWindow.setActuators(list<SofaObject> actuators, list<int> indicesInProblem, string valueType)
    #    Sets and links the actuators
    MyGui.MoveWindow.setActuators([cable.CableActuator.displacement], [0], 'displacement')
    MyGui.MoveWindow.setActuatorsLimits(0, 40)

    # __Plotting__ Window
    # 1. MyGui.PlottingWindow.addData(string description, SofaData (scalar) value)
    #    Adds SofaData (scalar) to plot
    MyGui.PlottingWindow.addData(' displacement', cable.CableActuator.displacement)
    MyGui.PlottingWindow.addData(' force', cable.CableActuator.force)

    # __Program__ Window
    # Sets a program to load at init
    MyGui.ProgramWindow.importProgram("mygui.crprog")
