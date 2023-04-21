#include <BiomechanicalAnalysis/DynamicalIK/InverseVelocityKinematics.h>

#include <BiomechanicalAnalysis/Logging/Logger.h>

#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

using namespace BiomechanicalAnalysis::DynamicalIK;

static Eigen::Vector3d initializeGravityVector()
{
    Eigen::Vector3d gravityVector;
    gravityVector << 0, 0, -9.81;
    return gravityVector;
}

static const Eigen::Vector3d GRAVITY_VECTOR = initializeGravityVector();

class InverseVelocityKinematics::Impl
{
public:

Impl(){};
~Impl() = default;

bool _initialized = false;


std::shared_ptr<iDynTree::Model> _model = nullptr;
std::vector<std::string> _jointNames;
std::string _baseFrame = "";
std::vector<std::shared_ptr<PositionTarget>> _positionTargets;
std::vector<std::shared_ptr<OrientationTarget>> _orientationTargets;

// iDynTree objects
std::shared_ptr<iDynTree::KinDynComputations> _kinDynComputations = std::make_shared<iDynTree::KinDynComputations>();

// Integrator utility
double _dt = 0;
std::chrono::nanoseconds _dt_as_chrono_ns;

// BLF objects
BipedalLocomotion::System::VariablesHandler _variablesHandler;
BipedalLocomotion::IK::QPInverseKinematics _ikSolver;
std::vector<std::shared_ptr<BipedalLocomotion::IK::R3Task>> _R3Tasks;
std::vector<std::shared_ptr<BipedalLocomotion::IK::SO3Task>> _SO3Tasks;
std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> _kinematics = std::make_shared<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>();
BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> _integrator;

// Buffer objects
manif::SO3Tangent<double> _bufferSO3tangent = manif::SO3Tangent<double>::Zero();

// Results
Eigen::VectorX<double> _jointPositions;
Eigen::VectorX<double> _jointVelocities;
Eigen::Matrix<double,4,4> _basePose;
Eigen::Matrix<double,3,1> _baseLinearVelocity;
Eigen::Vector3<double> _CoMPosition;
Eigen::Vector3<double> _CoMVelocity;


bool addTarget(std::shared_ptr<OrientationTarget>& target)
{
    _initialized = false;
    _orientationTargets.push_back(target);
    //TODO
    return true;
}

bool addTarget(std::shared_ptr<PositionTarget>& target)
{
    _initialized = false;
    _positionTargets.push_back(target);
    //TODO
    return true;
}


bool addConstraint(std::shared_ptr<Constraint>& constraint)
{
    _initialized = false;
    //TODO
    return true;
}

// IntegrationBasedSolver methods
bool setModel(std::shared_ptr<iDynTree::Model>& model)
{
    _initialized = false;
    _model = model;
    _jointNames.clear();

    size_t nrOfDOFs = getNumberOfDofs();
    for(size_t j=0; j<nrOfDOFs; j++){
        _jointNames.push_back(_model->getJointName(j));
    }

    return true;
}

bool setBaseFrame(const std::string& baseFrame)
{
    _initialized = false; //TODO
    _baseFrame = baseFrame;
    return true;
}

bool initialize()
{
    BiomechanicalAnalysis::log()->info("Starting main");

    //TODO check values
    if(!_model){
        BiomechanicalAnalysis::log()->error("Model has not been set!");
        return false;
    }

    if(_baseFrame == ""){
        BiomechanicalAnalysis::log()->info("Base frame has not been set, using default base frame: {}", _model->getFrameName(_model->getDefaultBaseLink())); //TODO check this
        return false;
    }

    //TODO Initialize buffers
    _jointPositions.resize(getNumberOfDofs());
    _jointVelocities.resize(getNumberOfDofs());
    _jointPositions.fill(0);
    _jointVelocities.fill(0);
    _basePose = Eigen::Matrix4d::Identity();
    _CoMPosition = Eigen::Vector3d::Zero();
    _CoMVelocity = Eigen::Vector3d::Zero();

    //TODO Initialize KinDyn object
    _kinDynComputations->loadRobotModel(*_model);
    _kinDynComputations->setFloatingBase(_baseFrame);

    // Initialize BLF objects

    // Variables handler
    _variablesHandler.addVariable("robotVelocity", getNumberOfDofs() + 6);
    
    // IK solver
    auto ikParameterHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ikParameterHandler->setParameter("robot_velocity_variable_name", "robotVelocity");
    ikParameterHandler->setParameter("verbosity", false);
    if(!_ikSolver.initialize(ikParameterHandler)){
        BiomechanicalAnalysis::log()->error("Unable to initialize the QPInverseKinematics");
        return false;
    }

    //TODO
    const Eigen::Vector3d constWeight = Eigen::Vector3d::Ones();

    // Create R3 tasks
    for(const auto& target : _positionTargets){
        auto R3ParameterHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        R3ParameterHandler->setParameter("robot_velocity_variable_name", "robotVelocity");

        // set parameters 
        R3ParameterHandler->setParameter("kp_linear", target->getFeedbackGain());
        R3ParameterHandler->setParameter("frame_name", target->getFrame());
        R3ParameterHandler->setParameter("mask", target->getControlledAxis());

        auto R3Task = std::make_shared<BipedalLocomotion::IK::R3Task>();
        R3Task->setKinDyn(_kinDynComputations);
        if (!R3Task->initialize(R3ParameterHandler)){
            BiomechanicalAnalysis::log()->error("Unable to initialize R3 task for frame: {}", target->getFrame());
            return false;
        }
        _ikSolver.addTask(R3Task, target->getName(), 0, constWeight);

        _R3Tasks.push_back(R3Task);
    }

    // Create SO3 tasks
    for(const auto& target : _orientationTargets){
        auto SO3ParameterHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        SO3ParameterHandler->setParameter("robot_velocity_variable_name", "robotVelocity");

        SO3ParameterHandler->setParameter("kp_angular", target->getFeedbackGain());
        SO3ParameterHandler->setParameter("frame_name", target->getFrame());
        SO3ParameterHandler->setParameter("mask", target->getControlledAxis());

        auto SO3Task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
        SO3Task->setKinDyn(_kinDynComputations);
        if (!SO3Task->initialize(SO3ParameterHandler)){
            BiomechanicalAnalysis::log()->error("Unable to initialize SO3 task for frame: {}", target->getFrame());
            return false;
        }
        _ikSolver.addTask(SO3Task, target->getName(), 0, constWeight);
        _SO3Tasks.push_back(SO3Task);
    }


    // Joint regularization task
    auto jointRegularizationHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    jointRegularizationHandler->setParameter("robot_velocity_variable_name", "robotVelocity");
    Eigen::VectorXd kpRegularization;
    kpRegularization.setConstant(getNumberOfDofs(), 0.);
    jointRegularizationHandler->setParameter("kp", kpRegularization);
    auto regularizationTask = std::make_shared<BipedalLocomotion::IK::JointTrackingTask>();
    regularizationTask->setKinDyn(_kinDynComputations);
    if(!regularizationTask->initialize(jointRegularizationHandler)){
        BiomechanicalAnalysis::log()->error("Unable to initialize the joint regularization task");
        return false;
    }
    _ikSolver.addTask(regularizationTask, "regularization_task", 1, kpRegularization);

    if(!_ikSolver.finalize(_variablesHandler)){
        BiomechanicalAnalysis::log()->error("Unable to finalize ik solver");
        return false;
    }

    //TODO state integrator
    BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics::State kinematicsState;
    kinematicsState.get<0>() = Eigen::Vector3d::Zero(); // p
    kinematicsState.get<1>() = manif::SO3d::Identity(); // R
    kinematicsState.get<2>() = _jointPositions; // s

    auto forwardEulerHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    _kinematics->initialize(forwardEulerHandler);
    _kinematics->setState(kinematicsState);
    _integrator.setIntegrationStep(_dt_as_chrono_ns);
    _integrator.setDynamicalSystem(_kinematics);

    _initialized = true;

    return true;
}

bool update()
{

    // update R3 tasks
    for(int targetIdx=0; targetIdx<_positionTargets.size();targetIdx++){
        _R3Tasks[targetIdx]->setSetPoint(_positionTargets[targetIdx]->getPosition(), _positionTargets[targetIdx]->getLinearVelocity());
    }

    // update SO3 tasks
    for(int targetIdx=0; targetIdx<_orientationTargets.size();targetIdx++){
        _bufferSO3tangent.ang() = _orientationTargets[targetIdx]->getAngularVelocity();
        _SO3Tasks[targetIdx]->setSetPoint(_orientationTargets[targetIdx]->getOrientation(), _bufferSO3tangent);
    }

    // advance ik solver
    if(!_ikSolver.advance()){
        BiomechanicalAnalysis::log()->error("Unable to advance the solver");
        return false;
    }

    // set control input for the dynamical system
    if(!_kinematics->setControlInput({_ikSolver.getOutput().baseVelocity.coeffs(), 
                                      _ikSolver.getOutput().jointVelocity})){
        BiomechanicalAnalysis::log()->error("Unable to set the control input");
        return false;
    }

    // integrate
    auto dt_as_chrono_s = std::chrono::duration<double>(_dt);
    std::chrono::nanoseconds dt_as_chrono_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(dt_as_chrono_s);
    if(!_integrator.integrate(std::chrono::nanoseconds(0), dt_as_chrono_ns)){ //TODO from step
        BiomechanicalAnalysis::log()->error("Unable to integrate solution");
        return false;
    }

    const auto& [basePosition, baseRotation, jointPositions]
                    = _integrator.getSolution();

    _kinDynComputations->setRobotState(_basePose,
                                        jointPositions,
                                        _ikSolver.getOutput().baseVelocity.coeffs(),
                                        _ikSolver.getOutput().jointVelocity,
                                        GRAVITY_VECTOR);


    // update output values
    _basePose.block<3,1>(0,3) = basePosition;
    _basePose.block<3,3>(0,0) = baseRotation.rotation();
    _jointPositions = jointPositions;
    _jointVelocities = _ikSolver.getOutput().jointVelocity;
    _baseLinearVelocity = _ikSolver.getOutput().baseVelocity.coeffs().block<3,1>(0,0);
    _CoMPosition = iDynTree::toEigen(_kinDynComputations->getCenterOfMassPosition());
    _CoMVelocity = iDynTree::toEigen(_kinDynComputations->getCenterOfMassVelocity());

    return true;
}

std::vector<std::string> getJointNames() const
{   
    return _jointNames;
}

std::string getBaseFrame() const
{
    return _baseFrame;
}

size_t getNumberOfDofs() const
{
    return _model->getNrOfDOFs();
}

const Eigen::VectorX<double>& getJointPositions() const
{
    return _jointPositions;
}

const Eigen::VectorX<double>& getJointVelocities() const
{
    return _jointVelocities;
}

const Eigen::Matrix<double,4,4>& getBasePose() const
{
    return _basePose;
}

const Eigen::Vector3<double>& getBaseVelocity() const
{
    return _baseLinearVelocity;
}

const Eigen::Vector3<double>& getCoMPosition() const
{
    return _CoMPosition;
}

const Eigen::Vector3<double>& getCoMVelocity() const
{
    return _CoMVelocity;
}

bool isValid() const 
{
    //TODO
    return true;
}

};

InverseVelocityKinematics::InverseVelocityKinematics()
{
    impl = std::make_unique<Impl>();
}

bool InverseVelocityKinematics::addTarget(std::shared_ptr<PositionTarget> target){ return impl->addTarget(target); }

bool InverseVelocityKinematics::addTarget(std::shared_ptr<OrientationTarget> target){ return impl->addTarget(target); }

bool InverseVelocityKinematics::addConstraint(std::shared_ptr<Constraint>& constraint){ return impl->addConstraint(constraint); }

// IntegrationBasedSolver methods
bool InverseVelocityKinematics::setModel(std::shared_ptr<iDynTree::Model>& model){ return impl->setModel(model); };

bool InverseVelocityKinematics::setBaseFrame(const std::string& baseFrame){ return impl->setBaseFrame(baseFrame); }

bool InverseVelocityKinematics::initialize(){ return impl->initialize(); }

bool InverseVelocityKinematics::update(){ return impl->update(); }

std::vector<std::string> InverseVelocityKinematics::getJointNames() const { return impl->getJointNames(); }

std::string InverseVelocityKinematics::getBaseFrame() const { return impl->getBaseFrame(); }

size_t InverseVelocityKinematics::getNumberOfDofs() const { return impl->getNumberOfDofs(); }

const Eigen::VectorX<double>& InverseVelocityKinematics::getJointPositions() const { return impl->getJointPositions(); }

const Eigen::VectorX<double>& InverseVelocityKinematics::getJointVelocities() const { return impl->getJointVelocities(); }

const Eigen::Matrix<double,4,4>& InverseVelocityKinematics::getBasePose() const { return impl->getBasePose(); }

const Eigen::Vector3<double>& InverseVelocityKinematics::getBaseVelocity() const { return impl->getBaseVelocity(); }

const Eigen::Vector3<double>& InverseVelocityKinematics::getCoMPosition() const { return impl->getCoMPosition(); }

const Eigen::Vector3<double>& InverseVelocityKinematics::getCoMVelocity() const { return impl->getCoMVelocity(); }

void InverseVelocityKinematics::setStep(const double dt)
{ 
    impl->_dt = dt; 
    auto dt_as_chrono_s = std::chrono::duration<double>(dt);
    impl->_dt_as_chrono_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(dt_as_chrono_s);    
}

double InverseVelocityKinematics::getStep() { return impl->_dt; }

bool InverseVelocityKinematics::isValid() const { return impl->isValid(); }

InverseVelocityKinematics::~InverseVelocityKinematics() = default;
