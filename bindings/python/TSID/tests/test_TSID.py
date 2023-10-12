import pytest
pytestmark = pytest.mark.tsid

import bipedal_locomotion_framework.bindings as blf
import idyntree.swig as idyn
import manifpy as manif
import numpy as np
import tempfile

import icub_models

def test_custom_task():
    class CustomTask(blf.tsid.TSIDLinearTask):
        def __init__(self):
            blf.tsid.TSIDLinearTask.__init__(self)

            self._description = "Custom Task"

        def set_variables_handler(self, variables_handler: blf.system.VariablesHandler):
            variable = variables_handler.get_variable("robotAcceleration")
            temp = np.zeros((10, variables_handler.get_number_of_variables()))

            # avoiding slicing
            for i in range(10):
                temp[i, variable.offset] = 1

            self._A = temp
            self._b = np.ones(10)
            return True

        def size(self):
            return 10

        def type(self):
            return blf.tsid.TSIDLinearTask.Type.equality

        def is_valid(self):
            return True

    # Set the parameters
    qp_tsid_param_handler = blf.parameters_handler.StdParametersHandler()
    qp_tsid_param_handler.set_parameter_string(name="robot_acceleration_variable_name", value="robotAcceleration")
    qp_tsid_param_handler.set_parameter_string(name="joint_torques_variable_name", value="jointTorque")
    qp_tsid_param_handler.set_parameter_vector_string(name="contact_wrench_variables_name", value=["contact_1"])

    # Initialize the QP inverse kinematics
    qp_tsid = blf.tsid.QPTSID()
    assert qp_tsid.initialize(handler=qp_tsid_param_handler)

    qp_tsid_param_handler = blf.system.VariablesHandler()
    assert qp_tsid_param_handler.add_variable("robotAcceleration",
                                              38) is True  # robot velocity size = 32 (joints) + 6 (base)
    assert qp_tsid_param_handler.add_variable("jointTorque",
                                              32) is True  # robot velocity size = 32 (joints) + 6 (base)
    assert qp_tsid_param_handler.add_variable("contact_1",
                                              6) is True  # robot velocity size = 32 (joints) + 6 (base)

    # add the custom task
    custom_task = CustomTask()
    assert qp_tsid.add_task(custom_task, "custom_task", 0)
    assert qp_tsid.finalize(qp_tsid_param_handler)


def get_kindyn():

    joints_list = ["neck_pitch", "neck_roll", "neck_yaw",
                   "torso_pitch", "torso_roll", "torso_yaw",
                   "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw","l_elbow",
                   "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw","r_elbow",
                   "l_hip_pitch", "l_hip_roll", "l_hip_yaw","l_knee", "l_ankle_pitch", "l_ankle_roll",
                   "r_hip_pitch", "r_hip_roll", "r_hip_yaw","r_knee", "r_ankle_pitch", "r_ankle_roll"]

    model_loader = idyn.ModelLoader()
    assert model_loader.loadReducedModelFromFile(str(icub_models.get_model_file("iCubGazeboV2_5")), joints_list)

    # create KinDynComputationsDescriptor
    kindyn = idyn.KinDynComputations()
    assert kindyn.loadRobotModel(model_loader.model())

    return joints_list, kindyn

def test_com_task():

    kp = 1.0
    kd = 0.5

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    com_param_handler = blf.parameters_handler.StdParametersHandler()
    com_param_handler.set_parameter_string(name="robot_acceleration_variable_name", \
                                           value="robotAcceleration")
    com_param_handler.set_parameter_float(name="kp_linear", value=kp)
    com_param_handler.set_parameter_float(name="kd_linear", value=kd)

    # Initialize the task
    com_task = blf.tsid.CoMTask()
    assert com_task.set_kin_dyn(kindyn)
    assert com_task.initialize(param_handler=com_param_handler)

    com_var_handler = blf.system.VariablesHandler()

    # robot velocity size = 26 (joints) + 6 (base)
    assert com_var_handler.add_variable("robotAcceleration", len(joints_list) + 6) is True
    assert com_task.set_variables_handler(variables_handler=com_var_handler)
    assert com_task.set_set_point(position=np.array([1.,1.,1.5]),
                                  velocity=np.array([0.,0.5,0.5]),
                                  acceleration=np.array([0.,-0.1,0.1]))

def test_se3_task():

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    se3_param_handler = blf.parameters_handler.StdParametersHandler()
    se3_param_handler.set_parameter_string(name="robot_acceleration_variable_name", value="robotAcceleration")
    se3_param_handler.set_parameter_string(name="frame_name", value="r_sole")
    se3_param_handler.set_parameter_float(name="kp_linear", value=10.0)
    se3_param_handler.set_parameter_float(name="kp_angular", value=10.0)
    se3_param_handler.set_parameter_float(name="kd_linear", value=5.0)
    se3_param_handler.set_parameter_float(name="kd_angular", value=5.0)

    # Initialize the task
    se3_task = blf.tsid.SE3Task()
    assert se3_task.set_kin_dyn(kindyn)
    assert se3_task.initialize(param_handler=se3_param_handler)
    se3_var_handler = blf.system.VariablesHandler()
    assert se3_var_handler.add_variable("robotAcceleration", len(joints_list) + 6) is True
    assert se3_task.set_variables_handler(variables_handler=se3_var_handler)

    # Set desiderata
    I_H_F = manif.SE3(position=[1.0, -2.0, 3.3], quaternion=[0, 0, -1, 0])
    mixed_velocity = manif.SE3Tangent([0.0]*6) # TODO: proper assignment
    mixed_acceleration = manif.SE3Tangent([0.0]*6) # TODO: proper assignment
    assert se3_task.set_set_point(I_H_F=I_H_F,
                                  mixed_velocity=mixed_velocity,
                                  mixed_acceleration=mixed_acceleration)
    # Update the task
    assert se3_task.update()

    # Check get_controller_output
    assert se3_task.get_controller_output().size == 6

def test_so3_task():

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    so3_param_handler = blf.parameters_handler.StdParametersHandler()
    so3_param_handler.set_parameter_string(name="robot_acceleration_variable_name", value="robotAcceleration")
    so3_param_handler.set_parameter_string(name="frame_name", value="chest")
    so3_param_handler.set_parameter_float(name="kp_angular", value=5.0)
    so3_param_handler.set_parameter_float(name="kd_angular", value=1.0)

    # Initialize the task
    so3_task = blf.tsid.SO3Task()
    assert so3_task.set_kin_dyn(kindyn)
    assert so3_task.initialize(param_handler=so3_param_handler)
    so3_var_handler = blf.system.VariablesHandler()
    assert so3_var_handler.add_variable("robotAcceleration", len(joints_list) + 6) is True
    assert so3_task.set_variables_handler(variables_handler=so3_var_handler)

    # Set desiderata
    I_R_F = manif.SO3(quaternion=[0, 0, -1, 0])
    angular_velocity = manif.SO3Tangent([0.0]*3) # TODO: proper assignment
    angular_acceleration = manif.SO3Tangent([0.0]*3) # TODO: proper assignment
    assert so3_task.set_set_point(I_R_F=I_R_F,
                                  angular_velocity=angular_velocity,
                                  angular_acceleration=angular_acceleration)

def test_joint_tracking_task():

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    joint_tracking_param_handler = blf.parameters_handler.StdParametersHandler()
    joint_tracking_param_handler.set_parameter_string(name="robot_acceleration_variable_name", value="robotAcceleration")
    joint_tracking_param_handler.set_parameter_vector_float(name="kp",value=[5.0]*kindyn.getNrOfDegreesOfFreedom())
    joint_tracking_param_handler.set_parameter_vector_float(name="kd",value=[1.0]*kindyn.getNrOfDegreesOfFreedom())

    # Initialize the task
    joint_tracking_task = blf.tsid.JointTrackingTask()
    assert joint_tracking_task.set_kin_dyn(kindyn)
    assert joint_tracking_task.initialize(param_handler=joint_tracking_param_handler)
    joint_tracking_var_handler = blf.system.VariablesHandler()

    # robot velocity size = 26 (joints) + 6 (base)
    assert joint_tracking_var_handler.add_variable("robotAcceleration", len(joints_list) + 6) is True
    assert joint_tracking_task.set_variables_handler(variables_handler=joint_tracking_var_handler)

    # Set desired joint pos
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn.getNrOfDegreesOfFreedom())]
    assert joint_tracking_task.set_set_point(joint_position=joint_values)

    # Set desired joint pos and vel
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn.getNrOfDegreesOfFreedom())]
    joint_velocities = [np.random.uniform(-0.5,0.5) for _ in range(kindyn.getNrOfDegreesOfFreedom())]
    assert joint_tracking_task.set_set_point(joint_position=joint_values,joint_velocity=joint_velocities)

def test_feasible_contact_wrench():

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    feasible_contact_wrench_param_handler = blf.parameters_handler.StdParametersHandler()
    feasible_contact_wrench_param_handler.set_parameter_int(name="number_of_slices", value=2)
    feasible_contact_wrench_param_handler.set_parameter_float(name="static_friction_coefficient",
                                                              value=0.3)
    feasible_contact_wrench_param_handler.set_parameter_vector_float(name="foot_limits_x",
                                                                     value=[-0.05, 0.1])
    feasible_contact_wrench_param_handler.set_parameter_vector_float(name="foot_limits_y",
                                                                     value=[-0.02, 0.01])
    feasible_contact_wrench_param_handler.set_parameter_string(name="variable_name",
                                                               value="left_foot_contact")
    feasible_contact_wrench_param_handler.set_parameter_string(name="frame_name",
                                                               value="l_sole")

    # Initialize the task
    feasible_contact_wrench_task = blf.tsid.FeasibleContactWrenchTask()
    assert feasible_contact_wrench_task.set_kin_dyn(kindyn)
    assert feasible_contact_wrench_task.initialize(param_handler=feasible_contact_wrench_param_handler)
    feasible_contact_wrench_var_handler = blf.system.VariablesHandler()

    assert feasible_contact_wrench_var_handler.add_variable("left_foot_contact", 6) is True
    assert feasible_contact_wrench_task.set_variables_handler(variables_handler=feasible_contact_wrench_var_handler)

def test_dynamics():

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    param_handler = blf.parameters_handler.StdParametersHandler()
    param_handler.set_parameter_string(name="robot_acceleration_variable_name",
                                       value="robotAcceleration")
    param_handler.set_parameter_string(name="joint_torques_variable_name",
                                       value="jointsTorque")
    param_handler.set_parameter_int(name="max_number_of_contacts", value=1)

    contact_group = blf.parameters_handler.StdParametersHandler()
    contact_group.set_parameter_string(name="variable_name",
                                       value="left_foot_contact")
    contact_group.set_parameter_string(name="frame_name",
                                       value="l_sole")
    assert param_handler.set_group("CONTACT_0", contact_group)

    var_handler = blf.system.VariablesHandler()
    var_handler.add_variable("robotAcceleration", len(joints_list) + 6)
    var_handler.add_variable("jointsTorque", len(joints_list))
    var_handler.add_variable("left_foot_contact", 6)


    # Initialize the task
    base_dynamics_task = blf.tsid.BaseDynamicsTask()
    assert base_dynamics_task.set_kin_dyn(kindyn)
    assert base_dynamics_task.initialize(param_handler=param_handler)
    assert base_dynamics_task.set_variables_handler(variables_handler=var_handler)

    # Initialize the task
    joint_dynamics_task = blf.tsid.JointDynamicsTask()
    assert joint_dynamics_task.set_kin_dyn(kindyn)
    assert joint_dynamics_task.initialize(param_handler=param_handler)
    assert joint_dynamics_task.set_variables_handler(variables_handler=var_handler)

def test_variable_regularization_task():

    # get  kindyn
    joints_list, kindyn = get_kindyn()

    # Set the parameters
    param_handler_1 = blf.parameters_handler.StdParametersHandler()
    param_handler_1.set_parameter_string(name="variable_name", value="variable_1")
    param_handler_1.set_parameter_int(name="variable_size", value=15)

    # Set the parameters
    param_handler_2 = blf.parameters_handler.StdParametersHandler()
    param_handler_2.set_parameter_string(name="variable_name", value="variable_2")
    param_handler_2.set_parameter_int(name="variable_size", value=3)
    param_handler_2.set_parameter_vector_string(name="elements_name",
                                                value = ["huey", "dewey", "louie"])


    var_handler = blf.system.VariablesHandler()
    var_handler.add_variable("variable_1", 15)
    var_handler.add_variable("variable_2", ["donald", "huey", "dewey", "louie", "daisy"])

    # Initialize the task
    regularizer_1 = blf.tsid.VariableRegularizationTask()
    assert regularizer_1.initialize(param_handler=param_handler_1)
    assert regularizer_1.set_variables_handler(variables_handler=var_handler)
    assert regularizer_1.set_variables_handler(variables_handler=var_handler)

    # Initialize the task
    regularizer_2 = blf.tsid.VariableRegularizationTask()
    assert regularizer_2.initialize(param_handler=param_handler_2)
    assert regularizer_2.set_variables_handler(variables_handler=var_handler)
