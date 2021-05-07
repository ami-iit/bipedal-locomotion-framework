import pytest
pytestmark = pytest.mark.ik

import bipedal_locomotion_framework.bindings as blf
import manifpy as manif
import numpy as np

def test_com_task():

    # create KinDynComputationsDescriptor
    kindyn_handler = blf.parameters_handler.StdParametersHandler()
    kindyn_handler.set_parameter_string("model_file_name", "./model.urdf")
    joints_list = ["neck_pitch", "neck_roll", "neck_yaw",
                   "torso_pitch", "torso_roll", "torso_yaw",
                   "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw","l_elbow",
                   "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw","r_elbow",
                   "l_hip_pitch", "l_hip_roll", "l_hip_yaw","l_knee", "l_ankle_pitch", "l_ankle_roll",
                   "r_hip_pitch", "r_hip_roll", "r_hip_yaw","r_knee", "r_ankle_pitch", "r_ankle_roll"]
    kindyn_handler.set_parameter_vector_string("joints_list", joints_list)
    kindyn_desc = blf.floating_base_estimators.construct_kindyncomputations_descriptor(kindyn_handler)
    assert kindyn_desc.is_valid()

    # Set the parameters
    com_param_handler = blf.parameters_handler.StdParametersHandler()
    com_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    com_param_handler.set_parameter_float(name="kp_linear", value=10.0)

    # Initialize the task
    com_task = blf.ik.CoMTask()
    assert com_task.set_kin_dyn(kindyn_desc.kindyn)
    assert com_task.initialize(paramHandler=com_param_handler)
    com_var_handler = blf.system.VariablesHandler()
    assert com_var_handler.add_variable("robotVelocity", 32) is True # robot velocity size = 26 (joints) + 6 (base)
    assert com_task.set_variables_handler(variablesHandler=com_var_handler)
    assert com_task.set_set_point(position=np.array([1.,1.,1.5]), velocity=np.array([0.,0.5,0.5]))


def test_se3_task():

    # create KinDynComputationsDescriptor
    kindyn_handler = blf.parameters_handler.StdParametersHandler()
    kindyn_handler.set_parameter_string("model_file_name", "./model.urdf")
    joints_list = ["neck_pitch", "neck_roll", "neck_yaw",
                   "torso_pitch", "torso_roll", "torso_yaw",
                   "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw","l_elbow",
                   "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw","r_elbow",
                   "l_hip_pitch", "l_hip_roll", "l_hip_yaw","l_knee", "l_ankle_pitch", "l_ankle_roll",
                   "r_hip_pitch", "r_hip_roll", "r_hip_yaw","r_knee", "r_ankle_pitch", "r_ankle_roll"]
    kindyn_handler.set_parameter_vector_string("joints_list", joints_list)
    kindyn_desc = blf.floating_base_estimators.construct_kindyncomputations_descriptor(kindyn_handler)
    assert kindyn_desc.is_valid()

    # Set the parameters
    se3_param_handler = blf.parameters_handler.StdParametersHandler()
    se3_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    se3_param_handler.set_parameter_string(name="frame_name", value="r_sole")
    se3_param_handler.set_parameter_float(name="kp_linear", value=10.0)
    se3_param_handler.set_parameter_float(name="kp_angular", value=10.0)

    # Initialize the task
    se3_task = blf.ik.SE3Task()
    assert se3_task.set_kin_dyn(kindyn_desc.kindyn)
    assert se3_task.initialize(paramHandler=se3_param_handler)
    se3_var_handler = blf.system.VariablesHandler()
    assert se3_var_handler.add_variable("robotVelocity", 32) is True  # robot velocity size = 26 (joints) + 6 (base)
    assert se3_task.set_variables_handler(variablesHandler=se3_var_handler)

    # Set desiderata
    I_H_F = manif.SE3(position=[1.0, -2.0, 3.3], quaternion=[0, 0, -1, 0])
    mixedVelocity = manif.SE3Tangent() # TODO
    assert se3_task.set_set_point(I_H_F=I_H_F, mixedVelocity=mixedVelocity)


def test_so3_task():

    # create KinDynComputationsDescriptor
    kindyn_handler = blf.parameters_handler.StdParametersHandler()
    kindyn_handler.set_parameter_string("model_file_name", "./model.urdf")
    joints_list = ["neck_pitch", "neck_roll", "neck_yaw",
                   "torso_pitch", "torso_roll", "torso_yaw",
                   "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw","l_elbow",
                   "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw","r_elbow",
                   "l_hip_pitch", "l_hip_roll", "l_hip_yaw","l_knee", "l_ankle_pitch", "l_ankle_roll",
                   "r_hip_pitch", "r_hip_roll", "r_hip_yaw","r_knee", "r_ankle_pitch", "r_ankle_roll"]
    kindyn_handler.set_parameter_vector_string("joints_list", joints_list)
    kindyn_desc = blf.floating_base_estimators.construct_kindyncomputations_descriptor(kindyn_handler)
    assert kindyn_desc.is_valid()

    # Set the parameters
    so3_param_handler = blf.parameters_handler.StdParametersHandler()
    so3_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    so3_param_handler.set_parameter_string(name="frame_name", value="chest")
    so3_param_handler.set_parameter_float(name="kp_angular", value=5.0)

    # Initialize the task
    so3_task = blf.ik.SO3Task()
    assert so3_task.set_kin_dyn(kindyn_desc.kindyn)
    assert so3_task.initialize(paramHandler=so3_param_handler)
    so3_var_handler = blf.system.VariablesHandler()
    assert so3_var_handler.add_variable("robotVelocity", 32) is True  # robot velocity size = 26 (joints) + 6 (base)
    assert so3_task.set_variables_handler(variablesHandler=so3_var_handler)

    # Set desiderata
    I_R_F = manif.SO3(quaternion=[0, 0, -1, 0])
    angularVelocity = manif.SO3Tangent() # TODO
    assert so3_task.set_set_point(I_R_F=I_R_F, angularVelocity=angularVelocity)


def test_joint_tracking_task():

    # create KinDynComputationsDescriptor
    kindyn_handler = blf.parameters_handler.StdParametersHandler()
    kindyn_handler.set_parameter_string("model_file_name", "./model.urdf")
    joints_list = ["neck_pitch", "neck_roll", "neck_yaw",
                   "torso_pitch", "torso_roll", "torso_yaw",
                   "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw","l_elbow",
                   "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw","r_elbow",
                   "l_hip_pitch", "l_hip_roll", "l_hip_yaw","l_knee", "l_ankle_pitch", "l_ankle_roll",
                   "r_hip_pitch", "r_hip_roll", "r_hip_yaw","r_knee", "r_ankle_pitch", "r_ankle_roll"]
    kindyn_handler.set_parameter_vector_string("joints_list", joints_list)
    kindyn_desc = blf.floating_base_estimators.construct_kindyncomputations_descriptor(kindyn_handler)
    assert kindyn_desc.is_valid()

    # Set the parameters
    joint_tracking_param_handler = blf.parameters_handler.StdParametersHandler()
    joint_tracking_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    joint_tracking_param_handler.set_parameter_vector_float(name="kp",value=[5.0]*kindyn_desc.get_nr_of_dofs())

    # Initialize the task
    joint_tracking_task = blf.ik.JointTrackingTask()
    assert joint_tracking_task.set_kin_dyn(kindyn_desc.kindyn)
    assert joint_tracking_task.initialize(paramHandler=joint_tracking_param_handler)
    joint_tracking_var_handler = blf.system.VariablesHandler()
    assert joint_tracking_var_handler.add_variable("robotVelocity", 32) is True  # robot velocity size = 26 (joints) + 6 (base)
    assert joint_tracking_task.set_variables_handler(variablesHandler=joint_tracking_var_handler)

    # Set desired joint pos
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.get_nr_of_dofs())]
    assert joint_tracking_task.set_set_point(jointPosition=joint_values)

    # Set desired joint pos and vel
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.get_nr_of_dofs())]
    joint_velocities = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.get_nr_of_dofs())]
    assert joint_tracking_task.set_set_point(jointPosition=joint_values,jointVelocity=joint_velocities)
