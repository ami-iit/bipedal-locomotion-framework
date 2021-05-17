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
    assert com_task.initialize(param_handler=com_param_handler)
    com_var_handler = blf.system.VariablesHandler()
    assert com_var_handler.add_variable("robotVelocity", 32) is True # robot velocity size = 26 (joints) + 6 (base)
    assert com_task.set_variables_handler(variables_handler=com_var_handler)
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
    assert se3_task.initialize(param_handler=se3_param_handler)
    se3_var_handler = blf.system.VariablesHandler()
    assert se3_var_handler.add_variable("robotVelocity", 32) is True  # robot velocity size = 26 (joints) + 6 (base)
    assert se3_task.set_variables_handler(variables_handler=se3_var_handler)

    # Set desiderata
    I_H_F = manif.SE3(position=[1.0, -2.0, 3.3], quaternion=[0, 0, -1, 0])
    mixed_velocity = manif.SE3Tangent([0.0]*6) # TODO: proper assignment
    assert se3_task.set_set_point(I_H_F=I_H_F, mixed_velocity=mixed_velocity)


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
    assert so3_task.initialize(param_handler=so3_param_handler)
    so3_var_handler = blf.system.VariablesHandler()
    assert so3_var_handler.add_variable("robotVelocity", 32) is True  # robot velocity size = 26 (joints) + 6 (base)
    assert so3_task.set_variables_handler(variables_handler=so3_var_handler)

    # Set desiderata
    I_R_F = manif.SO3(quaternion=[0, 0, -1, 0])
    angularVelocity = manif.SO3Tangent([0.0]*3) # TODO: proper assignment
    assert so3_task.set_set_point(I_R_F=I_R_F, angular_velocity=angularVelocity)


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
    joint_tracking_param_handler.set_parameter_vector_float(name="kp",value=[5.0]*kindyn_desc.kindyn.get_nr_of_dofs())

    # Initialize the task
    joint_tracking_task = blf.ik.JointTrackingTask()
    assert joint_tracking_task.set_kin_dyn(kindyn_desc.kindyn)
    assert joint_tracking_task.initialize(param_handler=joint_tracking_param_handler)
    joint_tracking_var_handler = blf.system.VariablesHandler()
    assert joint_tracking_var_handler.add_variable("robotVelocity", 32) is True  # robot velocity size = 26 (joints) + 6 (base)
    assert joint_tracking_task.set_variables_handler(variables_handler=joint_tracking_var_handler)

    # Set desired joint pos
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.kindyn.get_nr_of_dofs())]
    assert joint_tracking_task.set_set_point(joint_position=joint_values)

    # Set desired joint pos and vel
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.kindyn.get_nr_of_dofs())]
    joint_velocities = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.kindyn.get_nr_of_dofs())]
    assert joint_tracking_task.set_set_point(joint_position=joint_values,joint_velocity=joint_velocities)

def test_integration_based_ik_state():

    state = blf.ik.IntegrationBasedIKState()

    random_joint_velocity = [np.random.uniform(-0.5,0.5) for _ in range(26)]
    state.joint_velocity = random_joint_velocity
    assert state.joint_velocity == pytest.approx(random_joint_velocity)

    random_base_velocity = manif.SE3Tangent([0.0]*6) # TODO: proper assignment
    state.base_velocity = random_base_velocity
    assert state.base_velocity.coeffs() == pytest.approx(random_base_velocity.coeffs())

def test_qp_inverse_kinematics():

    # Set the parameters
    qp_ik_param_handler = blf.parameters_handler.StdParametersHandler()
    qp_ik_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")

    # Initialize the QP inverse kinematics
    qp_ik = blf.ik.QPInverseKinematics()
    assert qp_ik.initialize(handler=qp_ik_param_handler)

    # create KinDynComputationsDescriptor
    kindyn_handler = blf.parameters_handler.StdParametersHandler()
    kindyn_handler.set_parameter_string("model_file_name", "./model.urdf")
    joints_list = ['l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll', 'r_hip_pitch',
                   'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll', 'torso_pitch', 'torso_roll',
                   'torso_yaw', 'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', 'l_wrist_prosup',
                   'l_wrist_pitch', 'l_wrist_yaw', 'neck_pitch', 'neck_roll', 'neck_yaw', 'r_shoulder_pitch',
                   'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 'r_wrist_prosup', 'r_wrist_pitch', 'r_wrist_yaw']
    kindyn_handler.set_parameter_vector_string("joints_list", joints_list)
    kindyn_desc = blf.floating_base_estimators.construct_kindyncomputations_descriptor(kindyn_handler)
    assert kindyn_desc.is_valid()

    # Set the initial robot state
    world_T_base = np.asarray([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
    joint_values = [0.4147046680648114, 0.020109925966605414, -0.008920202688283316, -0.9417931898020863,
                    -0.5243497613177115, -0.021969353979888327, 0.41490863370740766, 0.019729971064546152,
                    -0.00875663754461344, -0.942238731363784, -0.5245880299799013, -0.021615580177534933,
                    0.10368807143683698, -1.7513061346176159e-06, -3.762644846952276e-05, -0.1599473872698949,
                    0.4341604471687158, 0.18350557928473807, 0.5390720970077302, 6.0766272665334824e-05,
                    -0.0010549820549572227, -6.457409077219657e-05, 0.0002255313408076292, 2.3204341497780863e-05,
                    5.920527253092593e-07, -0.1591984430641039, 0.4340643739146118, 0.18311274358002663,
                    0.5398797698508592, 5.367856260643534e-05, -0.0010455524092736824, 6.414366731688693e-05]
    s = np.array(joint_values)
    base_velocity = [0.0] * 6
    s_dot = [0.0] * kindyn_desc.kindyn.get_nr_of_dofs()
    world_gravity = [0.0, 0.0, blf.math.StandardAccelerationOfGravitation]
    assert kindyn_desc.kindyn.set_robot_state(world_T_base, s, base_velocity, s_dot, world_gravity)

    # Configure CoM task
    com_param_handler = blf.parameters_handler.StdParametersHandler()
    com_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    com_param_handler.set_parameter_float(name="kp_linear", value=10.0)
    com_task = blf.ik.CoMTask()
    assert com_task.set_kin_dyn(kindyn_desc.kindyn)
    assert com_task.initialize(param_handler=com_param_handler)
    com_var_handler = blf.system.VariablesHandler()
    assert com_var_handler.add_variable("robotVelocity", 38) is True # robot velocity size = 32 (joints) + 6 (base)
    assert com_task.set_variables_handler(variables_handler=com_var_handler)
    assert com_task.set_set_point(position=np.array([0.,0.,0.53]), velocity=np.array([0.,0.,0.]))

    # Add CoM task as hard constraint
    assert qp_ik.add_task(task=com_task, taskName="Com_task", priority=0)

    # Configure SE3 task (right foot)
    se3_param_handler = blf.parameters_handler.StdParametersHandler()
    se3_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    se3_param_handler.set_parameter_string(name="frame_name", value="r_sole")
    se3_param_handler.set_parameter_float(name="kp_linear", value=10.0)
    se3_param_handler.set_parameter_float(name="kp_angular", value=10.0)
    se3_task = blf.ik.SE3Task()
    assert se3_task.set_kin_dyn(kindyn_desc.kindyn)
    assert se3_task.initialize(param_handler=se3_param_handler)
    se3_var_handler = blf.system.VariablesHandler()
    assert se3_var_handler.add_variable("robotVelocity", 38) is True  # robot velocity size = 32 (joints) + 6 (base)
    assert se3_task.set_variables_handler(variables_handler=se3_var_handler)
    I_H_F = manif.SE3(position=[-0.02, 0.1, 0.0], quaternion=[1, 0, 0, 0])
    mixed_velocity = manif.SE3Tangent([0.0]*6)  # TODO: proper assignment
    assert se3_task.set_set_point(I_H_F=I_H_F, mixed_velocity=mixed_velocity)

    # Add SE3 task as hard constraint
    assert qp_ik.add_task(task=se3_task, taskName="se3_task", priority=0)

    # Configure joint tracking task (regularization)
    joint_tracking_param_handler = blf.parameters_handler.StdParametersHandler()
    joint_tracking_param_handler.set_parameter_string(name="robot_velocity_variable_name", value="robotVelocity")
    joint_tracking_param_handler.set_parameter_vector_float(name="kp",value=[5.0]*kindyn_desc.kindyn.get_nr_of_dofs())
    joint_tracking_task = blf.ik.JointTrackingTask()
    assert joint_tracking_task.set_kin_dyn(kindyn_desc.kindyn)
    assert joint_tracking_task.initialize(param_handler=joint_tracking_param_handler)
    joint_tracking_var_handler = blf.system.VariablesHandler()
    assert joint_tracking_var_handler.add_variable("robotVelocity", 38) is True  # robot velocity size = 32 (joints) + 6 (base)
    assert joint_tracking_task.set_variables_handler(variables_handler=joint_tracking_var_handler)
    joint_values_delta = [0.0011392677340292786, -0.0014866937971185989, -0.004861065064457352, 0.004832281257788355,
                          0.0005562676932744891, 0.0008835707784638569, -0.0013229485588518307, -0.00018524688500568074,
                          0.004053491738164755,0.0005340230664127431, 0.001292758583814022, -0.0011011443677521105,
                          -0.004805235090001216, 0.004329144865816151, 0.0030415719471654805, -0.0010637207416641682,
                          0.0012597145585905769, 0.0032216098557258663, -0.0016280333730231602, -0.004994650881888809,
                          -0.0019178037434056305, -0.004435872749372345, 0.0001002683234802371, 0.0038944749640689212,
                          0.004691540110817683, -0.0034115105556102344, 0.004406950159849346, 0.0019877633599394316,
                          -0.0027563483992913964, 0.004620313400111453, -0.0033082107094529693, -0.003320732925114648]
    assert joint_tracking_task.set_set_point(joint_position=np.add(joint_values,joint_values_delta))

    # Add joint tracking task as soft constraint
    assert qp_ik.add_task(task=joint_tracking_task, taskName="joint_tracking_task", priority=1, weight=[1.0]*kindyn_desc.kindyn.get_nr_of_dofs())

    # Check that all the tasks have been added
    task_names = qp_ik.get_task_names()
    assert len(task_names) == 3 and 'Com_task' in task_names and 'joint_tracking_task' in task_names and 'se3_task' in task_names

    # Finalize the qp inverse kinematics
    qp_ik_var_handler = blf.system.VariablesHandler()
    assert qp_ik_var_handler.add_variable("robotVelocity", 38) is True  # robot velocity size = 32 (joints) + 6 (base)
    assert qp_ik.finalize(handler=qp_ik_var_handler)

    # Solve the inverse kinematics
    assert qp_ik.advance()

    # Get the inverse kinematics output
    state = qp_ik.get_output()
    assert qp_ik.is_output_valid()

    # Update the desiderata for all the tasks
    assert com_task.set_set_point(position=np.array([0.,0.,0.5]), velocity=np.array([0.,0.,0.]))
    assert se3_task.set_set_point(I_H_F=manif.SE3(position=[-0.02, 0.2, 0.0], quaternion=[1, 0, 0, 0]),
                                  mixed_velocity=manif.SE3Tangent([0.0]*6)) # TODO: proper assignment
    joint_values_delta = [0.0007451200584672694, -0.001696359535882248, -5.733476699915606e-05, 0.0030691521323187162,
                          0.0016973417783450306, 0.004965361525630099, 0.002047452821820156, -0.0022178648205978,
                          -0.00499874605446799, -0.00021545355652612206, 0.002249046530448772, -0.0048592565096339324,
                          -0.001961899445335763, -0.00026554592901836555, -0.00317120389321911, -0.001743324389545805,
                          0.00482031870263355, 0.004596501967191832, -0.0023257148997022237, 0.0014664643967757674,
                          0.0038198161285948337, 0.0016210907365087892, 0.002195183413977628, -0.003929755444460801,
                          0.0018093019088196369, -0.0024510752337251096, 0.0033390174283683413, 0.001312063212095667,
                          -0.004458151270494718, -0.0008821295391808395,5.635798856415061e-05, -0.003568065094916102]
    assert joint_tracking_task.set_set_point(joint_position=np.add(joint_values,joint_values_delta))

    # Solve the updated inverse kinematics
    assert qp_ik.advance()

    # Get the updated inverse kinematics output
    updated_state = qp_ik.get_output()
    assert qp_ik.is_output_valid()

    # Check that with different desiderata the ik solution is different
    assert state.joint_velocity != pytest.approx(updated_state.joint_velocity)
    assert state.base_velocity.coeffs() != pytest.approx(updated_state.base_velocity.coeffs())