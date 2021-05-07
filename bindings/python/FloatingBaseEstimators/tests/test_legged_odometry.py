import pytest
pytestmark = pytest.mark.floating_base_estimators

import bipedal_locomotion_framework.bindings as blf
import numpy as np

def test_legged_odometry():
    # This function just performs an interface test for
    # the generated python bindings,

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

    # Check the number of degrees of freedom of the model
    assert kindyn_desc.get_nr_of_dofs() == len(joints_list)

    # Set the joint positions to random values
    joint_values = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.get_nr_of_dofs())]
    assert kindyn_desc.set_joint_pos(joint_values)

    # Check that the joint positions are all set to the desired values
    updated_joint_pos = np.zeros(kindyn_desc.get_nr_of_dofs())
    assert kindyn_desc.get_joint_pos(updated_joint_pos)
    assert updated_joint_pos == pytest.approx(joint_values)

    # Get the robot state
    # TODO: world_T_base.flags['F_CONTIGUOUS'] is required to be True
    world_T_base = np.asfortranarray(np.zeros((4,4)))
    s = np.zeros(kindyn_desc.get_nr_of_dofs())
    base_velocity = np.zeros(6)
    s_dot = np.zeros(kindyn_desc.get_nr_of_dofs())
    world_gravity = np.zeros(3)
    assert kindyn_desc.get_robot_state(world_T_base,s,base_velocity,s_dot,world_gravity)

    # Set the robot state
    # TODO: world_T_base.flags['F_CONTIGUOUS'] is required to be True
    updated_world_T_base = np.asfortranarray([[1., 0., 0., 0.],[0., 0., -1., 0.],[0., 1., 0., 0.],[0., 0., 0., 1.]])
    updated_s = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.get_nr_of_dofs())]
    updated_base_velocity = [np.random.uniform(-0.5,0.5) for _ in range(6)]
    updated_s_dot = [np.random.uniform(-0.5,0.5) for _ in range(kindyn_desc.get_nr_of_dofs())]
    updated_world_gravity = [np.random.uniform(-0.5,0.5) for _ in range(3)]
    assert kindyn_desc.set_robot_state(updated_world_T_base,updated_s,updated_base_velocity,
                                       updated_s_dot,updated_world_gravity)

    # Check that the robot state is set to the desired values
    assert kindyn_desc.get_robot_state(world_T_base,s,base_velocity,s_dot,world_gravity)
    assert world_T_base == pytest.approx(updated_world_T_base)
    assert s == pytest.approx(updated_s)
    assert base_velocity == pytest.approx(updated_base_velocity)
    assert s_dot == pytest.approx(updated_s_dot)
    assert world_gravity == pytest.approx(updated_world_gravity)

    dt = 0.01
    # create configuration parameters handler for legged odometry
    lo_params_handler = blf.parameters_handler.StdParametersHandler()
    lo_params_handler.set_parameter_float("sampling_period_in_s", dt)

    model_info_group = blf.parameters_handler.StdParametersHandler()
    model_info_group.set_parameter_string("base_link", "root_link")
    # use same frame for the IMU - requirement due to the software design
    model_info_group.set_parameter_string("base_link_imu", "root_link")
    # unused parameters but required for configuration (will be deprecated soon)
    model_info_group.set_parameter_string("left_foot_contact_frame", "l_foot")
    model_info_group.set_parameter_string("right_foot_contact_frame", "r_foot")
    assert(lo_params_handler.set_group("ModelInfo", model_info_group))

    lo_group = blf.parameters_handler.StdParametersHandler()
    lo_group.set_parameter_string("initial_fixed_frame", "l_sole")
    lo_group.set_parameter_string("initial_ref_frame_for_world", "l_sole")
    lo_group.set_parameter_vector_float("initial_world_orientation_in_ref_frame",  [1.0, 0.0, 0.0, 0.0])
    lo_group.set_parameter_vector_float("initial_world_position_in_ref_frame",  [0.0, 0.0, 0.0])
    lo_group.set_parameter_string("switching_pattern",  "useExternal")
    assert lo_params_handler.set_group("LeggedOdom", lo_group)

    # instantiate legged odometry
    legged_odom = blf.floating_base_estimators.LeggedOdometry()
    empty_handler = blf.parameters_handler.StdParametersHandler()
    # assert passing an empty parameter handler to false
    assert legged_odom.initialize(empty_handler, kindyn_desc.kindyn) == False

    # assert passing an properly configured parameter handler to true
    assert legged_odom.initialize(lo_params_handler, kindyn_desc.kindyn) == True

    # shape of the robot for the above specified joints list
    encoders = np.array([-0.0001, 0.0000, 0.0000,
                          0.1570, 0.0003, -0.0000,
                         -0.0609, 0.4350, 0.1833,
                          0.5375,
                         -0.0609, 0.4349, 0.1834,
                          0.5375,
                          0.0895, 0.0090, -0.0027,
                         -0.5694, -0.3771, -0.0211,
                          0.0896, 0.0090, -0.0027,
                         -0.5695, -0.3771, -0.0211])
    encoder_speeds = np.zeros_like(encoders)

    for time in np.arange(start=0, step=dt, stop=10*dt):
        fixed_frame_idx = legged_odom.get_fixed_frame_index()

        # here we only fill measurement buffers
        assert(legged_odom.set_kinematics(encoders, encoder_speeds))
        contact_name = "l_sole"
        contact_status = True
        switch_time = time
        time_now = time
        assert legged_odom.set_contact_status(contact_name, contact_status, switch_time, time_now)

        # since we set "switching_pattern" parameter to "useExternal"
        # we can change the fixed frame manually from an external script
        # such as this script, where for example, we change the same
        # frame as we had earlier initialized the estimator
        # if the "switching_pattern" is not set to `useExternal`
        # this function call will return false
        legged_odom.change_fixed_frame(fixed_frame_idx)

        # computations given the set measurements
        assert legged_odom.advance()

        # sample asserts for show-casing outputs
        out = legged_odom.get_output()
        assert len(out.base_twist) == 6
        assert len(out.state.imu_linear_velocity) == 3
