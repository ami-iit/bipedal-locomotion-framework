import pytest
import bipedal_locomotion_framework.bindings as blf
import numpy as np
import manifpy as manif
from datetime import timedelta

pytestmark = pytest.mark.mpc

def test_centroidal_mpc():

    centroidal_mpc = blf.reduced_model_controllers.CentroidalMPC()

    # Defining the param handler
    time_horizon = timedelta(seconds=1.2)
    dT = timedelta(milliseconds= 100)
    mpc_param_handler = blf.parameters_handler.StdParametersHandler()
    mpc_param_handler.set_parameter_datetime("sampling_time",dT)
    mpc_param_handler.set_parameter_datetime("time_horizon",time_horizon)
    mpc_param_handler.set_parameter_float("contact_force_symmetry_weight",1.0)
    mpc_param_handler.set_parameter_int("verbosity",0)
    mpc_param_handler.set_parameter_int("number_of_maximum_contacts",2)
    mpc_param_handler.set_parameter_int("number_of_slices",1)
    mpc_param_handler.set_parameter_float("static_friction_coefficient", 0.33)
    mpc_param_handler.set_parameter_string("linear_solver", "mumps")

    ## Defining MPC contact handler
    contact_0_handler = blf.parameters_handler.StdParametersHandler()
    contact_0_handler.set_parameter_int("number_of_corners",4)
    contact_0_handler.set_parameter_string("contact_name", "left_foot")
    contact_0_handler.set_parameter_vector_float("corner_0", [0.1,0.05,0.0])
    contact_0_handler.set_parameter_vector_float("corner_1", [0.1,-0.05,0.0])
    contact_0_handler.set_parameter_vector_float("corner_2", [-0.1,-0.05,0.0])
    contact_0_handler.set_parameter_vector_float("corner_3", [-0.1,0.05,0.0])
    contact_0_handler.set_parameter_vector_float("bounding_box_lower_limit", [0.0,0.0,0.0])
    contact_0_handler.set_parameter_vector_float("bounding_box_upper_limit", [0.0,0.0,0.0])

    contact_1_handler = blf.parameters_handler.StdParametersHandler()
    contact_1_handler.set_parameter_int("number_of_corners",4)
    contact_1_handler.set_parameter_string("contact_name", "right_foot")
    contact_1_handler.set_parameter_vector_float("corner_0", [0.1,0.05,0.0])
    contact_1_handler.set_parameter_vector_float("corner_1", [0.1,-0.05,0.0])
    contact_1_handler.set_parameter_vector_float("corner_2", [-0.1,-0.05,0.0])
    contact_1_handler.set_parameter_vector_float("corner_3", [-0.1,0.05,0.0])
    contact_1_handler.set_parameter_vector_float("bounding_box_lower_limit", [0.0,0.0,0.0])
    contact_1_handler.set_parameter_vector_float("bounding_box_upper_limit", [0.0,0.0,0.0])

    mpc_param_handler.set_group("CONTACT_0", contact_0_handler)
    mpc_param_handler.set_group("CONTACT_1", contact_1_handler)

    mpc_param_handler.set_parameter_vector_float("com_weight", [1000,100,1000])
    mpc_param_handler.set_parameter_float("contact_position_weight", 1e3)
    mpc_param_handler.set_parameter_vector_float("force_rate_of_change_weight", [10.0,10.0,10.0])
    mpc_param_handler.set_parameter_float("angular_momentum_weight", 1e5)

    #Defining com and angular momentum reference trajectory
    com_ref_traj = [np.zeros(3), np.zeros(3), np.zeros(3)]
    angular_ref_traj = [np.zeros(3), np.zeros(3), np.zeros(3)]
    centroidal_mpc.set_reference_trajectory(com_ref_traj,angular_ref_traj)

    # Defining the contact phase list
    contact_list_left_foot = blf.contacts.ContactList()
    contact = blf.contacts.PlannedContact()
    leftPosition = np.zeros(3)
    quaternion = [0.0, 0.0, 0.0, 1.0]
    contact.pose = manif.SE3(position=leftPosition, quaternion=quaternion)
    contact.activation_time = timedelta(seconds=0.0)
    contact.deactivation_time = timedelta(seconds=1.0)
    contact.name = "contactLeft1"
    contact_list_left_foot.add_contact(contact)

    # Right Foot
    contact_list_right_foot = blf.contacts.ContactList()
    contact = blf.contacts.PlannedContact()
    rightPosition = np.zeros(3)
    contact.pose = manif.SE3(position = rightPosition, quaternion = quaternion)
    contact.activation_time = timedelta(seconds=0.0)
    contact.deactivation_time = timedelta(seconds=3.0)
    contact.name = "contactRight1"
    contact_list_right_foot.add_contact(contact)

    contact_list_map ={}
    contact_list_map.update({"left_foot":contact_list_left_foot})
    contact_list_map.update({"right_foot":contact_list_right_foot})
    contact_phase_list = blf.contacts.ContactPhaseList()
    contact_phase_list.set_lists(contact_list_map)

    # Testing the mpc
    assert centroidal_mpc.initialize(mpc_param_handler)
    assert centroidal_mpc.set_state(np.zeros(3), np.zeros(3), np.zeros(3))
    assert centroidal_mpc.set_contact_phase_list(contact_phase_list)
    assert centroidal_mpc.advance()
    assert centroidal_mpc.is_output_valid()
