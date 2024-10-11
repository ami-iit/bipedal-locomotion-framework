"""
File: utils.py
Authors: Giulio Romualdi
Copyright (C): 2021 Fondazione Istituto Italiano di Tecnologia
Licensed under either the GNU Lesser General Public License v3.0 :
https://www.gnu.org/licenses/lgpl-3.0.html
or the GNU Lesser General Public License v2.1 :
https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
at your option.
"""
import bipedal_locomotion_framework as blf
import idyntree.swig as idyn

def create_tsid(kindyn: idyn.KinDynComputations,
                param_handler: blf.parameters_handler.IParametersHandler):
    """create_tsid is a function that will help you to create a TSID problem. It creates a TSID object with all the
    associated tasks. The function returns also the variable handler associated

    Parameters
    ----------
    kindyn : idyn.KinDynComputations a `kindyn` computation object. The object will be shared among all the tasks
    param_handler : bipedal_locomotion_framework.parameters_handler.IParametersHandler
       The handler containing all the required parameter.
       The following configuration file can be used as example

       tasks = ["COM_TASK", "LF_TASK"]

       [TSID]
       robot_acceleration_variable_name = "robot_acceleration"
       joint_torques_variable_name = "joint_torques"
       contact_wrench_variables_name = ["lf_wrench", "rf_wrench"]

       [VARIABLES]
       variables_name = ["robot_acceleration", "joint_torques", "lf_wrench", "rf_wrench"]
       variables_size = [29, 23, 6, 6]

       # the following parameter is optional and it can be used to give a name to each element of a specific variable
       lf_wrench_elements_name = ["fx", "fy", "fz", "tx", "ty", "tz"]

       [COM_TASK]
       name = "com"
       type = "CoMTask"
       priority = 1
       weight = [10.0, 10.0, 10.0]

       # The following parameters are required by the specific task
       robot_acceleration_variable_name = "robot_acceleration"
       kp_linear = 10.0
       kd_linear = 2.0

       [LF_TASK]
       name = "left_foot"
       type = "SE3Task"
       priority = 0

       # The following parameters are required by the specific task
       robot_acceleration_variable_name = "robot_acceleration"
       frame_name = "left_sole_link"
       kp_linear = 10.0
       kd_linear = 2.0
       kp_angular = 10.0
       kd_angular = 2.0

    Returns
    -------
    tuple
        a tuple containing the solver, a dictionary of the tasks and the variables handler
    """
    import warnings
    warnings.warn("The function is deprecated. It will be removed in the next release. "
                  "Please use bipedalLocomotion.tsid.QPTSID.build() instead.",
                  DeprecationWarning)
    solver = blf.tsid.QPTSID()
    tasks = dict()
    variables_handler = blf.system.VariablesHandler()

    # initialize the variable handler
    assert variables_handler.initialize(param_handler.get_group("VARIABLES"))

    # initialize the solver
    assert solver.initialize(param_handler.get_group("TSID"))

    # retrieve all the tasks
    task_groups_name = param_handler.get_parameter_vector_string("tasks")
    for task_group_name in task_groups_name:
        task_group = param_handler.get_group(task_group_name)
        name = task_group.get_parameter_string("name")
        if name in tasks.keys():
            raise AssertionError("The task named ", name, " is not unique.")

        # instantiate and initialize the the task
        tasks[name] = getattr(blf.tsid, task_group.get_parameter_string("type"))()

        # a task may not have set_kin_dyn method
        if hasattr(tasks[name], "set_kin_dyn") and callable(getattr(tasks[name], "set_kin_dyn")):
            assert tasks[name].set_kin_dyn(kindyn)

        assert tasks[name].initialize(task_group)

        # add the task to the optimization problem
        priority = task_group.get_parameter_int("priority")
        assert priority == 1 or priority == 0
        if priority == 1:
            assert solver.add_task(tasks[name], name, priority, task_group.get_parameter_vector_float("weight"))
        else:
            assert solver.add_task(tasks[name], name, priority)

    # All the tasks have been added. It's time to finalize the problem
    assert solver.finalize(variables_handler)

    return solver, tasks, variables_handler


def create_ik(kindyn: idyn.KinDynComputations,
              param_handler: blf.parameters_handler.IParametersHandler):
    """create_ik is a function that will help you to create a IK problem. It creates a IK object with all the
    associated tasks. The function returns also the variable handler associated

    Parameters
    ----------
    kindyn : bipedal_locomotion_framework.floating_base_estimators.KinDynComputationsDescriptor
       a `kindyn` computation object. The object will be shared among all the tasks
    param_handler : bipedal_locomotion_framework.parameters_handler.IParametersHandler
       The handler containing all the required parameter.
       The following configuration file can be used as example

       tasks = ["COM_TASK", "LF_TASK"]

       [VARIABLES]
       variables_name = ["robot_velocity"]
       variables_size = [29]

       [IK]
       robot_velocity_variable_name = "robot_velocity"

       [COM_TASK]
       name = "com"
       type = "CoMTask"
       priority = 1
       weight = [10.0, 10.0, 10.0]

       # The following parameters are required by the specific task
       robot_velocity_variable_name = "robot_velocity"
       kp_linear = 10.0

       [LF_TASK]
       name = "left_foot"
       type = "SE3Task"
       priority = 0

       # The following parameters are required by the specific task
       robot_velocity_variable_name = "robot_velocity"
       frame_name = "left_sole_link"
       kp_linear = 10.0
       kp_angular = 10.0

    Returns
    -------
    tuple
        a tuple containing the solver, a dictionary of the tasks and the variables handler
    """

    import warnings
    warnings.warn("The function is deprecated. It will be removed in the next release. "
                  "Please use bipedalLocomotion.ik.QPInverseKinematics.build() instead.",
                  DeprecationWarning)

    solver = blf.ik.QPInverseKinematics()
    tasks = dict()
    variables_handler = blf.system.VariablesHandler()

    # initialize the variable handler
    assert variables_handler.initialize(param_handler.get_group("VARIABLES"))

    # initialize the solver
    assert solver.initialize(param_handler.get_group("IK"))

    # retrieve all the tasks
    task_groups_name = param_handler.get_parameter_vector_string("tasks")
    for task_group_name in task_groups_name:
        task_group = param_handler.get_group(task_group_name)
        name = task_group.get_parameter_string("name")
        if name in tasks.keys():
            raise AssertionError("The task named ", name, " is not unique.")

        # instantiate and initialize the the task
        tasks[name] = getattr(blf.ik, task_group.get_parameter_string("type"))()

        # a task may not have set_kin_dyn method
        if hasattr(tasks[name], "set_kin_dyn") and callable(getattr(tasks[name], "set_kin_dyn")):
            assert tasks[name].set_kin_dyn(kindyn)

        assert tasks[name].initialize(task_group)

        # add the task to the optimization problem
        priority = task_group.get_parameter_int("priority")
        assert priority == 1 or priority == 0
        if priority == 1:
            assert solver.add_task(tasks[name], name, priority, task_group.get_parameter_vector_float("weight"))
        else:
            assert solver.add_task(tasks[name], name, priority)

    # All the tasks have been added. It's time to finalize the problem
    assert solver.finalize(variables_handler)

    return solver, tasks, variables_handler
