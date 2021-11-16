# 🐍 Python bindings {#python-additional-info}

## Introduction
**bipedal-locomotion-framework** provides also python bindings.
You can easily use them by importing `bipedal-locomotion-framework` in a python interpreter as
follows

~~~~~~~~~~~~~{.py}
import bipedal_locomotion_framework.bindings as blf
~~~~~~~~~~~~~

Once you have import it you can check the content of the bindings using `help(blf)`.
We tried to implement a 1-to-1 mapping between the c++ structure and the python one, this means that
every python submodule is associated to the equivalent c++ submodule. For instance `blf.tsid`
represents the python bindings of `BipedalLocomotion::TSID` component.

The python bindings has been designed to be compliant to the
[PEP-8](https://www.python.org/dev/peps/pep-0008/). This means that:
1. **function** and the **variables** are lowercase words separated by underscore (`snake_case`)
2. The **class** names starts with a capital letter (`CammelCase`)
3. The **modules** are lowercase words separated by underscore (`snake_case`)

To give an example, the following code shows how to retrieve a parameter from a `toml`
configuration file in c++ and in python


~~~~~~~~~~~~~{.cpp}
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <memory>
#include <string>

int main()
{
    namespace blf = ::BipedalLocomotion;

    const std::string parameterFile = "./config.toml";
    std::vector<int> param;
    auto paramHandler = std::make_shared<blf::ParametersHandler::TomlImplementation>();
    paramHandler->setFromFile(parameterFile);
    paramHandler->getParameter("vector", param);
    return EXIT_SUCCESS;
}
~~~~~~~~~~~~~

~~~~~~~~~~~~~{.py}
import bipedal_locomotion_framework.bindings as blf


parameter_file = './config.toml'
param_handler = blf.parameters_handler.TomlParametersHandler()
assert param_handler.set_from_file(parameter_file)

vector = param_handler.get_vector_of_int('vector')
~~~~~~~~~~~~~

~~~~~~~~~~~~~{.ini}
# config.toml file

vector = [1, 2, 3, 4, 5]
~~~~~~~~~~~~~

## Some utilities
If you are a `python` user you can easily create a inverse kinematics (IK) or a task based inverse
dynamics (TSID) problem writing just a configuration file.
The configuration file should contains the name of the tasks and the parameters of the associated
tasks. This is possible thanks to two utilities function available only in python (`create_ik` and
`create_tsid`)

For instance you can create a IK problem with
~~~~~~~~~~~~~{.py}
import bipedal_locomotion_framework.bindings as blf
import bipedal_locomotion_framework.utils as utils

# create a param handler
kindyn_handler = blf.parameters_handler.TomlParametersHandler()
assert kindyn_handler.set_from_file("parameter_file.toml")
ik_handler = blf.parameters_handler.TomlParametersHandler()
assert ik_handler.set_from_file("parameter_file_ik.toml")

kindyn_descriptor = blf.floating_base_estimators.construct_kindyncomputations_descriptor(kindyn_handler)

solver, tasks, variables_handler = utils.create_ik(kindyn=kindyn_descriptor, param_handler=ik_handler)
~~~~~~~~~~~~~

An example of the configuration file is
~~~~~~~~~~~~~{.ini}
tasks = ["COM_TASK", "LF_TASK"]
[VARIABLES]
variables_name = ["robot_velocity"]
variables_size = [29]

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
~~~~~~~~~~~~~

You can find further details calling
~~~~~~~~~~~~~{.py}
import bipedal_locomotion_framework.utils as utils


help(utils.create_tsid)
help(utils.create_ik)
~~~~~~~~~~~~~
