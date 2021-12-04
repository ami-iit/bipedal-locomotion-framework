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

### Create a custom TSID or IK task directly in Python
If you are a `python` user and you want create a inverse kinematics (IK) or a task based inverse
dynamics (TSID) tasks, you do not need to write a `C++` code. Indeed it is possible to write a class that inherits from `TSIDLinearTask` or `IKLinearTask`  and pass it to `tsid` or `ik` solvers. The following snippet can be used as starting point to build a custom TSID task, a similar approach can be used to define a custom IK task

~~~~~~~~~~~~~{.py}
import bipedal_locomotion_framework.bindings as blf


class CustomTSIDTask(blf.tsid.TSIDLinearTask):
    """ CustomTSIDTask represents a custom task for the tsid solver.

    The class must define the following functions:
    __init__(self): the constructor
    size(self) -> Get the size of the task. (I.e the number of rows of the vector b)
    is_valid(self) -> Determines the validity of the objects retrieved with getA() and getB()
    type(self) -> Get the type of the task. Namely equality or inequality

    Furthemore you have to populate the following attrubutes defined in TSIDLinearTask:
    self._description:str -> contains the description of the task
    self._A:numpy.ndarray[float64[m, n]]) -> matrix that describes the task A * x = b
    self._b: numpy.ndarray[float64[m, 1]]) -> vector that describes the task A * x = b

    Depending on the content of the task you may need to reimplement the following methods
    initialize(self, param_handler: blf.parameters_handler.IParameterHandler) -> usefull function to set parameters to the task
    set_variables_handler(self, variables_handler: blf.system.VariablesHandler) -> usefull function to set the variables
    update(self) -> update the content of the task
    """

    def __init__(self):

        blf.tsid.TSIDLinearTask.__init__(self)  # Without this, a TypeError is raised.

        # set the description of the task
        self._description = "Custom task"

    def size(self):
        # this is the number of rows of the vector b (in this example is equal to 1)
        return 1

    def is_valid(self):
        return True

    def type(self):
        # the types are defined in blf.tsid.TSIDLinearTask.Type
        return blf.tsid.TSIDLinearTask.Type.equality
~~~~~~~~~~~~~

Then you can use your custom task as follows

~~~~~~~~~~~~~{.py}
import bipedal_locomotion_framework.bindings as blf

# Instantiate the task
custom_task = CustomTSIDTask()

# Initialize your task
custom_task.initialize(param_handler)

# Create the solver
solver = blf.tsid.QPTSID()
priority = 0
solver.add_task(custom_task, 'custom_task', priority)
~~~~~~~~~~~~~

### Automatically build a TSID or IK from configuration file

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

[IK]
robot_velocity_variable_name = "robot_velocity"

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
