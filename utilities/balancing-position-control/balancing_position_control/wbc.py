# This software may be modified and distributed under the terms of the BSD-3-Clause license.
# Authors: Giulio Romualdi

import bipedal_locomotion_framework as blf
import idyntree.swig as idyn


class WBC:
    def __init__(
        self,
        param_handler: blf.parameters_handler.IParametersHandler,
        kindyn: idyn.KinDynComputations,
    ):
        self.solver, self.tasks, self.variables_handler = blf.utils.create_ik(
            kindyn=kindyn, param_handler=param_handler
        )
