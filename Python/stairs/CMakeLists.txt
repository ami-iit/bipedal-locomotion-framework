# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if(FRAMEWORK_GENERATE_urdf_models)

  generate_and_add_urdf_model(
    SCRIPT_NAME generate_stairs_urdf
    INSTALLATION_FOLDER stairs
    EXTRA_FILES model.config
    )

endif()
