#!/usr/bin/env python3

# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import yaml
import os

if __name__ == "__main__":

    dict_file = {':book: documentation' : 'docs/**/*',
                 ':computer: utilities' : 'utilities/**/*',
                 ':snake: bindings' : 'bindings/**/*',
                 ':building_construction: GitHub action' : '.github/**/*'}

    src_dir = '../src'
    for file in os.listdir(src_dir):
        d = os.path.join(src_dir, file)
        if os.path.isdir(d):
            dict_file[":hammer: component: " + os.path.basename(d)] = "src/" + os.path.basename(d) + "/**/*"


    with open(r'../.github/labeler.yml', 'w') as file:
        yaml.dump(dict_file, file, default_flow_style=False)

