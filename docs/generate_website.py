#!/bin/python3

# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import toml
import subprocess
import os
import argparse

from generate_documentation_files import generate_documentation_files

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='generate_website.py is a python script useful to generate the '
                                                 'documentation of the bipedal-locomotion-framework repo.')
    parser.add_argument('--mcss_path',
                        type=str,
                        required=True,
                        help='Path to \'m.css/documentation/doxygen.py\' file')
    args = parser.parse_args()

    parameters = toml.load("config.toml")
    for key, value in parameters.items():

        directory = './site/' + key
        if not os.path.exists(directory) and key != 'master':
            os.makedirs(directory)

        os.chdir('bipedal-locomotion-framework')
        subprocess.Popen(["git", "checkout", key], stdout=subprocess.PIPE)
        os.chdir('..')

        root_folder = "../"
        if key != 'master':
            root_folder = "bipedal-locomotion-framework/"

        additional_pages = []
        if 'additional_pages' in value.keys():
            additional_pages = [root_folder + page for page in value['additional_pages']]

        generate_documentation_files(tag=key,
                                     src_folder=root_folder + value['src_folder'],
                                     main_page=root_folder + value['main_page'],
                                     additional_pages=additional_pages,
                                     input_files_path="./",
                                     output_files_path="./")

        subprocess.Popen(["python3", args.mcss_path, "conf-" + key + ".py"],
                         stdout=subprocess.PIPE)
