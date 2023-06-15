#!/bin/python3

# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import toml
import subprocess
import os
import argparse

from generate_documentation_files import generate_documentation_files

def generate_documentation(tag, options):
    print("###### Generating documentation for", tag, " ######")

    root_folder = "../"
    if tag != 'master':
        os.chdir('bipedal-locomotion-framework')
        subprocess.Popen(["git", "checkout", tag], stdout=subprocess.PIPE)
        os.chdir('..')
        root_folder = "bipedal-locomotion-framework/"

    additional_pages = []
    if 'additional_pages' in options.keys():
        additional_pages = [root_folder + page for page in options['additional_pages']]

    generate_documentation_files(tag=tag,
                                 src_folder=root_folder + options['src_folder'],
                                 main_page=root_folder + options['main_page'],
                                 additional_pages=additional_pages,
                                 input_files_path="./",
                                 output_files_path="./")

    try:
        subprocess.check_call(["python3", args.mcss_path, "conf-" + tag + ".py"])
    except subprocess.CalledProcessError as error:
        raise ValueError(error)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='generate_website.py is a python script useful to generate the '
                                                 'documentation of the bipedal-locomotion-framework repo.')
    parser.add_argument('--mcss_path',
                        type=str,
                        required=True,
                        help='Path to \'m.css/documentation/doxygen.py\' file')
    args = parser.parse_args()

    parameters = toml.load("config.toml")

    # create the site directory structure
    os.makedirs('./site', exist_ok=True)
    for key, value in parameters.items():
        if key != 'master':
            directory = './site/' + key
            os.makedirs(directory, exist_ok=True)

    # build the documentation
    for key, value in parameters.items():
        generate_documentation(key, value)

