# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

#!/usr/bin/env python3

import argparse
import os

import xml.etree.ElementTree as et
from xml.dom import minidom


class Step:
    """Step contain the definition of a step on a stair.

    Attributes:
        name (str): associated to the step.
        length (double): horizontal dimension, in meters, of the step (along the x axis).
        width (double): horizontal dimension, in meters, of the step (along the y axis).
        height (double): vertical dimension, in meters, of the step (along the z axis).
    """
    def __init__(self, step_index, size):
        """Constructor of the step method
        Args:
            step_index (int): index representing the step.
            size (list): 3d vector containing the length (x-dimension)
                         width (y-dimension) and height (z-dimension) of the step.
        """
        self.name = 's_' + str(step_index)

        if ((len(size) != 3) and all(element > 0 for element in size)):
            raise ValueError('The size has to be a 3d-vector containing only positive numbers.')

        self.length = size[0]
        self.width = size[1]
        self.height = size[2]
        self.mass = 1

    def append_urdf(self, model_urdf):
        link = et.SubElement(model_urdf, 'link', name = self.name)

        inertial = et.SubElement(link, "inertial")
        inertial_origin =  et.SubElement(inertial, 'origin',
                                         xyz = str(self.length / 2.0) + ' 0.0 ' + str(self.height / 2.0),
                                         rpy = '0.0 0.0 0.0')
        mass = et.SubElement(inertial, "mass", value = str(self.mass))
        inertia = et.SubElement(inertial, "inertia", ixx = str(self.mass/3.0 * (self.width ** 2 + self.height ** 2)),
                                ixy = '0.0', ixz='0.0',
                                iyy = str(self.mass/3.0 * (self.length ** 2 + self.height ** 2)),
                                iyz= '0.0',
                                izz=str(self.mass/3.0 * (self.length ** 2 + self.width ** 2)))


        visual =  et.SubElement(link, 'visual')
        visual_origin =  et.SubElement(visual, 'origin',
                                       xyz = str(self.length / 2.0) + ' 0.0 ' + str(self.height / 2.0),
                                       rpy = '0.0 0.0 0.0')

        visual_geometry = et.SubElement(visual, 'geometry')
        visual_box = et.SubElement(visual_geometry, 'box',
                                   size = str(self.length) + ' ' + str(self.width) + ' ' + str(self.height))

        collision = et.SubElement(link, 'collision')
        collision_origin =  et.SubElement(collision, 'origin',
                                       xyz = str(self.length / 2.0) + ' 0.0 ' + str(self.height / 2.0),
                                       rpy = '0.0 0.0 0.0')

        collision_geometry = et.SubElement(collision, 'geometry')
        collision_box = et.SubElement(collision_geometry, 'box',
                                   size = str(self.length) + ' ' + str(self.width) + ' ' + str(self.height))

def add_joint_to_urdf(parent_step, child_step, model_urdf):
    joint_name = 'joint_' + parent_step.name + '_' + child_step.name

    joint = et.SubElement(model_urdf, 'joint', name = joint_name, type = 'fixed')
    et.SubElement(joint, 'origin',
                  xyz = str(parent_step.length) + ' 0 ' + str(parent_step.height), rpy = '0 0 0')
    et.SubElement(joint, 'parent' ,link = parent_step.name)
    et.SubElement(joint, 'child' ,link = child_step.name)

    return model_urdf


def save_urdf(model_urdf, output_file):

    xmlstr = minidom.parseString(et.tostring(model_urdf, encoding='UTF-8')).toprettyxml(indent='    ')
    with open(output_file, 'w') as f:
        f.write(xmlstr)


def generate_urdf(model_name, step_size, steps_number):
    model = et.Element('robot', name = model_name)
    et.SubElement(model, 'link', name='world')

    # create the steps
    steps = [Step(i, step_size) for i in range(steps_number)]

    # Populate the urdf model with the steps
    for step in steps:
        step.append_urdf(model)

    for i in range(len(steps) - 1):
        add_joint_to_urdf(steps[i], steps[i+1], model)

    joint = et.SubElement(model, 'joint', name = 'joint_world_' + steps[0].name, type = 'fixed')
    et.SubElement(joint, 'origin',
                  xyz = '0 0 0', rpy = '0 0 0')
    et.SubElement(joint, 'parent', link = 'world')
    et.SubElement(joint, 'child', link = steps[0].name)

    return model

def main():

    parser = argparse.ArgumentParser(description='generate_stairs_urdf.py is a python script useful to generate a simple urdf model of stair.')
    parser.add_argument('--step_length', type = float, default = 0.2, required = False, help='Length of the single step in meters (x-axis). E.g. 0.2')
    parser.add_argument('--step_width', type = float, default = 0.3, required = False, help='Width of the single step in meters (y-axis). E.g. 0.3')
    parser.add_argument('--step_height', type = float, default = 0.025, required = False, help='Height of the single step in meters (z-axis). E.g. 0.025')
    parser.add_argument('--steps_number', type = int, default = 5, required = False, help='Number of steps')
    parser.add_argument('--install_folder', type = str, default = './', required = False, help='File output')
    args = parser.parse_args()

    model_name = 'stairs'
    install_folder = args.install_folder


    model = generate_urdf(model_name,
                          [args.step_length, args.step_width, args.step_height],
                          args.steps_number)

    if not os.path.exists(os.path.normpath(args.install_folder)):
        os.makedirs(os.path.normpath(args.install_folder))

    save_urdf(model,
              os.path.normpath(args.install_folder + '/' + model_name + '.urdf'))

if __name__ == '__main__':
    main()
