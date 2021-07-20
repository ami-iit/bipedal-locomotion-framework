# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

#!/usr/bin/env python3

import argparse
import os

def main():

    parser = argparse.ArgumentParser(description='generate_doxyfile.py is a python script useful to generate the Doxyfile from Doxyfile.in.')
    parser.add_argument('--tag', type = str, default = 'master', required = False, help='Version of project.')
    parser.add_argument('--input_files_path', type = str, default = './', required = False, help='Path of the input files.')
    parser.add_argument('--output_files_path', type = str, default = './', required = False, help='Path of the output files.')
    parser.add_argument('--src_folder', type = str, required = True, help='Path of the src folder.')
    parser.add_argument('--main_page', type = str, required = True, help='Path to the the main page.')

    args = parser.parse_args()
    src_path = os.path.normpath(args.src_folder)
    input_file_path = os.path.normpath(args.input_files_path)
    output_file_path = os.path.normpath(args.output_files_path)
    main_page = os.path.normpath(args.main_page)
    _, main_page_extension = os.path.splitext(main_page)

    main_page_md =''
    if main_page_extension == '.md':
        main_page_md = main_page

    folders = [os.path.normpath(folder[0])  for folder in os.walk(src_path)
               if os.path.basename(os.path.normpath(folder[0])) == 'include']

    output_doxyfile_filename = 'Doxyfile-' + args.tag
    output_doxyfile_mcss_filename = 'Doxyfile-mcss-' + args.tag
    output_conf_filename = 'conf-' + args.tag + '.py'

    BLF_VERSION_PATH = '.'
    if args.tag != 'master':
        BLF_VERSION_PATH = args.tag

    BLF_DOCUMENTATION_ROOT = '.'
    if args.tag != 'master':
        BLF_DOCUMENTATION_ROOT = '..'

    try:
        input_file = open(os.path.join(input_file_path, 'Doxyfile.in'), 'rt')
        output_file = open(os.path.join(input_file_path, output_doxyfile_filename), 'wt')

        for line in input_file:
            output_file.write(line.replace('@BLF_INCLUDE_FOLDERS@',
                                               "\""+"\" \"".join(folders) + "\"")
                                       .replace('@BLF_MAIN_PAGE@', main_page))

    # Do something with the file
    except FileNotFoundError:
        print("Invalid doxyfile input file.")
    finally:
        input_file.close()
        output_file.close()



    try:
        input_file = open(os.path.join(input_file_path, 'Doxyfile-mcss.in'), 'rt')
        output_file = open(os.path.join(input_file_path, output_doxyfile_mcss_filename), 'wt')


        for line in input_file:
            output_file.write(line.replace('@BLF_TAG@', args.tag).replace('@BLF_VERSION_PATH@', BLF_VERSION_PATH).replace('@BLF_MAIN_PAGE@', main_page_md))

         # Do something with the file
    except FileNotFoundError:
        print("Invalid doxyfile-mcss input file.")
    finally:
        input_file.close()
        output_file.close()


    try:
        input_file = open(os.path.join(input_file_path, 'conf.py.in'), 'rt')
        output_file = open(os.path.join(input_file_path, output_conf_filename), 'wt')


        for line in input_file:
            output_file.write(line.replace('@BLF_TAG@', args.tag).replace('@BLF_DOCUMENTATION_ROOT@', BLF_DOCUMENTATION_ROOT))

         # Do something with the file
    except FileNotFoundError:
        print("Invalid conf.py input file.")
    finally:
        input_file.close()
        output_file.close()

if __name__ == '__main__':
    main()
