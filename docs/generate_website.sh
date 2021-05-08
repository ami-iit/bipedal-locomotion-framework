#!/bin/bash

mcss_script=$1

versions=(v0.1.1)
main_pages=(README.md)
src_folder=(bipedal-locomotion-framework)
number_of_versions=0

for version in ${versions}
do
    mkdir -p site/${version}
done


for i in "${!versions[@]}"
do
    cd bipedal-locomotion-framework
    git checkout ${versions[i]}
    cd ..
    python3 generate_documentation_files.py --tag ${versions[i]} --src_folder=bipedal-locomotion-framework/src --main_page=bipedal-locomotion-framework/${main_pages[i]}

    python3 ${mcss_script} conf-${versions[i]}.py

done

python3 generate_documentation_files.py --src_folder=../src --main_page=./pages/main-page.dox
python3 ${mcss_script} conf-master.py
