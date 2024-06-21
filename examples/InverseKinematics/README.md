# 🤖 Inverse Kinematics

This tutorial aims to give the user a receipt that can be followed to design inverse kinematics exploiting `bipedal-locomotion-framework`.

The tutorial requires the following python package installed.

- `bipedal_locomotion_framework` to solve the IK problem;
- [`robot_descriptions`](https://github.com/robot-descriptions/robot_descriptions.py) is required to load the model of the `iiwa`;
- [`iDynTree`](https://github.com/robotology/idyntree) for visualization and to execute dynamics algorithms;
- [`manifpy`](https://github.com/artivis/manif) that is in charge handle rotation matrices and homogeneous transformations.

If you installed `bipedal-locomotion-framework` with the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild) you should have all the dependencies already satisfied except for `robot_descriptions` that you can easily install with `pip` as
```console
pip install robot_descriptions
```

In case you do want to run this tutorial without compiling `bipedal-locomotion-framework` we suggest installing the dependencies with `conda` as follows
```console
conda create -n blf-ik-tutorial
conda activate blf-ik-tutorial
conda install -c conda-forge bipedal-locomotion-framework meshcat-python notebook robot_descriptions
```

Then you can run the notebook by typing the following command in a terminal
```console
jupyter notebook inverse_kinematics.ipynb
```
