# 🤖 MANN

This tutorial aims to give the user a receipt that can be followed to use the `MANNAutoregressive` and `MANNTrajectoryGenerator` classes to generate a trajectory for the `ergoCub` robot.

The tutorial requires the following Python package installed:
- `bipedal_locomotion_framework` to use MANN network;
- [`iDynTree`](https://github.com/robotology/idyntree) for visualization and to execute dynamics algorithms;
- [`manifpy`](https://github.com/artivis/manif) that is in charge handle rotation matrices and homogeneous transformations;
- [`ergocub-software`](https://github.com/icub-tech-iit/ergocub-software) to load the model of the `ergoCub` robot;
- [`resolve_robotics_uri_py`](https://github.com/ami-iit/resolve-robotics-uri-py) to correctly load the `ergoCub` model.

If you installed `bipedal-locomotion-framework` with the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild) you should have all the dependencies already satisfied.

In case you do want to run this tutorial without compiling `bipedal-locomotion-framework` we suggest installing the dependencies with `conda` as follows
```console
conda create -n blf-mann-tutorial
conda activate blf-mann-tutorial
conda install -c conda-forge -c robotology bipedal-locomotion-framework meshcat-python notebook resolve_robotics_uri_py ergocub-software
```

Then you can run the notebooks by typing the following command in a terminal:
1. `MANNTrajectoryGenerator`
    ```console
    jupyter notebook MANNTrajectoryGenerator.ipynb
    ```

    https://github.com/ami-iit/bipedal-locomotion-framework/assets/16744101/b3b921b9-efc8-4090-9f75-178cd0f9d119

2. `MANNAutoregressive`
    ```console
    jupyter notebook MANNAutoregressive.ipynb
    ```

    https://github.com/ami-iit/bipedal-locomotion-framework/assets/16744101/7ba65d66-68f9-4bd9-aede-a8edd00d82d5
