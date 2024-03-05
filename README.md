# Trajectory Planning and Tracking for Unmanned Aerial Vehicles

This repository includes source files together with the esential parameters of controllers and simulation models together with some experimental files for both Trajectory Tracking and Planning. All files were created as part of a dissertation on Trajectory Planning and Tracking for Unmanned Aerial Vehicles. The text of Doctoral Thesis is included in file `thesis.pdf`.


## UAV Trajectory Planning
Trajectory planning files are in the `traj_plan` folder with a separate README. Data that were obtained during the thesis work for UAV trajectory planning are published together with the Singularity/Apptainer container at 
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10669835.svg)](https://doi.org/10.5281/zenodo.10669835)



## UAV Trajectory Tracking
In folder `interpolating-control-matlab` are files with MATLAB implementation of Interpolating Control and files for experiments that were performed in MATLAB. Also it includes script for acquisition of IC for python trajectory tracking.

In `uav-traj-ic-py` there is python implementation of IC trajectory tracking together with files to perform simulation experiments together with laboratory experiments using Crazyflie 2.0 microUAV with CFlib (https://github.com/bitcraze/crazyflie-lib-python). Folder also includes Jupyter notebooks with evaluation of experiments.

For both parts of the trajectory tracking, files with the results and accompanying data that were generated during the evaluation are posted at 
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10683586.svg)](https://doi.org/10.5281/zenodo.10683586)

