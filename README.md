# Trajectory Planning and Tracking for Unmanned Aerial Vehicles

This repository includes source files together with the esential parameters of controllers and simulation models together with some experimental files for both Trajectory Tracking and Planning. All files were created as part of a dissertation on Trajectory Planning and Tracking for Unmanned Aerial Vehicles. The text of Doctoral Thesis is included in file `thesis.pdf`.

## Citation

If you use any part of this repository (code, data, or results), please cite the dissertation:

> Z. Bouček, *Trajectory Planning and Tracking for Unmanned Aerial Vehicles* [online]. Doctoral theses, Dissertations, University of West Bohemia, Faculty of Applied Sciences, Pilsen, 2024. Available: https://theses.cz/id/8dxjh3/

```bibtex
@PhdThesis{BOUCEK2024thesis,
  author   = {BOUČEK, Zdeněk},
  title    = {Trajectory Planning and Tracking for Unmanned Aerial Vehicles [online]},
  year     = {2024},
  type     = {Doctoral theses, Dissertations},
  school   = {University of West Bohemia, Faculty of Applied Sciences, Pilsen},
  note     = {SUPERVISOR: Doc. Ing. Ondřej Straka, Ph.D., CONSULTANT: Ing. Miroslav Flídr, Ph.D.},
  url      = {https://theses.cz/id/8dxjh3/}
}
```

## UAV Trajectory Planning
Trajectory planning files are in the `traj_plan` folder with a separate README. Data that were obtained during the thesis work for UAV trajectory planning are published together with the Singularity/Apptainer container at 
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10669835.svg)](https://doi.org/10.5281/zenodo.10669835)



## UAV Trajectory Tracking
In folder `interpolating-control-matlab` are files with MATLAB implementation of Interpolating Control and files for experiments that were performed in MATLAB. Also it includes script for acquisition of IC for python trajectory tracking.

In `uav-traj-ic-py` there is python implementation of IC trajectory tracking together with files to perform simulation experiments together with laboratory experiments using Crazyflie 2.0 microUAV with CFlib (https://github.com/bitcraze/crazyflie-lib-python). Folder also includes Jupyter notebooks with evaluation of experiments.

For both parts of the trajectory tracking, files with the results and accompanying data that were generated during the evaluation are posted at 
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10683586.svg)](https://doi.org/10.5281/zenodo.10683586)

### Video with UAV Trajectory Tracking
[![Watch the video](https://img.youtube.com/vi/guzADKNLg90/maxresdefault.jpg)](https://youtu.be/guzADKNLg90?si=b59eJHqcwh9xkjY6)
