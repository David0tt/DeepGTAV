# DeepGTA
## Ground Truth Data Generation for UAV Object detection

The DeepGTAV framework can be used to produce ground truth training data with
high accuracy for different tasks, by leveraging the high visual fidelity
environment of GTAV.

In particular HD images, LiDAR, depth, stencil, labels, object segmentation and
bounding boxes can be exported for self-driving car and aerial scenarios.
Furthermore the system implements easy control through messages from a python
client, that make it suitable for custom scenarios and in principle even
reinforcement learning tasks. 


# Simple Use
In the following the steps that are necessary to just export data using DeepGTAV
are described. In principle DeepGTAV should work with any version of GTAV, if
the correct newest version of ScriptHookV is obtained. The last working version
for me employed GTAV from the Epic store in version 1.0.2060.1

## Installation of DeepGTAV
1. Obtain the newest version of ScriptHookV from
   (http://dev-c.com/GTAV/scripthookv) and copy the files from `bin/` to
   `DeepGTAV-PreSIL/Release/` overwriting the old files (this is not necessery
   when using GTAV in version 1.0.2060.1)
2. Copy the contents of `DeepGTAV-PreSIL/bin/Release/` to the GTAV install
   directory
3. Replace the save game data in `Documents/Rockstar Games/GTA V/Profiles/` with
   the contents of `DeepGTAV-PreSIL/bin/SaveGame`
4. Download paths.xml
   (https://drive.google.com/file/d/0B6pR5O2YrmHnNU9EMDBSSFpMV00/view?usp=sharing)
   and store it in the GTAV install directory

5. Create environment variables (GTAV_INSTALL_DIR, DEEPGTAV_DIR,
   DEEPGTAV_LOG_FILE, DEEPGTAV_EXPORT_DIR) (TODO those should not be necessary
   anymore!)
6. Download opencv and place opencv_world343.dll in the GTAV install directory
   (TODO is this necessary?)

## Game Settings
1. Set the game in windowed mode.
2. In the graphics setting MSAA has to be disabled, otherwise no objects are
   detected. The correct settings should be loaded by replacing the files in
   `Documents/Rockstar Games/GTA V/Profiles/`, but keep this in mind if you
   modify the game settings.


## Data Generation using VPilot
VPilot uses simple Python commands to interact with DeepGTAV. Examples of how
VPilot can be used are shown in `VPilot/drive_FOR_PRESENTATION.py`. The
currently best ready to use data generation script is
`VPilot/drive_LOW_USED_GENERATION.py`. With data generated with this script good
performances have been achieved in object detection training, as can be seen in
my Bachelors Thesis. 


## Modification of GTAV
In general DeepGTAV should work stable with modifications of GTAV. 

For my works I used the following modifications:
1. Simple Increase Traffic(and Pedestrian)
2. HeapLimitAdjuster
3. Balanced Classes


The modifications and install instructions can be found in the folder `Mods`

In the future additional modifications will be added, mainly for graphics
improvements. If you have use this repository in conjunction with graphics
enhancing mods I would love it if you send me a quick email telling me what mods
you used and whether they worked. 






# In Depth Use
## General Module Structure
This framework consists of different moving parts, that are organized in
different folders. Their interaction is described in the following:

`DeepGTAV-PreSIL` contains the code to build the scripts that start the DeepGTAV
server in GTAV. Communication with those scripts is done via a TCP-Port. In
`VPilot` the functionalities to communicate with DeepGTAV are implemented, to
control DeepGTAV from a Python script. 

`DeepGTAV-PreSIL` uses the functionalities implemented in
`GTAVisionExport-DepthExtractor` to extract depth and stencil information. To
communicate with GTAV DeepGTAV uses `SkriptHookV`. 

In the `Mods/Readme.md`, installation instructions for different mods that were
used to improve GTAV are shown. Mainly those are changes to spawn more objects
in the game world (e.g. cars and pedestrians) but graphics enhancements will be
added in the future.



In depth explanations can be found in bachelors thesis. In the future I will try
to do better documentation, but for now this has to suffice.





# Developement
If you want to compile the binaries by yourself or make your own changes you
have to do the following things:

DeepGTAV was built using MS Visual Studio 2017 and I recommend to also use it to
circumvent compatability issues.

1. clone this repository
2. TODO get all requirements for DeepGTAV-PreSIL and copy them to this
   directory, in particular those are: 
   1. GTAVisionExport-DepthExtractor (already contained in this repository)
   2. eigen-3.3.7 (https://eigen.tuxfamily.org/index.php?title=Main_Page get the
      source of version 3.3.7)
   3. opencv343 (https://opencv.org/releases/ get the source of version 3.4.3)
   4. boost_1_73_0 (https://www.boost.org/users/history/version_1_73_0.html get
      the source of verions 1.73.0)
3. Build GTAVisionExport-DepthExtractor as described. Copy the files from
   `GTAVisionExport-DepthExtractor\native\build\src\Release` to
   `DeepGTAV-PreSIL\bin\Release` (This is not necessary, if DepthExtractor has
   not been changes).
4. Set environment variable `GTAV_INSTALL_DIR` to point to the root directory of
   GTAV. This is done so that on building DeepGTAV-PreSIL the newly built files
   are automatically copied into the GTAV install directory.
5. Set environment variables `DEEPGTAV_EXPORT_DIR` and `DEEPGTAV_LOG_FILE`
   (those are legacy requirements and should be removed in future versions)






# Repository History
This repository is built upon different other works. If you use this repository
please cite those respective works and this repository.

The initial DeepGTAV framework was built in this repo:
https://github.com/aitorzip/DeepGTAV


In the work "Driving in the Matrix: Can Virtual Worlds Replace Human-Generated
Annotations for Real World Tasks?" (https://arxiv.org/abs/1610.01983) object
segmentation exportation from GTAV was provided:
https://github.com/umautobots/GTAVisionExport

In "Precise Synthetic Image and LiDAR (PreSIL) Dataset for Autonomous Vehicle
Perception" (https://arxiv.org/abs/1905.00160) those two functionalities were
combined: https://github.com/bradenhurl/DeepGTAV-PreSIL


Primarily this repository contains changes that were made to allow the capturing
of data from UAV perspective. Additionally some quality of life changes have
been made:

- The capturing speed has been increased by not writing to disk and only
  exporting the data through the TCP connection with VPilot.
- Many commands have been added to VPilot to allow a more intuitive control of
  DeepGTAV

A detailed description of the changes is presented in my bachelors thesis. 




# TODO 
## Bug Fixes / Urgent
- Refactor and improve code quality
- Full removal of disk writing functionality
- Implement testing
- Fix Bugs in the bounding box exportation, in particular for pedestrians
- Fix Bugs that result in crashes when having LiDAR active for heights with
  relatively flat camera angles.
- Provide more documentation


## Improvements
- Implement object spawning 
- increase amount of spawned objects 
- balance spawned object classes

- Improve capturing speed (atm at ~6s per captured frame) by optimizing the
  raycasting
- Improve the graphics quality 
- Improve the water quality 
- Adapt for higher resolution capturing 


## Things that I will not do in the near future that would be easy to implement/useful
- Allow capturing of different objects (Traffic signs, Animals, houses,...)




