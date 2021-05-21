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
   when using GTAV in version 1.0.2245.0 or older)
2. Copy the contents of `DeepGTAV-PreSIL/bin/Release/` to the GTAV install
   directory
3. Replace the save game data in `Documents/Rockstar Games/GTA V/Profiles/` with
   the contents of `DeepGTAV-PreSIL/bin/SaveGame`

4. Done!

5. Only if you are using the reinforcement learning rewarder: Download paths.xml
   (https://drive.google.com/file/d/0B6pR5O2YrmHnNU9EMDBSSFpMV00/view?usp=sharing)
   and store it in the GTAV install directory

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
   5. zeroMQ TODO 

   Build GTAVisionExport-DepthExtractor and zeroMQ. Specific build instructions can be found in the individual projects.
   In principle it suffices to build using cmake and then building them in Visual Studio. A detailed description is as follows:

   4. Run cmake (cmake-gui) from your Windows start menu.
   5. Hit 'Browse Source' and select your GTAVisionExport/native folder.
   6. Hit 'Browse Build', create GTAVisionExport/native/build folder and select it.
   7. Hit 'configure' (first time around it will fail but dont worry).
   8. Choose project generator 'Visual Studio 15 2017 Win64' and keep the option 'use default native compilers'
   9. After the fail dialog, modify the EIGEN3_INCLUDE_DIR to point to your Eigen3 folder.
   10. Run 'configure' followed by 'generate'.
   11. cmake should now have generated the Visual Studio solution into GTAVisionExport/build.
   12. Open 'GTANativePlugin.sln' in Visual Studio.
   13. Select 'release' from the 'Solution Configurations' drop down.
   14. Edit GTAVisionNative project properties/configuration properties/c/c++/additional include dirs in VS to add the GTAVisionExport/native/src folder (this allows VS to find MinHook.h)
   15. Edit GTAVisionNative project properties/configuration properties/linker/input/additional dependencies to add : 
   `"..\..\deps\libMinHook.x64.lib"` 
   16. Press F6 to build the solution. it should now succeed and the products should be in 'GTAVisionExport\native\build\src\Release'
   17. Copy GTAVisionNative.asi & GTAVisionNative.lib to your GTAV exe folder.


   It is important to mimic this exact file structure, otherwise the realtive
   "Additional Include Directories" do not fit correctly. If this is the case
   and for some reason you want to include those directories from different
   locations, you have to change the "Additional Include Directories" in the
   following way: 
   
   Right click DeepGTAV in the SolutionExplorer in Visual Studio -> Properties
   -> C/C++ -> Additional Include Directories

   Here the include directories have to match.

   Furthermore in 

   Properties -> Linker -> Input -> Additional Dependencies 

   the correct locations of the libraries have to be set 

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
been made, a comprehensive list of the improvements is given in the following:

- The TCP server functionality has been replaced with ZeroMQ for more stability
  and speed.
- All data export functionality by writing to disk has been replaced by sending
  this data over the TCP socket to Python. This increased capturing speeds and
  usability.
- Multiple commands have been added to VPilot to allow fine grained control of
  what data is being captured and of the game world (e.g. spawning pedestrians,
  moving to ingame locations, changing the camera perspective).
- The code quality has been improved, some bugs were fixed and some functions
  have been refactored for more efficiency. 
- The total capturing speed has been improved by a factor of 3-4, resulting in
  almost no overhead to running the game natively.
- Different skripts are provided to capture data in vastly different scenarios. 
- Compatability with the newest game version has been tested.
- The installation of DeepGTAV to the game files has been simplified
<!-- A detailed description of the changes is presented in my bachelors thesis.  -->




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



## Things to note
- The LiDAR simulation currently uses the ingame raycasting and the depth map
  from the graphics. For the purposes of the ingame ray casting low poly objects
  are used (e.g. tree tops are only balls). The cast rays are then updated with
  the correct depth from the graphics depth map, if it is available. Of course
  this depth map is only available for the objects that are in the current
  frame. Therefore the LiDAR is not exact for objects outside of the current
  frame image.



