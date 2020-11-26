# Instructions on How to build the Working DeepGTAV-PreSIL

In principle follow the install instructions at https://github.com/bradenhurl/DeepGTAV-PreSIL and at https://github.com/bradenhurl/GTAVisionExport-DepthExtractor (in native)

But slightly different:

Build VisionNative-DepthExtractor by instructions. The dependencies have to be supplied

Build DeepGTAV-PreSIL. Many dependencies have to be supplied. In particular eigen, boost, opencv. Care for the versions.


Create an environment variable (GTAV_INSTALL_DIR) pointing to the GTAV installation directory.
Create an environment variable (DEEPGTAV_DIR) pointing to the directory of this repository.
Create an environment variable (DEEPGTAV_LOG_FILE) pointing to the file for logging (debugging purposes).
Create an environment variable (DEEPGTAV_EXPORT_DIR) pointing to the export location.

Get the newest version of SkriptHookV


make a directory with all the files to copy to GTAV folder:

| File                        |   From where                                                                        |
| --------------------------- |   --------------------------------------------------------------------------------- |
| DeepGTAV.asi                |   From compiled DeepGTAV-PreSIL\bin\Release                                         |
| DeepGTAV.iobj               |   From compiled DeepGTAV-PreSIL\bin\Release                                         |
| DeepGTAV.ipdb               |   From compiled DeepGTAV-PreSIL\bin\Release                                         |
| input8.dll                  |   From DeepGTAV-PreSIL\bin\Release or most recent SkripHook version (is the same)   |
| GTAVisionNative.asi         |   From compiled GTAVisionExport-DepthExtractor\native\build\src\Release             |
| GTAVisionNative.exp         |   From compiled GTAVisionExport-DepthExtractor\native\build\src\Release             |
| GTAVisionNative.lib         |   From compiled GTAVisionExport-DepthExtractor\native\build\src\Release             |
| GTAVLauncherBypass.asi      |   From DeepGTAV-PreSIL\bin\Release                                                  |
| GTAVLauncherBypass.ini      |   From DeepGTAV-PreSIL\bin\Release                                                  |
| opencv_world343.dll         |   From opencv install (care for version 3.4.3)                                      |
| paths.xml                   |   https://drive.google.com/file/d/0B6pR5O2YrmHnNU9EMDBSSFpMV00/view?usp=sharing     |
| ScripHookV.dll              |   From most recent SkriptHook version                                               |
