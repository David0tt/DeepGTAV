
1. Make compatible with drone footage: How could this be done (simplest to hardest):
   - Teleport Player with start Position in VPilot
   - Use something like flying car mode and set player Position

   - change vehicle Model to something that flies?
    https://gtaforums.com/topic/795380-qnetai-drive-to-coords-fly-helicopter-to-location-and-land-on-gr/


   - Use something like 

            PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
            while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

        - Additionally allow a VPilot command to change player Position
        - Moving the player by teleporting could be very slow!

        - Does 
            
                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f), vehicleHash, _drivingMode, 50.f, true);
            
            work in the air?

    - For the movement i basically just have to understand the loop in `Scenario.cpp` where `generate_n_random_points()` is used.

2. Fix Movement:
   - I Want to be able to send "go_to()" and "teleport_to()" Messages in VPilot
     - Or Better yet: "capture_at()"

3. Integrate loss calculation in loop: How could this be done (simplest to hardest):
    - do loss calculation in VPilot script

4. Use 1080px Screen to fix screen resolution issues

5. Get exact camera position and rotation with 

        CAM::GET_CAM_COORD()
        CAM::GET_CAM_ROT()



Questions:
1. Do the generated 2d-BB already work or do i have to use the python script?
2. How exactly does the data generation world random walk work right now? -> Let it drive around a little


Would be nice to have:
   - Debug Model loss ingame
   - or at least show model loss in the VPilot script (for current location)

   - extract UAV height and metadata (rotations etc.)
   - Send data in messages
   - In DeepGTAV-PreSIL the TCP-messages are broken. Fix this?



Speed Comparison (EXPORT_DIR on HDD vs SSD):
in 15 min Runtime:

drive | produced PNG Images (this includes all types!) | object folder size
----- | ---------------------------------------------- | ------------------
HDD | 734 | 2.5 GB
SSD | 847 | 2.9 GB
SSD + Only one Agent | 1060 | 3.13 GB

SSD is a speedup by 1 - 2.9/2.5 = 16%

Used file space is OK