
# Changes I have made to DeepGTAV-PreSIL

- Added Functionality for UAV Recording
- Added Handeling for Messages `GoToLocation`, `SetCameraPositionAndRotation`, `TeleportToLocation`
- Restored messaging functionality (The fields in the returned JSON `frama`, `location`,... (This worked in DeepGTAV, but not in DeepGTAV-PreSIL)


# Things to note

- DeepGTAV-PreSIL has the setting `GENERATE_SECONDARY_PERSPECTIVES` which also records from the position of all cars in the proximity of the player vehicle. This would not work for the UAV-perspective. Some of the changes I have made could possibly clash with this. 
  - From some Benchmarking I have made this setting could not even be that useful. I record ~20% more images/minute only from player perspective without this setting.