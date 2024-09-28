# Summary

This folder contains the code for a Unity project with the following parts:
- Implementation of the Kalman Filter for the localization estimate of a robot in an unknown scene, using a LIDAR
- Implementation of a mapping algorithm using grid maps: a static map for static objects and a dynamic map for dynamic objects

# Scenes

The project is made of different scenes to test the algorithms that were implemented:

- *SimpleScene*: Contains a simple scene to test the kalman filter and the dynamic construction of a map for the scene
- *MediumScene*: Contains a more complex scene to test the kalman filter and the mapping algorithms
- *ComplexScene*: Contains the most complex scene, filled with static and dynamic objects. The scene was taken from the following asset: https://assetstore.unity.com/packages/3d/environments/industrial/unity-warehouse-276394
- *EmptyScene*: Contains an almost empty scene, used to evaluate the performance of the mapping using geometric primitives, by logging and recording useful data during the mapping
- *DataloaderScene*: Contains an almost empty scene used to play recorded data (more details below)

# Setup:

1. Download and install the following asset: https://assetstore.unity.com/packages/3d/environments/industrial/unity-warehouse-276394, that is required for the complex scene
2. Each scene among the *SimpleScene*, *MediumScene* and *ComplexScene* contains three robot kinds: KalmanRobot, SimulatedRobot and RobotRecorder (more details below). Only one robot should be toggled active for performance reasons. Each robot can be placed at any location in the scene, without breaking the scripts (the localization estimate is updated accordingly). The robots can be controlled using the arrow keys (up, down, left, right).

# Robot kinds:

There are four different robot kinds that can be used in this project:
- *KalmanRobot* and *SimulatedRobot*: Can be controlled with the arrow keys, can be used for the mapping (see the script RobotManager)
- *DataloaderRobot*: Cannot be controlled with the arrow keys, since the robot plays recorded data. This data can be used for the mapping of the scene (see the script RobotManager)
- *RobotRecorder*: Special kind used to record LIDAR captures, cannot be used for mapping

## DataloaderRobot

This robot can be used to play recorded LIDAR data, saved as JSON files. The script was designed to specifically match a LIDAR capture external to this project, that is not even compatible with the data recorded by the RobotRecorder

To be compatible with this script, the data should respect the following conditions:

- All the frames should be located inside the same folder
- The frames should be named as follows:
    - step0.frame_data.json
    - step1.frame_data.json
    - step2.frame_data.json...
- capture[0] is expected to contain data for the rear LIDAR of the robot
- capture[2] is expected to contain data for the front LIDAR of the robot
- An example of frame is available in this file: dataloader_frame_example.json

### Usage

1. Open the *DataloaderScene*
2. In the Robot GameObject, in the script "Dataloader Robot", make sure that the path "DataFolder" points to a right folder, with LIDAR captures defined as above
3. Press the "Play" button in Unity
4. In the "Dataloader Robot" script, press the "run" button to start the mapping from the recorded captures
5. Toggle the display of the grid maps using the M-key
6. Toggle the drawing of the different components of the maps using geometric primitives at the bottom of the script "Robot Manager"

## Kalman Robot

This robot can be used to estimate it's pose from the LIDAR observations. This allows to run the whole algorithm (localization + mapping), even if this is currently quite slow.

### Usage

1. Select a scene among *SimpleScene*, *MediumScene* and *ComplexScene*
2. Make sure that the *KalmanRobot* is the only robot that is set active in Unity 
3. Press the "Play" button in Unity
4. Move the robot using the arrow keys of the keyboard
5. Toggle the display of the grid maps using the M-key
6. Toggle the drawing of the different components of the maps using geometric primitives at the bottom of the script "Robot Manager"

## Simulated Robot

This robot is used to focus on the mapping part, and uses the exact pose of the robot (given by Unity), instead of trying to estimate the pose with the Kalman Filter.

This is much faster and accurate than the KalmanRobot, but also less realistic, since in reality the global pose of the robot is unknown.

### Usage

1. Select a scene among *SimpleScene*, *MediumScene* and *ComplexScene*
2. Make sure that the *SimulatedRobot* is the only robot that is set active in Unity 
3. Press the "Play" button in Unity
4. Move the robot using the arrow keys of the keyboard
5. Toggle the display of the grid maps using the M-key
6. Toggle the drawing of the different components of the maps using geometric primitives at the bottom of the script "Robot Manager"

## RobotRecorder

This robot can be used to record LIDAR captures for the mapping in the Python implementation. It was designed to only record data, not to perform the mapping.

### Usage

1. Select a scene among *SimpleScene*, *MediumScene* and *ComplexScene*
2. Make sure that the *RobotRecorder* is the only robot that is set active in Unity
3. Select the *RobotRecorder*
4. Make sure that the folder defined in "SaveFolder" is a valid path (the folder exists)
5. Press the "Play" button in Unity
6. Move the robot using the arrow keys of the keyboard
7. When the recording is done, press again the "Play" button in Unity (to stop the recording and the simulation)
8. You can now use the recorded capture directly in the Python implementation (see the corresponding README for more information)
