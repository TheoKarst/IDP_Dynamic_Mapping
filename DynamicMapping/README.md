# Summary

This code contains an implementation of two mapping algorithms:
- The first one uses grid maps to represent the environment of the robot
- The second one uses geometric primitives (lines and circles)

The data used to run the algorithms can be found in the zip folder LidarCaptures.zip. The simulations
are run using pygame for the display.

# Lidar captures

The folder LidarCaptures.zip contains two LIDAR captures, made using Unity:
- StaticCapture_0: Contains recorded data in a static warehouse
- DynamicCapture_0: Contains recorded data in a dynamic warehouse

# Usage

Unzip the LIDAR captures (recorded using Unity):
```console
$ cd ..
$ unzip LidarCaptures.zip
```

Install the requirements:
```console
$ cd DynamicMapping
$ pip install -r requirements.txt
```

Help message
```console
$ python3 dynamic_mapping/dynamic_mapping.py -h
```

Run the code with a given datafolder to use for recorded LIDAR captures. If a folder is
not provided, the folder DynamicCapture_0 will be used by default

```console
$ python3 dynamic_mapping/dynamic_mapping.py -f ../LidarCaptures/DynamicCapture_0/
```

# Mouse and key controls:

Use the mouse scroll to zoom in the scene and the mouse left button to drag the scene.

Key controls:
- Press the R key to toggle the drawing of LIDAR rays
- Press the G key to change the draw mode for the grid maps used for dynamic mapping. There are 4 different modes: drawing the static map only, the dynamic map only, both maps or none
- Press the P key to toggle the drawing of the mapping using geometric primitives
- Press the W key to toggle the drawing of the wipe-shape in the mapping using geometric primitives
- Press the M key to toggle the drawing of the match grid used in the mapping using geometric primitives
- Press the S key to toggle the drawing of the speed estimates of the primitives in the mapping using geometric primitives
- Press the L key to toggle the drawing of lines in the mapping using geometric primitives
- Press the C key to toggle the drawing of circles in the mapping using geometric primitives

# Notes

By default, the mapping using grid maps and the mapping using geometric primitives are both activated, which can be slow in practice. This behaviour can be changed in the dynamic_mapping.py file with the variables USE_GRIDS_MAPPING and USE_GEOMETRY_MAPPING.
