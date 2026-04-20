# p2_lizijun

CS 424/524 Assignment 2 repository for:
1. **Part 1**: autonomous navigation through three user-defined map locations and return to start.
2. **Part 2**: RGB-D red ball following while maintaining about 1 meter distance.

## Repository structure
- `launch/`: launch files required by the assignment
- `script/`: Python ROS nodes
- `src/`: reserved for C++ source files (left empty intentionally)
- `misc/`: required assignment materials
- `config/`: editable parameters for map path and goals

## Quick use
### Part 1
1. Build a map first using gmapping and save it as a `.yaml` + `.pgm` pair.
2. Edit `config/p2a_params.yaml`:
   - set `map_file` to your saved map yaml path
   - set the `goals` list to your actual L1, L2, L3, and return-to-L1 poses in the AMCL map frame
3. Launch:
   ```bash
   roslaunch p2_lizijun p2a.launch
   ```

### Part 2
1. Start the robot with cameras connected.
2. Launch:
   ```bash
   roslaunch p2_lizijun p2b.launch
   ```
3. Hold a red ball in front of the robot.

## Notes
- Part 1 is fully autonomous after startup, but it still depends on a valid saved map and correct goal coordinates for your own lab.
- Part 2 uses RGB + depth, publishes velocity on `/cmd_vel`, and also publishes a debug image on `/p2b/debug_image`.
- This package is written in Python to keep deployment simple.
