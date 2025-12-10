# Running the janusseraphim stack

These notes assume a ROS 2 workspace at `/home/syntropy/ros2_ws` with the source tree in `src/janusseraphim`. Adjust paths if your workspace differs.

## Build and source
1. From the workspace root, build the packages:
   ```bash
   cd /home/syntropy/ros2_ws
   colcon build --packages-select janusseraphim janusseraphim_description
   ```
2. Source the overlay (do this in every new shell before running nodes):
   ```bash
   source /home/syntropy/ros2_ws/install/setup.bash
   ```

## Visualize the robot (RViz + GUI joints)
- Launch the description with GUI joint sliders (no hardware required):
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_hardware:=false
  ```
- The launch file loads `rviz2` with the provided config and runs `joint_state_publisher_gui` so you can move joints interactively.

## Use hardware serial input
- Start the launch with hardware enabled (expects Arduino on `/dev/ttyACM0`):
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_hardware:=true simulate_serial:=false
  ```
- The launch executes the serial driver script directly from source:
  `/home/syntropy/ros2_ws/src/janusseraphim/janusseraphim_description/scripts/serial_driver.py`
- The driver reads comma-separated ADC values (0–1023) for six channels and publishes `sensor_msgs/JointState` on `joint_states`.
- If your serial device uses a different port, edit the path in `serial_driver.py` (default: `/dev/ttyACM0`).

## Simulate hardware serial data
- You can still launch with `use_hardware:=true` but simulate input instead of reading Arduino:
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_hardware:=true simulate_serial:=true
  ```
- The driver will generate dummy sine-wave data for the six channels.

## Run the serial driver standalone
- From the source tree (after sourcing the workspace):
  ```bash
  cd /home/syntropy/ros2_ws/src/janusseraphim
  python3 janusseraphim_description/scripts/serial_driver.py --use-dummy true   # simulate data
  python3 janusseraphim_description/scripts/serial_driver.py                    # real serial on /dev/ttyACM0
  ```
- This publishes `joint_states`; start `rviz2` separately if you want visualization:
  ```bash
  ros2 run rviz2 rviz2 -d /home/syntropy/ros2_ws/src/janusseraphim/janusseraphim_description/rviz/config.rviz
  ```

## Troubleshooting
- If `joint_states` is not publishing, check that the serial device is accessible and sending comma-separated values.
- For hardware mode, verify permissions on the serial port (e.g., add your user to the `dialout` group or `sudo chmod` the device).
- Rebuild after changing package names or URDF assets, then re-source `install/setup.bash`.
