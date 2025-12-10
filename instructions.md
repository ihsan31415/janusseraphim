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

## Quick mode picker
- GUI only (no hardware): `ros2 launch janusseraphim_description display.launch.py use_hardware:=false`
- Simulated serial node (no GUI, no hardware): `ros2 launch janusseraphim_description display.launch.py use_sim_node:=true`
- Hardware serial: `ros2 launch janusseraphim_description display.launch.py use_hardware:=true simulate_serial:=false`
- Hardware flow but dummy data: `ros2 launch janusseraphim_description display.launch.py use_hardware:=true simulate_serial:=true`

### Visualize the robot (RViz + GUI joints)
- Launch with GUI joint sliders:
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_hardware:=false
  ```
- Loads `rviz2` with the provided config and runs `joint_state_publisher_gui` so you can move joints interactively.

### Simulated serial node (no hardware, no GUI)
- Launch the dedicated simulated serial node that publishes `joint_states` continuously:
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_sim_node:=true
  ```
- This disables the GUI slider node and runs `scripts/serial_sim_node.py` to emit smooth motion on all joints.
- Run it directly with tunable params:
  ```bash
  cd /home/syntropy/ros2_ws/src/janusseraphim
  python3 janusseraphim_description/scripts/serial_sim_node.py --ros-args -p rate_hz:=30.0 -p amplitude_rad:=1.2 -p prismatic_amp:=0.04
  ```
  `rate_hz` sets publish rate, `amplitude_rad` scales revolute joint amplitude, `prismatic_amp` sets linear stroke (meters).

### Hardware serial input
- Hardware enabled (expects Arduino on `/dev/ttyACM0`):
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_hardware:=true simulate_serial:=false
  ```
- Uses `scripts/serial_driver.py` (adjust port in that file if needed). Reads comma-separated ADC values (0–1023) for six channels and publishes `sensor_msgs/JointState` on `joint_states`.

### Hardware flow with dummy data
- Same process as hardware, but dummy sine data from the driver:
  ```bash
  ros2 launch janusseraphim_description display.launch.py use_hardware:=true simulate_serial:=true
  ```

## Run nodes standalone
- Serial driver only (after sourcing):
  ```bash
  cd /home/syntropy/ros2_ws/src/janusseraphim
  python3 janusseraphim_description/scripts/serial_driver.py --use-dummy true   # simulate data
  python3 janusseraphim_description/scripts/serial_driver.py                    # real serial on /dev/ttyACM0
  ```
- Simulated serial node only:
  ```bash
  cd /home/syntropy/ros2_ws/src/janusseraphim
  python3 janusseraphim_description/scripts/serial_sim_node.py --ros-args -p rate_hz:=30.0 -p amplitude_rad:=1.0 -p prismatic_amp:=0.03
  ```
- RViz only (visualize `joint_states` you publish elsewhere):
  ```bash
  ros2 run rviz2 rviz2 -d /home/syntropy/ros2_ws/src/janusseraphim/janusseraphim_description/rviz/config.rviz
  ```

## Troubleshooting
- If `joint_states` is not publishing, confirm the chosen node is running and visible via `ros2 topic echo /joint_states`.
- For hardware mode, verify serial permissions (e.g., add user to `dialout` or adjust `/dev/ttyACM*` perms) and confirm the port matches your device.
- Rebuild after changing package names or URDF assets, then re-source `install/setup.bash`.
