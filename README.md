# My Bot Hackathon Submission

This is a ROS 2 Jazzy project for the ... hackathon.

## How to Build
1. Clone this repo into the `src` folder of a ROS 2 workspace:
   `git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git`
2. Move to the workspace root: `cd ..`
3. Install dependencies: `rosdep install -i --from-path src -y`
4. Build the package: `colcon build`

## How to Run
1. Source the workspace: `source install/setup.bash`
2. Launch the simulation: `ros2 launch my_bot launch_sim.launch.py`
3. Run teleop (in a new terminal): `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
