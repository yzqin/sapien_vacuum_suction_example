# Suction Gripper Simulation in SAPIEN

This repository contains an implementation of a suction gripper simulation in SAPIEN, available in
a [single file](sapien_vaccum_gripper_example.py). The simulation utilizes the robot model of XArm6 + XArm vacuum
gripper to achieve pick and place tasks. The only dependency required is `pip3 install sapien`.

![A gif showing the suction gripper in action](doc/suction.gif)

The main code for the suction gripper is shown below:

```python
# Do something before suction

# Suction
suction_drive = scene.create_drive(ee_link, Pose(), grasped_box, Pose())
suction_drive.set_x_properties(1e4, 1e2, 1e2)
suction_drive.set_y_properties(1e4, 1e2, 1e2)
suction_drive.set_z_properties(1e4, 1e2, 1e2)

# Do something after suction

# Release suction
suction_drive.set_x_properties(0, 0, 0)
suction_drive.set_y_properties(0, 0, 0)
suction_drive.set_z_properties(0, 0, 0)
``` 