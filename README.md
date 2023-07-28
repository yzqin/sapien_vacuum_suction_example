# Suction Gripper Simulation in SAPIEN

This repo provides a [single file implementation](sapien_vaccum_gripper_example.py) of simulating a suction gripper in
SAPIEN. It utilizes the robot model of XArm6 + XArm vacuum gripper to achieve some pick and place tasks. The only
dependency is `pip3 install sapien`

![teaser](doc/suction.gif)

The main code is shown as follow:

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

