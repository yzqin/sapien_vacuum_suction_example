from pathlib import Path

import numpy as np
import sapien.core as sapien
from sapien.asset import create_dome_envmap
from sapien.core import Pose
from sapien.utils import Viewer

BOX_SIZE = 0.03


def create_box(scene: sapien.Scene, renderer: sapien.VulkanRenderer, color):
    # Load box
    box_size = np.array([BOX_SIZE, BOX_SIZE, BOX_SIZE])
    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = color
    material.roughness = 0.1
    material.metallic = 1.0
    builder.add_box_visual(material=material, half_size=box_size)
    builder.add_box_collision(half_size=box_size)
    box = builder.build()
    return box


def create_table(scene: sapien.Scene, renderer: sapien.VulkanRenderer, table_height=1.0,
                 table_half_size=(0.8, 0.8, 0.025)):
    builder = scene.create_actor_builder()

    # Top
    top_pose = sapien.Pose([0, 0, -table_half_size[2]])
    top_material = scene.create_physical_material(1, 0.5, 0.01)
    builder.add_box_collision(pose=top_pose, half_size=table_half_size, material=top_material)
    # Leg
    asset_dir = Path(__file__).parent / "assets"
    table_map_path = asset_dir / "misc" / "table_map.jpg"
    table_cube_path = asset_dir / "misc" / "cube.obj"
    table_visual_material = renderer.create_material()
    table_visual_material.set_metallic(0.0)
    table_visual_material.set_specular(0.3)
    table_visual_material.set_diffuse_texture_from_file(str(table_map_path))
    table_visual_material.set_roughness(0.3)
    leg_size = np.array([0.025, 0.025, (table_height / 2 - table_half_size[2])])
    leg_height = -table_height / 2 - table_half_size[2]
    x = table_half_size[0] - 0.1
    y = table_half_size[1] - 0.1

    # Build visual
    builder.add_visual_from_file(str(table_cube_path), pose=top_pose, material=table_visual_material,
                                 scale=table_half_size, name="surface")
    builder.add_box_visual(pose=sapien.Pose([x, y, leg_height]), half_size=leg_size,
                           material=table_visual_material, name="leg0")
    builder.add_box_visual(pose=sapien.Pose([x, -y, leg_height]), half_size=leg_size,
                           material=table_visual_material, name="leg1")
    builder.add_box_visual(pose=sapien.Pose([-x, y, leg_height]), half_size=leg_size,
                           material=table_visual_material, name="leg2")
    builder.add_box_visual(pose=sapien.Pose([-x, -y, leg_height]), half_size=leg_size,
                           material=table_visual_material, name="leg3")
    return builder.build_static("table")


def compute_inverse_kinematics(delta_pose_world, ee_jacobian, damping=0.05):
    lmbda = np.eye(6) * (damping ** 2)
    delta_qpos = ee_jacobian.T @ \
                 np.linalg.lstsq(ee_jacobian.dot(ee_jacobian.T) + lmbda, delta_pose_world, rcond=None)[0]

    return delta_qpos


def move_robot(robot: sapien.Articulation, delta_pos: np.ndarray, end_link_index: int, scene, viewer):
    delta_rot = np.zeros(3)  # No need for gripper rotation
    delta_pose = np.concatenate([delta_pos, delta_rot])
    jacobian = robot.compute_world_cartesian_jacobian()[end_link_index * 6 - 6:end_link_index * 6]
    delta_qpos = compute_inverse_kinematics(delta_pose, jacobian)
    robot.set_drive_target(robot.get_drive_target() + delta_qpos)
    robot.set_qf(robot.compute_passive_force(external=False))
    scene.step()
    scene.update_render()
    viewer.render()


def wait(n, robot: sapien.Articulation, scene, viewer):
    for _ in range(n):
        robot.set_qf(robot.compute_passive_force(external=False))
        scene.step()
        scene.update_render()
        viewer.render()


def main():
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)
    scene = engine.create_scene()

    # Sky and ground
    scene.set_environment_map(create_dome_envmap(sky_color=[0.529, 0.808, 0.922], ground_color=[0.607, 0.462, 0.325]))
    scene.set_ambient_light([0.1, 0.1, 0.1])
    scene.add_point_light([1, 0, 0.5], [1, 1, 1])
    scene.add_directional_light([0, -1, -1], [1, 1, 1])
    ground_material = renderer.create_material()
    ground_material.base_color = np.array([0.607, 0.462, 0.325, 1])
    ground_material.specular = 0.5
    scene.add_ground(-1, render_material=ground_material)
    scene.set_timestep(1 / 240)

    # Load robot
    loader = scene.create_urdf_loader()
    robot = loader.load("assets/xarm6/xarm6_vacuum.urdf")
    robot.set_pose(Pose([-0.5, 0, 0]))
    robot.set_qpos(np.array([0, 0, -0.4, 0, 0.4, 0]))
    for joint in robot.get_active_joints():
        joint.set_drive_property(1e6, 1e4, 1e3)
    robot.set_drive_target(robot.get_qpos())

    # Load box and table
    table = create_table(scene, renderer)
    box1 = create_box(scene, renderer, [0.8, 0, 0, 1])
    box1.set_pose(Pose([0, 0, BOX_SIZE]))
    box2 = create_box(scene, renderer, [0.0, 0.8, 0, 1])
    box2.set_pose(Pose([0, 0.3, BOX_SIZE]))
    box3 = create_box(scene, renderer, [0.0, 0, 0.8, 1])
    box3.set_pose(Pose([0, -0.3, BOX_SIZE]))
    boxes = [box2, box3, box1]
    scene.step()

    # Viewer
    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    camera = scene.add_camera(name="camera", width=3692, height=2032, fovy=1.57, near=0.1, far=100)
    camera.set_local_pose(Pose([-0.493444, 0.472136, 0.335562], [0.832326, 0.164863, 0.334857, -0.409786]))
    viewer.focus_camera(camera)

    # Whether to use lock_motion for suction simulation
    use_lock_motion = True

    end_link_index = len(robot.get_links()) - 1
    ee_link = robot.get_links()[-1]
    object_z_offset = BOX_SIZE + 0.02
    while not viewer.closed:
        for i in range(2):
            n = 200
            grasped_box = boxes[i]
            placed_box = box1

            # Move to target box before suction
            delta_pos = (grasped_box.get_pose().p - ee_link.get_pose().p + np.array([0, 0, object_z_offset])) / n
            for k in range(n):
                move_robot(robot, delta_pos, end_link_index, scene, viewer)
            wait(50, robot, scene, viewer)

            # Suction
            relative_pose = ee_link.get_pose().inv() * grasped_box.get_pose()
            relative_pos = [0, 0, -BOX_SIZE]
            relative_pose.set_p(relative_pos)
            suction_drive = scene.create_drive(ee_link, relative_pose, grasped_box, Pose())
            if use_lock_motion:
                suction_drive.lock_motion(1, 1, 1, 1, 1, 1)
            else:
                suction_drive.set_x_properties(1e4, 1e2, 1e2)
                suction_drive.set_y_properties(1e4, 1e2, 1e2)
                suction_drive.set_z_properties(1e4, 1e2, 1e2)

            # Move the grasped box to the box for place
            delta_pos = (placed_box.get_pose().p - grasped_box.get_pose().p + np.array(
                [0, 0, object_z_offset + (i * 2 + 1) * BOX_SIZE])) / n
            for k in range(n):
                move_robot(robot, delta_pos, end_link_index, scene, viewer)
            wait(50, robot, scene, viewer)

            # Release suction
            if use_lock_motion:
                suction_drive.free_motion(1, 1, 1, 1, 1, 1)
            else:
                suction_drive.set_x_properties(0, 0, 0)
                suction_drive.set_y_properties(0, 0, 0)
                suction_drive.set_z_properties(0, 0, 0)

        print("Finished")
        wait(10000, robot, scene, viewer)


if __name__ == '__main__':
    main()
