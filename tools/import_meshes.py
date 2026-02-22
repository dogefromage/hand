import os
from pathlib import Path
import shutil
import numpy as np
import pandas
from urchin import URDF, Joint, Link
import trimesh
from scipy.spatial.transform import Rotation as R

############ CUSTOMIZATION

dir_meshes = Path("import/meshes2")
path_urdf = Path("import/urdf/robot.urdf")
path_csv = Path("import/urdf/robot.csv")
path_tree = Path("import/urdf/tree.txt")

dir_output_meshes = Path("src/hand_description/meshes")
path_output_urdf = Path("src/hand_description/urdf/robot.urdf")

package_relative_mesh_prefix = "package://hand_description/meshes/"
robot_name = "hand"

part_name_to_mesh_name = lambda part_name: f"arm - {part_name.replace("/", " ")}.STL"

############ CUSTOMIZATION END


def T_base_link(urdf: URDF, link_name: str):

    joints_by_children: dict[Link, Joint] = {}
    for joint in urdf.joints:
        joints_by_children[joint.child] = joint

    T = np.eye(4)

    curr = link_name
    while curr in joints_by_children:
        joint = joints_by_children[curr]
        assert joint.joint_type == "revolute"

        # q = 0
        # R_revolute = R.from_rotvec(joint.axis * q)
        # joint.axis, joint.origin
        # joint.
        T_next = joint.origin
        T = T_next @ T

        curr = joint.parent

    return T


def clean():
    print(f"Cleaning: {dir_output_meshes}")
    shutil.rmtree(dir_output_meshes)
    dir_output_meshes.mkdir()

    print(f"Deleting: {path_output_urdf}")
    path_output_urdf.unlink(missing_ok=True)


def do_import():

    robot = URDF.load(str(path_urdf), lazy_load_meshes=True)
    robot.name = robot_name
    for link in robot.links:
        for visual in link.visuals:
            if visual.geometry.mesh is not None:
                visual.geometry.mesh.filename = package_relative_mesh_prefix + f"{link.name}.STL"
        for collision in link.collisions:
            if collision.geometry.mesh is not None:
                collision.geometry.mesh.filename = package_relative_mesh_prefix + f"{link.name}.STL"

    with open(path_tree, "r") as f:
        tree_paths = [line.strip() for line in f.readlines()]

    robot_df = pandas.read_csv(path_csv)
    for idx, (link_name, sw_components_str) in robot_df[["Link Name", "SW Components"]].iterrows():
        sw_components = sw_components_str.split(";")
        parts = []
        for sw_component in sw_components:
            parts.extend(filter(lambda line: line.startswith(sw_component), tree_paths))

        parts = list(set(parts))
        # print(f"Link '{link_name}' contains:")
        # print(parts)
        # print()
        # print()

        meshes = []
        for part in parts:
            mesh_name = part_name_to_mesh_name(part)
            path_mesh = dir_meshes / mesh_name
            print(f"Loading: {path_mesh}")
            mesh = trimesh.load(path_mesh)
            meshes.append(mesh)

        print(f"Combining...")
        combined_mesh = trimesh.util.concatenate(meshes)

        T = T_base_link(robot, link_name)
        # print(T)
        T_inv = np.linalg.inv(T)
        combined_mesh.apply_transform(T_inv)

        path_combined_mesh = dir_output_meshes / f"{link_name}.STL"
        print(f"Saving: {path_combined_mesh}")
        combined_mesh.export(path_combined_mesh)

    print(f"Saving: {path_output_urdf}")
    robot.save(path_output_urdf)


if __name__ == "__main__":
    clean()
    do_import()
