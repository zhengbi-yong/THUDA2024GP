"""
Last modified date: 2023.06.06
Author: Jialiang Zhang
Description: Generate object point clouds
这段代码是一个用于生成物体点云的脚本。它通过从多个角度对3D模型进行投影，以模拟相机拍摄，从而生成物体的点云。以下是它的主要步骤和组件：

主要步骤
环境设置：首先，脚本改变当前工作目录到脚本文件所在的父目录。
解析命令行参数：使用 argparse 库来解析命令行参数，这些参数包括数据根路径、姿态数量、最大点数、样本数、CPU数量、GPU列表、相机设置等。
准备对象列表：脚本列出数据根路径下的所有对象目录，并为每个对象创建一个任务列表。
多进程处理：使用 multiprocessing.Pool 来并行处理每个对象，生成点云。
设置模拟环境：对于每个对象，脚本使用 Sapien 引擎创建一个模拟环境，并设置渲染器、场景、灯光等。
加载3D模型：通过从文件加载3D模型的可视化部分来创建一个演员（actor）。
设置和使用相机：设置一个挂载相机，模拟不同的视角来观察并拍摄3D模型。
生成和处理点云：使用相机拍摄的数据生成点云，通过对点云进行下采样和远点采样，得到一定数量的样本点。
保存点云数据：将处理后的点云数据保存到 .npy 文件中。
关键组件
Sapien Engine：用于物理模拟和渲染的引擎，这里主要用于创建场景和模拟相机观察3D模型。
相机模拟：通过设置相机参数（如视角、距离、高度等）和使用 Sapien 提供的相机功能，模拟真实世界的拍摄过程。
点云生成：根据相机拍摄得到的位置纹理，提取有效的点并转换到世界坐标系中，生成点云。
多进程：利用 Python 的 multiprocessing 库并行处理多个对象，提高效率。
点云处理：对生成的点云进行下采样和最远点采样（farthest point sampling），以减少点的数量并尝试保持点云的覆盖范围。
如何修改使用自己的数据
如果你想使用自己的数据，需要确保你的数据以适当的结构存放在指定的数据根路径下。具体来说，你的数据应该包含3D模型文件（如 .obj 文件）和相应的姿态信息（如 .npy 文件存储的姿态矩阵）。你还需要根据你的数据调整命令行参数，例如：

--data_root_path：指向你的数据所在的根目录。
--scale：根据你的模型大小调整缩放比例。
其他相机和渲染设置，如 --camera_distance、--camera_height 等，也可能需要根据你的具体需求进行调整。
最后，根据你的硬件配置，适当调整 --gpu_list 和 --n_cpu 参数，以充分利用你的计算资源。
"""

import argparse
import os
from multiprocessing import Pool, current_process

import numpy as np
import pytorch3d.io
import pytorch3d.ops
import pytorch3d.structures
import sapien.core as sapien
import torch
import transforms3d
from tqdm import tqdm

# os.chdir(os.path.dirname(os.path.dirname(__file__)))
# 获取脚本所在目录的路径
script_dir = os.path.dirname(os.path.abspath(__file__))
# print(f"当前工作目录：{script_dir}")
# 改变当前工作目录到脚本所在目录的父目录
os.chdir(os.path.dirname(script_dir))
data_dir = os.path.join(os.path.dirname(script_dir), "data/DFCData/meshes")


def sample_projected(_):
    args, object_code, idx = _

    worker = current_process()._identity[0]
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu_list[
        (worker - 1) % len(args.gpu_list)
    ]
    # print(idx)

    object_path = os.path.join(
        args.data_root_path, object_code, "coacd", "decomposed.obj"
    )
    # print(f"object code:{object_code}")

    # set simulator

    engine = sapien.Engine()
    engine.set_log_level("critical")
    # renderer = sapien.SapienRenderer(offscreen_only=True)
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()

    rscene = scene.get_renderer_scene()
    if hasattr(rscene, "set_ambient_light"):
        rscene.set_ambient_light([0.5, 0.5, 0.5])
    if hasattr(rscene, "add_directional_light"):
        rscene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    if hasattr(rscene, "add_point_light"):
        rscene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
        rscene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
        rscene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        object_path, scale=[args.scale, args.scale, args.scale]
    )
    object_actor = builder.build_kinematic()

    camera_mount_actor = scene.create_actor_builder().build_kinematic()
    camera = scene.add_mounted_camera(
        name="camera",
        actor=camera_mount_actor,
        pose=sapien.Pose(),  # relative to the mounted actor
        width=args.width,
        height=args.height,
        fovx=0,
        fovy=np.deg2rad(35),
        near=args.near,
        far=args.far,
    )
    # print('Intrinsic matrix\n', camera.get_camera_matrix())

    # camera_eye = np.array([[0.2, -0.5, 1.0], [1.0, 0.2, 1.0], [0.2, 0.2, 1.4]])
    # camera_forward = np.array([[0, 4.0, 0], [-4.0, 0, 0], [0, -0.01, -2.0]]
    camera_forward = np.array(
        [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float32
    )
    camera_eye = (
        np.array([0, 0, args.camera_height]) - args.camera_distance * camera_forward
    )
    # angles = np.linspace(0, 2 * np.pi, args.n_cameras, endpoint=False)
    # cam_pos_array = np.stack([
    #     args.camera_distance * np.sin(args.theta) * np.cos(angles),
    #     args.camera_distance * np.sin(args.theta) * np.sin(angles),
    #     args.camera_distance * np.cos(args.theta).repeat(args.n_cameras)
    # ], axis=1)
    # cam_pos_array = np.stack(np.meshgrid([-args.camera_distance, args.camera_distance], [-args.camera_distance, args.camera_distance], [-args.camera_distance, args.camera_distance]), axis=-1).reshape(-1, 3)
    # print(f'n_camera: {len(cam_pos_array)}')

    # load poses

    pose_matrices = np.load(os.path.join(args.data_root_path, object_code, "poses.npy"))
    pose_matrices = (
        pose_matrices
        if len(pose_matrices) <= args.n_poses
        else pose_matrices[: args.n_poses]
    )

    # sample pc

    pcs = []

    if os.path.exists(os.path.join(args.data_root_path, object_code, "pcs.npy")):
        pcs_old = np.load(os.path.join(args.data_root_path, object_code, "pcs.npy"))
        for pc in pcs_old:
            pcs.append(pc)
        if len(pcs) >= args.n_poses:
            return

    for pose_matrix in pose_matrices[len(pcs) :]:

        pc = []

        translation = pose_matrix[:3, 3]
        translation *= args.scale
        translation[:2] = 0
        rotation_matrix = pose_matrix[:3, :3]
        rotation_quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)

        # for cam_pos in cam_pos_array:
        for idx_camera in range(len(camera_eye)):

            # Compute the camera pose by specifying forward(x), left(y) and up(z)
            cam_pos = camera_eye[idx_camera]
            forward = camera_forward[idx_camera]
            # forward = -cam_pos / np.linalg.norm(cam_pos)
            left = np.cross([0, 0, 1], forward)
            left = np.cross([0, 1, 0], forward) if np.linalg.norm(left) < 0.01 else left
            left = left / np.linalg.norm(left)
            up = np.cross(forward, left)
            mat44 = np.eye(4)
            mat44[:3, :3] = np.stack([forward, left, up], axis=1)
            mat44[:3, 3] = cam_pos
            camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(mat44))

            # render

            object_actor.set_pose(sapien.Pose(translation, rotation_quaternion))
            scene.step()  # make everything set
            scene.update_render()
            camera.take_picture()

            # Each pixel is (x, y, z, is_valid) in camera space (OpenGL/Blender)
            position = camera.get_float_texture("Position")  # [H, W, 4]
            # OpenGL/Blender: y up and -z forward
            points_opengl = position[..., :3][position[..., 2] != 0]
            # Model matrix is the transformation from OpenGL camera space to SAPIEN world space
            # camera.get_model_matrix() must be called after scene.update_render()!
            model_matrix = camera.get_model_matrix()
            points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]
            pc.append(points_world)

        pc = np.concatenate(pc)
        pc_down_sampled = (
            pc[np.random.choice(len(pc), args.max_n_points, replace=False)]
            if len(pc) > args.max_n_points
            else pc
        )
        pc_sampled = pytorch3d.ops.sample_farthest_points(
            torch.tensor(pc_down_sampled).unsqueeze(0), K=args.num_samples
        )[0][0]
        pc_sampled = (pc_sampled - translation) @ rotation_matrix / args.scale
        pcs.append(pc_sampled)
        # print(f'n_points: {len(pc)}')

    pcs = np.stack(pcs)

    np.save(os.path.join(args.data_root_path, object_code, "pcs.npy"), pcs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # experiments settings
    parser.add_argument("--data_root_path", type=str, default=data_dir)
    parser.add_argument("--n_poses", type=int, default=100)
    parser.add_argument("--max_n_points", type=int, default=9000)
    parser.add_argument("--num_samples", type=int, default=3000)
    parser.add_argument("--n_cpu", type=int, default=8)
    # parser.add_argument('--n_cameras', type=int, default=6)
    # parser.add_argument('--theta', type=float, default=np.pi / 4)
    parser.add_argument("--scale", type=float, default=0.1)
    parser.add_argument("--gpu_list", type=str, nargs="*", default=["0"])
    # camera settings
    parser.add_argument("--camera_distance", type=float, default=0.5)
    parser.add_argument("--camera_height", type=float, default=0.05)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--near", type=float, default=0.1)
    parser.add_argument("--far", type=float, default=100)
    args = parser.parse_args()

    object_category_list = os.listdir(args.data_root_path)
    object_code_list = []
    for object_category in object_category_list:
        object_code_list += [
            os.path.join(object_category, object_code)
            for object_code in sorted(
                os.listdir(os.path.join(args.data_root_path, object_category))
            )
        ]
    # object_code_list = [object_code for object_code in object_code_list if not os.path.exists(os.path.join(args.data_root_path, object_code, 'pcs.npy'))]

    # object_code_list = object_code_list[:1]

    parameters = []
    for idx, object_code in enumerate(object_code_list):
        parameters.append((args, object_code, idx))

    with Pool(args.n_cpu) as p:
        it = tqdm(
            p.imap(sample_projected, parameters), desc="sampling", total=len(parameters)
        )
        list(it)
