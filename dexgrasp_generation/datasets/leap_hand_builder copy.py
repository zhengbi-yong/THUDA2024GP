import os

import numpy as np
import pytorch_kinematics as pk
import torch
import trimesh
import urdf_parser_py.urdf as urdf
from pytorch3d.structures import Meshes, join_meshes_as_batch


class LeapHandBuilder:
    # 定义手部各关节名称
    joint_names = [
        "joint0",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
        "joint8",
        "joint9",
        "joint10",
        "joint11",
        "joint12",
        "joint13",
        "joint14",
        "joint15",
    ]
    mesh_filenames = [
        "dip.stl",
        "fingertip.stl",
        "mcp_joint.stl",
        "palm_lower.stl",
        "pip.stl",
        "thumb_dip.stl",
        "thumb_fingertip.stl",
        "thumb_pip.stl",
    ]

    def __init__(
        self, urdf_path="data/leap_hand/robot.urdf", mesh_dir="data/leap_hand/meshes"
    ):
        print("开始构建LEAP Hand")
        print(f"当前工作目录：{os.getcwd()}")
        print(f"URDF绝对路径：{os.path.abspath(urdf_path)}")
        urdf_path = (
            "/home/sisyphus/UniDexGrasp/dexgrasp_generation/data/leap_hand/robot.urdf"
        )
        print(f"URDF_path：{urdf_path}")
        # self.chain = pk.build_serial_chain_from_urdf(open(urdf_path).read()).to(
        #     dtype=torch.float
        # )

        self.chain = pk.build_serial_chain_from_urdf(
            open(urdf_path).read(), end_link_name="fingertip"
        ).to(dtype=torch.float)
        self.mesh_dir = mesh_dir
        self.mesh = {}
        self._build_meshes()

    def _build_meshes(self):
        device = "cpu"
        # 遍历URDF文件中的所有链接，加载相应的网格
        for link in self.chain.links:
            # 获取链接名和网格文件名
            link_name = link.link.name
            mesh_filename = link.link.visual[0].geometry.filename
            full_mesh_path = os.path.join(self.mesh_dir, mesh_filename)
            print(f"文件路径：{full_mesh_path}")

            # 加载网格文件
            link_mesh = trimesh.load(full_mesh_path, process=False)
            vertices = torch.tensor(
                link_mesh.vertices, dtype=torch.float, device=device
            )
            faces = torch.tensor(link_mesh.faces, dtype=torch.long, device=device)

            # 存储网格信息
            self.mesh[link_name] = {"vertices": vertices, "faces": faces}

    def qpos_to_qpos_dict(self, qpos, hand_qpos_names=None):
        if hand_qpos_names is None:
            hand_qpos_names = LeapHandBuilder.joint_names
        assert len(qpos) == len(hand_qpos_names)
        return dict(zip(hand_qpos_names, qpos))

    def qpos_dict_to_qpos(self, qpos_dict, hand_qpos_names=None):
        if hand_qpos_names is None:
            hand_qpos_names = LeapHandBuilder.joint_names
        return np.array([qpos_dict[name] for name in hand_qpos_names])

    def get_hand_mesh(
        self,
        rotation_mat,
        world_translation,
        qpos=None,
        hand_qpos_dict=None,
        hand_qpos_names=None,
        without_arm=False,
    ):
        if qpos is None:
            if hand_qpos_names is None:
                hand_qpos_names = LeapHandBuilder.joint_names
            assert hand_qpos_dict is not None, "Both qpos and qpos_dict are None!"
            qpos = np.array(
                [hand_qpos_dict[name] for name in hand_qpos_names], dtype=np.float32
            )

        current_status = self.chain.forward_kinematics(qpos[np.newaxis, :])
        meshes = []

        for link_name in self.mesh:
            v = current_status[link_name].transform_points(
                self.mesh[link_name]["vertices"]
            )
            v = v @ rotation_mat.T + world_translation
            f = self.mesh[link_name]["faces"]
            meshes.append(Meshes(verts=[v], faces=[f]))

        if without_arm:
            meshes = join_meshes_as_batch(meshes[1:])  # each link is a "batch"
        else:
            meshes = join_meshes_as_batch(meshes)  # each link is a "batch"
        return Meshes(
            verts=[meshes.verts_packed().type(torch.float32)],
            faces=[meshes.faces_packed()],
        )
