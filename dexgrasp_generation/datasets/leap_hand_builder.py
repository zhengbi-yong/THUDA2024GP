import os

import numpy as np
import pytorch_kinematics as pk
import torch
import trimesh
from pytorch3d.structures import Meshes, join_meshes_as_batch


class LeapHandBuilder:
    joint_names = [
        "0",
        "1",
        "2",
        "3",
        "4",
        "5",
        "6",
        "7",
        "8",
        "9",
        "10",
        "11",
        "12",
        "13",
        "14",
        "15",
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
        print(f"LeapHandBuilder接收到的urdf_path: {urdf_path}")
        self.chain = pk.build_serial_chain_from_urdf(
            open(urdf_path).read(), end_link_name="fingertip"
        ).to(dtype=torch.float)
        self.mesh_dir = mesh_dir
        self.mesh = {}
        self._build_meshes()

    def _build_meshes(self):
        device = "cpu"
        self._build_meshes_recurse(self.chain._root, device)

    def _build_meshes_recurse(self, root_frame, device):
        for child in root_frame.children:
            if child.name in LeapHandBuilder.joint_names:
                # 假设网格文件名与关节名称有一定的对应关系，或者有其他逻辑来确定具体的网格文件名
                mesh_filename = self._get_mesh_filename_for_joint(child.name)
                print(f"关节名称：{child.name}，网格文件名：{mesh_filename}")
                if mesh_filename:
                    full_mesh_path = os.path.join(self.mesh_dir, mesh_filename)
                    print(f"文件路径：{full_mesh_path}")

                    # 加载网格文件
                    link_mesh = trimesh.load(full_mesh_path, process=False)
                    vertices = torch.tensor(
                        link_mesh.vertices, dtype=torch.float, device=device
                    )
                    faces = torch.tensor(
                        link_mesh.faces, dtype=torch.long, device=device
                    )

                    # 存储网格信息
                    self.mesh[child.name] = {"vertices": vertices, "faces": faces}

                self._build_meshes_recurse(child, device)

    def _get_mesh_filename_for_joint(self, joint_name):
        # 这里需要实现逻辑来根据关节名称选择正确的网格文件名
        # 以下是一个简化的示例，具体实现可能会更复杂
        return {
            "joint0": "dip.stl",
            # 添加其它关节名称到网格文件名的映射
        }.get(joint_name)

    def qpos_to_qpos_dict(self, qpos, hand_qpos_names=None):
        if hand_qpos_names is None:
            hand_qpos_names = LeapHandBuilder.joint_names
        assert len(qpos) == len(hand_qpos_names)
        return dict(zip(hand_qpos_names, qpos))

    def qpos_dict_to_qpos(self, qpos_dict):
        hand_qpos_names = [f"{i}" for i in range(16)]  # Assuming 16 joints
        missing_joints = [name for name in hand_qpos_names if name not in qpos_dict]
        if missing_joints:
            raise ValueError(f"Missing joint data for: {missing_joints}")
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
