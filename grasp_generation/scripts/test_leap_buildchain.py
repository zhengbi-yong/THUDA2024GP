import json
import os

import numpy as np
import plotly.graph_objects as go
import pytorch3d.ops
import pytorch3d.structures

# from utils.rot6d import robust_compute_rotation_matrix_from_ortho6d
import pytorch_kinematics as pk
import torch
import trimesh as tm
from torchsdf import compute_sdf, index_vertices_by_faces

urdf_path = "leaphand/robot.urdf"
device = "cpu"
chain = pk.build_chain_from_urdf(open(urdf_path).read()).to(
    dtype=torch.float, device=device
)
print(type(chain))
print(chain)
