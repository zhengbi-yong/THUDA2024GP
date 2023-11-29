import mujoco

# MuJoCo 模型文件路径
model_path = '/home/sisyphus/UniDexGrasp/dexgrasp_generation/data/leap_hand/robot.urdf'


# 加载模型
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# 创建一个模拟器窗口来查看模型
while True:
    mujoco.mj_step(model, data)
    # 这里可以添加代码以在视窗中渲染模型，例如使用 OpenGL
