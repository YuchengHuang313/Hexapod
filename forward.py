import numpy as np


def forward_kinematics(theta1, theta2, theta3):
    # 定义连杆长度（假设值）
    l1 = 0.5  # 髋关节到膝关节的距离
    l2 = 0.4  # 膝关节到踝关节的距离
    l3 = 0.3  # 踝关节到末端执行器的距离

    # 计算每个关节的位置
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    z1 = 0

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    z2 = 0

    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    z3 = 0

    return np.array([x3, y3, z3])


# 示例关节角度
theta1 = np.deg2rad(30)  # 将角度转换为弧度
theta2 = np.deg2rad(45)
theta3 = np.deg2rad(60)

# 计算末端执行器的位置
position = forward_kinematics(theta1, theta2, theta3)
print("末端执行器的位置：", position)
