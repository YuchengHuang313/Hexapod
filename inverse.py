import numpy as np


def forward_kinematics(theta1, theta2, theta3):
    l1 = 0.5  # 髋关节到膝关节的距离
    l2 = 0.4  # 膝关节到踝关节的距离
    l3 = 0.3  # 踝关节到末端执行器的距离

    x = np.cos(theta1) * (l1 + l3 *
                          np.cos(theta2 + theta3) + l2 * np.cos(theta2))
    y = np.sin(theta1) * (l1 + l3 *
                          np.cos(theta2 + theta3) + l2 * np.cos(theta2))
    z = l3 * np.sin(theta2 + theta3) + l2 * np.sin(theta2)

    return np.array([x, y, z])


def inverse_kinematics(x, y, z):
    l1 = 0.5
    l2 = 0.4
    l3 = 0.3

    # 计算 theta1
    theta1 = np.arctan2(y, x)

    # 计算腿的水平面投影长度
    r = np.sqrt(x**2 + y**2) - l1
    s = z

    # 计算 theta3
    D = (r**2 + s**2 - l2**2 - l3**2) / (2 * l2 * l3)
    theta3 = np.arccos(np.clip(D, -1.0, 1.0))

    # 计算 theta2
    theta2 = np.arctan2(s, r) - np.arctan2(l3 *
                                           np.sin(theta3), l2 + l3 * np.cos(theta3))

    return np.array([theta1, theta2, theta3])


# 示例关节角度
theta1_original = np.deg2rad(30)
theta2_original = np.deg2rad(45)
theta3_original = np.deg2rad(30)

# 正运动学计算末端执行器位置
position = forward_kinematics(
    theta1_original, theta2_original, theta3_original)
print("末端执行器位置：", position)

# 逆运动学计算关节角度
calculated_angles = inverse_kinematics(position[0], position[1], position[2])
print("计算出的关节角度：", np.rad2deg(calculated_angles))

# 比较原始关节角度和计算出的关节角度
print("原始关节角度：", np.rad2deg(
    [theta1_original, theta2_original, theta3_original]))
print("差异：", np.round(np.rad2deg([theta1_original, theta2_original,
      theta3_original]) - np.rad2deg(calculated_angles), 3))
