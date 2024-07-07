from sympy import symbols, Eq, sin, cos, solve, Interval, pi
from math import atan2, degrees

def position_callback():
    # 从键盘接收输入
    x = float(input("请输入x的值:"))
    y = float(input("请输入y的值:"))
    z = float(input("请输入z的值:"))

    huizhuan = degrees(atan2(y, x))
    R = (((x**2) + (y**2))**0.5)

    # 定义未知数，x1是大臂角度，x2是小臂角度
    x1, x2 = symbols('x1 x2')

    # 定义方程
    eq1 = Eq(116.7 - 41 + 175*sin(x1) - 150*sin(x2), z)
    eq2 = Eq(27.9 + 175*cos(x1) + 150*cos(x2) + 146, R)

    # 解方程
    solution = solve([eq1, eq2], (x1, x2))

    # 检查解
    if not solution:
        print("方程无解。")
    else:
        print(degrees(solution[0][0]),degrees(solution[0][1]),huizhuan)

# 调用函数
position_callback()