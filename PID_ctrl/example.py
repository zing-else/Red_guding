#
# CtrlInfo  testFun()
# {
#     CtrlInfo  m_ctrl;
# double  radius = 1;
# const double g = 9.7;
# / /固定半径转弯
# static PID  m_radi;
# m_radi.setParam(1, 0, 0, 1);
# m_radi.setLimits(-1.57, 1.57);
# / /auto roll_exp = m_radi.update();
#
# auto  roll_exp = 35 * atan2(flightData.Vc * flightData.Vc / radius ,  g) / 180 * 3.14;
# std::cout << "roll_exp: " << roll_exp << std::endl;
# m_ctrl.dwYpos = rollCtrl(roll_exp).dwYpos;
#
# m_ctrl.dwRpos = slipCtrl(0).dwRpos;
# / /for test
# static auto altitude_exp = flightData.altitude;
# static auto spd_exp = flightData.Vc;
# auto ctrl_ = altiCtrl(altitude_exp, spd_exp);
# m_ctrl.dwXpos = ctrl_.dwXpos;
# m_ctrl.dwZpos = ctrl_.dwZpos;
#
# return m_ctrl;
#
#
#
# }

import math

x = -4  # 向量的 x 分量
y = 4  # 向量的 y 分量

angle_rad = math.atan2(y, x)
angle_deg = math.degrees(angle_rad)

if x >= 0 and y >= 0:  # 第一象限
    angle_deg = 90 - angle_deg
elif x < 0 and y > 0:  # 第二象限
    angle_deg = 360+90 - angle_deg
elif x < 0 and y == 0:
    angle_deg = 270
elif x < 0 and y < 0:  # 第三象限
    angle_deg = 90 - angle_deg
elif x >= 0 and y < 0:  # 第四象限
    angle_deg = 90 - angle_deg

print("夹角（度数）：", angle_deg)
