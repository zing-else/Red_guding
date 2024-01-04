import math

from PID_ctrl.PID_baseControl import FlightData
from redAgent.lowAgent import lowAgent


class highAgent:
    def __init__(self):
        self.lowAgent = lowAgent()  # 创建 lowAgent 对象
        self.m_state = 0

    def action(self, self_aircraft, enemy_aircraft):

        ###############################局势解析-begin####################################################################
        self_aircraft, enemy_aircraft = self_aircraft, enemy_aircraft
        # 根据敌我高度，升降
        # 根据敌我速度距离，加减
        # 根据敌机在左右加减，攻击角
        cur_self_heading = self_aircraft['Heading']  # 朝向
        cur_self_pitch = self_aircraft['Pitch']  # 俯仰
        cur_self_roll = self_aircraft['Roll']  # 滚转


        print("滚转角：",cur_self_roll)

        cur_self_v_n = self_aircraft['V_N']  # 北向速度(m/s)
        cur_self_v_e = self_aircraft['V_E']  # 东向速度(m/s)
        cur_self_v_d = self_aircraft['V_D']  # 地向速度(m/s)
        cur_self_v_real = (
                                  cur_self_v_n * cur_self_v_n + cur_self_v_e * cur_self_v_e + cur_self_v_d * cur_self_v_d) ** 0.5
        cur_self_alt = self_aircraft['Altitude']  # 高度(m)

        cur_enemy_v_n = enemy_aircraft['V_N']  # 北向速度(m/s)
        cur_enemy_v_e = enemy_aircraft['V_E']  # 东向速度(m/s)
        cur_enemy_v_d = enemy_aircraft['V_D']  # 地向速度(m/s)
        cur_enemy_v_real = (
                                   cur_enemy_v_n * cur_enemy_v_n + cur_enemy_v_e * cur_enemy_v_e + cur_enemy_v_d * cur_enemy_v_d) ** 0.5
        cur_enemy_alt = enemy_aircraft['Altitude']  # 高度(m)

        relative_X = enemy_aircraft["Relative_X"]  # 考虑通过经纬度换算
        relative_Y = enemy_aircraft["Relative_Y"]  # 考虑通过经纬度换算
        relative_Z = enemy_aircraft["Relative_Z"]  # 考虑通过经纬度换算

        # print("X,Y:", relative_X, "xy", relative_Y)

        # 两机距离
        cur_deta_range = (relative_X * relative_X + relative_Y * relative_Y + relative_Z * relative_Z) ** 0.5

        cur_attack_angle = math.acos(
            (cur_self_v_e * relative_X + cur_self_v_n * relative_Y - cur_self_v_d * relative_Z) / (
                    cur_deta_range * cur_self_v_real))  # 攻击角

        # print(cur_deta_range)
        # print(cur_enemy_v_real)

        cur_escape_angle = math.acos(
            (cur_enemy_v_e * relative_X + cur_enemy_v_n * relative_Y - cur_enemy_v_d * relative_Z) / (
                    cur_deta_range * cur_enemy_v_real))  # [逃逸角]

        ###############################局势解析-end#######################################################################

        ###############################做决策-begin######################################################################

        # 相较于直线，现在对手有更多机动，转弯，高度升降，速度变化
        # 限制空战距离在10km内，即更精细化的机动决策在这一范围内施展



        # print("cur_attack_angle",cur_attack_angle)
        # print(math.pi / 2)
        x = relative_X  # 向量的 x 分量
        y = relative_Y  # 向量的 y 分量

        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)

        # 调整夹角范围
        if x >= 0 and y >= 0:  # 第一象限
            angle_deg = 90 - angle_deg
        elif x < 0 and y > 0:  # 第二象限
            angle_deg = 360 + 90 - angle_deg
        elif x < 0 and y == 0:
            angle_deg = 270
        elif x < 0 and y < 0:  # 第三象限
            angle_deg = 90 - angle_deg
        elif x >= 0 and y < 0:  # 第四象限
            angle_deg = 90 - angle_deg

        angle_rad = math.radians(angle_deg)
        # print(angle_rad, "dddd", cur_self_heading)
        if cur_self_heading == 2 * math.pi:
            cur_self_heading = 0
        # angle_rad距离矢量与y轴正向的夹角
        # cur_self_heading当前本机朝向
        if angle_rad > cur_self_heading:
            tmp = angle_rad - cur_self_heading
            if tmp >= math.pi:
                m_rollCtrl = self.lowAgent.turnLeft(tmp)
            else:
                m_rollCtrl = self.lowAgent.turnRight(tmp)
        elif angle_rad < cur_self_heading:
            tmp = cur_self_heading - angle_rad
            if tmp >= math.pi:
                m_rollCtrl = self.lowAgent.turnRight(tmp)
            else:
                m_rollCtrl = self.lowAgent.turnLeft(tmp)
        else:
            m_rollCtrl = self.lowAgent.autoDriver.rollCtrl(0)

        alti_exp = cur_enemy_alt
        # print("当前距离:",cur_deta_range)
        if cur_deta_range > 20000:
            spd_exp = cur_enemy_v_real + 100
        elif cur_deta_range > 10000:
            # spd_exp=300
            spd_exp = cur_enemy_v_real + 50 + 50 * (cur_deta_range - 10000) / 10000
        elif cur_deta_range > 5000 and cur_deta_range <= 10000:
            # spd_exp = 250
            spd_exp = cur_enemy_v_real + 20 + 30 * (cur_deta_range - 5000) / 5000
        elif cur_deta_range > 1000 and cur_deta_range <= 5000:
            # spd_exp = 200
            spd_exp = cur_enemy_v_real + 20 * (cur_deta_range - 1000) / 2000
        else:
            spd_exp = cur_enemy_v_real
        m_altiCtrl = self.lowAgent.autoDriver.altiCtrl(alti_exp, spd_exp)
        m_pidtrl = m_altiCtrl
        m_pidtrl.dwYpos = m_rollCtrl.dwYpos

        return m_pidtrl

    ###############################做决策-end######################################################################
