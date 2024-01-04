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

        # print("滚转角：", cur_self_roll)
        cur_self_v_n = self_aircraft['V_N']  # 北向速度(m/s)
        cur_self_v_e = self_aircraft['V_E']  # 东向速度(m/s)
        cur_self_v_d = self_aircraft['V_D']  # 地向速度(m/s)
        cur_self_v_real = (
                                  cur_self_v_n * cur_self_v_n + cur_self_v_e * cur_self_v_e + cur_self_v_d * cur_self_v_d) ** 0.5
        cur_self_alt = self_aircraft['Altitude']  # 高度(m)

        cur_enemy_v_n = enemy_aircraft['V_N']  # 北向速度(m/s)
        cur_enemy_v_e = enemy_aircraft['V_E']  # 东向速度(m/s)
        cur_enemy_v_d = enemy_aircraft['V_D']  # 地向速度(m/s) 上下升高的作用
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
        # print(enemy_aircraft)

        cur_escape_angle = math.acos(
            (cur_enemy_v_e * relative_X + cur_enemy_v_n * relative_Y - cur_enemy_v_d * relative_Z) / (
                    cur_deta_range * cur_enemy_v_real))  # [逃逸角]

        ###############################局势解析-end#######################################################################

        ###############################做决策-begin######################################################################

        # 相较于直线，现在对手有更多机动，转弯，高度升降，速度变化
        # 限制空战距离在10km内，即更精细化的机动决策在这一范围内施展
        # 控制三个期望量:高度,速度,转向

        # 高度
        # alti_exp = cur_enemy_alt+cur_enemy_v_d*0.1
        alti_exp = cur_enemy_alt + 10
        alti_exp = max(alti_exp, 3000)
        # alti_exp = min(alti_exp, 8000)

        # 速度
        # 当两机距离大于10km时，应该主要考虑拉近两机距离,这时距离矢量造成的影响大于敌机速度矢量
        if cur_deta_range > 20000:
            spd_exp = cur_enemy_v_real + 100
        elif cur_deta_range >= 10000 and cur_deta_range < 20000:
            # spd_exp=300
            spd_exp = cur_enemy_v_real + 50 + 50 * (cur_deta_range - 10000) / 10000
        elif cur_deta_range >= 5000 and cur_deta_range < 10000:
            spd_exp = cur_enemy_v_real + 20 + 50 * (cur_deta_range - 5000) / 5000
        elif cur_deta_range >= 1000 and cur_deta_range < 5000:
            spd_exp = cur_enemy_v_real + 20 * (cur_deta_range - 1000) / 4000
        else:
            # 两机距离在10km以内，这时应该主要考虑转向的灵敏度,因此需要降速
            # 这里考虑敌机的速度的水平面投影的矢量,以及我机朝向的矢量,来使得速度判定和转向判定更加精准和灵敏
            # 当敌机速度矢量和我方不一致时,敌机速度将不具备唯一参考性
            # x = cur_enemy_v_e  # 向量的 x 分量
            # y = cur_enemy_v_n  # 向量的 y 分量
            # angle_deg = self.lowAgent.transfer_angel(x, y)
            # angle_rad = math.radians(angle_deg)
            spd_exp = cur_enemy_v_real
        spd_exp = max(100, spd_exp)
        spd_exp = min(500, spd_exp)

        # 转向
        x = relative_X  # 向量的 x 分量
        y = relative_Y  # 向量的 y 分量
        angle_deg = self.lowAgent.transfer_angel(x, y)
        angle_rad = math.radians(angle_deg)
        if cur_self_heading == 2 * math.pi:
            cur_self_heading = 0

        if angle_rad > cur_self_heading:
            tmp = angle_rad - cur_self_heading
            if tmp >= math.pi:
                tmp = 2 * math.pi - tmp
                m_rollCtrl = self.lowAgent.turnLeft(tmp)  # 我机朝向矢量+我机和敌机距离矢量=====判定其在左侧
            else:
                m_rollCtrl = self.lowAgent.turnRight(tmp)  # 我机朝向矢量+我机和敌机距离矢量=====判定其在右侧
        elif angle_rad < cur_self_heading:
            tmp = cur_self_heading - angle_rad
            if tmp >= math.pi:
                tmp = 2 * math.pi - tmp
                m_rollCtrl = self.lowAgent.turnRight(tmp)  # 我机朝向矢量+我机和敌机距离矢量=====判定其在右侧
            else:
                m_rollCtrl = self.lowAgent.turnLeft(tmp)  # 我机朝向矢量+我机和敌机距离矢量=====判定其在左侧
        else:
            m_rollCtrl = self.lowAgent.autoDriver.rollCtrl(0)  # 我机朝向矢量+我机和敌机距离矢量=====判定其在正前方

        # 方法1:敌机矢量和在距离矢量上的投影
        # 方法2:考虑另一种思想,当角度差距过大时,且距离近时,降速,来获得一定距离内更大的转弯性能
        # if tmp > math.pi / 6 and cur_deta_range < 20000:
        #     spd_exp = max(100, cur_enemy_v_real - (tmp / math.pi) * 200)

        # # 蓝方飞直线
        # alti_exp=5000
        # spd_exp=200
        # m_rollCtrl = self.lowAgent.autoDriver.rollCtrl(0)
        # print("tmp************************************",tmp)
        # print("alti_exp",alti_exp)
        # print("spd_exp",spd_exp)

        # # 分段爬高加速
        # def stage_climb(self, alti_exp, spd_exp, alti_cur, spd_cur):
        # 把高度变化限制在10，速度变化限制在2，但是这只是每一帧的
        m_pidtrl = self.lowAgent.stage_climb(alti_exp, spd_exp, cur_self_alt, cur_self_v_real)

        # 0`0`0`1 四个值依次对应[纵向杆位移、横向杆位移、油门位移、脚蹬位移]XYZR
        # 高度控制
        # def altiCtrl(self, alti_exp, spd_exp):
        if cur_self_alt < 3000:
            if cur_self_alt < 2000:
                if cur_self_v_real < 300:
                    m_pidtrl = self.lowAgent.autoDriver.altiCtrl(2000, min(cur_self_v_real + 50, 300))
                    # m_pidtrl.dwZpos=0.5
                    # m_pidtrl.dwXpos = -0.1
                    m_rollCtrl = self.lowAgent.autoDriver.rollCtrl(0.01)
                else:
                    m_pidtrl = self.lowAgent.autoDriver.altiCtrl(min(cur_self_alt + 500, 3000), cur_self_v_real)
                    m_rollCtrl = self.lowAgent.autoDriver.rollCtrl(-0.01)

        # m_pidtrl.dwXpos = -0.1
        # m_pidtrl.dwZpos = 0.5
        m_pidtrl.dwYpos = m_rollCtrl.dwYpos
        # m_pidtrl.dwYpos = 0
        # m_pidtrl.dwRpos = 0

        return m_pidtrl

    ###############################做决策-end######################################################################
