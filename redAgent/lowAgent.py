import math
import json
import struct
from environment.combatEnv import combatEnv_Red
from PID_ctrl.PID_baseControl import FlightData, AutoDriver


# 发送给平台的控制信息
class st_HotasinfoOrder:
    def __init__(self):
        self.strObjID = ""
        self.dwXpos = ""
        self.dwYpos = ""
        self.dwZpos = ""
        self.dwRpos = ""


class lowAgent:
    def __init__(self):
        # 创建与平台的链接
        self.combatEnv_Red = combatEnv_Red()  # 和平台交互的类combatEnv_Red
        self.combatEnv_Red.init_connect()
        self.autoDriver = AutoDriver()  # 掌握多种基础控制PID的类autoDriver,
        # self.autoDriver.flightData中存着当前飞机信息
        self.cur_time = 0
        # self.flightData=FlightData()

    def toFlightData(self, self_aircraft):  #todo：拿到飞机的信息，传到PID？
        flightdata = FlightData()

        flightdata.VE[0] = self_aircraft['V_N']
        flightdata.VE[1] = self_aircraft['V_E']
        flightdata.VE[2] = self_aircraft['V_D']

        flightdata.longitude = self_aircraft['Longitude']
        flightdata.latitude = self_aircraft['Latitude']
        flightdata.altitude = self_aircraft['Altitude']

        flightdata.psi = self_aircraft['Heading']
        flightdata.theta = self_aircraft['Pitch']
        flightdata.phi = self_aircraft['Roll']

        flightdata.alpha = self_aircraft['Alpha']
        flightdata.beta = self_aircraft['Beta']

        p = self_aircraft['p']
        q = self_aircraft['q']
        r = self_aircraft['r']
        flightdata.omega[0] = p + q * math.sin(flightdata.phi) * math.tan(
            flightdata.theta) + r * math.cos(flightdata.phi) * math.tan(flightdata.theta)
        flightdata.omega[1] = q * math.cos(flightdata.phi) - r * math.sin(flightdata.phi)
        flightdata.omega[2] = q * math.sin(flightdata.phi) / math.cos(flightdata.theta) + r * math.cos(
            flightdata.phi) / math.cos(flightdata.theta)
        flightdata.Vc = math.sqrt(
            flightdata.VE[0] * flightdata.VE[0] + flightdata.VE[1] * flightdata.VE[1] + flightdata.VE[
                2] * flightdata.VE[2])
        return flightdata

    def updateFlightData(self, self_aircraft):
        flightdata = self.toFlightData(self_aircraft)
        self.autoDriver.update_output(flightdata)

    # 左转
    def turnLeft(self, roll_exp):

        roll_exp = roll_exp * 1.5
        roll_exp = min(1.4, roll_exp)
        roll_exp = -max(0.5, roll_exp)

        m_rollCtrl = self.autoDriver.rollCtrl(roll_exp)
        return m_rollCtrl

    # 右转
    def turnRight(self, roll_exp):
        roll_exp = roll_exp * 1.5
        roll_exp = min(1.4, roll_exp)
        roll_exp = max(0.5, roll_exp)
        m_rollCtrl = self.autoDriver.rollCtrl(roll_exp)
        return m_rollCtrl

    # #掉头
    def turnHeading(self):
        roll_exp = 1
        m_rollCtrl = self.autoDriver.rollCtrl(roll_exp)
        return m_rollCtrl

    # altiCtrl(self, alti_exp, spd_exp):
    # 升高,降低,加速,减速
    def alti_spd(self, alti_exp, spd_exp):
        m_altiCtrl = self.autoDriver.altiCtrl(alti_exp, spd_exp)
        return m_altiCtrl

    # 分段爬高加速
    def stage_climb(self, alti_exp, spd_exp, alti_cur, spd_cur):
        alti_limit = 2000
        tmp_alti = alti_exp - alti_cur
        if tmp_alti > 2000:
            alti_exp = alti_cur + alti_limit
        if tmp_alti < -2000:
            alti_exp = alti_cur - alti_limit

        spd_limit = 100
        tmp_spd = spd_exp - spd_cur
        if tmp_spd > 100:
            spd_exp = spd_cur + spd_limit
        if tmp_spd < -100:
            spd_exp = spd_cur + spd_limit

        m_altiCtrl = self.autoDriver.altiCtrl(alti_exp, spd_exp)
        return m_altiCtrl

    def transfer_angel(self, x, y):  # 注意.输入是角度制
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)

        # 这里的变换是为了和平台的本机朝向的变化范围相适应
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

        return angle_deg
