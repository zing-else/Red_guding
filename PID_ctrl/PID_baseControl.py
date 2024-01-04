import math
import sys
from PID_ctrl.PID_definition import PID


class CtrlInfo:
    def __init__(self, m_dwXpos=0.0, m_dwYpos=0.0, m_dwRpos=0.0, m_dwZpos=0.0):
        self.dwXpos = m_dwXpos
        self.dwYpos = m_dwYpos
        self.dwRpos = m_dwRpos
        self.dwZpos = m_dwZpos


class FlightData:
    def __init__(self):
        self.latitude = 0   #纬度
        self.longitude = 0  #经度
        self.altitude = 0   #海拔高度
        self.theta = 0      #俯仰角
        self.phi = 0        #滚转角
        self.psi = 0        #偏航角
        self.alpha = 0      #攻角 机翼的前进方向(相当与气流的方向)和翼弦(与机身轴线不同)的夹角叫迎角，也称为攻角，（往上飞所遇到的气流阻力）
        self.beta = 0       #滑移角 左右飞遇到的气流阻力
        self.omega = [0, 0, 0]      #飞机角速度，包含x,y,z三个方向
        self.VE = [0, 0, 0]     #飞机的速度，包含N（北向）、E（东向）、D（地向）的速度分量
        self.Vc = 0         #飞机的空速（总速度）

#todo：飞机控制，给定一些初始值，后序具体实现
class AutoDriver:
    def __init__(self):
        self.flightData = FlightData()
        #Pitch（俯仰）控制器 m控制俯仰角
        self.m_fPitchCtrl_m_fPitch_u = PID()
        self.m_fPitchCtrl_m_fPitch_u.setParam(1, 0, 0, 1)
        self.m_fPitchCtrl_m_fPitch_u.setLimits(-20, 20)
        
        self.m_fPitchCtrl_u_ctrl = PID()
        self.m_fPitchCtrl_u_ctrl.setParam(-5, 0, 0, 1)
        self.m_fPitchCtrl_u_ctrl.setLimits(-1, 1)
        #Roll（滚转）控制器 u控制滚转角
        self.rollCtrl_roll_u = PID()
        self.rollCtrl_roll_u.setParam(2, 0, 0, 1)
        self.rollCtrl_roll_u.setLimits(-9, 9)

        self.rollCtrl_u_ctrl = PID()
        self.rollCtrl_u_ctrl.setParam(-1, 0, 0, 1)
        self.rollCtrl_u_ctrl.setLimits(-1, 1)
        #Slip（滑移）控制器
        self.slipCtrl_slipPID = PID()
        self.slipCtrl_slipPID.setParam(0.5, 0.00001, 0, 1)
        self.slipCtrl_slipPID.setLimits(-1, 1)

        self.acceSpdCtrl_u_ctrl = PID()
        self.acceSpdCtrl_u_ctrl.setParam(0.5, 0.00001, 0, 1)
        self.acceSpdCtrl_u_ctrl.setLimits(0, 1)

        self.vertSpdCtrl_Vspd_u = PID()
        self.vertSpdCtrl_Vspd_u.setParam(0.040, 0.0002, 0, 1)
        self.vertSpdCtrl_Vspd_u.setLimits(-1.57, 1.57)
        #高度控制器
        self.altiCtrl_alt_u = PID()
        self.altiCtrl_alt_u.setParam(0.005, 0.00001, 0.09, 1)
        self.altiCtrl_alt_u.setLimits(-1.57, 1.57)

        self.m_fPitchCtrlSpd_spd_m_fPitch = PID()
        self.m_fPitchCtrlSpd_spd_m_fPitch.setParam(0.30, 0, 0.5, 1)
        self.m_fPitchCtrlSpd_spd_m_fPitch.setLimits(-1.57, 1.57)
        #偏航角控制器
        self.psiCtrl_psi_roll = PID()
        self.psiCtrl_psi_roll.setParam(0.13, 0, 0.1, 1)
        self.psiCtrl_psi_roll.setLimits(-9, 9)

    def update_output(self, flightData):
        self.flightData = flightData

    def thetaCtrl(self, theta_exp):
        #俯仰角控制
        ctrl = CtrlInfo()
        self.m_fPitchCtrl_m_fPitch_u.setPid(theta_exp, self.flightData.theta)
        uc = self.m_fPitchCtrl_m_fPitch_u.update()

        self.m_fPitchCtrl_u_ctrl.setPid(uc, self.flightData.omega[1])
        ctrl.dwXpos = self.m_fPitchCtrl_u_ctrl.update()

        return ctrl

    def rollCtrl(self, roll_exp, params: list = []):
        # 滚转角控制
        if len(params) != 0:
            self.rollCtrl_roll_u.setParam(params[0], params[1], params[2], 1)
            self.rollCtrl_u_ctrl.setParam(params[3], params[4], params[5], 1)
        ctrl = CtrlInfo()

        self.rollCtrl_roll_u.setPid(roll_exp, self.flightData.phi)
        uc = self.rollCtrl_roll_u.update()

        self.rollCtrl_u_ctrl.setPid(uc, self.flightData.omega[0])
        ctrl.dwYpos = self.rollCtrl_u_ctrl.update()
        return ctrl

    def slipCtrl(self, slip_exp):
        # 侧滑角控制
        ctrl = CtrlInfo()

        self.slipCtrl_slipPID.setPid(slip_exp, self.flightData.beta)
        ctrl.dwRpos = self.slipCtrl_slipPID.update()
        return ctrl
        # pass

    def vertSpdCtrl(self, vert_spd_exp, spd_exp):
        # 垂直速度控制
        ctrl = CtrlInfo()

        self.vertSpdCtrl_Vspd_u.setPid(vert_spd_exp, self.flightData.VE[2])
        u_m_fPitch = -self.vertSpdCtrl_Vspd_u.update()
        ctrl.dwXpos = self.thetaCtrl(u_m_fPitch).dwXpos
        ctrl.dwZpos = self.acceSpdCtrl(spd_exp, u_m_fPitch).dwZpos
        return ctrl

    def altiCtrl(self, alti_exp, spd_exp):
        # 高度控制
        ctrl = CtrlInfo()

        self.altiCtrl_alt_u.setPid(alti_exp, self.flightData.altitude)
        u_m_fPitch = self.altiCtrl_alt_u.update()
        ctrl.dwXpos = self.thetaCtrl(u_m_fPitch).dwXpos
        ctrl.dwZpos = self.acceSpdCtrl(spd_exp, u_m_fPitch).dwZpos
        ctrl.dwYpos = self.rollCtrl(0).dwYpos
        # print(ctrl.dwYpos)
        return ctrl

    def thetaCtrlSpd(self, spd_exp):
        # 俯仰控制速度
        ctrl = CtrlInfo()

        self.m_fPitchCtrlSpd_spd_m_fPitch.setPid(spd_exp, self.flightData.Vc)
        theta_exp = -self.m_fPitchCtrlSpd_spd_m_fPitch.update()
        ctrl.dwXpos = self.thetaCtrl(theta_exp).dwXpos
        return ctrl

    def acceSpdCtrl(self, spd_exp, theta_exp):
        # 油门控制速度
        ctrl = CtrlInfo()

        self.acceSpdCtrl_u_ctrl.setPid(spd_exp, self.flightData.Vc)
        ctrl.dwZpos = self.acceSpdCtrl_u_ctrl.update()
        ctrl.dwXpos = self.thetaCtrl(theta_exp).dwXpos
        return ctrl

    def oilCtrl(self, spd_exp):
        # 油门控制速度
        ctrl = CtrlInfo()

        self.acceSpdCtrl_u_ctrl.setPid(spd_exp, self.flightData.Vc)
        ctrl.dwZpos = self.acceSpdCtrl_u_ctrl.update()
        return ctrl

    def psiCtrl(self, psi_ctrl, alt_exp, spd_exp):
        # 偏航角控制
        ctrl = CtrlInfo()

        self.psiCtrl_psi_roll.setPid(psi_ctrl, self.flightData.psi)
        roll_exp = self.psiCtrl_psi_roll.update()
        ctrl.dwYpos = self.rollCtrl(roll_exp).dwYpos
        ctrl.dwRpos = self.slipCtrl(0).dwRpos
        ctrl_ = self.altiCtrl(alt_exp, spd_exp)
        ctrl.dwXpos = ctrl_.dwXpos
        ctrl.dwZpos = ctrl_.dwZpos
        return ctrl

    def rasCtrl(self, roll_exp, alt_exp, spd_exp):
        ctrl = CtrlInfo()

        ctrl.dwYpos = self.rollCtrl(roll_exp).dwYpos
        ctrl.dwRpos = self.slipCtrl(0).dwRpos
        ctrl_ = self.altiCtrl(alt_exp, spd_exp)
        ctrl.dwXpos = ctrl_.dwXpos
        ctrl.dwZpos = ctrl_.dwZpos
        return ctrl

    def reset_theta(self):
        self.m_fPitchCtrl_m_fPitch_u.reset()
        self.m_fPitchCtrl_u_ctrl.reset()

    def reset_roll(self):
        self.rollCtrl_roll_u.reset()
        self.rollCtrl_u_ctrl.reset()

    def reset_slip(self):
        self.slipCtrl_slipPID.reset()

    def reset_oil(self):
        self.acceSpdCtrl_u_ctrl.reset()
        self.reset_theta()

    def reset_vert(self):
        self.vertSpdCtrl_Vspd_u.reset()
        self.reset_theta()
        self.reset_oil()

    def reset_alti(self):
        self.vertSpdCtrl_Vspd_u.reset()
        self.reset_theta()
        self.reset_oil()

    def reset_theta_spd(self):
        self.m_fPitchCtrlSpd_spd_m_fPitch.reset()
        self.reset_theta()

    def reset_psi(self):
        self.psiCtrl_psi_roll.reset()
        self.reset_roll()
        self.reset_alti()
        self.reset_slip()

    def reset_ras(self):
        self.reset_roll()
        self.reset_alti()
        self.reset_slip()

    def reset_all(self):
        self.m_fPitchCtrl_m_fPitch_u.reset()
        self.m_fPitchCtrl_u_ctrl.reset()
        self.rollCtrl_roll_u.reset()

        self.rollCtrl_u_ctrl.reset()

        self.slipCtrl_slipPID.reset()
        self.acceSpdCtrl_u_ctrl.reset()

        self.vertSpdCtrl_Vspd_u.reset()

        self.altiCtrl_alt_u.reset()

        self.m_fPitchCtrlSpd_spd_m_fPitch.reset()

        self.psiCtrl_psi_roll.reset()
