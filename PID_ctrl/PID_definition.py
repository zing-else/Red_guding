import math
import sys

class PID:
    def __init__(self, m_kp=0, m_ki=0, m_kd=0, m_dt=1):
        self.kp = m_kp  #比例
        self.ki = m_ki  #积分
        self.kd = m_kd  #微分项增益
        self.dt = m_dt  #采样时间间隔

        self.integral = 0   #积分项累积
        self.itag = True    #是否启用积分项
        self.out = 0    #控制器输出

        # Physical limits
        self.low_limit = -sys.float_info.max    #输出的上下限
        self.high_limit = sys.float_info.max

        # Derivative parameters
        self.preerror = 0   #上一次的误差，用于计算微分项
        self.alpha = 0  #微分项参数

        self.setPoint = 0.0 #期望值
        self.value = 0.0    #实际值

    def setLimits(self, low, high):
        self.low_limit = low
        self.high_limit = high

    def setFilter(self, m_alpha):
        self.alpha = m_alpha

    def setPid(self, m_setPoint, m_value):
        self.setPoint = m_setPoint
        self.value = m_value

    def setParam(self, m_kp, m_ki, m_kd, m_dt):
        self.kp = m_kp
        self.ki = m_ki
        self.kd = m_kd
        self.dt = m_dt

    def update(self):
        error = self.setPoint - self.value
        P = self.getP(error)
        I = self.getI(error)
        D = self.getD(error)
        self.out = P + I + D
        adjustout = self.adjustedOut(self.out)
        # print("pidout :", self.out, "adjustout:", adjustout)
        return adjustout

    def getP(self, error):
        return self.kp * error

    def getI(self, error):
        self.setITag(error)
        if self.itag:
            self.integral += self.ki * error * self.dt
        return self.integral

    def getD(self, error):
        filtered_D = 0.0
        tmp = self.kd * (error - self.preerror) / self.dt
        self.preerror = error
        # Add low-pass filter
        filtered_D = self.alpha * filtered_D + (1 - self.alpha) * tmp
        return filtered_D

    #输出超过上下限时禁用积分项
    def setITag(self, error):
        if self.out > self.high_limit or self.out < self.low_limit:
            if (error > 0 and self.out > 0) or (error < 0 and self.out < 0):
                self.itag = False
                return
        self.itag = True

    def adjustedOut(self, m_out):
        if m_out > self.high_limit:
            return self.high_limit
        elif m_out < self.low_limit:
            return self.low_limit
        else:
            return m_out

    def reset(self):
        self.integral = 0
        self.itag = True
        self.preerror = 0
        self.alpha = 0