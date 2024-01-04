import math


# 写奖励函数
class ObsToState:
    def __init__(self):
        self.done = 0
        self.theta = 0
        self.count = 0
        self.pre_state = 0
        self.maxCount=0

    def get_state_reward(self, self_aircraft, exp_info, curtime):  # 俯仰角为角度制
        abs_info = math.fabs(self_aircraft['Pitch'] - exp_info)  # 这里是否需要考虑角度的情况 180度之类的
        obs_state = [self_aircraft['Pitch'] / math.pi,
                     abs_info / math.pi]
        # print("当前高度：",flightData.altitude)
        if curtime > 7200:
            self.done = 3
            print("时间达到400s")
            cur_step_reward_total = 30
            return obs_state, cur_step_reward_total, self.done
        if self_aircraft['Altitude'] < 1000 or self_aircraft['Altitude'] > 30000:
            self.done = 2
            cur_step_reward_total = -50
            # print("达到的最大帧数：", self.maxCount)
            self.maxCount = 0
            # print("当前高度越界，高度为：", self_aircraft['Altitude'], "存活时间为：", curtime)
            return obs_state, cur_step_reward_total, self.done
        # if abs_info <= 0.005:  # 加上达到目标的持续时间且持续时间越长，奖励越大
        #     self.count += 1
        #
        #     if self.count>self.maxCount:
        #         self.maxCount=self.count
        #         print("达到的最大帧数：",self.maxCount)
        #
        #     if self.count > 9:  # 持续时间
        #         self.done = 1
        #         cur_step_reward_total = 50 + self.count * 0.1
        #         print("达到期望的俯仰角", exp_info, "累计帧数", self.count)
        #         self.maxCount=0
        #         return obs_state, cur_step_reward_total, self.done
        #     self.pre_state = 1
        #
        # if abs_info > 0.005:
        #     self.pre_state = 0
        #     self.count = 0


        Reward_theta = -2 * math.log(math.fabs(abs_info) * 200 + 1 / math.e, math.e)  # 这里奖励函数应该再看看怎么写
        cur_step_reward_total = Reward_theta * 0.001
        return obs_state, cur_step_reward_total, self.done
