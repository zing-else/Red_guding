import math
from redAgent.lowAgent import st_HotasinfoOrder, lowAgent
# from redAgent.highAgent import highAgent
from redAgent.highAgent import highAgent
from obsState import ObsToState
import random

if __name__ == '__main__':
    Episode = 10000
    hotas = st_HotasinfoOrder()
    # m_dispatch = lowAgent()
    agent = highAgent()
    obs = ObsToState()
    exp_theta = 0.17  # 所有参数都是弧度制
    # action_msg = {
    #     "msg_info": "驾驶操控,1001,2,0,Delay,Force,0|0`0`0.8`0",
    #     "msg_type": "manu_ctrl",
    #     "done": "0"
    # }
    # action_msg = {
    #     "msg_info": ["驾驶操控,1001,2,0,Delay,Force,0|0`0`0.8`0"],
    #     "msg_type": "manu_ctrl",
    #     "done": "0"
    # }

    for i in range(Episode):
        print("第", i + 1, "局开始")

        action_msg = {
            "msg_info": ["驾驶操控,1001,2,0,Delay,Force,0|0`0`0.8`0"],
            "msg_type": "manu_ctrl",
            "done": "0"
        }
        done = 0
        total_reward = 0
        # action = [1, 0, -5, 0]  # 初始化四个参数值，action[0]=[0,2],action[1]=[-1,1],action[2]=[-6,-4],action[3]=[-1,1]
        obs.done = 0
        jieshu = 0
        curTime = 0

        """初始化状态信息"""
        agent.lowAgent.combatEnv_Red.dispatch_action(action_msg)  # 最开始初始化飞机的状态信息
        self_aicraft, enemy_aircraft, msg_type, curTime,recv_msg = agent.lowAgent.combatEnv_Red.msg_packet_2_observertion_dict()  # 将接收的信息格式化
        # print("heading", self_aircraft['Heading'])

        while jieshu == 0:  # 要仔细想想这个逻辑顺序
            """获取到动作之后，传入控制函数，得到新的动作指令"""
            # m_ctrlinfo_thetactrl = m_dispatch.autoDriver.thetaCtrl(exp_theta, action[0], action[1], action[2],
            #                                                          action[3])  # 加上限制范围
            # psiCtrl(self, psi_ctrl, alt_exp, spd_exp):
            agent.lowAgent.combatEnv_Red.dispatch_action(action_msg)  # 传动作
            self_aicraft, enemy_aircraft, msg_type, curTime,recv_msg = agent.lowAgent.combatEnv_Red.msg_packet_2_observertion_dict()  # 接收数据
            if msg_type == "result":
                if recv_msg["msg_info"]["result"] != 0:
                    jieshu = 1
                    break

            # 判定空包
            if recv_msg["msg_info"] == []:
                action_msg = {
                    "msg_info": ["驾驶操控,1002,2,0,Delay,Force,0|0`0`0.8`0"],
                    "msg_type": "manu_ctrl",
                    "done": "0"
                }
                continue

            agent.lowAgent.updateFlightData(self_aicraft)  # 更新PID的数据

            # psiCtrl(self, psi_ctrl, alt_exp, spd_exp):
            # +左转 -右转
            # psi_ctrl = -0.51
            # spd_exp = 200
            # alt_exp = 5000
            # altiCtrl(self, alti_exp, spd_exp):
            # hotas.strObjID = "1001"
            # m_ctrlinfo_altiCtrl = agent.lowAgent.autoDriver.altiCtrl(alt_exp, spd_exp)

            m_ctrlinfo = agent.action(self_aicraft, enemy_aircraft)
            hotas.strObjID = "1001"
            hotas.dwXpos = str(m_ctrlinfo.dwXpos)
            hotas.dwYpos = str(m_ctrlinfo.dwYpos)
            hotas.dwZpos = str(m_ctrlinfo.dwZpos)
            hotas.dwRpos = str(m_ctrlinfo.dwRpos)

            # radius = 1
            # g = 9.7
            # +右转 -左转
            # roll_exp = -1
            # m_ctrlinfo_rollCtrl = agent.lowAgent.autoDriver.rollCtrl(roll_exp)
            # m_ctrlinfo = m_ctrlinfo_altiCtrl
            # hotas.dwYpos = str(m_ctrlinfo.dwYpos)
            # print(hotas.dwXpos, "+", hotas.dwYpos, "+", hotas.dwZpos, "+", hotas.dwRpos)

            action_msg["msg_info"] = agent.lowAgent.combatEnv_Red.OrderInfoToString(hotas)
            action_msg["msg_info"] = [action_msg["msg_info"]]
            """执行动作之后，飞机的状态信息会相应改变，此时获取奖励值"""

            next_state, reward, done = obs.get_state_reward(self_aicraft, exp_theta, curTime)

            action_msg["done"] = done
            # if done == 2:
            #     i = 0

            # action_msg["done"] = done
            # agent.lowAgent.do_process_ready_read(action_msg)  # 执行动作 改变飞机的数据信息

        print("第", i + 1, "局结束,当前奖励值", total_reward)
