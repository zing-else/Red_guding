import socket
import json
import struct

RA_IDENTIFY = ("red", "blue")


class combatEnv_Red(object):
    def __init__(self, port: int = 8868, identify: RA_IDENTIFY = "red"):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = port
        self.identify_dict = {
            "msg_type": "identify",
            "msg_info": {
                "identify": identify
            }
        }
        self.cur_time = 0
        self.rcv_msg_buffer = bytes()

    def init_connect(self):
        try:
            self.client.connect(('127.0.0.1', self.port))
        except ConnectionRefusedError:
            print("仿真平台连接失败！请检查是否正确开始！")
            raise
        else:
            print("连接仿真平台成功！")
        identify = json.dumps(self.identify_dict)  # 将self.identify_dict字典转换为JSON格式的字符串。
        msg_len = len(identify.encode("utf-8"))
        msg_len_data = struct.pack('i', msg_len)  # 使用struct.pack函数将消息长度打包为4字节的二进制数据。
        self.client.send(msg_len_data)
        self.client.send(identify.encode("utf-8"))
        print("平台初始化成功！")

    def dispatch_action(self, action_info):

        # print("蓝传给平台的动作信息：", action_info)

        msg_len = len(json.dumps(action_info).encode("utf-8"))
        msg_len_data = struct.pack('i', msg_len)
        self.client.send(msg_len_data)
        self.client.send(json.dumps(action_info).encode("utf-8"))
        # print('蓝方动作输出完成')

    def getInfo(self):
        cur_msg = self.client.recv(1024 * 10)
        # print(cur_msg)
        self.rcv_msg_buffer = self.rcv_msg_buffer + cur_msg
        total_len = len(self.rcv_msg_buffer)
        while total_len > 4:
            msg_len = int.from_bytes(self.rcv_msg_buffer[0:4], byteorder="little", signed=False)
            if msg_len + 4 <= total_len:
                # 一帧数据
                one_frame_msg = self.rcv_msg_buffer[4:4 + msg_len]
                # 去除一帧后的剩余数据
                self.rcv_msg_buffer = self.rcv_msg_buffer[4 + msg_len: total_len]
                total_len = len(self.rcv_msg_buffer)

                self.recv_msg = json.loads(one_frame_msg)

                # print(self.recv_msg)

                return self.recv_msg["msg_info"], self.recv_msg["msg_type"], self.recv_msg["msg_time"],self.recv_msg
            else:
                break

    def msg_packet_2_observertion_dict(self):
        self_aicraft = {}
        enemy_aircraft = {}
        cur_observation_msg, msg_type, curTime,recv_msg = self.getInfo()
        if msg_type == "result":
            return self_aicraft, enemy_aircraft, msg_type, curTime,recv_msg
        msg_len = len(cur_observation_msg)
        # print(cur_observation_msg)

        msg_len = len(cur_observation_msg)
        # print(cur_observation_msg)
        for index in range(0, msg_len):
            data = cur_observation_msg[index]
            # 探测信息
            if data['data_tp'] == 'DetectedInfo':
                # print(1)
                data_info = data["data_info"]
                data_len = len(data_info)
                for index_1 in range(0, data_len):
                    # print(12)
                    detected_targets = data_info[index_1]
                    if detected_targets['ID'] == '1001':
                        # print(123)
                        # print(detected_targets)
                        detected_1001_info = detected_targets['DetectedTargets']
                        detected_1001_len = len(detected_1001_info)
                        # print(detected_1001_info)
                        for index_2 in range(0, detected_1001_len):
                            one_target = detected_1001_info[index_2]
                            # print("!!!!!",one_target)
                            if one_target['TYPE'] == 'Aircraft':
                                enemy_aircraft['V_N'] = one_target['V_N']
                                enemy_aircraft['V_E'] = one_target['V_E']
                                enemy_aircraft['V_D'] = one_target['V_D']

                                enemy_aircraft['Longitude'] = one_target['Longitude']
                                enemy_aircraft['Latitude'] = one_target['Latitude']
                                enemy_aircraft['Altitude'] = one_target['Altitude']

                                enemy_aircraft['Relative_X'] = one_target['Relative_X']
                                enemy_aircraft['Relative_Y'] = one_target['Relative_Y']
                                enemy_aircraft['Relative_Z'] = one_target['Relative_Z']

                                break
                        break
            # 武器信息
            elif data['data_tp'] == 'WeaponSystem':
                cur_WeaponSystem = data['data_info']
            # 自身航迹信息
            elif data["data_tp"] == 'track':
                tracks = data["data_info"]
                track_num = len(tracks)
                for index in range(0, track_num):
                    cur_track = tracks[index]
                    if cur_track['ID'] == '1001':
                        self_aicraft['V_N'] = cur_track['V_N']
                        self_aicraft['V_E'] = cur_track['V_E']
                        self_aicraft['V_D'] = cur_track['V_D']

                        self_aicraft['Longitude'] = cur_track['Longitude']
                        self_aicraft['Latitude'] = cur_track['Latitude']
                        self_aicraft['Altitude'] = cur_track['Altitude']

                        self_aicraft['Heading'] = cur_track['Heading']
                        self_aicraft['Pitch'] = cur_track['Pitch']
                        self_aicraft['Roll'] = cur_track['Roll']

                        self_aicraft['Alpha'] = cur_track['alpha']
                        self_aicraft['Beta'] = cur_track['beta']

                        self_aicraft['p'] = cur_track['p']
                        self_aicraft['q'] = cur_track['q']
                        self_aicraft['r'] = cur_track['r']
            # 火控信息
            elif data["data_tp"] == "SFC":
                cur_SFC = data["data_info"]
            # 导弹信息
            elif data["data_tp"] == "MissileTrack":
                cur_MissileTrack = data["data_info"]

        return self_aicraft, enemy_aircraft, msg_type, curTime,recv_msg

    def convertToPositionString(self, order):
        positionString = order.dwXpos + "`" + order.dwYpos + "`" + order.dwZpos + "`" + order.dwRpos
        return positionString

    def OrderInfoToString(self, order):
        result = "驾驶操控, " + order.strObjID + ", 2, 0, Delay, Force, 0 | " + self.convertToPositionString(order)
        return result
