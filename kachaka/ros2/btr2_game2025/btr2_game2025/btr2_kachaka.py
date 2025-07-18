import kachaka_api
import os
import sys

def main(args=None):
    args = sys.argv
    kachaka_command = args[1]

    kachakaIP = os.getenv('kachaka_IP')
    client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")


    # 関数名に対応する関数の辞書
    kachaka_functions = {
            "speak": kachaka_speak,
            "move_to_pose": kachaka_move_to_pose,
            "set_robot_pose": kachaka_set_robot_pose,
            "return_home": kachaka_return_home,
            "get_battery_info": kachaka_get_battery_info,
    }
 
    # 該当する関数があれば実行
    if kachaka_command in kachaka_functions:
        kachaka_functions[kachaka_command](client, args)
        print(kachaka_command)
    else:
        print(f"Unknown command: {kachaka_command}")

def kachaka_speak(client, kachaka_command):
    print("[kachaka]: ", kachaka_command[2])
    client.speak(kachaka_command[2])

def kachaka_move_to_pose(client, kachaka_command):
    client.move_to_pose(float(kachaka_command[2]), float(kachaka_command[3]), float(kachaka_command[4]))

def kachaka_set_robot_pose(client, kachaka_command):
    client.set_robot_pose({ "x": float(kachaka_command[2]), "y": float(kachaka_command[3]), "theta": float(kachaka_command[4])})

def kachaka_return_home(client, kachaka_command):
    client.return_home()

def kachaka_get_battery_info(client, kachaka_command):
    battery = client.get_battery_info()
    print("[kachaka]: ", battery)
    kachaka_speak(client, ["", "", "バッテリーの残量は，" + str(int(battery[0])) + "%です．"])
    if battery[1] == 1:
        kachaka_speak(client, ["", "", "充電中です．"])
    if battery[1] == 2:
        kachaka_speak(client, ["", "", "放電中です．"]) 


# main
#
if __name__ == '__main__':
    main()
