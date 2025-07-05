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
    }
 
    # 該当する関数があれば実行
    if kachaka_command in kachaka_functions:
        kachaka_functions[kachaka_command](client, args)
    else:
        print(f"Unknown command: {kachaka_command}")

def kachaka_speak(client, kachaka_command):
    client.speak(kachaka_command[2])

def kachaka_move_to_pose(client, kachaka_command):
    client.move_to_pose(float(kachaka_command[2]), float(kachaka_command[3]), float(kachaka_command[4]))


# main
#
if __name__ == '__main__':
    main()
