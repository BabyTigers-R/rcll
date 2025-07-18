import kachaka_api
import os

kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")

def get_target_pose():
    """
    目的地の座標を返す関数（例: (x, y, θ)）
    将来的に外部から取得する設計も可能。
    """
    # サンプル: 目的地
    return 0.5, 0.0, 0.0

def move_kachaka_to_pose(x: float, y: float, theta: float):
    """
    指定座標にカチャカを移動させる。
    """
    current_pose = client.get_robot_pose()
    print(f"[INFO] 現在位置: {current_pose}")
    
    client.speak("カチャカ、目的地に向かいます")
    client.move_to_pose(x, y, theta)
    print(f"[INFO] 指定座標 ({x:.2f}, {y:.2f}, {theta:.2f}) に移動しました")

def main():
    # navigatorから目標座標を取得
    x, y, theta = get_target_pose()

    # controllerでカチャカを移動
    move_kachaka_to_pose(x, y, theta)

if __name__ == "__main__":
    main()

