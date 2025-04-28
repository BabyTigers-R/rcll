from kachaka_api import KachakaApiClient

client = KachakaApiClient(target="192.168.18.30:26400")

def move_kachaka_to_pose(x: float, y: float, theta: float):
    """
    指定座標にカチャカを移動させる。
    """
    current_pose = client.get_robot_pose()
    print(f"[INFO] 現在位置: {current_pose}")
    
    client.speak("カチャカ、目的地に向かいます")
    client.move_to_pose(x, y, theta)
    print(f"[INFO] 指定座標 ({x:.2f}, {y:.2f}, {theta:.2f}) に移動しました")