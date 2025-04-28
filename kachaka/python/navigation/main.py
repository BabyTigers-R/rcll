from navigator import get_target_pose
from controller import move_kachaka_to_pose

def main():
    # navigatorから目標座標を取得
    x, y, theta = get_target_pose()

    # controllerでカチャカを移動
    move_kachaka_to_pose(x, y, theta)

if __name__ == "__main__":
    main()