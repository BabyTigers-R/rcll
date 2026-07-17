from pymycobot import MyPalletizerSocket
import time
mc = MyPalletizerSocket("192.168.0.212",9000)

print(mc.get_coords())
mc.sync_send_coords([150,0,80,0],10,15)
time.sleep(1)
mc.set_gripper_state(10,100)
mc.release_all_servos()


