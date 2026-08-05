import qi
import time

master = qi.Session()
master.connect("tcp://192.168.14.70:9559")

slave = qi.Session()
slave.connect("tcp://192.168.13.71:9559")

master_motion = master.service("ALMotion")
master_memory = master.service("ALMemory")
slave_motion  = slave.service("ALMotion")

slave_motion.setStiffnesses("Body", 1.0)

names = [
    "HeadYaw",
    "HeadPitch",

    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",

    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll"
]

names = master_motion.getBodyNames("Body")
soft_mode = False

while True:

    angles = master_motion.getAngles(
        names,
        True
    )

    slave_motion.setAngles(
        names,
        angles,
        0.2
    )

    # print(angles)
    head_middle = master_memory.getData(
        "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value"
    )
    head_rear = master_memory.getData(
        "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value"
    )
    if (head_middle > 0.5 and not soft_mode):
        master_motion.setStiffnesses("Body", 0.0)
        soft_mode = True
        print(0.0)
    elif (head_middle <= 0.5 and soft_mode):
        master_motion.setStiffnesses("Body", 0.2)
        soft_mode = False
        print(0.2)
    if head_rear > 0.5:
        print("Exit requested")
        master_motion.setStiffnesses("Body", 0.2)
        slave_motion.setStiffnesses("Body", 0.2)
        break
    time.sleep(0.05)
