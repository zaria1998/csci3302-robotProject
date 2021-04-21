from controller import Robot, Motor, Camera, RangeFinder, Lidar
import math

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.5725 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10 # Left wheel index
MOTOR_RIGHT = 11 # Right wheel index
N_PARTS = 12 # Total joints


robot = Robot()


timestep = int(robot.getBasicTimeStep())

part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")



robot_parts = []

for i in range(N_PARTS):
        robot_parts.append(robot.getDevice(part_names[i]))


pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

waypoints = [[0.5,0],[2.0,-0.5],[2.0,-3.0],[1.7,-3.8]]
i = 0

        
while robot.step(timestep) != -1:

    targets = waypoints[i]


    d = math.sqrt((targets[0]-pose_x)**2 + (targets[1]-pose_y)**2 )
    if d < 0.2:
        i = i+1
        if (i > 3):
            break
    alpha = pose_theta - math.atan2(targets[1]-pose_y,targets[0]-pose_x)

    


    x = d
    T = 2 * alpha 

    vL = (2*x - T*0.4044)/2
    vR = (2*x + T*0.4044)/2

    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    #print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta))
    
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)