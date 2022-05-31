import pybullet as p
import pybullet_data
import time
from math import cos, sin, tan, asin, acos, atan, sqrt, pi

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)

pos=[0,0,0.1]
system = p.loadURDF("fkik.urdf", pos)

def inverseKinematics(target, theta):

    y=target
    if abs(theta-pi)<0.0001:
        return pi, 0
     
    theta1 = asin(y/4)
    theta2 = pi - 2*theta1

    q2 = theta2
    alpha = acos(((target)*(target)+3.2*3.2-0.8*0.8)/(2*target*3.2))
    q1=abs(theta)-(pi/2)+theta1+alpha

    if(theta<0):
        return 2*pi-q1, 2*pi-q2

    return q1, q2

theta = 0
theta1, theta2 = inverseKinematics(2.4, theta)
p.resetJointState(system, 2, theta1)
p.resetJointState(system, 3, theta2)
while(True):
    temp1, temp2 = theta1, theta2
    p.setJointMotorControl2(system, 0, p.POSITION_CONTROL, targetPosition=-theta, force=10)

    actual_theta=((theta*180/pi)%360)*pi/180
    if actual_theta>=pi:
        actual_theta-=2*pi

    target=sqrt((0.8)*(0.8)+(3.2)*(3.2)-2*0.8*3.2*cos(abs(actual_theta)))

    theta1, theta2 = inverseKinematics(target, actual_theta)

    if (temp1-theta1)>0.01:
        p.resetJointState(system, 2, theta1)
        p.resetJointState(system, 3, theta2)
    p.setJointMotorControl2(system, 2, p.POSITION_CONTROL, targetPosition=theta1, force=10)
    p.setJointMotorControl2(system, 3, p.POSITION_CONTROL, targetPosition=theta2, force=10)
    
    print(theta1, theta2)
    p.stepSimulation()
    # time.sleep(1./2400.)
    theta+=0.0001