# PyBullet - Inverse Kinematics
For a link-joint system in robotics, the link at the end of the chain is usually called as the end effector. If I have the desired position and orientation of the end effector in the world space, then I can calculate the joint values required to reach that desired position. This is referred to as Inverse Kinematics (IK). **Note that the joint values should reside within the joint limits.**

Athough there is a built-in function in pybullet for IK, still we can implement one on our own depending on the problem statement.

### Problem scenario
The task is to pin a 2R planar arm on the boundary of a rotating disc. While the disc is rotating, the end effector of the arm should be at a fixed point.
I have attached the [URDF file](fkik.urdf) and the solution [fkik.py](fkik.py) file.
