'''
Created on 1 mai 2018

@author: PASTOR Robert

Euler Angles to Rotation Matrices
The easiest way to think about 3D rotation is the axis-angle form. Any arbitrary rotation can be defined by an axis of rotation and an angle the describes the amount of rotation. 
Let’s say you want to rotate a point or a reference frame about the x axis by angle \theta_x. 
The rotation matrix corresponding to this rotation is given by

    \[ \mathbf{R_x} = \begin{bmatrix}     1       & 0 & 0 \\     0 & \cos(\theta_x) & -\sin(\theta_x)\\     0 & \sin(\theta_x) & (\cos\theta_x) \end{bmatrix} \]

Rotations by \theta_y and \theta_z about the y and z axes can be written as

    \[ \mathbf{R_y} = \begin{bmatrix}     \cos(\theta_y)       & 0 & \sin(\theta_y) \\     0 & 1 & 0\\     -\sin(\theta_y) & 0 & (\cos\theta_y) \end{bmatrix} \]

    \[ \mathbf{R_z} = \begin{bmatrix}     \cos(\theta_z) & -\sin(\theta_z) & 0 \\     \sin(\theta_z) & (\cos\theta_z) & 0 \\      0       & 0 & 1 \\ \end{bmatrix} \]

A rotation \mathbf{R} about any arbitrary axis can be written in terms of successive rotations about the Z, Y, and finally X axes using the matrix multiplication shown below.

    \[ \mathbf{R} = \mathbf{R_z R_y R_x} \]

In this formulation \theta_x, \theta_y and \theta_z are the Euler angles. 
Given these three angles you can easily find the rotation matrix by first finding \mathbf{R_x}, \mathbf{R_y} and \mathbf{R_z} and then multiply them to obtain \mathbf{R}. The code below shows an example

'''

import numpy as np
import math

# Calculates Rotation Matrix given Euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


'''

Converting a rotation matrix to Euler angles is a bit tricky. 
The solution is not unique in most cases. 
Using the code in the previous section you can verify that rotation matrices corresponding to 
Euler angles [0.1920,  2.3736,   1.1170] ( or [[11, 136, 64] in degrees) 
and [-2.9496, 0.7679, -2.0246] ( or [-169, 44, -116] in degrees) 
are actually the same even though the Euler angles look very different. 
The code below shows a method to find the Euler angles given the rotation matrix. 
The output of the following code should exactly match the output of MATLAB’s rotm2euler but the order of x and z are swapped.

'''

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to Euler angles
# The result is the same as MATLAB except the order
# of the Euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])