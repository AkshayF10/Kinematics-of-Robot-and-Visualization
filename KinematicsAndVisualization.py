# -*- coding: utf-8 -*-
"""
@author: Akshay
"""

'''
The sample robot selected here is a FANUC R-2000iC with 6 DOF. All joints are revolute joints.
'''

'''
The forward kinematic calculations are being done using a method taught by Dr. Sangram Redkar.

According to his method, the rules for making kinematic diagrams are:
    1. Z axis is the joint axis
    2. Xn is perpendicular to Zn and Zn-1, and Xn intersects Zn and Zn-1 (there are exceptions, which leads to frames being translated)
    3. Use right-hand co-ordinate system, i.e. the index finger represents the Y axis, the middle finger represents X axis, the thumb represents Z axis, and when the thumb shows the Z axis, curling the fingers will represent the positive direction of rotation.


Assumptions:
    1. The forward kinematics will give the position of the end effector with respect to the centre of the first joint
    2. There are fixed distances between the centres of each joints, represnted by a1, a2, a3, a4, a5
    3. There is a fixed distance between the centre of the last joint and the centre of the end effector, a6
    4. If there is any exceptions to the rules, the assumption is made that there is no rotation
'''


import numpy as np
import sympy as sp

# Defining the basic rotation matrices with theta as a symbolic variable
ThetaVariables = [sp.symbols(f'theta{i}') for i in range(1, 7)]

# Create a list to store the rotation matrices
RotationMatrices = []

# Generate six instances of the rotation matrix with different thetas
# These matrices will be multiplied with transformation matrices to find the rotation of the nth frame with respect to the (n-1)th frame
for theta in ThetaVariables:
    RotationMatrix = np.array([[sp.cos(theta), -sp.sin(theta), 0],
                                [sp.sin(theta), sp.cos(theta), 0],
                                [0, 0, 1]])
    RotationMatrices.append(sp.Matrix(RotationMatrix))

# Defining each transformation matrix to find the rotation of the nth frame with respect to the (n-1)th frame
TransformationMatrices = [
    sp.Matrix(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])),
    sp.Matrix(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
    sp.Matrix(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
    sp.Matrix(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])),
    sp.Matrix(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])),
    sp.Matrix(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
]

# Multiplying each rotation matrix with its corresponding transformation matrix in order to check whether each resultant rotation is correct
ResultantMatrices = [RotMat * TransMat for RotMat, TransMat in zip(RotationMatrices, TransformationMatrices)]

# Printing the resultant matrices which will show roatation
ResultantMatrixSequence = ResultantMatrices[0]
for i, (RotMat, TransMat, ResultMat) in enumerate(zip(RotationMatrices, TransformationMatrices, ResultantMatrices), start=1):
    label = f"{i - 1}R{i}"  # Label format: (i-1)Ri
    print(f"Resultant Matrix {label} after multiplying Rotation {i} with Transformation {i}: {ResultMat}")

    ResultantMatrixSequence = ResultantMatrixSequence * ResultMat
    print(f"Cumulative Resultant Matrix 0R6 showing rotation of end effector with respect to the first joint: {ResultantMatrixSequence}")

# Assuming a1 is a symbol or a specific value you want to add to the matrix
a1, a2, a3, a4, a5, a6 = sp.symbols('a1 a2 a3 a4 a5 a6')

# Take the first ResultMat matrix
HomMat1 = ResultantMatrices[0]
HomMat2 = ResultantMatrices[1]
HomMat3 = ResultantMatrices[2]
HomMat4 = ResultantMatrices[3]
HomMat5 = ResultantMatrices[4]
HomMat6 = ResultantMatrices[5]

# Create a 4x4 matrix by adding a row and a column
HomogeneousMatrix1 = sp.Matrix.hstack(sp.Matrix.vstack(HomMat1, sp.Matrix([[0, 0, 0]])), sp.Matrix([[0], [0], [a1], [1]]))
HomogeneousMatrix2 = sp.Matrix.hstack(sp.Matrix.vstack(HomMat2, sp.Matrix([[0, 0, 0]])), sp.Matrix([[-a2*sp.sin(ThetaVariables[1])], [a2*sp.cos(ThetaVariables[1])], [0], [1]]))
HomogeneousMatrix3 = sp.Matrix.hstack(sp.Matrix.vstack(HomMat3, sp.Matrix([[0, 0, 0]])), sp.Matrix([[0], [0], [0], [1]]))
HomogeneousMatrix4 = sp.Matrix.hstack(sp.Matrix.vstack(HomMat4, sp.Matrix([[0, 0, 0]])), sp.Matrix([[0], [0], [a3 + a4], [1]]))
HomogeneousMatrix5 = sp.Matrix.hstack(sp.Matrix.vstack(HomMat5, sp.Matrix([[0, 0, 0]])), sp.Matrix([[0], [0], [0], [1]]))
HomogeneousMatrix6 = sp.Matrix.hstack(sp.Matrix.vstack(HomMat6, sp.Matrix([[0, 0, 0]])), sp.Matrix([[0], [0], [a5 + a6], [1]]))

# Print the resulting 4x4 matrix
print(f"Resulting Homogeneous Matrix 0H1: {HomogeneousMatrix1}")
print(f"Resulting Homogeneous Matrix 1H2: {HomogeneousMatrix2}")
print(f"Resulting Homogeneous Matrix 2H3: {HomogeneousMatrix3}")
print(f"Resulting Homogeneous Matrix 3H4: {HomogeneousMatrix4}")
print(f"Resulting Homogeneous Matrix 4H5: {HomogeneousMatrix5}")
print(f"Resulting Homogeneous Matrix 5H6: {HomogeneousMatrix6}")

# Multiplying and printing the final homogeneous matrix which will show the the answer for the forward kinematics by showing the rotation and translation of the end effector with respect to the first joint
FinalHomogeneousMatrix = HomogeneousMatrix1 * HomogeneousMatrix2 * HomogeneousMatrix3 * HomogeneousMatrix4 * HomogeneousMatrix5 * HomogeneousMatrix6

# Print the final homogeneous matrix
print(f"Final Homogeneous Matrix 0H6: {FinalHomogeneousMatrix}")

# Asking the user to input values for link lengths a1 to a6
a1Value = float(input("Enter value for a1: "))
a2Value = float(input("Enter value for a2: "))
a3Value = float(input("Enter value for a3: "))
a4Value = float(input("Enter value for a4: "))
a5Value = float(input("Enter value for a5: "))
a6Value = float(input("Enter value for a6: "))

# Asking the user to input values for joint angles theta1 to theta6
ThetaValues = [float(input(f"Enter value for theta{i}: ")) for i in range(1, 7)]

# Substituting the user-provided values into the symbolic expressions
HomogeneousMatrix1 = HomogeneousMatrix1.subs({a1: a1Value, ThetaVariables[0]: ThetaValues[0]})
HomogeneousMatrix2 = HomogeneousMatrix2.subs({a2: a2Value, ThetaVariables[1]: ThetaValues[1]})
HomogeneousMatrix3 = HomogeneousMatrix3.subs({ThetaVariables[2]: ThetaValues[2]})
HomogeneousMatrix4 = HomogeneousMatrix4.subs({a3: a3Value, a4: a4Value, ThetaVariables[3]: ThetaValues[3]})
HomogeneousMatrix5 = HomogeneousMatrix5.subs({ThetaVariables[4]: ThetaValues[4]})
HomogeneousMatrix6 = HomogeneousMatrix6.subs({a5: a5Value, a6: a6Value, ThetaVariables[5]: ThetaValues[5]})

# Multiplying the updated homogeneous matrices to get the final homogeneous matrix
FinalHomogeneousMatrix = HomogeneousMatrix1 * HomogeneousMatrix2 * HomogeneousMatrix3 * HomogeneousMatrix4 * HomogeneousMatrix5 * HomogeneousMatrix6

# Printing the final homogeneous matrix
print(f"Final Homogeneous Matrix 0H6: {FinalHomogeneousMatrix}")



'''
Visualtion part

As joints 3 and 5 are exceptions to the rules, they have been handled differently in order to enable their visualization in the plot
'''



import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

H02 = HomogeneousMatrix1 * HomogeneousMatrix2
H03 = H02 * HomogeneousMatrix3
H04 = H03 * HomogeneousMatrix4
H05 = H04 * HomogeneousMatrix5
H06 = FinalHomogeneousMatrix

# Extract the translation vector
Link1EndCoordinates = HomogeneousMatrix1[:3, 3]
Link2EndCoordinates = H02[:3, 3]
Link3EndCoordinates = H03[:3, 3]
Link4EndCoordinates = H04[:3, 3]
Link5EndCoordinates = H05[:3, 3]
Link6EndCoordinates = H06[:3, 3]


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot coordinate system for the centre of each joint
ax.scatter(0, 0, 0, c='red', marker='o', label='Joint 1')
ax.scatter(Link1EndCoordinates[0], Link1EndCoordinates[1], Link1EndCoordinates[2], c='blue', marker='o', label='Joint 2')
ax.scatter(Link2EndCoordinates[0], Link2EndCoordinates[1], [a1Value + a2Value], c='green', marker='o', label='Joint 3') #Assumption
ax.scatter(Link3EndCoordinates[0], Link3EndCoordinates[1], Link3EndCoordinates[2], c='orange', marker='o', label='Joint 4')
ax.scatter(Link4EndCoordinates[0], Link4EndCoordinates[1], [a1Value + a2Value + a3Value + a4Value], c='purple', marker='o', label='Joint 5') #Assumption
ax.scatter(Link5EndCoordinates[0], Link5EndCoordinates[1], Link5EndCoordinates[2], c='brown', marker='o', label='Joint 6')
ax.scatter(Link6EndCoordinates[0], Link6EndCoordinates[1], Link6EndCoordinates[2], c='cyan', marker='o', label='End effector')


# Connecting all the joints, and the final joint with the end effector
ax.plot([0, Link1EndCoordinates[0]], [0, Link1EndCoordinates[1]], [0, Link1EndCoordinates[2]], color='black')
ax.plot([Link1EndCoordinates[0], Link2EndCoordinates[0]],
        [Link1EndCoordinates[1], Link2EndCoordinates[1]],
        [Link1EndCoordinates[2], a1Value + a2Value], color='black') #Assumption
ax.plot([Link2EndCoordinates[0], Link3EndCoordinates[0]],
        [Link2EndCoordinates[1], Link3EndCoordinates[1]],
        [a1Value + a2Value, Link3EndCoordinates[2]], color='black') #Assumption
ax.plot([Link3EndCoordinates[0], Link4EndCoordinates[0]],
        [Link3EndCoordinates[1], Link4EndCoordinates[1]],
        [Link3EndCoordinates[2], a1Value + a2Value + a3Value + a4Value], color='black') #Assumption
ax.plot([Link4EndCoordinates[0], Link5EndCoordinates[0]],
        [Link4EndCoordinates[1], Link5EndCoordinates[1]],
        [a1Value + a2Value + a3Value + a4Value, Link5EndCoordinates[2]], color='black') #Assumption
ax.plot([Link5EndCoordinates[0], Link6EndCoordinates[0]],
        [Link5EndCoordinates[1], Link6EndCoordinates[1]],
        [Link5EndCoordinates[2], Link6EndCoordinates[2]], color='black')

# Set axis labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Add legend
ax.legend()

# Show the plot
plt.show()

asdafs
