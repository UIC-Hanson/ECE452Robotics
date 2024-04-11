import cv2
import numpy as np
import math

# ======== TO DO ========
#Fill out each functions based on the materials from the lecture.

def vec2hat(x):
    # Find x^
    x_hat = np.array([[0, -x[2], x[1]],
                      [x[2], 0, -x[0]],
                      [-x[1], x[0], 0]])
    
    return x_hat

def cvdata2transmtx(rvec,tvec):
    # Rotation and translation of Camera to ArUco
    R_temp = cv2.Rodrigues(rvec)[0]
    p_temp = tvec.reshape((3, 1))
    
    # Find the Rotation and translation of ArUco to Camera
    R = np.transpose(R_temp) # R_T
    p = -np.dot(R, p_temp) # -R_T.p
    g = np.vstack((np.hstack((R, p)), [0, 0, 0, 1]))
    
    return g, R, p

def transmtx2twist(g):
    # Rotation and translation from g
    R = g[:3, :3]
    p = g[:3, 3]
    
    # Convert the rotation matrix to rotation vector (including theta)
    rvec = cv2.Rodrigues(R)[0]
    
    # Find the twist coordinate
    th = np.linalg.norm(p)
    
    # Compute angular velocity (w) and linear velocity (v)
    if th < 1e-6:  # Check if translation vector is close to zero
        w = np.zeros(3)
        v = np.zeros(3)
    else:
        w = np.flatten(rvec / th)
        v = np.dot(np.linalg.inv(np.eye(3) - R), p) / th

    print("v shape:", v.shape)
    print("w shape:", w.shape)
        
    return v, w, th

def twist2screw(v,w,th):
    # Convert the twist coordinate to screw motion
    q = np.cross(v, w) # Point on screw axis
    h = np.linalg.norm(v) # Pitch
    u = w / np.linalg.norm(w) # Normalized w
    M = th # Magnitude
    
    return q, h, u, M

def distance(xdiff,zdiff):
    # Find the distance using delta(x) and delta(z) in the x-z plane
    dist = np.sqrt(xdiff**2 + zdiff**2)
    return dist
