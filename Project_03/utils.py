import cv2
import numpy as np
import math

# ======== TO DO ========
#Fill out each functions based on the materials from the lecture.

def vec2hat(x):
    # Find x^
    x_hat = 
    return x_hat

def cvdata2transmtx(rvec,tvec):
    # Rotation and translation of Camera to ArUco
    R_temp = 
    p_temp = 
    # Find the Rotation and translation of ArUco to Camera
    R = 
    p = 
    g = 
    return g, R, p

def transmtx2twist(g):
    # Rotation and translation from g
    R = 
    p = 
    # Convert the rotation matrix to rotation vector (including theta)
    rvec = 
    # Find the twist coordinate
    th = 
    w = 
    v = 
    return v, w, th

def twist2screw(v,w,th):
    # Convert the twist coordinate to screw motion
    q = 
    h = 
    u = 
    M = 
    return q, h, u, M

def distance(xdiff,zdiff):
    # Find the distance using delta(x) and delta(z) in the x-z plane
    dist = 
    return dist
