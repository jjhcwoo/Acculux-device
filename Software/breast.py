import config
import numpy as np
from quaternion import Quaternion

# Using settings defined in config.py, represents a breast as a spheroid.

a = config.BREAST_A / 2
b = config.BREAST_B / 2
c = config.BREAST_C

def get_mesh(n_theta = 20, n_phi = 12):
    theta = np.linspace(0, 2*np.pi, n_theta)
    phi = np.linspace(0, np.pi/2, n_phi)   # Only upper half
    theta, phi = np.meshgrid(theta, phi)

    # Cartesian coordinates
    x = a * np.sin(phi) * np.cos(theta)
    y = b * np.sin(phi) * np.sin(theta)
    z = c * np.cos(phi)

    # Convert grid to vertices
    verts = np.vstack([x.ravel(), y.ravel(), z.ravel()]).T

    # Build triangular faces
    faces = []
    for i in range(n_phi - 1):
        for j in range(n_theta - 1):
            v0 = i * n_theta + j
            v1 = v0 + 1
            v2 = v0 + n_theta
            v3 = v2 + 1

            faces.append([v0, v1, v2])
            faces.append([v1, v2, v3])

    faces = np.array(faces)
    
    return verts, faces

def get_projection(q, offset=0):
    # given a quaternion orientation, returns the projection of the vector onto the surface

    # quaternion x and y are pure quaternions
    vx = np.array([0, 1, 0, 0])
    vy = np.array([0, 0, 1, 0])

    # V = QVQ'
    vx = Quaternion.multiply(
        Quaternion.multiply(q, vx), 
        Quaternion.conjugate(q)
        )[1:4]
    vy = Quaternion.multiply(
        Quaternion.multiply(q, vy), 
        Quaternion.conjugate(q)
        )[1:4]
    
    vz = np.cross(vx, vy)
    vz = vz / np.linalg.norm(vz)

    a = config.BREAST_A / 2
    b = config.BREAST_B / 2
    c = config.BREAST_C

    x, y, z = vz
    scale = 1 / np.sqrt(
        (x ** 2) / (a ** 2) +
        (y ** 2) / (b ** 2) +
        (z ** 2) / (c ** 2)
    )
    vz = scale * vz
    if offset != 0:
        vz[0] += offset
        vz[2] += config.PCB_OFFSET

    return np.array(vz)