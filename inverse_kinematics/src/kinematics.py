import numpy as np

class ArmDimensions:

    def __init__(self, a, b):
        self.a = a
        self.b = b

DIM = ArmDimensions(10, 10)


# Cartesian <-> Cylindrical

def cartesian_to_cylindrical(cartesian):
    x, y, z = cartesian
    alpha = np.arctan2(y, x)
    r = np.hypot(x, y)
    return np.array([r, alpha, z])

def cylindrical_to_cartesian(cylindrical):
    r, alpha, z = cylindrical
    x = r*np.cos(alpha)
    y = r*np.sin(alpha)
    return np.array([x, y, z])


# Angles -> Position

def angles_to_cylindrical(angles):
    alpha, beta, gamma = angles
    r = DIM.a*np.cos(beta) + DIM.b*np.cos(beta - gamma)
    z = DIM.a*np.sin(beta) + DIM.b*np.sin(beta - gamma)
    return np.array([r, alpha, z])

def angles_to_cartesian(angles):
    return cylindrical_to_cartesian(angles_to_cylindrical(angles))


# Position -> Angles

def cylindrical_to_angles(cylindrical):
    r, alpha, z = cylindrical

    R = np.hypot(r, z)
    max_R = DIM.a + DIM.b
    if R>max_R:
        return np.full(3, np.nan)
    elif R==0:
        return np.array([0, 0.5*np.pi, np.pi])

    # Intermediate angles named according to the lines they are the angle
    # between, where
    #   a = lower arm, b = upper arm
    #   r = r axis, R = origin -> position

    rR = np.arctan2(z, r)
    Ra = np.arccos((DIM.a**2 + R**2 - DIM.b**2)/(2*DIM.a*R))
    beta = rR + Ra
    ab = np.arccos((DIM.a**2 + DIM.b**2 - R**2)/(2*DIM.a*DIM.b))
    gamma = np.pi - ab

    return np.array([alpha, beta, gamma])

def cartesian_to_angles(cartesian):
    return cylindrical_to_angles(cartesian_to_cylindrical(cartesian))


# Start + End Pos -> Path of Angles

def format_angle(theta):
    if theta<-np.pi:
        return theta + 2*np.pi
    elif theta>np.pi:
        return theta - 2*np.pi
    else:
        return theta

def path_cartesian_to_angles(start_pos, end_pos, n):
    if n<=1 or np.linalg.norm(start_pos-end_pos)<1e-3:
        positions =  [start_pos, end_pos]
    else:
        positions = np.linspace(start_pos, end_pos, n)
    return [cartesian_to_angles(pos) for pos in positions]


def path_cylindrical_to_angles(start_pos, end_pos, n):
    if n<=2 or np.linalg.norm(start_pos-end_pos)<1e-3:
        positions = [start_pos, end_pos]
    else:
        r = np.linspace(start_pos[0], end_pos[0], n)

        start_theta = format_angle(start_pos[1])
        end_theta = format_angle(end_pos[1])
        delta_theta = format_angle(end_theta - start_theta)
        theta = start_theta + np.linspace(0, delta_theta, n)
        for i in range(theta.size):
            theta[i] = format_angle(theta[i])

        z = np.linspace(start_pos[2], end_pos[2], n)

        r = r.reshape(n, 1)
        theta = theta.reshape(n, 1)
        z = z.reshape(n, 1)
        positions = np.concatenate((r, theta, z), axis=1) 
    
    return [cylindrical_to_angles(pos) for pos in positions]


"""
Alternative path for cylindrical coordinates
Make the arm pass closer to the centre to minimise distance
by making r decrease to a minimum, then increase again to the target r.

eg: Fit a quadratic to ra to rb, with zero gradient at rmin
r = a*(i-imin)^2 + rmin
ra = a*i_min^2 + rmin
rb = a*((n-1)-i_min)^2 + rmin
imin = (n-1)/(1 + sqrt((rb-rmin)/(ra-rmin)))
a = (ra - rmin)/imin^2

Code that would be used:
r_min = 2 (eg)
i_min = (n_steps - 1)/(1 + sqrt((end_pos[0] - r_min)/(start_pos[0] - r_min)))
a = (start_pos[0] - r_min)/i_min**2
i = np.arange(0, n_steps)
r = a*(i-i_min)**2 + r_min
theta = ...
z = ...

"""