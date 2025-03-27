# 2D geometry helper functions

from math import atan2, sin, cos, pi, sqrt

def p2r(r, theta):
    """Convert polar coords (r, theta) to rectangular coords (x,y)"""
    x = r * cos(theta)
    y = r * sin(theta)
    return (x, y)

def r2p(x, y):
    """Convert rect coords (x, y) to polar (r, theta)"""
    r = sqrt(x*x + y*y)
    theta = atan2(y, x)
    return (r, theta)


def pt_coords(pose, dist, sensor):
    """Representing vectors as complex numbers, calculate &
    return (x, y) coords of point detected by robot at pose,
    measured distance = dist by sensor = 'R' | 'L' | 'F'
    """
    xr, yr, ar = pose

    # convert robot location from rect coords to polar coords
    a, alpha = r2p(xr, yr)

    b = dist  # measured dist from robot to point

    if sensor == 'R':
        beta = ar - pi/2
    elif sensor == 'L':
        beta = ar + pi/2
    else:
        beta = ar

    delta = atan2(a * sin(alpha) + b * sin(beta),
                  a * cos(alpha) + b * cos(beta))
    d = sqrt((a * cos(alpha) + b * cos(beta))**2 +\
             (a * sin(alpha) + b * sin(beta))**2)

    return p2r(d, delta)

if __name__ == "__main__":
    pose = (4, 4, pi/4)
    dist = 2.0
    print("point on right at coords", pt_coords(pose, dist, 'R'))
    print("point on left at coords", pt_coords(pose, dist, 'L'))
    print("point in front at coords", pt_coords(pose, dist, 'F'))
