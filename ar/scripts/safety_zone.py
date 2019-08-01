import math
import numpy as np
from scipy.spatial import ConvexHull
import cv2

def PointsInCircum(r,ox,oy,n=50):
    return [(math.cos(2*math.pi/n*x)*r+ox, math.sin(2*math.pi/n*x)*r+oy) for x in xrange(0,n+1)]


def calculate_safety_zone(link_locations, circle_rad, visualization):
    orig_xs = [elem[0] for elem in link_locations]
    orig_ys = [elem[1] for elem in link_locations]
    xs = []
    ys = []
    xhull = []
    yhull = []
    # calculate circle points for each joint position
    for idx in xrange(0, len(orig_xs)):
        vec = PointsInCircum(circle_rad, orig_xs[idx], orig_ys[idx])
        xs.append([it[0] for it in vec])
        ys.append([it[1] for it in vec])
    r_contour_x = [item for sublist in xs for item in sublist]
    r_contour_y = [item for sublist in ys for item in sublist]

    # calculate the convex hull
    points = np.column_stack((r_contour_x, r_contour_y))  # for convex hull
    hull = ConvexHull(points)
    vs = np.zeros((len(points[hull.vertices, 0]), 2))
    vs[:, 0] = points[hull.vertices, 0]
    vs[:, 1] = points[hull.vertices, 1]
    xhull.append(vs[:, 0])
    yhull.append(vs[:, 1])

    # add more points to the hull line
    idx = 0
    while idx < len(vs):
        if (idx == len(vs) - 1):
            c1 = vs[-1:]
            c1 = vs[0]
        else:
            c1 = vs[idx]
            c2 = vs[idx + 1]
        dist = abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])
        N = dist / 0.01
        xhull = np.append(xhull, np.linspace(c1[0], c2[0], N))
        yhull = np.append(yhull, np.linspace(c1[1], c2[1], N))
        idx += 1

    # xhull = xhull[::5]
    # yhull = yhull[::5]
    convex_hull_final_pt = []
    convex_hull_final_str = ""
    for i in range(0, len(xhull)):
        convex_hull_final_pt.append([xhull[i], yhull[i]])
        if i != 0:
            convex_hull_final_str += ':'
        convex_hull_final_str += "{:0.3f}".format(xhull[i]) + ':' + "{:0.3f}".format(yhull[i])

    if visualization:
        blank_image = np.zeros((2200, 2000, 3), np.uint8)
        blank_image[:] = (0, 0, 0)
        for p in convex_hull_final_pt:
            p = np.array(p) * 1000 + 1500
            cv2.circle(blank_image, (int(p[0]), int(p[1])), 9, (255, 0, 0), -1)
        viz_img = cv2.resize(blank_image, (800, 800), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("asd", viz_img)
        cv2.waitKey(5)
    return convex_hull_final_pt, convex_hull_final_str
