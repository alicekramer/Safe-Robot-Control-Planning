import numpy as np
from math import atan2, sqrt
from shapely import Polygon

def orientation(p1, p2, p3):
    
    # m1 = (p2[1] - p1[1])/ (p2[0] -p1[0]) #division by zero problem
    # m2 = (p3[1]-p2[1])/ (p3[0]- p2[0])

    orientation = (p2[1] - p1[1])*(p3[0]- p2[0]) - (p2[0] -p1[0])*(p3[1]-p2[1]) 

    if orientation == 0:
        #collinear
        return 0
    elif orientation > 0:
        #clockwise
        return 1
    else:
        #counterclockwise
        return 2

def furtherstPoint(poi_sorted, P_ref):
    angles = np.array([atan2(point[1]-P_ref[1], point[0]-P_ref[0]) for point in poi_sorted])
    distances = np.array([sqrt((point[1]-P_ref[1])**2 + (point[0]-P_ref[0])**2) for point in poi_sorted])
    indices = np.array([i for i in range(len(angles))])
    
    # Store Duplicates:
    unique_angle = []
    duplicate_anlge = []
    final_points = []

    # Loop through every point
    for angle in angles:
        if angle not in unique_angle:
            mask_angle = angle == angles
            max_distance = np.max(distances[mask_angle])
            mask_distance = max_distance == distances
            index_keep = indices[mask_distance]
            final_points.append(poi_sorted[index_keep[0]])

            unique_angle.append(angle)
        else:
            duplicate_anlge.append(angle)
            pass
    # print("duplicate_angle", duplicate_anlge)
    return final_points

def graham_scan(pts):
    # Tuple of points
    pts = [(pt[0], pt[1]) for pt in pts] 
    
    # Lowest point as reference - min y & if repeated min x
    P_ref = min(pts, key=lambda p: (p[0], p[1])) 
    
    # Function that calculates the angle between the point of reference and other angles (use as key function when sorting)
    def angle_distance(p2):
        y_delta = p2[1] - P_ref[1]
        x_delta = p2[0] - P_ref[0]
        angle = atan2(y_delta, x_delta)

        euc_distance = sqrt(y_delta**2 + x_delta**2)

        return angle, euc_distance

    # Remove Tuple from list
    poi = pts
    poi.remove(P_ref) 
    
    # Points of Interest sorted by angles and if same angle, closest to P_ref
    poi_sorted = sorted(poi, key= angle_distance) # if the angles are the same, put nearest point first

    poi_sorted = furtherstPoint(poi_sorted, P_ref)

    # Initiate Hull
    point2 =poi_sorted[0]
    point3 = poi_sorted[1]
    convex_hull = [P_ref, point2, point3]
    
    for pt in poi_sorted[2:]:
        ort = orientation(convex_hull[-2], convex_hull[-1], pt)
        # print("point:", pt, "orient:", ort)
        while len(convex_hull) > 1 and ort != 2:
            # Pop if clockwise turn/ right turn
            convex_hull.pop()
            
            # Update the orientation
            ort = orientation(convex_hull[-2], convex_hull[-1], pt) #check the next orientation between pooint of interest and point on the hull

        convex_hull.append(pt)
    pts_sorted = np.array([[pt[0], pt[1]] for pt in poi_sorted])
    
    # print("convex_hull", convex_hull)
    return Polygon(convex_hull), pts_sorted