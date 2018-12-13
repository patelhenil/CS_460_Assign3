# -*- coding: utf-8 -*-
import math
import copy


# square root of 2 for diagonal distance
SQRT2 = math.sqrt(2)


def backtrace(node):
    """
    Backtrace according to the parent records and return the path.
    (including both start and end nodes)
    """
    path = [(node.x, node.y)]
    while node.parent:
        node = node.parent
        path.append((node.x, node.y))
    path.reverse()
    return path


def bresenham(coords_a, coords_b):
    '''
    Given the start and end coordinates, return all the coordinates lying
    on the line formed by these coordinates, based on Bresenham's algorithm.
    http://en.wikipedia.org/wiki/Bresenham's_line_algorithm#Simplification
    '''
    line = []
    x0, y0 = coords_a
    x1, y1 = coords_b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        line += [[x0, y0]]
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err = err - dy
            x0 = x0 + sx
        if e2 < dx:
            err = err + dx
            y0 = y0 + sy

    return line

def inside_polygon(x, y, polygon):
    '''Check if a point (x,y) lies inside a polygon with given vertices.
	Using ideas derived from: http://paulbourke.net/geometry/polygonmesh/'''

    num_vertices = len(polygon)

    inside = False
    p1x, p1y = polygon[0]
    for i in range(num_vertices + 1):
        p2x, p2y = polygon[i % num_vertices]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def on_polygon(x,y,polygon):
	pair_vertices = zip(polygon,polygon[1:]+polygon[:1])
	for item in pair_vertices:
		if abs(distance(item[0],item[1]) - (distance(item[0],(x,y))+distance((x,y),item[1]))) < 0.1:
			return True
	return False

def distance(pt1,pt2):
	'''Returns distance between two points.'''
	return ((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)**0.5


# Ray tracing
def ray_tracing_method(x,y,poly):
    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside