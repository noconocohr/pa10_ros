import math

def degToRad(angle_degree):
	return angle_degree * math.pi / 180

def radToDeg(angle_rad):
	return angle_rad * 180 / math.pi

def tupleToRad(t):
	lst = map(degToRad, list(t))
	return tuple(lst)

def tupleToDeg(t):
	lst = map(radToDeg, list(t))
	return tuple(lst)

