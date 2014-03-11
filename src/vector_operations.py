#!/usr/bin/env python

# Jon Rovira
# Summer 2013

import math
import numpy as np

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

"""
Vector Operations

This library holds necessary vector operations used throughout 
other NxR Baxter scripts.
"""

def make_vector_from_POINTS(p1, p2):
	return [p2.x-p1.x, p2.y-p1.y, p2.z-p1.z]

def inverse_rotate(q, v):
	"""
	Returns a vector v_prime rotated by the inverse of a
	quaternion q


	Inverse quaternion q^-1:
	    q^-1 = q.w - q.x(i) - q.y(j) - q.z(k)
     
	Vector v rotated by quaternion q:
			t = 2 * cross(q.xyz, v)
			v' = v + q.w * t + cross(q.xyz, t)
	"""
	# Create inverse quaternion
	divisor = ((q.x**2) + (q.y**2) + (q.z**2) + (q.w**2))**0.5
	q.x = -1*q.x/divisor
	q.y = -1*q.y/divisor
	q.z = -1*q.z/divisor
	q.w = q.w/divisor

	# Algorithm
	q_array = [q.x, q.y, q.z]
	q_cross_v = np.cross(q_array, v)
	t = 2 * np.array(q_cross_v)
	q_cross_t = np.cross(q_array, t)
	v_prime = v + (q.w * np.array(t)) + q_cross_t

	# Account for inprecision
	if math.fabs(v_prime[0]) < 0.000001: v_prime[0] = 0
	if math.fabs(v_prime[1]) < 0.000001: v_prime[1] = 0
	if math.fabs(v_prime[2]) < 0.000001: v_prime[2] = 0

	return v_prime

def angle_between_vectors(A, B):
	"""
	Returns the angle theta between two vectors A and B

	A . B = |a| * |b| * cos(theta)
	"""
	A_dot_B = np.dot(A, B)
	a = np.linalg.norm(A)
	b = np.linalg.norm(B)
	theta = np.arccos(A_dot_B/a/b)

	return theta

def vector_projection_onto_vector(v, A):
	"""
	Returns the vector v_prime obtained by orthogonally projecting a
	vector v onto a vector A

	A^ = A / |A|
	v_prime = (v . A^) * A^
	"""
	# Vector A of unit length
	A_hat = np.array(A) / np.linalg.norm(A)
	# Projected vector
	v_prime = np.dot(v, A_hat) * A_hat

	return v_prime	

def vector_projection_onto_plane(v, A, B):
	"""
	Returns the vector v_prime obtained by orthogonally projecting a
	vector v onto a plane defined by vectors A and B

	n = A x B
	a = n x (v X n)
	v_prime = [(a . v) / (a . a)] * a
	"""
	# Vector orthogonal to plane
	n = np.cross(A, B)
	# Vector in the direction of the projection
	a = np.cross(n, np.cross(v, n))
	# Projected vector
	v_prime = (np.dot(a, v) / np.dot(a, a)) * a

	return v_prime


def shortest_vector_from_point_to_vector(p, v1, v2, p_v1):
	"""
	A vector v1's endpoint is the origin of vector v2. If v1 were to
	continue past its magnitude, this function returns the vector of
	shortest magnitude between point p (v2's endpoint) and the
	continuing v1.
	"""
	# Extends the vector's length so that its endpoint is the closest
	#    point on the vector to the point (as if in real space)
	v1_extended = v1 + vector_projection_onto_vector(v2, v1)
	p_v1 = [p_v1.x, p_v1.y, p_v1.z]
	p2 = p_v1 + v1_extended
	p = [p.x, p.y, p.z]
	v_prime = p - p2

	return v_prime
		

"""
Main:

Only for testing functions as they are written. Set this library as
executable in order for it to run. In the terminal:

                      chmod +x vector_operations.py
"""
if __name__=='__main__':
	print "\n---------------------"

	print "TEST: inverse_rotate"
	q = Quaternion(math.sqrt(0.5), 0, 0, math.sqrt(0.5))
	v0 = [0,1,0]
	v_prime = inverse_rotate(q,v0)
	print "x: " + str(v_prime[0])
	print "y: " + str(v_prime[1])
	print "z: " + str(v_prime[2])
	print "---------------------\n"

	print "TEST: angle_between_vectors"
	A = [0,0,1]
	B = [0,1,0]
	theta = angle_between_vectors(A,B)
	print "theta = " + str(theta)
	print "---------------------\n"

	print "TEST: vector_projection_onto_plane"
	v = [3,7,2]
	A = [0,0,1]
	B = [1,0,0]
	v_prime = vector_projection_onto_plane(v,A,B)
	print "x: " + str(v_prime[0])
	print "y: " + str(v_prime[1])
	print "z: " + str(v_prime[2])
	print "---------------------\n"

	print "TEST: vector_projection_onto_vector"
	v = [1,5,0]
	A = [1,0,0]
	v_prime = vector_projection_onto_vector(v,A)
	print "x: " + str(v_prime[0])
	print "y: " + str(v_prime[1])
	print "z: " + str(v_prime[2])
	print "---------------------\n"

	print "TEST: shortest_vector_from_point_to_vector"
	p = Point(10,5,0)
	v1 = [5,0,0]
	v2 = [5,5,0]
	p_v1 = Point(0,0,0)
	v_prime = shortest_vector_from_point_to_vector(p,v1,v2,p_v1)
	print "x: " + str(v_prime[0])
	print "y: " + str(v_prime[1])
	print "z: " + str(v_prime[2])
	print "---------------------\n"
