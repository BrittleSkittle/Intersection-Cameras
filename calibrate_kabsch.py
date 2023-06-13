import pyrealsense2 as rs
import calculate_rmsd_kabsch as rmsd
import numpy as np
def calculate_transformation_kabsch(src_points, dst_points):
	"""
	Calculates the optimal rigid transformation from src_points to
	dst_points
	(regarding the least squares error)

	Parameters:
	-----------
	src_points: array
		(3,N) matrix
	dst_points: array
		(3,N) matrix
	
	Returns:
	-----------
	rotation_matrix: array
		(3,3) matrix
	
	translation_vector: array
		(3,1) matrix

	rmsd_value: float

	"""
	assert src_points.shape == dst_points.shape
	if src_points.shape[0] != 3:
		raise Exception("The input data matrix had to be transposed in order to compute transformation.")
		
	src_points = src_points.transpose()
	dst_points = dst_points.transpose()
	
	src_points_centered = src_points - rmsd.centroid(src_points)
	dst_points_centered = dst_points - rmsd.centroid(dst_points)

	rotation_matrix = rmsd.kabsch(src_points_centered, dst_points_centered)
	rmsd_value = rmsd.kabsch_rmsd(src_points_centered, dst_points_centered)

	translation_vector = rmsd.centroid(dst_points) - np.matmul(rmsd.centroid(src_points), rotation_matrix)

	return rotation_matrix.transpose(), translation_vector.transpose(), rmsd_value