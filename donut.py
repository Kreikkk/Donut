import numpy as np
import pygame
from config import mrx_cfg


class Thorus:
	K1 = 5
	K2 = 5
	
	def __init__(self, R1, R2):
		self.R1 = R1	#r
		self.R2 = R2	#R

	def get_coord(self, phi, alp):
		self.cosphi = np.cos(phi)
		self.cosalp = np.cos(alp)

		brack = self.R2 + self.R1*self.cosphi


	def get_projection(self):
		pass



class RotationMatrix:
	self._x_mask = [[0, 1, 1, 2, 2], [0, 1, 2, 1, 2]]
	self._y_mask = [[1, 0, 2, 0, 2], [1, 0, 0, 2, 2]]
	self._z_mask = [[2, 0, 0, 1, 1], [2, 0, 1, 0, 1]]		#1, cos, -sin, sin, cos

	def __init__(self, phi):
		self.clear_matrix()
		self._cosphi = np.cos(phi)
		self._sinphi = np.sin(phi)

	def clear_matrix(self):
		self._matrix = np.zeros((3, 3))

	@property
	def x_matrix(self, phi):
		self.clear_matrix()

		self._matrix[0, 0] =  1
		self._matrix[1, 1] =  self._cosphi
		self._matrix[1, 2] = -self._sinphi
		self._matrix[2, 1] =  self._sinphi
		self._matrix[2, 2] =  self._cosphi

		return self._matrix

	def get_y_mrx(self, phi):
		self.clear_matrix()

	def get_z_mrx(self, phi):
		self.clear_matrix()