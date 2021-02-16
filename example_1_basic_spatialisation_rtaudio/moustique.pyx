# distutils: language = c++
cimport Moustique
from libcpp cimport bool
from libcpp cimport CVector3
cdef class PyMoustique:
	cdef Moustique c_moust

	def __cinit__(self,int x, int y, int z, bool ask, float speed):
		self.c_moust = Moustique()
		self.c_moust.init(x,y,z,ask,speed)
	def setDestination(self,int x, int y, int z):
		self.c_moust.setDestination(x,y,z)

	def stop(self):
		self.c_moust.stop()