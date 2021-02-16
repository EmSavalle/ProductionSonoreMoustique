cdef extern from "src/BasicSpatialisationRTAudio.cpp":
	pass

cdef extern from "src/BasicSpatialisationRTAudio.h" namespace "sounds":
	cdef cppclass Moustique:
		CVector3 position
		CVector3 destination
#		vector<tuple<int, int, int>> path
		bool authorizeMovement
		bool waitForAuthorization
		bool askPosition
		bool ended
		float speed
		void init(int posX, int posY, int posZ, bool askPos, float speed) except +
		void stop()
		void setDestination(int x, int y, int z)
		void setDestination(CVector3 vec)
		void updatePosition()
		float vectorLength(CVector3 vec)
		CVector3 getDirection()
		CVector3 normalize(CVector3 vec)
		CVector3 getPosition()
		void setDefaultPath()
		void getNextDestination()