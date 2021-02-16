
#include "BasicSpatialisationRTAudio.h"
#include "../projects/vstudio/AudioFile.h"
#include "../projects/vstudio/boost/python.hpp"

BOOST_PYTHON_MODULE(audioSimu) {
	using namespace boost::python;
	def("test", main);
}