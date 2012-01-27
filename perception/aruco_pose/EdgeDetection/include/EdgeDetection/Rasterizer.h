#ifndef __Rasterizer_H__
#define __Rasterizer_H__

#include "Image.h"
#include "Rasterizable.h"

namespace EdgeDetection
{
	// This is the abstract base class for all rasterizers. They can rasterize any
	// object implementing the 'Rasterizable' interface.
	class Rasterizer
	{	
		public: virtual Image* Rasterize(Rasterizable* object, int width, int height) = 0;
	};
};

#endif
