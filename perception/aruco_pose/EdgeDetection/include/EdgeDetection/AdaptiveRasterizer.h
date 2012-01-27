#ifndef __AdaptiveRasterizer_H__
#define __AdaptiveRasterizer_H__

#include "Rasterizer.h"
#include "Rasterizable.h"
#include "Image.h"

namespace EdgeDetection
{
	// The AdaptiveRasterizer uses recursive subdivision to rasterize given objects.
	class AdaptiveRasterizer : public Rasterizer
	{
		private: double threshold;

		public: AdaptiveRasterizer(double threshold)
		{
			if (threshold <= 0) throw "The parameter 'threshold' was out of range.";

			this->threshold= threshold;
		}
		public: ~AdaptiveRasterizer() { }
	
		public: Image* Rasterize(Rasterizable* object, int width, int height);

		private: double GetValue(Rasterizable* object, Rectangle rectangle);
	};
};

#endif
