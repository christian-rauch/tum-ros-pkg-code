#ifndef __Rasterizable_H__
#define __Rasterizable_H__

#include "Rectangle.h"

namespace EdgeDetection
{
	// This is the interface objects need to implement in order to be rasterized.
	class Rasterizable
	{
		// Gets the region of the object that should be rasterized.
		public: virtual Rectangle GetRegion() = 0;

		// Returns a value indicating whether the 'CalculateIntegral' method can be
		// used to calculate the exact value of the integral of the image function
		// over the given rectangle.
		public: virtual bool CanCalculateIntegral(Rectangle rectangle) = 0;
		// Calculates the exact value of the integral of the image function over the
		// given rectangle.
		public: virtual double CalculateIntegral(Rectangle rectangle) = 0;
		// Estimates the value of the integral of the image function over the given
		// rectangle.
		public: virtual double EstimateIntegral(Rectangle rectangle) = 0;
	};
};

#endif
