#ifndef __MomentMask_H__
#define __MomentMask_H__

#include "Rasterizable.h"
#include "Rectangle.h"

namespace EdgeDetection
{
	// Represents a mask for calculating moments of discrete two-dimensional
	// functions defined on the unit circle.
	class MomentMask : public Rasterizable
	{
		private: int degreeX;
		private: int degreeY;
	
		public: Rectangle GetRegion() { return Rectangle(-1, +1, -1, +1); }

		public: MomentMask(int degreeX, int degreeY)
		{
			if (degreeX < 0) throw "The parameter 'degreeX' was out of range.";
			if (degreeY < 0) throw "The parameter 'degreeY' was out of range.";
	
			this->degreeX = degreeX;
			this->degreeY = degreeY;
		}
		public: ~MomentMask() { }

		public: double GetValue(Rectangle rectangle);
		public: bool CanCalculateIntegral(Rectangle rectangle);
		public: double CalculateIntegral(Rectangle rectangle);
		public: double EstimateIntegral(Rectangle rectangle);
	};
};

#endif
