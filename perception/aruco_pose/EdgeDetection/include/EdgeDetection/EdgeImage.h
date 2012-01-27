#ifndef __EdgeImage_H__
#define __EdgeImage_H__

#include "Rasterizable.h"
#include "Line.h"
#include "Rectangle.h"

namespace EdgeDetection
{
	// Represents the image of a two-dimensional edge.
	class EdgeImage : public Rasterizable
	{
		private: Line line;
		private: Rectangle region;

		public: Line GetLine() { return line; }
		public: Rectangle GetRegion() { return region; }

		public: EdgeImage(Line line, Rectangle region)
		{
			this->line = line;
			this->region = region;
		}
		public: ~EdgeImage() { }

		public: bool CanCalculateIntegral(Rectangle rectangle);
		public: double CalculateIntegral(Rectangle rectangle);
		public: double EstimateIntegral(Rectangle rectangle);
	};
};

#endif
