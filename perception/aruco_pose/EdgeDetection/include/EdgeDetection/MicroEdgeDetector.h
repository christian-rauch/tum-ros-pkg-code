#ifndef __MicroEdgeDetector_H__
#define __MicroEdgeDetector_H__

#include "Image.h"
#include "Rasterizer.h"
#include "Edge.h"
#include "MomentMask.h"

namespace EdgeDetection
{
	// Detects two-dimensional edges on the unit circle.
	class MicroEdgeDetector
	{
		private: int size;
		private: Image* mask00;
		private: Image* mask10;
		private: Image* mask01;
		private: Image* mask11;
		private: Image* mask20;
		private: Image* mask02;
	
		public: int GetSize() { return size; }

		public: MicroEdgeDetector(Rasterizer* rasterizer, int size)
		{
			if (rasterizer == 0) throw "The parameter 'rasterizer' cannot be NULL.";
			if (size < 0) throw "The parameter 'size' was out of range.";
	
			this->size = size;
	
			MomentMask* mask;

			mask = new MomentMask(0, 0);
			this->mask00 = rasterizer->Rasterize(mask, size, size);
			delete mask;
			mask = new MomentMask(1, 0);
			this->mask10 = rasterizer->Rasterize(mask, size, size);
			delete mask;
			mask = new MomentMask(0, 1);
			this->mask01 = rasterizer->Rasterize(mask, size, size);
			delete mask;
			mask = new MomentMask(1, 1);
			this->mask11 = rasterizer->Rasterize(mask, size, size);
			delete mask;
			mask = new MomentMask(2, 0);
			this->mask20 = rasterizer->Rasterize(mask, size, size);
			delete mask;
			mask = new MomentMask(0, 2);
			this->mask02 = rasterizer->Rasterize(mask, size, size);
			delete mask;
		}
		public: ~MicroEdgeDetector()
		{
			delete this->mask00;
			delete this->mask10;
			delete this->mask01;
			delete this->mask11;
			delete this->mask20;
			delete this->mask02;
		}

		public: Edge DetectEdge(Image* image);
	};
};

#endif
