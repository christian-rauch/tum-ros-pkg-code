#ifndef __MacroEdgeDetector_H__
#define __MacroEdgeDetector_H__

#include "MicroEdgeDetector.h"
#include "MicroEdgeCorrectionTable.h"
#include "PointCorrector.h"
#include "Image.h"
#include "Line.h"
#include "BoundedLine.h"
#include "Vector2.h"

#include <cmath>

namespace EdgeDetection
{
	// Detects two-dimensional macro edges.
	class MacroEdgeDetector
	{
		private: MicroEdgeDetector* microEdgeDetector;
		private: MicroEdgeCorrectionTable* microEdgeCorrectionTable;
		private: PointCorrector* pointCorrector;

		public: MacroEdgeDetector(MicroEdgeDetector* microEdgeDetector, MicroEdgeCorrectionTable* microEdgeCorrectionTable, PointCorrector* pointCorrector)
		{
			if (microEdgeDetector == 0) throw "The parameter 'microEdgeDetector' cannot be NULL.";
			if (microEdgeCorrectionTable == 0) throw "The parameter 'microEdgeCorrectionTable' cannot be NULL.";
			if (pointCorrector == 0) throw "The parameter 'pointCorrector' cannot be NULL.";

			this->microEdgeDetector = microEdgeDetector;
			this->microEdgeCorrectionTable = microEdgeCorrectionTable;
			this->pointCorrector = pointCorrector;
		}
		public: ~MacroEdgeDetector() { }

		public: Line DetectEdge(Image* image, BoundedLine estimatedEdge);

		private: static int Round(double value)
		{
			return value >= 0 ? ceil(value - 0.5) : floor(value + 0.5);
		}
	};
};

#endif
