#ifndef __MicroEdgeCorrectionTable_H__
#define __MicroEdgeCorrectionTable_H__

#include "Edge.h"
#include "Rectangle.h"
#include "MicroEdgeMap.h"

#include <vector>
#include <sstream>
#include <cmath>

using namespace std;

namespace EdgeDetection
{
	// Corrects the errors made on two-dimensional edge detection.
	class MicroEdgeCorrectionTable
	{
		private: Rectangle area;
		private: int offsetCount;
		private: int angleCount;
		private: vector<Edge> entries;

		private: double GetOffsetOffset() { return area.GetLeft(); }
		private: double GetOffsetStep() { return area.GetWidth() / (offsetCount - 1); }
		private: double GetAngleOffset() { return area.GetTop(); }
		private: double GetAngleStep() { return area.GetHeight() / (angleCount - 1); }

		public: MicroEdgeCorrectionTable(MicroEdgeMap* microEdgeMap, Rectangle area, int offsetCount, int angleCount)
		{
			if (microEdgeMap == 0) throw "The parameter 'microEdgeMap' cannot be NULL.";

			Initialize(area, offsetCount, angleCount);
			PopulateTable(microEdgeMap);
		}
		private: MicroEdgeCorrectionTable(Rectangle area, int offsetCount, int angleCount)
		{
			Initialize(area, offsetCount, angleCount);
		}
		public: ~MicroEdgeCorrectionTable() { }
	
		public: Edge Correct(Edge measuredEdge);
	
		private: void Initialize(Rectangle area, int offsetCount, int angleCount);
		private: void PopulateTable(MicroEdgeMap* microEdgeMap);

		public: static istream* Serialize(MicroEdgeCorrectionTable* microEdgeCorrectionTable);
		public: static MicroEdgeCorrectionTable* Deserialize(istream* data);
	};
};

#endif
