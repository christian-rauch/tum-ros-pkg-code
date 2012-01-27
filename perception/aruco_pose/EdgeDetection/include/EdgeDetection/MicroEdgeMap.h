#ifndef __MicroEdgeMap_H__
#define __MicroEdgeMap_H__

#include "Edge.h"
#include "Rasterizer.h"
#include "MicroEdgeDetector.h"
#include "Rectangle.h"
#include "Vector2.h"

#include <vector>

using namespace std;

namespace EdgeDetection
{
	// Finds the errors made on two-dimensional edge detection.
	class MicroEdgeMap
	{
		class Entry
		{
			private: Edge realEdge;
			private: Edge measuredEdge;
		
			public: Edge GetRealEdge() { return realEdge; }
			public: Edge GetMeasuredEdge() { return measuredEdge; }
		
			public: Entry(Edge realEdge, Edge measuredEdge)
			{
				this->realEdge = realEdge;
				this->measuredEdge = measuredEdge;
			}
			public: ~Entry() { }
		};
	
		private: Rasterizer* rasterizer;
		private: MicroEdgeDetector* microEdgeDetector;
		private: vector<Entry> entries;
	
		public: MicroEdgeMap(Rasterizer* rasterizer, MicroEdgeDetector* microEdgeDetector, Rectangle area, int offsetCount, int angleCount)
		{
			if (rasterizer == 0) throw "The parameter 'rasterizer' cannot be NULL.";
			if (microEdgeDetector == 0) throw "The parameter 'microEdgeDetector' cannot be NULL.";
	
			this->rasterizer = rasterizer;
			this->microEdgeDetector = microEdgeDetector;
			this->entries = vector<Entry>();
		
			PopulateTable(area, offsetCount, angleCount);
		}
		public: ~MicroEdgeMap() { }
	
		public: Edge FindRealEdge(Edge measuredEdge);

		private: void PopulateTable(Rectangle area, int offsetCount, int angleCount);
		private: Edge MeasureEdge(Edge realEdge);
	};
};

#endif
