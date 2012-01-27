#include "EdgeDetection/MicroEdgeMap.h"

#include "EdgeDetection/EdgeImage.h"
#include "EdgeDetection/Image.h"

namespace EdgeDetection
{
	Edge MicroEdgeMap::FindRealEdge(Edge measuredEdge)
	{
		Edge bestEdge = Edge(0, 0);
		Edge bestEdgeMeasured = MeasureEdge(bestEdge);

		for (size_t index = 0; index < entries.size(); index++)
		{
			Edge sampleEdge = entries[index].GetRealEdge();
			Edge sampleEdgeMeasured = entries[index].GetMeasuredEdge();
		
			if (Edge::GetDistance(measuredEdge, sampleEdgeMeasured) <= Edge::GetDistance(measuredEdge, bestEdgeMeasured))
			{
				bestEdge = sampleEdge;
				bestEdgeMeasured = sampleEdgeMeasured;
			}
		}

		return bestEdge;
	}

	void MicroEdgeMap::PopulateTable(Rectangle area, int offsetCount, int angleCount)
	{
		if (area.GetLeft() < -1 || area.GetRight() > +1 || area.GetTop() < -M_PI || area.GetBottom() > +M_PI) throw "The parameter 'area' was out of range.";
		if (offsetCount <= 0) throw "The parameter 'offsetCount' was out of range.";
		if (angleCount <= 0) throw "The parameter 'angleCount' was out of range.";

		for (int offsetIndex = 0; offsetIndex < offsetCount; offsetIndex++)
			for (int angleIndex = 0; angleIndex < angleCount; angleIndex++)
			{
				double offset = area.GetLeft() + offsetIndex * (area.GetWidth() / (offsetCount - 1));
				double angle = area.GetTop() + angleIndex * (area.GetHeight() / (angleCount - 1));

				Edge realEdge = Edge(offset, angle);
				Edge measuredEdge = MeasureEdge(realEdge);

				entries.push_back(Entry(realEdge, measuredEdge));
			}
	}
	Edge MicroEdgeMap::MeasureEdge(Edge realEdge)
	{	
		EdgeImage* realEdgeImage = new EdgeImage(Line::FromEdge(realEdge), Rectangle(-1, +1, -1, +1));
		Image* image = rasterizer->Rasterize(realEdgeImage, microEdgeDetector->GetSize(), microEdgeDetector->GetSize());
		Edge measuredEdge = microEdgeDetector->DetectEdge(image);
	
		delete realEdgeImage;
		delete image;
	
		return measuredEdge;
	}
};
