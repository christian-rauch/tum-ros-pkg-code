#include "EdgeDetection/MicroEdgeCorrectionTable.h"

#include "EdgeDetection/Interpolation.h"

#include <cmath>

namespace EdgeDetection
{
	Edge MicroEdgeCorrectionTable::Correct(Edge measuredEdge)
	{
		double offsetIndex = Interpolation::InterpolateReverse(area.GetLeft(), area.GetRight(), measuredEdge.GetOffset()) * (offsetCount - 1);
		int offsetIndexLow = floor(offsetIndex);
		int offsetIndexHigh = ceil(offsetIndex);
		double offsetFraction = offsetIndex - offsetIndexLow;

		double angleIndex = Interpolation::InterpolateReverse(area.GetTop(), area.GetBottom(), measuredEdge.GetAngle()) * (angleCount - 1);
		int angleIndexLow = floor(angleIndex);
		int angleIndexHigh = ceil(angleIndex);
		double angleFraction = angleIndex - angleIndexLow;

		Edge leftTop = entries[offsetIndexLow * angleCount + angleIndexLow];
		Edge rightTop = entries[offsetIndexHigh * angleCount + angleIndexLow];
		Edge leftBottom = entries[offsetIndexLow * angleCount + angleIndexHigh];
		Edge rightBottom = entries[offsetIndexHigh * angleCount + angleIndexHigh];

		Edge offsetTop = Edge::Interpolate(leftTop, rightTop, offsetFraction);
		Edge offsetBottom = Edge::Interpolate(leftBottom, rightBottom, offsetFraction);
		Edge offsetAngle = Edge::Interpolate(offsetTop, offsetBottom, angleFraction);

		// Debug outputs
/*
		cout << "OffsetFraction: " << offsetFraction << ", AngleFraction: " << angleFraction << endl;
		cout << "Indices: [" << offsetIndexLow << ", " << offsetIndexHigh << "], [" << angleIndexLow << ", " << angleIndexHigh << "]" << endl;
		cout << "LeftTop: [" << leftTop.GetOffset() << ", " << leftTop.GetAngle() << "], RightTop: [" << rightTop.GetOffset() << ", " << rightTop.GetAngle() << "], LeftBottom: [" << leftBottom.GetOffset() << ", " << leftBottom.GetAngle() << "], RightBottom: [" << rightBottom.GetOffset() << ", " << rightBottom.GetAngle() << "]" << endl;
		cout << "OffsetTop: [" << offsetTop.GetOffset() << ", " << offsetTop.GetAngle() << "], OffsetBottom: [" << offsetBottom.GetOffset() << ", " << offsetBottom.GetAngle() << "], OffsetAngle: [" << offsetAngle.GetOffset() << ", " << offsetAngle.GetAngle() << "]" << endl;
*/

		return offsetAngle;
	}

	void MicroEdgeCorrectionTable::Initialize(Rectangle area, int offsetCount, int angleCount)
	{
		if (area.GetLeft() < -1 || area.GetRight() > +1 || area.GetTop() < -M_PI || area.GetBottom() > +M_PI) throw "The parameter 'area' was out of range.";
		if (offsetCount <= 0) throw "The parameter 'offsetCount' was out of range.";
		if (angleCount <= 0) throw "The parameter 'angleCount' was out of range.";

		this->area = area;
		this->offsetCount = offsetCount;
		this->angleCount = angleCount;
		this->entries = vector<Edge>();
	}
	void MicroEdgeCorrectionTable::PopulateTable(MicroEdgeMap* microEdgeMap)
	{
		for (int offsetIndex = 0; offsetIndex < offsetCount; offsetIndex++)
			for (double angleIndex = 0; angleIndex < angleCount; angleIndex++)
			{
				double offset = GetOffsetOffset() + offsetIndex * GetOffsetStep();
				double angle = GetAngleOffset() + angleIndex * GetAngleStep();

				Edge measuredEdge = Edge(offset, angle);
				Edge realEdge = microEdgeMap->FindRealEdge(measuredEdge);

				entries.push_back(realEdge);
			}
	}

	istream* MicroEdgeCorrectionTable::Serialize(MicroEdgeCorrectionTable* microEdgeCorrectionTable)
	{
		stringstream* data = new stringstream(stringstream::in | stringstream::out | stringstream::binary);

		double areaLeft = microEdgeCorrectionTable->area.GetLeft();
		double areaRight = microEdgeCorrectionTable->area.GetRight();
		double areaTop = microEdgeCorrectionTable->area.GetTop();
		double areaBottom = microEdgeCorrectionTable->area.GetBottom();
		int offsetCount = microEdgeCorrectionTable->offsetCount;
		int angleCount = microEdgeCorrectionTable->angleCount;

		data->write((const char*)&areaLeft, sizeof(double));
		data->write((const char*)&areaRight, sizeof(double));
		data->write((const char*)&areaTop, sizeof(double));
		data->write((const char*)&areaBottom, sizeof(double));
		data->write((const char*)&offsetCount, sizeof(int));
		data->write((const char*)&angleCount, sizeof(int));
	
		for (vector<Edge>::iterator edge = microEdgeCorrectionTable->entries.begin(); edge < microEdgeCorrectionTable->entries.end(); edge++)
		{
			double offset = edge->GetOffset();
			double angle = edge->GetAngle();

			data->write((const char*)&offset, sizeof(double));
			data->write((const char*)&angle, sizeof(double));
		}

		return data;
	}
	MicroEdgeCorrectionTable* MicroEdgeCorrectionTable::Deserialize(istream* data)
	{
		double areaLeft;
		double areaRight;
		double areaTop;
		double areaBottom;
		int offsetCount;
		int angleCount;

		data->read((char*)&areaLeft, sizeof(double));
		data->read((char*)&areaRight, sizeof(double));
		data->read((char*)&areaTop, sizeof(double));
		data->read((char*)&areaBottom, sizeof(double));
		data->read((char*)&offsetCount, sizeof(int));
		data->read((char*)&angleCount, sizeof(int));

		MicroEdgeCorrectionTable* microEdgeCorrectionTable = new MicroEdgeCorrectionTable(Rectangle(areaLeft, areaRight, areaTop, areaBottom), offsetCount, angleCount);
	
		while (!data->eof())
		{
			double offset;
			double angle;
		
			data->read((char*)&offset, sizeof(double));
			data->read((char*)&angle, sizeof(double));
		
			microEdgeCorrectionTable->entries.push_back(Edge(offset, angle));
		}
	
		return microEdgeCorrectionTable;
	}
};
