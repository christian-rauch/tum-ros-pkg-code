#include "EdgeDetection/MacroEdgeDetector.h"

#include "EdgeDetection/Vector2.h"
#include "EdgeDetection/Edge.h"

#include <vector>

namespace EdgeDetection
{
	Line MacroEdgeDetector::DetectEdge(Image* image, BoundedLine estimatedEdge)
	{
		// Debug outputs
		//cout << "Refining (" << estimatedEdge.GetStart().GetX() << ", " << estimatedEdge.GetStart().GetY() << ") -> (" << estimatedEdge.GetEnd().GetX() << ", " << estimatedEdge.GetEnd().GetY() << ")" << endl;

		if (estimatedEdge.GetLength() < 3 * microEdgeDetector->GetSize()) throw "The estimated edge cannot be shorter than three times the window size.";

		vector<Vector2> edgePoints;

		double stepFraction = microEdgeDetector->GetSize() / estimatedEdge.GetLength();
		double start = 0 + stepFraction;
		double end = 1 - stepFraction;

		for (double fraction = start; fraction <= end; fraction += stepFraction)
		{
			Vector2 position = estimatedEdge.GetPosition(fraction);

			int windowLeft = Round(position.GetX() + 0.5 - 0.5 * microEdgeDetector->GetSize());
			int windowRight = Round(position.GetX() + 0.5 + 0.5 * microEdgeDetector->GetSize());
			int windowTop = Round(position.GetY() + 0.5 - 0.5 * microEdgeDetector->GetSize());
			int windowBottom = Round(position.GetY() + 0.5 + 0.5 * microEdgeDetector->GetSize());

			Image* windowRegion = image->GetRegion(windowLeft, windowRight, windowTop, windowBottom);

			Edge measuredEdge = microEdgeDetector->DetectEdge(windowRegion);
			Edge correctedEdge = microEdgeCorrectionTable->Correct(measuredEdge);

			delete windowRegion;

			Vector2 windowCenter = Vector2(0.5 * ((windowLeft - 0.5) + (windowRight - 0.5)), 0.5 * ((windowTop - 0.5) + (windowBottom - 0.5)));
			Vector2 edgeCenter = Vector2::Add(windowCenter, Vector2::Multiply(0.5 * microEdgeDetector->GetSize(), correctedEdge.GetCenter()));
			Vector2 correctedEdgeCenter = pointCorrector->CorrectPoint(edgeCenter);

			// Debug outputs
//			cout << "(" << edgeCenter.GetX() << ", " << edgeCenter.GetY() << ") -> (" << correctedEdgeCenter.GetX() << ", " << correctedEdgeCenter.GetY() << ")" << endl;

			edgePoints.push_back(correctedEdgeCenter);

			// Debug outputs
/*
			cout << "Position: (" << position.GetX() << ", " << position.GetY() << ")" << endl;
			cout << "Window: [" << windowLeft << ", " << windowRight << ", " << windowTop << ", " << windowBottom << "]" << endl;
			cout << "Window Center: (" << windowCenter.GetX() << ", " << windowCenter.GetY() << ")" << endl;
			cout << "Edge Center: (" << correctedEdge.GetCenter().GetX() << ", " << correctedEdge.GetCenter().GetY() << ")" << endl;
			cout << "Measured: [" << measuredEdge.GetOffset() << ", " << measuredEdge.GetAngle() << "], Corrected: [" << correctedEdge.GetOffset() << ", " << correctedEdge.GetAngle() << "], Center: (" << edgeCenter.GetX() << ", " << edgeCenter.GetY() << ")" << endl;
*/
		}

		double averageX = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++) averageX += edgePoint->GetX();
		averageX /= edgePoints.size();

		double averageY = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++) averageY += edgePoint->GetY();
		averageY /= edgePoints.size();

		double momentXX = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++) momentXX += (edgePoint->GetX() - averageX) * (edgePoint->GetX() - averageX);
		momentXX /= edgePoints.size() - 1;

		double momentYY = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++) momentYY += (edgePoint->GetY() - averageY) * (edgePoint->GetY() - averageY);
		momentYY /= edgePoints.size() - 1;

		double momentXY = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++) momentXY += (edgePoint->GetX() - averageX) * (edgePoint->GetY() - averageY);
		momentXY /= edgePoints.size() - 1;

		double a = momentXX - momentYY;
		double b = momentXY + momentXY;
		double c = a * a + b * b;

		double factorX = - b * averageX + a * averageY - sqrt(c) * averageY;
		double factorY = + b * averageY + a * averageX + sqrt(c) * averageX;
		double offset = b * averageX * averageX - 2 * a * averageX * averageY - b * averageY * averageY;

		// Debug outputs
/*
		Line estimatedLine = Line::Normalize(estimatedEdge);
		Line fittedLine = Line::Normalize(Line(factorX, factorY, offset));

		cout << "Estimated line: (" << estimatedLine.GetFactorX() << ") * x + (" << estimatedLine.GetFactorY() << ") * y + (" << estimatedLine.GetOffset() << ") = 0" << endl;
		cout << "Fitted line: (" << fittedLine.GetFactorX() << ") * x + (" << fittedLine.GetFactorY() << ") * y + (" << fittedLine.GetOffset() << ") = 0" << endl;

		double estimatedError = 0;
		int estimatedAbove = 0;
		int estimatedBelow = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++)
		{
			estimatedError += abs(estimatedLine.GetSide(*edgePoint));
			if (estimatedLine.GetSide(*edgePoint) >= 0) estimatedAbove++;
			if (estimatedLine.GetSide(*edgePoint) < 0) estimatedBelow++;
		}
		double fittedError = 0;
		int fittedAbove = 0;
		int fittedBelow = 0;
		for (vector<Vector2>::iterator edgePoint = edgePoints.begin(); edgePoint < edgePoints.end(); edgePoint++)
		{
			fittedError += abs(fittedLine.GetSide(*edgePoint));
			if (fittedLine.GetSide(*edgePoint) >= 0) fittedAbove++;
			if (fittedLine.GetSide(*edgePoint) < 0) fittedBelow++;
		}

		cout << "Estimated error: " << estimatedError << ", Estimated distribution: " << estimatedAbove << "/" << estimatedBelow << ", Fitted error: " << fittedError << ", Fitted distribution: " << fittedAbove << "/" << fittedBelow << endl;

		return fittedLine;
*/

		return Line(factorX, factorY, offset);
	}
};
