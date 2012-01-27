#include "EdgeDetection/MomentMask.h"

#include <cmath>

namespace EdgeDetection
{
	double MomentMask::GetValue(Rectangle rectangle)
	{
		double p = degreeX + 1;
		double q = degreeY + 1;

		double valueX = pow(rectangle.GetRight(), p) - pow(rectangle.GetLeft(), p);
		double valueY = pow(rectangle.GetBottom(), q) - pow(rectangle.GetTop(), q);

		return (valueX * valueY) / (p * q);
	}
	bool MomentMask::CanCalculateIntegral(Rectangle rectangle)
	{
		bool leftTopInside = rectangle.GetLeftTop().GetLength() <= 1;
		bool rightTopInside = rectangle.GetRightTop().GetLength() <= 1;
		bool leftBottomInside = rectangle.GetLeftBottom().GetLength() <= 1;
		bool rightBottomInside = rectangle.GetRightBottom().GetLength() <= 1;

		return (leftTopInside && rightTopInside && leftBottomInside && rightBottomInside) || (!leftTopInside && !rightTopInside && !leftBottomInside && !rightBottomInside);
	}
	double MomentMask::CalculateIntegral(Rectangle rectangle)
	{
		return (rectangle.GetCenter().GetLength() <= 1 ? 1.00 : 0.00) * GetValue(rectangle);
	}
	double MomentMask::EstimateIntegral(Rectangle rectangle)
	{
		return (rectangle.GetCenter().GetLength() <= 1 ? 0.75 : 0.25) * GetValue(rectangle);
	}
};
