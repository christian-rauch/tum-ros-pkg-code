#include "EdgeDetection/EdgeImage.h"

namespace EdgeDetection
{
	bool EdgeImage::CanCalculateIntegral(Rectangle rectangle)
	{
		bool leftTopInside = line.GetSide(rectangle.GetLeftTop()) >= 0;
		bool rightTopInside = line.GetSide(rectangle.GetRightTop()) >= 0;
		bool leftBottomInside = line.GetSide(rectangle.GetLeftBottom()) >= 0;
		bool rightBottomInside = line.GetSide(rectangle.GetRightBottom()) >= 0;

		return (leftTopInside && rightTopInside && leftBottomInside && rightBottomInside) || (!leftTopInside && !rightTopInside && !leftBottomInside && !rightBottomInside);
	}
	double EdgeImage::CalculateIntegral(Rectangle rectangle)
	{
		return (line.GetSide(rectangle.GetCenter()) >= 0 ? 1.00 : 0.00) * rectangle.GetArea();
	}
	double EdgeImage::EstimateIntegral(Rectangle rectangle)
	{
		return (line.GetSide(rectangle.GetCenter()) >= 0 ? 0.75 : 0.25) * rectangle.GetArea();
	}
};
