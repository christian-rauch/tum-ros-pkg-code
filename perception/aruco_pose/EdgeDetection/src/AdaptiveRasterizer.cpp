#include "EdgeDetection/AdaptiveRasterizer.h"

#include "EdgeDetection/OwnedImage.h"

namespace EdgeDetection
{
	Image* AdaptiveRasterizer::Rasterize(Rasterizable* object, int width, int height)
	{
		if (object == 0) throw "The parameter 'object' cannot be NULL.";
		if (width < 0) throw "The parameter 'threshold' was out of range.";
		if (height < 0) throw "The parameter 'threshold' was out of range.";
	
		OwnedImage* image = new OwnedImage(width, height);

		Rectangle region = object->GetRegion();

		double pixelWidth = region.GetWidth() / width;
		double pixelHeight = region.GetHeight() / height;

		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				double left = region.GetLeft() + (x + 0) * pixelWidth;
				double right = region.GetLeft() + (x + 1) * pixelWidth;
				double top = region.GetTop() + (y + 0) * pixelHeight;
				double bottom = region.GetTop() + (y + 1) * pixelHeight;

				double value = GetValue(object, Rectangle(left, right, top, bottom));			

				image->Set(x, y, value);
			}

		return image;
	}

	double AdaptiveRasterizer::GetValue(Rasterizable* object, Rectangle rectangle)
	{
		if (object->CanCalculateIntegral(rectangle)) return object->CalculateIntegral(rectangle);
		if (rectangle.GetArea() < threshold) return object->EstimateIntegral(rectangle);

		Rectangle leftTop = Rectangle(rectangle.GetLeft(), rectangle.GetCenter().GetX(), rectangle.GetTop(), rectangle.GetCenter().GetY());
		Rectangle rightTop = Rectangle(rectangle.GetCenter().GetX(), rectangle.GetRight(), rectangle.GetTop(), rectangle.GetCenter().GetY());
		Rectangle leftBottom = Rectangle(rectangle.GetLeft(), rectangle.GetCenter().GetX(), rectangle.GetCenter().GetY(), rectangle.GetBottom());
		Rectangle rightBottom = Rectangle(rectangle.GetCenter().GetX(), rectangle.GetRight(), rectangle.GetCenter().GetY(), rectangle.GetBottom());

		return GetValue(object, leftTop) + GetValue(object, rightTop) + GetValue(object, leftBottom) + GetValue(object, rightBottom);
	}
};
