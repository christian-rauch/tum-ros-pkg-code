#include "EdgeDetection/Image.h"

namespace EdgeDetection
{
	double Image::Combine(Image* image1, Image* image2)
	{
		if (image1 == 0) throw "The parameter 'image1' cannot be NULL.";
		if (image2 == 0) throw "The parameter 'image2' cannot be NULL.";
		if (image1->width != image2->width || image1->height != image2->height) throw "The two images are not the same size.";

		int width = image1->width;
		int height = image1->height;

		double result = 0;

		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
				result += image1->Get(x, y) * image2->Get(x, y);

		return result;
	}
};
