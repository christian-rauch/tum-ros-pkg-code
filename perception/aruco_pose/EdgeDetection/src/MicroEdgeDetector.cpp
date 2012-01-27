#include "EdgeDetection/MicroEdgeDetector.h"

#include <cmath>

namespace EdgeDetection
{
	Edge MicroEdgeDetector::DetectEdge(Image* image)
	{
		if (image == 0) throw "The parameter 'image' cannot be NULL.";
		if (image->GetWidth() != size || image->GetHeight() != size) throw "The parameter 'image' was out of range.";

		double moment00 = Image::Combine(image, mask00);
		double moment10 = Image::Combine(image, mask10);
		double moment01 = Image::Combine(image, mask01);
		double moment11 = Image::Combine(image, mask11);
		double moment20 = Image::Combine(image, mask20);
		double moment02 = Image::Combine(image, mask02);
	
		double rotatedMoment00 = moment00;
		double rotatedMoment10 = sqrt(moment10 * moment10 + moment01 * moment01);
		double rotatedMoment20 = (moment10 * moment10 * moment20 + 2 * moment10 * moment01 * moment11 + moment01 * moment01 * moment02) / (moment10 * moment10 + moment01 * moment01);
	
		double offset = (4 * rotatedMoment20 - rotatedMoment00) / (3 * rotatedMoment10);
		double angle = atan2(moment01, moment10);

		return Edge(offset, angle);
	}
};
