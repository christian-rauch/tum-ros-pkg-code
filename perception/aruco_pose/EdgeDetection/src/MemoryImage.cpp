#include "EdgeDetection/MemoryImage.h"

namespace EdgeDetection
{
	Image* RedByteGreenByteBlueByteMemoryImage::GetRegion(int left, int right, int top, int bottom)
	{
		if (left < 0 || left >= GetWidth()) throw "The parameter 'left' was out of range.";
		if (right < 0 || right >= GetWidth()) throw "The parameter 'right' was out of range.";
		if (top < 0 || top >= GetHeight()) throw "The parameter 'top' was out of range.";
		if (bottom < 0 || bottom >= GetHeight()) throw "The parameter 'bottom' was out of range.";
		if (right - left < 0) throw "The parameters 'left' and 'right' were out of range.";
		if (bottom - top < 0) throw "The parameters 'top' and 'bottom' were out of range.";

		return new RedByteGreenByteBlueByteMemoryImage(right - left, bottom - top, ((unsigned char*)GetData()) + left * 3 + top * GetStride(), GetStride());
	}
	Image* GrayDoubleMemoryImage::GetRegion(int left, int right, int top, int bottom)
	{
		if (left < 0 || left >= GetWidth()) throw "The parameter 'left' was out of range.";
		if (right < 0 || right >= GetWidth()) throw "The parameter 'right' was out of range.";
		if (top < 0 || top >= GetHeight()) throw "The parameter 'top' was out of range.";
		if (bottom < 0 || bottom >= GetHeight()) throw "The parameter 'bottom' was out of range.";
		if (right - left < 0) throw "The parameters 'left' and 'right' were out of range.";
		if (bottom - top < 0) throw "The parameters 'top' and 'bottom' were out of range.";

		return new GrayDoubleMemoryImage(right - left, bottom - top, ((double*)GetData()) + left * 1 + top * GetStride(), GetStride());
	}
};
