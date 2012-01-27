#ifndef __PointCorrector_H__
#define __PointCorrector_H__

#include "Vector2.h"

namespace EdgeDetection
{
	class PointCorrector
	{
		public: virtual Vector2 CorrectPoint(Vector2 point) = 0;
	};
};

#endif
