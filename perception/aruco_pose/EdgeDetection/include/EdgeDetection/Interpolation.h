#ifndef __Interpolation_H__
#define __Interpolation_H__

namespace EdgeDetection
{
	class Interpolation
	{
		public: static double InterpolateForward(double start, double end, double fraction)
		{
			if (fraction < 0) return start;
			if (fraction > 1) return end;

			return start + fraction * (end - start);
		}
		public: static double InterpolateReverse(double start, double end, double value)
		{
			if (value < start) return 0;
			if (value > end) return 1;

			return (value - start) / (end - start);
		}
	};
};

#endif
