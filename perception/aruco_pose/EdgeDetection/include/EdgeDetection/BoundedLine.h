#ifndef __BoundedLine_H__
#define __BoundedLine_H__

#include "Line.h"
#include "Vector2.h"

namespace EdgeDetection
{
	// Represents a bounded line in two-dimensional space.
	class BoundedLine : public Line
	{
		private: Vector2 start;
		private: Vector2 end;

		public: Vector2 GetStart() { return start; }
		public: Vector2 GetEnd() { return end; }
		public: double GetLength() { return Vector2::Subtract(start, end).GetLength(); }

		public: BoundedLine()
		{
			this->start = Vector2();
			this->end = Vector2();
		}
		public: BoundedLine(Vector2 start, Vector2 end) : Line(end.GetY() - start.GetY(), start.GetX() - end.GetX(), end.GetX() * start.GetY() - start.GetX() * end.GetY())
		{
			if (Vector2::Equals(start, end)) throw "The parameters 'start' and 'end' cannot be the same.";

			this->start = start;
			this->end = end;
		}
		public: ~BoundedLine() { }

		public: Vector2 GetPosition(double fraction)
		{
			if (fraction < 0 || fraction > 1) throw "The parameter 'fraction' was out of range.";

			return Vector2::Add(start, Vector2::Multiply(fraction, Vector2::Subtract(end, start)));
		}
	};
};

#endif
