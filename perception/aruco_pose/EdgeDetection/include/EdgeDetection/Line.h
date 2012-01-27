#ifndef __Line_H__
#define __Line_H__

#include "Vector2.h"
#include "Edge.h"

#include <cmath>

namespace EdgeDetection
{
	// Represents a line in two-dimensional space.
	class Line
	{
		private: double factorX;
		private: double factorY;
		private: double offset;

		public: double GetFactorX() { return factorX; }
		public: double GetFactorY() { return factorY; }
		public: double GetOffset() { return offset; }

		public: Line()
		{
			this->factorX = 0;
			this->factorY = 0;
			this->offset = 0;
		}
		public: Line(double factorX, double factorY, double offset)
		{
			if (factorX == 0 && factorY == 0) throw "The parameters 'factorX' and 'factorY' cannot both be 0.";

			this->factorX = factorX;
			this->factorY = factorY;
			this->offset = offset;
		}
		public: ~Line() { }

		public: double GetSide(Vector2 position)
		{
			return factorX * position.GetX() + factorY * position.GetY() + offset;
		}

		public: static Line FromEdge(Edge edge)
		{
			Vector2 start, end;

			if (edge.GetOffset() == 0)
			{
				start = Vector2(edge.GetAngle() - 0.5 * M_PI);
				end = Vector2(edge.GetAngle() + 0.5 * M_PI);
			}
			if (edge.GetOffset() < 0)
			{
				start = Vector2::Multiply(2 * edge.GetOffset(), Vector2(edge.GetAngle() + M_PI / 3));
				end = Vector2::Multiply(2 * edge.GetOffset(), Vector2(edge.GetAngle() - M_PI / 3));
			}
			if (edge.GetOffset() > 0)
			{
				start = Vector2::Multiply(2 * edge.GetOffset(), Vector2(edge.GetAngle() - M_PI / 3));
				end = Vector2::Multiply(2 * edge.GetOffset(), Vector2(edge.GetAngle() + M_PI / 3));
			}

			return Line(end.GetY() - start.GetY(), start.GetX() - end.GetX(), end.GetX() * start.GetY() - start.GetX() * end.GetY());
		}
		public: static Line Normalize(Line line)
		{
			if (line.offset != 0) return Line(line.factorX / line.offset, line.factorY / line.offset, line.offset / line.offset);
			if (line.factorX != 0) return Line(line.factorX / line.factorX, line.factorY / line.factorX, line.offset / line.factorX);
			if (line.factorY != 0) return Line(line.factorX / line.factorY, line.factorY / line.factorY, line.offset / line.factorY);

			throw "Invalid operation.";
		}
		public: static bool AreParallel(Line line1, Line line2)
		{
			return line1.factorX * line2.factorY - line1.factorY * line2.factorX == 0;
		}
		public: static Vector2 Intersect(Line line1, Line line2)
		{
			double determinant = line1.factorX * line2.factorY - line1.factorY * line2.factorX;
			double determinant1 = line1.factorY * line2.offset - line1.offset * line2.factorY;
			double determinant2 = line1.offset * line2.factorX - line1.factorX * line2.offset;

			if (determinant == 0) throw "The given lines are parallel.";

			return Vector2(determinant1 / determinant, determinant2 / determinant);
		}
	};
};

#endif
