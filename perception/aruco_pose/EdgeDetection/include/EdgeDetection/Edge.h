#ifndef __Edge_H__
#define __Edge_H__

#include "Vector2.h"
#include "Interpolation.h"

#include <cmath>

namespace EdgeDetection
{
	// Represents a two-dimensional edge by offset and angle
	class Edge
	{
		private: double offset;
		private: double angle;

		public: double GetOffset() { return offset; }
		public: double GetAngle() { return angle; }
		public: Vector2 GetCenter() { return Vector2::Multiply(offset, Vector2(angle)); }

		public: Edge()
		{
			this->offset = 0;
			this->angle = 0;
		}
		public: Edge(double offset, double angle)
		{
			this->offset = offset;
			this->angle = angle;

			while (this->angle < -M_PI) this->angle += 2 * M_PI;
			while (this->angle > +M_PI) this->angle -= 2 * M_PI;
		}
		public: ~Edge() { }

		public: static double GetDistance(Edge edge1, Edge edge2)
		{
			while (edge1.angle - edge2.angle < -M_PI) edge1.angle += 2 * M_PI;
			while (edge1.angle - edge2.angle > +M_PI) edge1.angle -= 2 * M_PI;

			double offsetDifference = edge1.offset - edge2.offset;
			double angleDifference = edge1.angle - edge2.angle;
	
			return Vector2(offsetDifference, angleDifference).GetLength();
		}
		public: static Edge Interpolate(Edge edge1, Edge edge2, double fraction)
		{
			while (edge1.angle - edge2.angle < -M_PI) edge1.angle += 2 * M_PI;
			while (edge1.angle - edge2.angle > +M_PI) edge1.angle -= 2 * M_PI;

			double offset = Interpolation::InterpolateForward(edge1.offset, edge2.offset, fraction);
			double angle = Interpolation::InterpolateForward(edge1.angle, edge2.angle, fraction);

			return Edge(offset, angle);
		}
	};
};

#endif
