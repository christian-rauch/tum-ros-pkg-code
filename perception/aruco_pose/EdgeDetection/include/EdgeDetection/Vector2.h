#ifndef __Vector2_H__
#define __Vector2_H__

#include <cmath>

namespace EdgeDetection
{
	// Represents a vector in two-dimensional space.
	class Vector2
	{
		private: double x;
		private: double y;

		public: double GetX() { return x; }
		public: double GetY() { return y; }
		public: double GetLength() { return sqrt(x * x + y * y); }

		public: Vector2()
		{
			this->x = 0;
			this->y = 0;
		}
		public: Vector2(double angle)
		{
			this->x = cos(angle);
			this->y = sin(angle);
		}
		public: Vector2(double x, double y)
		{
			this->x = x;
			this->y = y;
		}
		public: ~Vector2() { }
	
		public: static Vector2 Add(Vector2 vector1, Vector2 vector2) { return Vector2(vector1.x + vector2.x, vector1.y + vector2.y); }
		public: static Vector2 Subtract(Vector2 vector1, Vector2 vector2) { return Vector2(vector1.x - vector2.x, vector1.y - vector2.y); }
		public: static Vector2 Multiply(Vector2 vector, double factor) { return Vector2(vector.x * factor, vector.y * factor); }
		public: static Vector2 Multiply(double factor, Vector2 vector) { return Vector2(factor * vector.x, factor * vector.y); }
		public: static Vector2 Negate(Vector2 vector) { return Multiply(vector, -1); }
		public: static bool Equals(Vector2 vector1, Vector2 vector2) { return vector1.x == vector2.x && vector1.y == vector2.y; }
	};
};

#endif
