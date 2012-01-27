#ifndef __Rectangle_H__
#define __Rectangle_H__

#include "Vector2.h"

namespace EdgeDetection
{
	// Represents an axis-aligned rectangle in two-dimensional space.
	class Rectangle
	{
		private: Vector2 start;
		private: Vector2 end;
	
		public: Vector2 GetLeftTop() { return Vector2(GetLeft(), GetTop()); }
		public: Vector2 GetRightTop() { return Vector2(GetRight(), GetTop()); }
		public: Vector2 GetLeftBottom() { return Vector2(GetLeft(), GetBottom()); }
		public: Vector2 GetRightBottom() { return Vector2(GetRight(), GetBottom()); }
		public: Vector2 GetCenter() { return Vector2(0.5 * (GetLeft() + GetRight()), 0.5 * (GetTop() + GetBottom())); }
		public: double GetLeft() { return start.GetX(); }
		public: double GetRight() { return end.GetX(); }
		public: double GetTop() { return start.GetY(); }
		public: double GetBottom() { return end.GetY(); }
		public: double GetWidth() { return GetRight() - GetLeft(); }
		public: double GetHeight() { return GetBottom() - GetTop(); }
		public: double GetArea() { return GetWidth() * GetHeight(); }

		public: Rectangle()
		{
			this->start = Vector2();
			this->end = Vector2();
		}
		public: Rectangle(double left, double right, double top, double bottom)
		{
			this->start = Vector2(left, top);
			this->end = Vector2(right, bottom);
		}
		public: ~Rectangle() { }
	};
};

#endif
