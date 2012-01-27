#ifndef __OwnedImage_H__
#define __OwnedImage_H__

#include "Image.h"

namespace EdgeDetection
{
	class OwnedImage : public Image
	{
		private: double* values;
	
		public: double Get(int x, int y)
		{
			if (x < 0 || x >= GetWidth()) throw "The parameter 'x' was out of range.";
			if (y < 0 || y >= GetHeight()) throw "The parameter 'y' was out of range.";
	
			return values[x + y * GetWidth()];
		}
		public: void Set(int x, int y, double value)
		{
			if (x < 0 || x >= GetWidth()) throw "The parameter 'x' was out of range.";
			if (y < 0 || y >= GetHeight()) throw "The parameter 'y' was out of range.";
	
			values[x + y * GetWidth()] = value;
		}

		public: OwnedImage(int width, int height) : Image(width, height)
		{
			this->values = new double[GetWidth() * GetHeight()];
		
			for (int index = 0; index < GetWidth() * GetHeight(); index++) values[index] = 0;
		}
		public: OwnedImage(Image* image) : Image(image->GetWidth(), image->GetHeight())
		{
			this->values = new double[GetWidth() * GetHeight()];

			for (int y = 0; y < GetHeight(); y++)
				for (int x = 0; x < GetWidth(); x++)
					Set(x, y, image->Get(x, y));
		}
		public: ~OwnedImage()
		{
			delete[] this->values;
		}

		public: Image* GetRegion(int left, int right, int top, int bottom);
	};
};

#endif
