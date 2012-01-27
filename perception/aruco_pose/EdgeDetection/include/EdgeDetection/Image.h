#ifndef __Image_H__
#define __Image_H__

namespace EdgeDetection
{
	class Image
	{
		private: int width;
		private: int height;
	
		public: int GetWidth() { return width; }
		public: int GetHeight() { return height; }
		public: virtual double Get(int x, int y) = 0;
	
		protected: Image(int width, int height)
		{
			if (width < 0) throw "The parameter 'width' was out of range.";
			if (height < 0) throw "The parameter 'height' was out of range.";

			this->width = width;
			this->height = height;
		}

		public: virtual Image* GetRegion(int left, int right, int top, int bottom) = 0;

		public: static double Combine(Image* image1, Image* image2);
	};
};

#endif
