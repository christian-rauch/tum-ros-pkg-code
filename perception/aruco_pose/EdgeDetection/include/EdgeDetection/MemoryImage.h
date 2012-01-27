#ifndef __MemoryImage_H__
#define __MemoryImage_H__

#include "Image.h"

namespace EdgeDetection
{
	class MemoryImage : public Image
	{
		private: void* data;
		private: int stride;

		protected: void* GetData() { return data; }
		protected: int GetStride() { return stride; }

		protected: MemoryImage(int width, int height, void* data, int stride) : Image(width, height)
		{
			if (data == 0) throw "The parameter 'data' cannot be NULL.";
			if (stride <= 0) throw "The parameter 'stride' was out of range.";

			this->data = data;
			this->stride = stride;
		}
	};
	class RedByteGreenByteBlueByteMemoryImage : public MemoryImage
	{
		public: double Get(int x, int y)
		{
			if (x < 0 || x >= GetWidth()) throw "The parameter 'x' was out of range.";
			if (y < 0 || y >= GetHeight()) throw "The parameter 'y' was out of range.";
	
			unsigned char red = ((unsigned char*)GetData())[0 + x * 3 + y * GetStride()];
			unsigned char green = ((unsigned char*)GetData())[1 + x * 3 + y * GetStride()];
			unsigned char blue = ((unsigned char*)GetData())[2 + x * 3 + y * GetStride()];

			return (0.299 * red + 0.587 * green + 0.114 * blue) / 255;
		}

		public: RedByteGreenByteBlueByteMemoryImage(int width, int height, void* data, int stride) : MemoryImage(width, height, data, stride) { }

		public: Image* GetRegion(int left, int right, int top, int bottom);
	};
	class GrayDoubleMemoryImage : public MemoryImage
	{
		public: double Get(int x, int y)
		{
			if (x < 0 || x >= GetWidth()) throw "The parameter 'x' was out of range.";
			if (y < 0 || y >= GetHeight()) throw "The parameter 'y' was out of range.";
	
			return ((double*)GetData())[0 + x * 1 + y * GetStride()];
		}

		public: GrayDoubleMemoryImage(int width, int height, void* data, int stride) : MemoryImage(width, height, data, stride) { }

		public: Image* GetRegion(int left, int right, int top, int bottom);
	};
};

#endif
