#ifndef __MarkerRefiner__
#define __MarkerRefiner__

#include <string>
#include <fstream>

#include <opencv/cv.h>

#include "aruco/aruco.h"

#include "EdgeDetection/AdaptiveRasterizer.h"
#include "EdgeDetection/MicroEdgeDetector.h"
#include "EdgeDetection/MicroEdgeCorrectionTable.h"
#include "EdgeDetection/PointCorrector.h"
#include "EdgeDetection/MacroEdgeDetector.h"
#include "EdgeDetection/Vector2.h"

class MarkerRefiner
{
	private: static unsigned char microEdgeCorrectionTableData[];

	private: EdgeDetection::Rasterizer* rasterizerFine;
	private: EdgeDetection::MicroEdgeDetector* microEdgeDetector;
	private: EdgeDetection::MicroEdgeCorrectionTable* microEdgeCorrectionTable;
	private: EdgeDetection::MacroEdgeDetector* macroEdgeDetector;

	public: MarkerRefiner(std::string correctionTablePath, EdgeDetection::PointCorrector* pointCorrector)
	{
		this->rasterizerFine = new EdgeDetection::AdaptiveRasterizer(0.00000001);
		this->microEdgeDetector = new EdgeDetection::MicroEdgeDetector(rasterizerFine, 5);
		std::ifstream microEdgeCorrectionTableFile(correctionTablePath.c_str(), std::ifstream::in | std::ifstream::binary);
		this->microEdgeCorrectionTable = EdgeDetection::MicroEdgeCorrectionTable::Deserialize(&microEdgeCorrectionTableFile);
		microEdgeCorrectionTableFile.close();
		this->macroEdgeDetector = new EdgeDetection::MacroEdgeDetector(microEdgeDetector, microEdgeCorrectionTable, pointCorrector);
	}
	public: MarkerRefiner(EdgeDetection::PointCorrector* pointCorrector)
	{
		this->rasterizerFine = new EdgeDetection::AdaptiveRasterizer(0.00000001);
		this->microEdgeDetector = new EdgeDetection::MicroEdgeDetector(rasterizerFine, 5);
		std::stringstream data(std::stringstream::in | std::stringstream::out | std::stringstream::binary);
		data.write((const char*)microEdgeCorrectionTableData, 40040);
		this->microEdgeCorrectionTable = EdgeDetection::MicroEdgeCorrectionTable::Deserialize(&data);
		this->macroEdgeDetector = new EdgeDetection::MacroEdgeDetector(microEdgeDetector, microEdgeCorrectionTable, pointCorrector);
	}
	public: ~MarkerRefiner()
	{
		delete rasterizerFine;
		delete microEdgeDetector;
		delete microEdgeCorrectionTable;
		delete macroEdgeDetector;
	}

	public: aruco::Marker refineMarker(aruco::Marker marker, Mat matrixImage);

	private: static bool isCloseToBorder(Mat matrixImage, EdgeDetection::Vector2 point);
};

#endif
