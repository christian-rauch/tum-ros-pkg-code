#include "EdgeDetection/AdaptiveRasterizer.h"
#include "EdgeDetection/MicroEdgeDetector.h"
#include "EdgeDetection/MicroEdgeMap.h"
#include "EdgeDetection/MicroEdgeCorrectionTable.h"

#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;
using namespace EdgeDetection;

int main(int argc, char** argv)
{
	cout << "Creating Rasterizer (coarse)..." << endl;
	Rasterizer* rasterizerCoarse = new AdaptiveRasterizer(0.0001);
	cout << "Creating Rasterizer (fine)..." << endl;
	Rasterizer* rasterizerFine = new AdaptiveRasterizer(0.00000001);
	cout << "Creating MicroEdgeDetector..." << endl;
	MicroEdgeDetector* microEdgeDetector = new MicroEdgeDetector(rasterizerFine, 5);
	cout << "Creating MicroEdgeMap..." << endl;
	MicroEdgeMap* microEdgeMap = new MicroEdgeMap(rasterizerCoarse, microEdgeDetector, Rectangle(-0.6, +0.6, -M_PI, +M_PI), 1000, 1000);
	cout << "Creating MicroEdgeCorrectionTable..." << endl;
	MicroEdgeCorrectionTable* microEdgeCorrectionTable = new MicroEdgeCorrectionTable(microEdgeMap, Rectangle(-0.6, +0.6, -M_PI, +M_PI), 50, 50);
	cout << endl;

	cout << "Saving MicroEdgeCorrectionTable..." << endl;
	istream* microEdgeCorrectionTableData = MicroEdgeCorrectionTable::Serialize(microEdgeCorrectionTable);
	ofstream microEdgeCorrectionTableFile("MicroEdgeCorrectionTable.bin", ofstream::out | ofstream::binary);
	microEdgeCorrectionTableFile << microEdgeCorrectionTableData->rdbuf();
	microEdgeCorrectionTableFile.close();
	delete microEdgeCorrectionTableData;

	delete rasterizerCoarse;
	delete rasterizerFine;
	delete microEdgeDetector;
	delete microEdgeMap;
	delete microEdgeCorrectionTable;

	return 0;
}
