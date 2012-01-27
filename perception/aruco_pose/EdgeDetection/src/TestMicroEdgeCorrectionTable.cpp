#include "EdgeDetection/AdaptiveRasterizer.h"
#include "EdgeDetection/MicroEdgeDetector.h"
#include "EdgeDetection/MicroEdgeCorrectionTable.h"
#include "EdgeDetection/EdgeImage.h"
#include "EdgeDetection/Image.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace EdgeDetection;

double MeasureError(Rasterizer* rasterizer, MicroEdgeDetector* microEdgeDetector, MicroEdgeCorrectionTable* microEdgeCorrectionTable, Edge realEdge)
{
	EdgeImage* realEdgeImage = new EdgeImage(Line::FromEdge(realEdge), Rectangle(-1, +1, -1, +1));
	Image* realImage = rasterizer->Rasterize(realEdgeImage, 5, 5);
	Edge measuredEdge = microEdgeDetector->DetectEdge(realImage);
	Edge correctedEdge = microEdgeCorrectionTable->Correct(measuredEdge);

	delete realEdgeImage;
	delete realImage;

	double error = Edge::GetDistance(realEdge, correctedEdge);

/*
	if (error > 0.1)
	{
		cout << "Real: [" << realEdge.GetOffset() << ", " << realEdge.GetAngle() << "]" << endl;
		cout << "Measured: [" << measuredEdge.GetOffset() << ", " << measuredEdge.GetAngle() << "]" << endl;
		cout << "Corrected: [" << correctedEdge.GetOffset() << ", " << correctedEdge.GetAngle() << "]" << endl;
		cout << "Error: " << error << endl;
		cout << endl;
	}
*/

	return error;
}
double NextDouble()
{
	return (double)rand() / (double)RAND_MAX;
}

int main(int argc, char** argv)
{
	cout << "Creating Rasterizer (fine)..." << endl;
	Rasterizer* rasterizerFine = new AdaptiveRasterizer(0.00000001);
	cout << "Creating MicroEdgeDetector..." << endl;
	MicroEdgeDetector* microEdgeDetector = new MicroEdgeDetector(rasterizerFine, 5);
	cout << "Loading MicroEdgeCorrectionTable..." << endl;
	ifstream microEdgeCorrectionTableFile("MicroEdgeCorrectionTable.bin", ifstream::in | ifstream::binary);
	MicroEdgeCorrectionTable* microEdgeCorrectionTable = MicroEdgeCorrectionTable::Deserialize(&microEdgeCorrectionTableFile);
	microEdgeCorrectionTableFile.close();
	cout << endl;

	cout << "Generating error statistics..." << endl;

	double errorMinimum = -1;
	double errorMaximum = -1;
	double errorSum = 0;

	srand(time(NULL));

	int sampleCount = 10000;

	for (int i = 0; i < sampleCount; i++)
	{
		Edge realEdge = Edge((1.2 * NextDouble()) - 0.6, (2 * M_PI * NextDouble()) - M_PI);

		double error = MeasureError(rasterizerFine, microEdgeDetector, microEdgeCorrectionTable, realEdge);

		if (errorMinimum == -1 || error < errorMinimum) errorMinimum = error;
		if (errorMaximum == -1 || error > errorMaximum) errorMaximum = error;
		errorSum += error;
	}

	double errorAverage = errorSum / sampleCount;

	cout << "Errors: Minimum: " << errorMinimum << ", Maximum: " << errorMaximum << ", Average: " << errorAverage << endl;

	delete rasterizerFine;
	delete microEdgeDetector;
	delete microEdgeCorrectionTable;

	return 0;
}
