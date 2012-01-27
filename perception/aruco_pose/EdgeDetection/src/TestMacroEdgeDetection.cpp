#include "EdgeDetection/AdaptiveRasterizer.h"
#include "EdgeDetection/MicroEdgeDetector.h"
#include "EdgeDetection/MicroEdgeCorrectionTable.h"
#include "EdgeDetection/PointCorrector.h"
#include "EdgeDetection/MacroEdgeDetector.h"
#include "EdgeDetection/EdgeImage.h"
#include "EdgeDetection/Image.h"
#include "EdgeDetection/OwnedImage.h"
#include "EdgeDetection/Edge.h"
#include "EdgeDetection/Line.h"
#include "EdgeDetection/BoundedLine.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace EdgeDetection;

class NullPointCorrector : public PointCorrector
{
	public: Vector2 CorrectPoint(Vector2 point) { return point; }
};

double NextDouble()
{
	return (double)rand() / (double)RAND_MAX;
}
double NextDouble(double minimum, double maximum)
{
	return minimum + (maximum - minimum) * NextDouble();
}
double Clamp(double value, double minimum, double maximum)
{
	value = max(value, minimum);
	value = min(value, maximum);

	return value;
}
void SaveBitmap(Image* image, string path)
{
	ifstream header("Header.bmp", ifstream::in | ifstream::binary);

	ofstream bitmap(path.c_str(), ofstream::out | ofstream::binary);

	bitmap << header.rdbuf();

	for (int y = 0; y < image->GetHeight(); y++)
		for (int x = 0; x < image->GetWidth(); x++)
		{
			unsigned char value = (unsigned char)(image->Get(x, y) * 255);

			bitmap.write((const char*)&value, sizeof(unsigned char));
		}

	bitmap.close();

	header.close();
}
double MeasureError(MacroEdgeDetector* macroEdgeDetector, BoundedLine realEdge, Image* image, double noiseLevel)
{
	Line normalizedRealEdge = Line::Normalize(realEdge);

	OwnedImage* noiseImage = new OwnedImage(image);	
	for (int y = 0; y < noiseImage->GetHeight(); y++)
		for (int x = 0; x < noiseImage->GetWidth(); x++)
			noiseImage->Set(x, y, Clamp(noiseImage->Get(x, y) + NextDouble(-noiseLevel, +noiseLevel), 0, 1));

	//stringstream path;
	//path << "test-" << noiseLevel << ".bmp";
	//SaveBitmap(noiseImage, path.str());

	BoundedLine estimatedEdge = BoundedLine(Vector2(realEdge.GetStart().GetX() + NextDouble(-0.5, +0.5), realEdge.GetStart().GetY() + NextDouble(-0.5, +0.5)), Vector2(realEdge.GetEnd().GetX() + NextDouble(-0.5, +0.5), realEdge.GetEnd().GetY() + NextDouble(-0.5, +0.5)));
	Line normalizedEstimatedEdge = Line::Normalize(estimatedEdge);

	Line measuredEdge = macroEdgeDetector->DetectEdge(noiseImage, estimatedEdge);
	Line normalizedMeasuredEdge = Line::Normalize(measuredEdge);

	double realSlope = normalizedRealEdge.GetFactorX() / normalizedRealEdge.GetFactorY();
	double measuredSlope = normalizedMeasuredEdge.GetFactorX() / normalizedMeasuredEdge.GetFactorY();

	if (abs(realSlope) > 1)
	{
		realSlope = 1 / realSlope;
		measuredSlope = 1 / measuredSlope;
	}
	
	double error = abs(realSlope - measuredSlope);

/*
	if (error > 0.01)
	{
		cout << "RealEdge: " << "[" << "(" << realEdge.GetStart().GetX() << ", " << realEdge.GetStart().GetY() << ")" << ", " << "(" << realEdge.GetEnd().GetX() << ", " << realEdge.GetEnd().GetY() << ")" << "]" << endl;
		cout << "RealEdge: (" << normalizedRealEdge.GetFactorX() << ") * x + (" << normalizedRealEdge.GetFactorY() << ") * y + (" << normalizedRealEdge.GetOffset() << ") = 0" << endl;
		cout << "EstimatedEdge: " << "[" << "(" << estimatedEdge.GetStart().GetX() << ", " << estimatedEdge.GetStart().GetY() << ")" << ", " << "(" << estimatedEdge.GetEnd().GetX() << ", " << estimatedEdge.GetEnd().GetY() << ")" << "]" << endl;
		cout << "EstimatedEdge: (" << normalizedEstimatedEdge.GetFactorX() << ") * x + (" << normalizedEstimatedEdge.GetFactorY() << ") * y + (" << normalizedEstimatedEdge.GetOffset() << ") = 0" << endl;
		cout << "MeasuredEdge: (" << normalizedMeasuredEdge.GetFactorX() << ") * x + (" << normalizedMeasuredEdge.GetFactorY() << ") * y + (" << normalizedMeasuredEdge.GetOffset() << ") = 0" << endl;
		cout << "Error: " << error << endl;
		cout << endl;
	}
*/

	delete noiseImage;

	return error;
}
void GenerateStatistics(Rasterizer* rasterizer, MacroEdgeDetector* macroEdgeDetector, double noiseLevel, int edgeSampleCount)
{
	cout << "Generating statistics (Noise level: " << noiseLevel << ")... ";

    double errorMinimum = -1;
    double errorMaximum = -1;
    double errorSum = 0;

    for (int i = 0; i < edgeSampleCount; i++)
    {
		BoundedLine line;
		while (line.GetLength() < 50) line = BoundedLine(Vector2(NextDouble(5, 95), NextDouble(5, 95)), Vector2(NextDouble(5, 95), NextDouble(5, 95)));

		EdgeImage* edgeImage = new EdgeImage(line, Rectangle(-0.5, +99.5, -0.5, +99.5));
		Image* image = rasterizer->Rasterize(edgeImage, 100, 100);

		double error = MeasureError(macroEdgeDetector, line, image, noiseLevel);

		if (errorMinimum == -1 || error < errorMinimum) errorMinimum = error;
		if (errorMaximum == -1 || error > errorMaximum) errorMaximum = error;
		errorSum += error;

		delete image;
		delete edgeImage;
    }

    double errorAverage = errorSum / edgeSampleCount;

    cout << "Errors: Minimum: " << errorMinimum << ", Maximum: " << errorMaximum << ", Average: " << errorAverage << endl;
}

int main(int argc, char** argv)
{
	cout << "Creating Rasterizer (coarse)..." << endl;
	Rasterizer* rasterizerCoarse = new AdaptiveRasterizer(0.0001);
	cout << "Creating Rasterizer (fine)..." << endl;
	Rasterizer* rasterizerFine = new AdaptiveRasterizer(0.00000001);
	cout << "Creating MicroEdgeDetector..." << endl;
	MicroEdgeDetector* microEdgeDetector = new MicroEdgeDetector(rasterizerFine, 5);
	cout << "Loading MicroEdgeCorrectionTable..." << endl;
	ifstream microEdgeCorrectionTableFile("MicroEdgeCorrectionTable.bin", ifstream::in | ifstream::binary);
	MicroEdgeCorrectionTable* microEdgeCorrectionTable = MicroEdgeCorrectionTable::Deserialize(&microEdgeCorrectionTableFile);
	microEdgeCorrectionTableFile.close();
	cout << "Creating NullPointCorrector..." << endl;
	PointCorrector* nullPointCorrector = new NullPointCorrector();
	cout << "Creating MacroEdgeDetector..." << endl;
	MacroEdgeDetector* macroEdgeDetector = new MacroEdgeDetector(microEdgeDetector, microEdgeCorrectionTable, nullPointCorrector);
	cout << endl;

	cout << "Generating statistics..." << endl;

    srand(time(NULL));

	for (double noiseLevel = 0; noiseLevel <= 1; noiseLevel += 0.1) GenerateStatistics(rasterizerCoarse, macroEdgeDetector, noiseLevel, 100);

	delete rasterizerCoarse;
	delete rasterizerFine;
	delete microEdgeDetector;
	delete microEdgeCorrectionTable;
	delete nullPointCorrector;
	delete macroEdgeDetector;

	return 0;
}
