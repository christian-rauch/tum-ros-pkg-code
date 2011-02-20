/***************************************************************************
 *            main.cpp
 *
 *
 *  Copyright  2010  Tristen Georgiou
 *  tristen_georgiou@hotmail.com
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <string.h>
#include <stdio.h>
//#include <time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "fast_match_template/FastMatchTemplate.h"

using namespace cv;

int searchForParameter(const char* param,
                       int         numArgs,
                       char**      argList);

void printOptions();

int main(int argc, char** argv)
{
    if( argc != 7 )
    {
        printf( "\nERROR: Required arguments missing.\n" );
        printOptions();
        return 1;
    }

    int sourceParam = searchForParameter("-s", argc, argv);
    if(sourceParam == -1)
    {
        printf( "\nERROR: Source argument missing.\n" );
        return 1;
    }

    int targetParam = searchForParameter("-t", argc, argv);
    if(targetParam == -1)
    {
        printf( "\nERROR: Target argument missing.\n" );
        return 1;
    }

    int nameParam = searchForParameter("-n", argc, argv);
    if(nameParam == -1)
      {
        printf( "\nERROR: Name argument missing.\n" );
        return 1;
    }

    Mat source = imread(argv[++sourceParam]);
    if(!source.data)
    {
        printf("\nERROR: Could not load image %s.\n", argv[sourceParam]);
        return 2;
    }

    Mat target = imread(argv[++targetParam]);
    if(!target.data)
    {
        printf("\nERROR: Could not load image %s.\n", argv[targetParam]);
        return 2;
    }

    // perform the match
    vector<Point> foundPointsList;
    vector<double> confidencesList;

    // start the timer
    //clock_t start = clock();
    if(!FastMatchTemplate(source,
                          target,
                          &foundPointsList,
                          &confidencesList))
    {
        printf("\nERROR: Fast match template failed.\n");
        return 3;
    }
    //clock_t elapsed = clock() - start;
    //long int timeMs = ((float)elapsed) / ((float)CLOCKS_PER_SEC) * 1000.0f;
    //printf("\nINFO: FastMatchTemplate() function took %ld ms.\n", timeMs);

    // create a color image to draw on
    Mat colorImage;

    // if the original is a grayscale image, convert it to color
    if(source.channels() == 1)
    {
        cvtColor(source, colorImage, CV_GRAY2RGB);
    }
    else
    {
        colorImage = source.clone();
    }

    DrawFoundTargets(&colorImage,
                     target.size(),
                     foundPointsList,
                     confidencesList);

    // wait for both windows to be closed before releasing images
    imshow("Results", colorImage);
    waitKey(0);
    imwrite(string(argv[++nameParam]) + ".png", colorImage);
    return 0;
}

int searchForParameter(const char* param,
                       int         numArgs,
                       char**      argList)
{
    for(int currArg = 0; currArg < numArgs; currArg++)
    {
        if(strcmp( param, argList[currArg] ) == 0)
        {
            return currArg;
        }
    }

    // argument not found
    return -1;
}

void printOptions()
{
    printf("\nFAST MATCH TEMPLATE EXAMPLE PROGRAM\n");
    printf("-----------------------------------\n");
    printf("\nProgram arguments:\n\n" );
    printf("     -s source image name (image to be searched)\n\n");
    printf("     -t target image name (image we are trying to find)\n\n");
    printf("     -n result image name (name of result image we are saving)\n\n");
    printf("Example: FastMatchTemplate -s source.bmp -t target.bmp -n result.png \n\n");
}
