/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "cop.h"
#include "cpp/HalconCpp.h"

#include "ShapeModel.h"
#include "ShapeBased3D.h"
#include "TwoInOneAlg.h"
#include "CheckColorClass.h"
#include "Blob.h"
#include "BlobLocalizer.h"
#include "ColorBased.h"
#include "RFAColorByShape.h"
#include "RFAPointDescrByShape.h"
#include "PRAShapeVsPointDescr.h"
#include <signal.h>
#define STD_CONFIG_FILENAME "bla.xml"
#ifdef OPENCV_USED
#include <cv.h>
#endif /*OPENCV_USED*/

#ifndef USE_YARP_COMM
#include <ros/ros.h>
#endif

#include "ShapeModelDownloader.h"

/*#include <pcrecpparg.h>
namespace pcrecpp
{

    Arg no_arg((void*)NULL);
}*/
#define XML_ATTRIBUTE_HASPTU "HasPTU"
#define XML_ATTRIBUTE_ISSTOC "IsSTOC"
#define XML_ATTRIBUTE_GRABBERNAME "GrabberName"
#define XML_ATTRIBUTE_CAMERATYPE "CameraType"
#define XML_ATTRIBUTE_CAMCOLOR "CamColor"
#define XML_ATTRIBUTE_IMGWIDTH "SetImageWidth"
#define XML_ATTRIBUTE_IMGHEIGHT "SetImageHeight"
#define XML_ATTRIBUTE_PANTOTILTHEIGHT "PanToTiltHeight"
#define XML_ATTRIBUTE_PANTOTILTWIDTH  "PanToTiltWidth"
#define XML_ATTRIBUTE_TILTTOENDHEIGHT "TiltToEndHeight"
#define XML_ATTRIBUTE_TILTTOENDWIDTH  "TiltToEndWidth"
#define XML_ATTRIBUTE_CAMPORT         "CamPort"
#define XML_ATTRIBUTE_CALIBFILE       "CalibFileName"
#define XML_ATTRIBUTE_HRESOLUTION     "Hresolution"
#define XML_ATTRIBUTE_VRESOLUTION     "Vresolution"
#define XML_ATTRIBUTE_STARTROW        "Startrow"
#define XML_ATTRIBUTE_STARTCOLUMN     "Startcolumn"
#define XML_ATTRIBUTE_FIELD           "Field"
#define XML_ATTRIBUTE_BITSPERCHANNEL  "Bitsperchannel"
#define XML_ATTRIBUTE_COLORSPACE       "Colospace"
#define XML_ATTRIBUTE_GAIN            "Gain"
#define XML_ATTRIBUTE_DEVICE          "Device"
#define XML_ATTRIBUTE_EXTERNALTRIGGER "Externaltrigger"
#define XML_ATTRIBUTE_LINEIN          "Linein"

using namespace cop;

/*void CalcMeshCenterBox(const Mesh_t &mesh, double &x, double &y, double &z, double &height,double &width, double &depth);*/
Mesh_t ReadMesh(std::string filename, double scale_halcon, Matrix m, int &points);
int main(int argc, char* argv[])
{
#ifndef USE_YARP_COMM
printf("seg\n");
    ros::init(argc, argv, "cop");
    printf("seg\n");
#endif
    try
    {
    if(argc > 0)
    {
        XMLTag* config = NULL;
        if(argc > 1)
            config = XMLTag::ReadFromFile(argv[1]);
        else
            config = XMLTag::ReadFromFile(STD_CONFIG_FILENAME);
        if(config == NULL)
            config = new XMLTag("config");
        /*Blob* blob = new Blob(235, 255, 0, 35, 0, 40, 750, 5000, 0.4, 0.095, 0.05);*/

        printf("Init start:\n");
        cop_world copWorld(config);
        /*XMLTag tag(XML_NODE_CAMERADRIVER);
        tag.AddProperty(XML_ATTRIBUTE_GRABBERNAME, "DirectShow");
        tag.AddProperty(XML_ATTRIBUTE_EXTERNALTRIGGER, "false");
        tag.AddProperty(XML_ATTRIBUTE_CAMERATYPE, "RGB24 (640x480)");
        tag.AddProperty(XML_ATTRIBUTE_DEVICE, "USB PC Camera (SN9C102)");
        tag.AddProperty(XML_ATTRIBUTE_CALIBFILE, "C:/__CODE/radigsvn/cop/bin/fakeparam_usbwebcam.dat");
        tag.AddProperty(XML_ATTRIBUTE_CAMPORT, "0");
        tag.AddProperty(XML_ATTRIBUTE_COLORSPACE, "rgb");
        copWorld.s_inputSystem->AddCamera(new CameraDriver(&tag));
        copWorld.s_visFinder->AddAlgorithm(new ShapeBased3D());   */
/*        copWorld.s_visFinder->AddAlgorithm(new BlobLocalizer());
        Signature* sig = new Signature();
        Class* cl = new Class();
        Class* clOra = new Class();
        cl->SetName("EasterEgg");
        clOra->SetName("Orange");
        sig->SetClass(cl);
        sig->SetClass(clOra);
        sig->SetElem(blob);
        copWorld.s_sigDb->AddSignature(sig);*/
        printf("Inited\n");
#ifdef USE_YARP_COMM
        copWorld.StartListeningYarpPort((char*)"/tracking/in");
#else
        if(copWorld.s_visFinder->CountAlgorithms() == 0)
           printf("Warning: no algorithm loaded\n");
        copWorld.StartNodeSubscription((char*)"/tracking/in");

        printf("Returned successfully\n");
#endif
        if(g_stopall == true)
        {
          if(argc > 1)
            copWorld.SaveCop(STD_CONFIG_FILENAME);
          else
            copWorld.SaveCop(STD_CONFIG_FILENAME);
        }
      }
    }
#ifdef HALCONIMG
    catch(Halcon::HException ex)
    {
        printf("Error in cop_main: %s\n", ex.message);
    }
#endif /*HALCONIMG*/
    catch(char* text)
    {
      printf("Error in cop_main: %s\n", text);
    }
    catch(...)
    {
      printf("Unhabdled Exception: stopping program\n");
    }
    printf("exiting %s\n", argv[0]);
    return 0;
}

