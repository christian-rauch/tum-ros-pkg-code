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
#include "pluginlib/class_list_macros.h"


/*Includes for Sensor Plugins*/
  /*interface of cognitive_perception*/
#include "Sensor.h"

#include "CameraDriver.h"
#include "SimulatedCamera.h"
#include "ROSCamera.h"
#include "StereoSensor.h"

/*Includes for Reading Plugins*/
  /*interface of cognitive_perception*/
#include "Reading.h"

#include "Image.h"
/* Image Includes also for ReadingConverter Plugins*/



/*Includes for Descriptor Plugins*/
  /*interface of cognitive_perception*/
#include "Descriptor.h"

#include "ShapeModel.h"
#include "CalTab.h"
#include "DeformShapeModel.h"
#include "ColorClass.h"
#include "Blob.h"
#include "SupportingPlaneDescriptor.h"

/*Includes for LocaeAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "LocateAlgorithm.h"

#include "ShapeBased3D.h"
#include "FindCalTab.h"
#include "DeformShapeBased.h"
#include "CheckColorClass.h"
#include "BlobLocalizer.h"
#include "TwoInOneAlg.h"
#include "SupportingPlaneDetector.h"
#include "SimulatedLocate.h"

/*Includes for RefineAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "RefineAlgorithm.h"

#include "RFADeformByCluster.h"
#include "RFAPointDescrByShape.h"
#include "RFAColorByShape.h"
#include "ShapeModelDownloader.h"


#include "cpp/HalconCpp.h"
//This procedure simply hands the exception object to the C++ exception handling via throw:
void MyHalconExceptionHandler(const Halcon::HException& except)
{
  throw except;
}


using namespace cop;

void loadLib(void) __attribute__ ((constructor));

void loadLib(void)
{
    Image::RegisterImageConverter();
    Halcon::HException::InstallHHandler(&MyHalconExceptionHandler);
}

/*Sensor Plugins*/
PLUGINLIB_REGISTER_CLASS(CameraDriver, CameraDriver, Sensor);
PLUGINLIB_REGISTER_CLASS(CameraDriverRelay, CameraDriverRelay, Sensor);

PLUGINLIB_REGISTER_CLASS(SimulatedCamera, SimulatedCamera, Sensor);
PLUGINLIB_REGISTER_CLASS(ROSCOPCamera, ROSCOPCamera, Sensor);
PLUGINLIB_REGISTER_CLASS(StereoSensor, StereoSensor, Sensor);

/*Reading Plugin*/
PLUGINLIB_REGISTER_CLASS(ImageFile, Image, Reading);

/*Descriptor Plugins*/
PLUGINLIB_REGISTER_CLASS(ShapeModel, ShapeModel, Descriptor);
PLUGINLIB_REGISTER_CLASS(CalTab, CalTab, Descriptor);
PLUGINLIB_REGISTER_CLASS(DeformShapeModel, DeformShapeModel, Descriptor);
PLUGINLIB_REGISTER_CLASS(ColorClass, ColorClass, Descriptor);
PLUGINLIB_REGISTER_CLASS(Blob, Blob, Descriptor);
PLUGINLIB_REGISTER_CLASS(SupportingPlaneDescriptor, SupportingPlaneDescriptor, Descriptor);

/*LocateAlgorithm Plugins*/
PLUGINLIB_REGISTER_CLASS(ShapeBased3DAlg, ShapeBased3D, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(FindCalTab, FindCalTab, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(DeformShapeBased, DeformShapeBased, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(DeformShapeBasedAlg, DeformShapeBased, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(CheckColorClass, CheckColorClass, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(BlobLocalizer, BlobLocalizer, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(TwoInOneAlg, TwoInOneAlg, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(SupportingPlaneDetector, SupportingPlaneDetector, LocateAlgorithm);
PLUGINLIB_REGISTER_CLASS(SimulatedLocate, SimulatedLocate, LocateAlgorithm);


/*RefineAlgorithm Plugins*/
PLUGINLIB_REGISTER_CLASS(RFADeformByCluster, RFADeformByCluster, RefineAlgorithm);
PLUGINLIB_REGISTER_CLASS(RFAColorByShape, RFAColorByShape, RefineAlgorithm);
PLUGINLIB_REGISTER_CLASS(ShapeModelDownloader, ShapeModelDownloader, RefineAlgorithm);
