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

 

#include "VM_InTableCS.h"

#define D2R(X) (X*M_PI/180.0) 
#define precision 1e-9


using namespace Halcon;
using namespace std;
void
caltab2table_pose_incam (const HTuple & CaltabPose, HTuple * pTableCSPose)
{
//ROTATE AND TRANSLATE THE CALIBRATION PLATE POSE TO GET TABLE ORIGIN:
  HTuple id_matrix, rot_matrixZ, rot_matrixZY, fin_transf_matrix,
    CaltabPoseHM, TableCSHomMat;

  pose_to_hom_mat3d ((const HTuple) CaltabPose, &CaltabPoseHM);
  hom_mat3d_identity (&id_matrix);
  HTuple fin_transf_matrix_Z;
  HTuple FirstRot, SecondRot, TransX, TransY;


  /*if (IsRightTableCorner)
    {*/
	hom_mat3d_rotate ((const HTuple) id_matrix,
			(const HTuple) -PI / 2.0,
			(const HTuple) "Z",
			(const HTuple) 0,
			(const HTuple) 0, (const HTuple) 0, &rot_matrixZ);
      TransX = 0.1;//0.0965; (IF THE CALIB_TAB IS ACTUALLY 0.193X0.193M!!!)
      TransY = 0.1;//0.0965;
   // }
  /*else
    {
      rot_matrixZ = id_matrix;
      TransX = 0.1;
      TransY = -1.1;
    }*/
  hom_mat3d_rotate ((const HTuple) rot_matrixZ,
		    (const HTuple) PI,
		    (const HTuple) "Y",
		    (const HTuple) 0,
		    (const HTuple) 0, (const HTuple) 0, &rot_matrixZY);
  hom_mat3d_translate ((const HTuple) rot_matrixZY,
		       (const HTuple) TransX,
		       (const HTuple) TransY,
		       (const HTuple) 0, &fin_transf_matrix);
  hom_mat3d_compose ((const HTuple) CaltabPoseHM,
		     (const HTuple) fin_transf_matrix, &TableCSHomMat);
  hom_mat3d_to_pose (TableCSHomMat, pTableCSPose);
}

void	obj_pose_cam2ref (const HTuple RefCSPoseInCam, const HTuple ObjPoseInCam,
		  HTuple * pObjPoseInRefCS)
{
  //cerr << "RefCSPoseInCam: " << RefCSPoseInCam << endl;
  //cerr << "ObjPoseInCam: " << ObjPoseInCam << endl;
  HTuple TransfMat, RefCSHMInCam, ObjHMInCam, ObjHMInRefCS;
  pose_to_hom_mat3d (RefCSPoseInCam, &RefCSHMInCam);
  pose_to_hom_mat3d (ObjPoseInCam, &ObjHMInCam);
  hom_mat3d_invert (RefCSHMInCam, &TransfMat);
  hom_mat3d_compose (TransfMat, ObjHMInCam, &ObjHMInRefCS);
  hom_mat3d_to_pose (ObjHMInRefCS, pObjPoseInRefCS);
}

void obj_pose_ref2cam (const HTuple RefCSPoseInCam, const HTuple ObjPoseInRefCS,
		  HTuple * pObjPoseInCamCS)
{
  HTuple ObjHMInCamCS, RefCSHMInCam, ObjHMInRefCS;
  pose_to_hom_mat3d (RefCSPoseInCam, &RefCSHMInCam);
  pose_to_hom_mat3d (ObjPoseInRefCS, &ObjHMInRefCS);
  hom_mat3d_compose (RefCSHMInCam, ObjHMInRefCS, &ObjHMInCamCS);
  hom_mat3d_to_pose (ObjHMInCamCS, pObjPoseInCamCS);
	//cerr<<"Everything fine!!!!!"<<endl;
}

/////////////////////////////////////
/////////////////////////////////////

HTuple get_estd_pose_and_covmatrix(HTuple TableCSPoseInCam, HTuple * pMeanPoseInCam, 
HTuple InputMeanPoseInTable, HTuple RegionSize)
{ 
Matrix CovInTCS(6,6);
CovInTCS=0;
HTuple InputMeanHMInTable;

	  CovInTCS.element (0, 0) = RegionSize[0].D()/2;
	  CovInTCS.element (1, 1) = RegionSize[1].D()/2;
	  CovInTCS.element (2, 2) = RegionSize[2].D()/2;
	  CovInTCS.element (3, 3) = RegionSize[3].D()/2;
	  CovInTCS.element (4, 4) = RegionSize[4].D()/2;
	  CovInTCS.element (5, 5) = RegionSize[5].D()/2;
	  InputMeanPoseInTable[6]=0;
	  pose_to_hom_mat3d(InputMeanPoseInTable,&InputMeanHMInTable);

HTuple TableCSHMInObjCS,TableCSPoseInObjCS;

hom_mat3d_invert(InputMeanHMInTable,&TableCSHMInObjCS);
hom_mat3d_to_pose(TableCSHMInObjCS,&TableCSPoseInObjCS);

Matrix EPosesInTableCS(6,12),EPosesInObjectCS, EPosesInCamCS(6,12);

EPosesInTableCS=GetExtremePoses(CovInTCS);
//cerr<<"EPosesInTableCS"<<endl<<EPosesInTableCS<<endl;

EPosesInObjectCS = TableOffsetsToObjectOffsets(EPosesInTableCS,InputMeanPoseInTable);

obj_pose_ref2cam(TableCSPoseInCam,InputMeanPoseInTable,pMeanPoseInCam);


//EPosesInObjectCS=Trans12Poses(EPosesInTableCS,TableCSPoseInObjCS);
cerr<<"EPosesInObjectCS"<<endl<<EPosesInObjectCS<<endl;
//GET OBJECT CS IN CAM CS

EPosesInCamCS=Trans12Poses(EPosesInObjectCS,*pMeanPoseInCam);
cerr<<"EPosesInCamCS"<<endl<<EPosesInCamCS<<endl;

Matrix CovInCCS(6,6);
CovInCCS=0.0;

ColumnVector MeanPoseInCam(6);
for(int i=0;i<6;i++)
	MeanPoseInCam.element(i)=(*pMeanPoseInCam)[i];

CovInCCS=CreateCov(EPosesInCamCS,MeanPoseInCam);
HTuple TableAreaCovMatrix(36,0.0);

for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
			{
				TableAreaCovMatrix[i * 6 + j] = CovInCCS.element (i, j);
			}
	}
  return TableAreaCovMatrix;
}

ReturnMatrix Trans12Poses(Matrix EPInCS1, HTuple PoseCS1inCS2)
{
	//Takes a tuple containing 12 poses, transforms them into an other CS and creates a MATRIX of them
	HTuple EPInCS1Tuple(84,0.0),EPInCS2Tuple;
	HTuple CurrEPose(7,0.0),CurrPoseCS2(7,0.0);
	mat2tuple(EPInCS1, 0, &EPInCS1Tuple);
	Matrix EPInCS2(6,12);
	HTuple PoseCS2inCS1, HMCS2inCS1, HMCS1inCS2,CurrHMCS2,CurrOffHMCS2;

	pose_to_hom_mat3d(PoseCS1inCS2, &HMCS1inCS2);
	hom_mat3d_invert(HMCS1inCS2,&HMCS2inCS1);
	hom_mat3d_to_pose(HMCS2inCS1,&PoseCS2inCS1);

	for (int i =0;i<12;i++)
		{
			tuple_select_range(EPInCS1Tuple,i*7,i*7+6,&CurrEPose);
			//cerr<<"PoseCS2inCS1"<<endl<<PoseCS2inCS1<<endl;
			obj_pose_ref2cam(PoseCS1inCS2,CurrEPose+PoseCS2inCS1,&CurrPoseCS2);
			
				for(int f=0;f<3;f++)
				{
					if (CurrPoseCS2[3+f].D()>180)
							CurrPoseCS2[3+f]=(360-CurrPoseCS2[3+f].D())*(-1);
					if (CurrPoseCS2[3+f].D()==360)
							CurrPoseCS2[3+f]=0;
				}
			if (i==0)
				EPInCS2Tuple=CurrPoseCS2;
			else
			tuple_concat(EPInCS2Tuple,CurrPoseCS2,&EPInCS2Tuple);
		}	
	tuple2mat(EPInCS2Tuple, 0, &EPInCS2);
	return EPInCS2;
}

ReturnMatrix CreateCov (Matrix ExPoses, ColumnVector MeanPose)
{
	//Creates an offset matrix from a mean pose and extreme poses
	ColumnVector Offset2(6);
	HTuple MU;
	Matrix Cov2(6,6);
	Cov2=0.0;
	double veclen=0;
	for (int j=0;j<6;j++)
	{	
	Offset2=ExPoses.column(j*2+1);//-MeanPose;
	veclen=0;
	for (int n=0;n<6;n++)
		{
		veclen+=Offset2.element(n)*Offset2.element(n);		
		}
		veclen=sqrt(veclen);	
		Offset2/=veclen;		
		if (!veclen<0.000001)
		{Cov2+=Offset2*Offset2.t()*veclen;
		//cerr<<"COV:"<<endl<<Cov2<<endl;
		}
	}
	return Cov2;
}

void mat2tuple(Matrix MatrixIN,int type,HTuple * TupleOUT)
{
//	type: 0=12poses,1=hom_mat,2=cov.mat,3=columnvector;
	
if (type==0)//12poses
	{
	for (int i=0;i<12;i++)
		for(int j=0;j<7;j++)
		{		
			if (j==6)
			(*TupleOUT)[i*6+i+j]=0;
			else
			(*TupleOUT)[i*6+i+j]=MatrixIN.element(j,i);
		}
		
	}
else if (type==1)//hom_mat
	{
		for(int i=0;i<3;i++)
			for (int j =0 ;j<4;j++)
				{				
				(*TupleOUT)[i*4+j]=MatrixIN.element(i,j);				
				}
	}
else if (type==2)//cov_matrix
	{
	for(int i=0;i<6;i++)
		for (int j =0 ;j<6;j++)
			(*TupleOUT)[i*6+j]=MatrixIN.element(i,j);
	}
else
	cerr<<"ERROR WITH THE TUPLES AND MATRICES (mat2tuple)!!!"<<endl;
}

void tuple2mat(HTuple TupleIN,int type,Matrix * MatrixOUT)
{
	*MatrixOUT=0.0;
	
if (type==0)//12poses
	{
	for (int i=0;i<12;i++)
		{for(int j=0;j<6;j++)
			{
				MatrixOUT->element(j,i)=TupleIN[i*7+j].D();
			}
		}
	}
else if (type==1)//hom_mat
	{
		for(int i=0;i<3;i++)
			for (int j =0 ;j<4;j++)
				{				
					MatrixOUT->element(i,j)=TupleIN[i*4+j].D();				
				}
		MatrixOUT->element(3,3)=1;	
	}
else if (type==2)
	{
		for(int i=0;i<6;i++)
			for (int j =0 ;j<6;j++)
				MatrixOUT->element(i,j)=TupleIN[i*6+j].D();
	}
else
	cerr<<"ERROR WITH THE TUPLES AND MATRICES (tuple2mat) !!!"<<endl;
}


ReturnMatrix TableOffsetsToObjectOffsets(Matrix EPosesInTable,HTuple MeanPoseInTable)
{
	HTuple EPosesInTableTuple,ObjectEPosesTuple;
	Matrix ObjectEPosesMatrix(6,12);
	mat2tuple(EPosesInTable,0,&EPosesInTableTuple);
	HTuple CurrEPose(7,0.0),CurrTablePose(7,0.0);
	HTuple RotX,RotY, CurrTableHM,CurrTableHMRot;
	HTuple RotAxes;
	RotAxes[0]="X";
	RotAxes[1]="Y";
	RotAxes[2]="Z";
	HTuple TablePoses(7,0.0), MeanHM,EHMInObject,EPoseInObject,InvMeanPoseInTable;
	pose_to_hom_mat3d(MeanPoseInTable,&MeanHM);
	hom_mat3d_invert(MeanHM,&InvMeanPoseInTable);
	
	for (int i=0;i<12;i++)
		{				
				tuple_select_range(EPosesInTableTuple,i*7,i*7+6,&CurrEPose);
				cerr<<endl<<"CurrEPose\t\t"<<CurrEPose<<endl;		
				CurrTablePose=MeanPoseInTable;
				cerr<<"MeanPoseInTable:\t"<<MeanPoseInTable<<endl;
				for (int t=0; t<3;t++)								//Here the translation is added
					CurrTablePose[t]=MeanPoseInTable[t].D()+CurrEPose[t].D();
				
					pose_to_hom_mat3d(CurrTablePose,&CurrTableHM);
				//Rotations:
				//cerr<<"fabs(CurrEPose[4].D()): "<<CurrEPose[4].D()<<endl;
				for(int r=0;r<3;r++)
					{
						if((fabs(CurrEPose[3+r].D())>0.0001))
							{
								hom_mat3d_rotate ((const HTuple) CurrTableHM,
										(const HTuple) CurrEPose[3+r].D()/180*PI,
										(const HTuple) RotAxes[r].S(),
										(const HTuple) CurrTablePose[0],
										(const HTuple) CurrTablePose[1], 
										(const HTuple) CurrTablePose[2], 
										&CurrTableHMRot);	
							
								CurrTableHM=CurrTableHMRot;					
								cerr<<"Rotated in"<<RotAxes[r].S()<<" for "<<CurrEPose[5].D()<<endl;
							}
					}
			hom_mat3d_to_pose(CurrTableHM,&CurrTablePose);
			cerr<<"Pose After Transf:\t" <<CurrTablePose<<endl;	
			
			hom_mat3d_compose(InvMeanPoseInTable,CurrTableHM,&EHMInObject);
			hom_mat3d_to_pose(EHMInObject,&EPoseInObject);
			cerr<<"Offset in ObjectCS:\t" <<EPoseInObject<<endl;	
			
			for(int f=0;f<3;f++)
				{
					if (EPoseInObject[3+f].D()>180)
							EPoseInObject[3+f]=(360-EPoseInObject[3+f].D())*(-1);
					if (EPoseInObject[3+f].D()==360)
							EPoseInObject[3+f]=0;
				}
			cerr<<"floored:\t\t" <<EPoseInObject<<endl;	
				
			if (i==0)
				ObjectEPosesTuple=EPoseInObject;
			else
				tuple_concat(ObjectEPosesTuple,EPoseInObject,&ObjectEPosesTuple);
		}
		
	tuple2mat(ObjectEPosesTuple,0,&ObjectEPosesMatrix);
	return ObjectEPosesMatrix;
}
