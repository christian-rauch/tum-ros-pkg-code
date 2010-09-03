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

#include "pcl_cloud_tools/dxf_writer.h"

dxfwriter::dxfwriter(std::string FileName) :
	linfile::File(FileName, true, false),
	m_handle(67)
{
	WriteConstHeader();
}

dxfwriter::~dxfwriter(void)
{
	WriteConstTail();
}

///////////////////////////////////////////////////////////////////////////////////////
/////////////// call this function to start the conversion ////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
bool dxfwriter::WriteMesh(Mesh_t mesh,  std::string FileName)
{
  dxfwriter write(FileName);
  for(unsigned int i = 0; i < mesh.size(); i++)
  {
    Polygon_t poly = mesh[i].first;
    if(poly.size() == 4)
    {
      write.WriteSolid(poly[0].x, poly[0].y, poly[0].z, poly[1].x, poly[1].y, poly[1].z,
                       poly[2].x, poly[2].y, poly[2].z, poly[3].x, poly[3].y, poly[3].z);
    }
    else if(poly.size() == 3)
    {
      write.WriteSolid(poly[0].x, poly[0].y, poly[0].z, poly[1].x, poly[1].y, poly[1].z,
                       poly[2].x, poly[2].y, poly[2].z);
    }
  }
  return true;
}

void dxfwriter::Write3dLINE(double startx, double starty, double startz, double endx, double endy, double endz)
{
	WriteLine("LINE\n");
	WriteSingleInt(5);
	WriteLine("\n");
	WriteSingleIntAsHex(m_handle);
	WriteLine("\n");
	m_handle++;
	WriteSingleInt(6);
	WriteLine("\n");
	WriteLine("BYBLOCK\n");
	WriteSingleInt(8);
	WriteLine("\n");
	WriteSingleInt(0);
	WriteLine("\n");
	WriteSingleInt(10);
	WriteLine("\n");
	WriteDoubleLimLength(startx);
	WriteLine("\n");
	WriteSingleInt(20);
	WriteLine("\n");
	WriteDoubleLimLength(starty);
	WriteLine("\n");
	WriteSingleInt(30);
	WriteLine("\n");
	WriteDoubleLimLength(startz);
	WriteLine("\n");
	WriteSingleInt(11);
	WriteLine("\n");
	WriteDoubleLimLength(endx);
	WriteLine("\n");
	WriteSingleInt(21);
	WriteLine("\n");
	WriteDoubleLimLength(endy);
	WriteLine("\n");
	WriteSingleInt(31);
	WriteLine("\n");
	WriteDoubleLimLength(endz);
	WriteLine("\n");
	WriteLine("0\n");
}

void dxfwriter::WriteConstHeader()
{
	WriteLine("0\n\
SECTION\n\
  2\n\
HEADER\n\
  9\n\
$ACADVER\n\
  1\n\
AC1009\n\
  9\n\
$INSBASE\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  9\n\
$EXTMIN\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  9\n\
$EXTMAX\n\
 10\n\
60.0\n\
 20\n\
60.0\n\
 30\n\
60.0\n\
  9\n\
$LIMMIN\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
  9\n\
$LIMMAX\n\
 10\n\
30.0\n\
 20\n\
30.0\n\
  9\n\
$ORTHOMODE\n\
 70\n\
     0\n\
  9\n\
$REGENMODE\n\
 70\n\
     1\n\
  9\n\
$FILLMODE\n\
 70\n\
     1\n\
  9\n\
$QTEXTMODE\n\
 70\n\
     0\n\
  9\n\
$MIRRTEXT\n\
 70\n\
     0\n\
  9\n\
$DRAGMODE\n\
 70\n\
     2\n\
  9\n\
$LTSCALE\n\
 40\n\
1.0\n\
  9\n\
$OSMODE\n\
 70\n\
    37\n\
  9\n\
$ATTMODE\n\
 70\n\
     1\n\
  9\n\
$TEXTSIZE\n\
 40\n\
2.5\n\
  9\n\
$TRACEWID\n\
 40\n\
0.05\n\
  9\n\
$TEXTSTYLE\n\
  7\n\
STANDARD\n\
  9\n\
$CLAYER\n\
  8\n\
0\n\
  9\n\
$CELTYPE\n\
  6\n\
CONTINUOUS\n\
  9\n\
$CECOLOR\n\
 62\n\
   256\n\
  9\n\
$DIMSCALE\n\
 40\n\
1.0\n\
  9\n\
$DIMASZ\n\
 40\n\
2.5\n\
  9\n\
$DIMEXO\n\
 40\n\
0.625\n\
  9\n\
$DIMDLI\n\
 40\n\
0.38\n\
  9\n\
$DIMRND\n\
 40\n\
0.0\n\
  9\n\
$DIMDLE\n\
 40\n\
0.0\n\
  9\n\
$DIMEXE\n\
 40\n\
1.25\n\
  9\n\
$DIMTP\n\
 40\n\
0.0\n\
  9\n\
$DIMTM\n\
 40\n\
0.0\n\
  9\n\
$DIMTXT\n\
 40\n\
2.5\n\
  9\n\
$DIMCEN\n\
 40\n\
0.09\n\
  9\n\
$DIMTSZ\n\
 40\n\
0.0\n\
  9\n\
$DIMTOL\n\
 70\n\
     0\n\
  9\n\
$DIMLIM\n\
 70\n\
     0\n\
  9\n\
$DIMTIH\n\
 70\n\
     1\n\
  9\n\
$DIMTOH\n\
 70\n\
     1\n\
  9\n\
$DIMSE1\n\
 70\n\
     0\n\
  9\n\
$DIMSE2\n\
 70\n\
     0\n\
  9\n\
$DIMTAD\n\
 70\n\
     0\n\
  9\n\
$DIMZIN\n\
 70\n\
     0\n\
  9\n\
$DIMBLK\n\
  1\n\
\n\
  9\n\
$DIMASO\n\
 70\n\
     1\n\
  9\n\
$DIMSHO\n\
 70\n\
     1\n\
  9\n\
$DIMPOST\n\
  1\n\
\n\
  9\n\
$DIMAPOST\n\
  1\n\
\n\
  9\n\
$DIMALT\n\
 70\n\
     0\n\
  9\n\
$DIMALTD\n\
 70\n\
     2\n\
  9\n\
$DIMALTF\n\
 40\n\
25.399999999999999\n\
  9\n\
$DIMLFAC\n\
 40\n\
1.0\n\
  9\n\
$DIMTOFL\n\
 70\n\
     0\n\
  9\n\
$DIMTVP\n\
 40\n\
0.0\n\
  9\n\
$DIMTIX\n\
 70\n\
     0\n\
  9\n\
$DIMSOXD\n\
 70\n\
     0\n\
  9\n\
$DIMSAH\n\
 70\n\
     0\n\
  9\n\
$DIMBLK1\n\
  1\n\
\n\
  9\n\
$DIMBLK2\n\
  1\n\
\n\
  9\n\
$DIMSTYLE\n\
  2\n\
STANDARD\n\
  9\n\
$DIMCLRD\n\
 70\n\
     0\n\
  9\n\
$DIMCLRE\n\
 70\n\
     0\n\
  9\n\
$DIMCLRT\n\
 70\n\
     0\n\
  9\n\
$DIMTFAC\n\
 40\n\
1.0\n\
  9\n\
$DIMGAP\n\
 40\n\
0.625\n\
  9\n\
$LUNITS\n\
 70\n\
     2\n\
  9\n\
$LUPREC\n\
 70\n\
     4\n\
  9\n\
$SKETCHINC\n\
 40\n\
0.1\n\
  9\n\
$FILLETRAD\n\
 40\n\
0.0\n\
  9\n\
$AUNITS\n\
 70\n\
     0\n\
  9\n\
$AUPREC\n\
 70\n\
     2\n\
  9\n\
$MENU\n\
  1\n\
.\n\
  9\n\
$ELEVATION\n\
 40\n\
0.0\n\
  9\n\
$PELEVATION\n\
 40\n\
0.0\n\
  9\n\
$THICKNESS\n\
 40\n\
0.0\n\
  9\n\
$LIMCHECK\n\
 70\n\
     0\n\
  9\n\
$BLIPMODE\n\
 70\n\
     0\n\
  9\n\
$CHAMFERA\n\
 40\n\
0.0\n\
  9\n\
$CHAMFERB\n\
 40\n\
0.0\n\
  9\n\
$SKPOLY\n\
 70\n\
     0\n\
  9\n\
$TDCREATE\n\
 40\n\
2454169.4542739\n\
  9\n\
$TDUPDATE\n\
 40\n\
2454169.4545730208\n\
  9\n\
$TDINDWG\n\
 40\n\
0.0003005671\n\
  9\n\
$TDUSRTIMER\n\
 40\n\
0.0003005671\n\
  9\n\
$USRTIMER\n\
 70\n\
     1\n\
  9\n\
$ANGBASE\n\
 50\n\
0.0\n\
  9\n\
$ANGDIR\n\
 70\n\
     0\n\
  9\n\
$PDMODE\n\
 70\n\
     0\n\
  9\n\
$PDSIZE\n\
 40\n\
0.0\n\
  9\n\
$PLINEWID\n\
 40\n\
0.0\n\
  9\n\
$COORDS\n\
 70\n\
     1\n\
  9\n\
$SPLFRAME\n\
 70\n\
     0\n\
  9\n\
$SPLINETYPE\n\
 70\n\
     6\n\
  9\n\
$SPLINESEGS\n\
 70\n\
     8\n\
  9\n\
$ATTDIA\n\
 70\n\
     0\n\
  9\n\
$ATTREQ\n\
 70\n\
     1\n\
  9\n\
$HANDLING\n\
 70\n\
     1\n\
  9\n\
$HANDSEED\n\
  5\n\
10033\n\
  9\n\
$SURFTAB1\n\
 70\n\
     6\n\
  9\n\
$SURFTAB2\n\
 70\n\
     6\n\
  9\n\
$SURFTYPE\n\
 70\n\
     6\n\
  9\n\
$SURFU\n\
 70\n\
     6\n\
  9\n\
$SURFV\n\
 70\n\
     6\n\
  9\n\
$UCSNAME\n\
  2\n\
\n\
  9\n\
$UCSORG\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  9\n\
$UCSXDIR\n\
 10\n\
1.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  9\n\
$UCSYDIR\n\
 10\n\
0.0\n\
 20\n\
1.0\n\
 30\n\
0.0\n\
  9\n\
$PUCSNAME\n\
  2\n\
\n\
  9\n\
$PUCSORG\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  9\n\
$PUCSXDIR\n\
 10\n\
1.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  9\n\
$PUCSYDIR\n\
 10\n\
0.0\n\
 20\n\
1.0\n\
 30\n\
0.0\n\
  9\n\
$USERI1\n\
 70\n\
     0\n\
  9\n\
$USERI2\n\
 70\n\
     0\n\
  9\n\
$USERI3\n\
 70\n\
     0\n\
  9\n\
$USERI4\n\
 70\n\
     0\n\
  9\n\
$USERI5\n\
 70\n\
     0\n\
  9\n\
$USERR1\n\
 40\n\
0.0\n\
  9\n\
$USERR2\n\
 40\n\
0.0\n\
  9\n\
$USERR3\n\
 40\n\
0.0\n\
  9\n\
$USERR4\n\
 40\n\
0.0\n\
  9\n\
$USERR5\n\
 40\n\
0.0\n\
  9\n\
$WORLDVIEW\n\
 70\n\
     1\n\
  9\n\
$SHADEDGE\n\
 70\n\
     3\n\
  9\n\
$SHADEDIF\n\
 70\n\
    70\n\
  9\n\
$TILEMODE\n\
 70\n\
     1\n\
  9\n\
$MAXACTVP\n\
 70\n\
    64\n\
  9\n\
$PLIMCHECK\n\
 70\n\
     0\n\
  9\n\
$PEXTMIN\n\
 10\n\
1.0000000000000000E+020\n\
 20\n\
1.0000000000000000E+020\n\
 30\n\
1.0000000000000000E+020\n\
  9\n\
$PEXTMAX\n\
 10\n\
-1.0000000000000000E+020\n\
 20\n\
-1.0000000000000000E+020\n\
 30\n\
-1.0000000000000000E+020\n\
  9\n\
$PLIMMIN\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
  9\n\
$PLIMMAX\n\
 10\n\
210.0\n\
 20\n\
297.0\n\
  9\n\
$UNITMODE\n\
 70\n\
     0\n\
  9\n\
$VISRETAIN\n\
 70\n\
     1\n\
  9\n\
$PLINEGEN\n\
 70\n\
     0\n\
  9\n\
$PSLTSCALE\n\
 70\n\
     1\n\
  0\n\
ENDSEC\n\
  0\n\
SECTION\n\
  2\n\
TABLES\n\
  0\n\
TABLE\n\
  2\n\
VPORT\n\
 70\n\
     1\n\
  0\n\
VPORT\n\
  2\n\
*ACTIVE\n\
 70\n\
     0\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 11\n\
1.0\n\
 21\n\
1.0\n\
 12\n\
286.30555555555537\n\
 22\n\
277.18754423213011\n\
 13\n\
0.0\n\
 23\n\
0.0\n\
 14\n\
10.0\n\
 24\n\
10.0\n\
 15\n\
10.0\n\
 25\n\
10.0\n\
 16\n\
0.0\n\
 26\n\
0.0\n\
 36\n\
1.0\n\
 17\n\
0.0\n\
 27\n\
0.0\n\
 37\n\
0.0\n\
 40\n\
554.37508846426022\n\
 41\n\
1.0328947368421051\n\
 42\n\
50.0\n\
 43\n\
0.0\n\
 44\n\
0.0\n\
 50\n\
0.0\n\
 51\n\
0.0\n\
 71\n\
     0\n\
 72\n\
   100\n\
 73\n\
     1\n\
 74\n\
     3\n\
 75\n\
     1\n\
 76\n\
     1\n\
 77\n\
     0\n\
 78\n\
     0\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
LTYPE\n\
 70\n\
    19\n\
  0\n\
LTYPE\n\
  2\n\
CONTINUOUS\n\
 70\n\
     0\n\
  3\n\
Solid line\n\
 72\n\
    65\n\
 73\n\
     0\n\
 40\n\
0.0\n\
  0\n\
LTYPE\n\
  2\n\
DOT\n\
 70\n\
     0\n\
  3\n\
Dot . . . . . . . . . . . . . . . . . . . . . .\n\
 72\n\
    65\n\
 73\n\
     2\n\
 40\n\
6.3499999999999988\n\
 49\n\
0.0\n\
 49\n\
-6.3499999999999988\n\
  0\n\
LTYPE\n\
  2\n\
DOT2\n\
 70\n\
     0\n\
  3\n\
Dot (.5x) .....................................\n\
 72\n\
    65\n\
 73\n\
     2\n\
 40\n\
3.1749999999999989\n\
 49\n\
0.0\n\
 49\n\
-3.1749999999999989\n\
  0\n\
LTYPE\n\
  2\n\
DOTX2\n\
 70\n\
     0\n\
  3\n\
Dot (2x) .  .  .  .  .  .  .  .  .  .  .  .  .\n\
 72\n\
    65\n\
 73\n\
     2\n\
 40\n\
12.699999999999999\n\
 49\n\
0.0\n\
 49\n\
-12.699999999999999\n\
  0\n\
LTYPE\n\
  2\n\
DASHED\n\
 70\n\
     0\n\
  3\n\
Dashed __ __ __ __ __ __ __ __ __ __ __ __ __ _\n\
 72\n\
    65\n\
 73\n\
     2\n\
 40\n\
19.050000000000001\n\
 49\n\
12.699999999999999\n\
 49\n\
-6.3499999999999988\n\
  0\n\
LTYPE\n\
  2\n\
DASHED2\n\
 70\n\
     0\n\
  3\n\
Dashed (.5x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _\n\
 72\n\
    65\n\
 73\n\
     2\n\
 40\n\
9.5250000000000004\n\
 49\n\
6.3499999999999988\n\
 49\n\
-3.1749999999999989\n\
  0\n\
LTYPE\n\
  2\n\
DASHEDX2\n\
 70\n\
     0\n\
  3\n\
Dashed (2x) ____  ____  ____  ____  ____  ___\n\
 72\n\
    65\n\
 73\n\
     2\n\
 40\n\
38.100000000000001\n\
 49\n\
25.399999999999999\n\
 49\n\
-12.699999999999999\n\
  0\n\
LTYPE\n\
  2\n\
DASHDOT\n\
 70\n\
     0\n\
  3\n\
Dash dot __ . __ . __ . __ . __ . __ . __ . __\n\
 72\n\
    65\n\
 73\n\
     4\n\
 40\n\
25.399999999999999\n\
 49\n\
12.699999999999999\n\
 49\n\
-6.3499999999999988\n\
 49\n\
0.0\n\
 49\n\
-6.3499999999999988\n\
  0\n\
LTYPE\n\
  2\n\
DASHDOT2\n\
 70\n\
     0\n\
  3\n\
Dash dot (.5x) _._._._._._._._._._._._._._._.\n\
 72\n\
    65\n\
 73\n\
     4\n\
 40\n\
12.699999999999999\n\
 49\n\
6.3499999999999988\n\
 49\n\
-3.1749999999999989\n\
 49\n\
0.0\n\
 49\n\
-3.1749999999999989\n\
  0\n\
LTYPE\n\
  2\n\
DASHDOTX2\n\
 70\n\
     0\n\
  3\n\
Dash dot (2x) ____  .  ____  .  ____  .  ___\n\
 72\n\
    65\n\
 73\n\
     4\n\
 40\n\
50.799999999999997\n\
 49\n\
25.399999999999999\n\
 49\n\
-12.699999999999999\n\
 49\n\
0.0\n\
 49\n\
-12.699999999999999\n\
  0\n\
LTYPE\n\
  2\n\
DIVIDE\n\
 70\n\
     0\n\
  3\n\
Divide ____ . . ____ . . ____ . . ____ . . ____\n\
 72\n\
    65\n\
 73\n\
     6\n\
 40\n\
31.75\n\
 49\n\
12.699999999999999\n\
 49\n\
-6.3499999999999988\n\
 49\n\
0.0\n\
 49\n\
-6.3499999999999988\n\
 49\n\
0.0\n\
 49\n\
-6.3499999999999988\n\
  0\n\
LTYPE\n\
  2\n\
DIVIDE2\n\
 70\n\
     0\n\
  3\n\
Divide (.5x) __..__..__..__..__..__..__..__.._\n\
 72\n\
    65\n\
 73\n\
     6\n\
 40\n\
15.875\n\
 49\n\
6.3499999999999988\n\
 49\n\
-3.1749999999999989\n\
 49\n\
0.0\n\
 49\n\
-3.1749999999999989\n\
 49\n\
0.0\n\
 49\n\
-3.1749999999999989\n\
  0\n\
LTYPE\n\
  2\n\
DIVIDEX2\n\
 70\n\
     0\n\
  3\n\
Divide (2x) ________  .  .  ________  .  .  _\n\
 72\n\
    65\n\
 73\n\
     6\n\
 40\n\
63.5\n\
 49\n\
25.399999999999999\n\
 49\n\
-12.699999999999999\n\
 49\n\
0.0\n\
 49\n\
-12.699999999999999\n\
 49\n\
0.0\n\
 49\n\
-12.699999999999999\n\
  0\n\
LTYPE\n\
  2\n\
CENTER\n\
 70\n\
     0\n\
  3\n\
Center ____ _ ____ _ ____ _ ____ _ ____ _ ____\n\
 72\n\
    65\n\
 73\n\
     4\n\
 40\n\
50.799999999999997\n\
 49\n\
31.75\n\
 49\n\
-6.3499999999999988\n\
 49\n\
6.3499999999999988\n\
 49\n\
-6.3499999999999988\n\
  0\n\
LTYPE\n\
  2\n\
CENTER2\n\
 70\n\
     0\n\
  3\n\
Center (.5x) ___ _ ___ _ ___ _ ___ _ ___ _ ___\n\
 72\n\
    65\n\
 73\n\
     4\n\
 40\n\
28.574999999999999\n\
 49\n\
19.050000000000001\n\
 49\n\
-3.1749999999999989\n\
 49\n\
3.1749999999999989\n\
 49\n\
-3.1749999999999989\n\
  0\n\
LTYPE\n\
  2\n\
CENTERX2\n\
 70\n\
     0\n\
  3\n\
Center (2x) ________  __  ________  __  _____\n\
 72\n\
    65\n\
 73\n\
     4\n\
 40\n\
101.59999999999999\n\
 49\n\
63.5\n\
 49\n\
-12.699999999999999\n\
 49\n\
12.699999999999999\n\
 49\n\
-12.699999999999999\n\
  0\n\
LTYPE\n\
  2\n\
BORDER\n\
 70\n\
     0\n\
  3\n\
Border __ __ . __ __ . __ __ . __ __ . __ __ .\n\
 72\n\
    65\n\
 73\n\
     6\n\
 40\n\
44.450000000000003\n\
 49\n\
12.699999999999999\n\
 49\n\
-6.3499999999999988\n\
 49\n\
12.699999999999999\n\
 49\n\
-6.3499999999999988\n\
 49\n\
0.0\n\
 49\n\
-6.3499999999999988\n\
  0\n\
LTYPE\n\
  2\n\
BORDER2\n\
 70\n\
     0\n\
  3\n\
Border (.5x) __.__.__.__.__.__.__.__.__.__.__.\n\
 72\n\
    65\n\
 73\n\
     6\n\
 40\n\
22.225000000000001\n\
 49\n\
6.3499999999999988\n\
 49\n\
-3.1749999999999989\n\
 49\n\
6.3499999999999988\n\
 49\n\
-3.1749999999999989\n\
 49\n\
0.0\n\
 49\n\
-3.1749999999999989\n\
  0\n\
LTYPE\n\
  2\n\
BORDERX2\n\
 70\n\
     0\n\
  3\n\
Border (2x) ____  ____  .  ____  ____  .  ___\n\
 72\n\
    65\n\
 73\n\
     6\n\
 40\n\
88.900000000000006\n\
 49\n\
25.399999999999999\n\
 49\n\
-12.699999999999999\n\
 49\n\
25.399999999999999\n\
 49\n\
-12.699999999999999\n\
 49\n\
0.0\n\
 49\n\
-12.699999999999999\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
LAYER\n\
 70\n\
     1\n\
  0\n\
LAYER\n\
  2\n\
0\n\
 70\n\
     0\n\
 62\n\
     7\n\
  6\n\
CONTINUOUS\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
STYLE\n\
 70\n\
     1\n\
  0\n\
STYLE\n\
  2\n\
STANDARD\n\
 70\n\
     0\n\
 40\n\
0.0\n\
 41\n\
0.75\n\
 50\n\
0.0\n\
 71\n\
     0\n\
 42\n\
2.5\n\
  3\n\
txt\n\
  4\n\
\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
VIEW\n\
 70\n\
     0\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
UCS\n\
 70\n\
     0\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
APPID\n\
 70\n\
     1\n\
  0\n\
APPID\n\
  2\n\
ACAD\n\
 70\n\
     0\n\
  0\n\
ENDTAB\n\
  0\n\
TABLE\n\
  2\n\
DIMSTYLE\n\
 70\n\
     1\n\
  0\n\
DIMSTYLE\n\
  2\n\
STANDARD\n\
 70\n\
     0\n\
  3\n\
\n\
  4\n\
\n\
  5\n\
\n\
  6\n\
\n\
  7\n\
\n\
 40\n\
1.0\n\
 41\n\
2.5\n\
 42\n\
0.625\n\
 43\n\
3.75\n\
 44\n\
1.25\n\
 45\n\
0.0\n\
 46\n\
0.0\n\
 47\n\
0.0\n\
 48\n\
0.0\n\
140\n\
2.5\n\
141\n\
2.5\n\
142\n\
0.0\n\
143\n\
0.03937007874016\n\
144\n\
1.0\n\
145\n\
0.0\n\
146\n\
1.0\n\
147\n\
0.625\n\
 71\n\
     0\n\
 72\n\
     0\n\
 73\n\
     0\n\
 74\n\
     0\n\
 75\n\
     0\n\
 76\n\
     0\n\
 77\n\
     1\n\
 78\n\
     8\n\
170\n\
     0\n\
171\n\
     3\n\
172\n\
     1\n\
173\n\
     0\n\
174\n\
     0\n\
175\n\
     0\n\
176\n\
     0\n\
177\n\
     0\n\
178\n\
     0\n\
  0\n\
ENDTAB\n\
  0\n\
ENDSEC\n\
  0\n\
SECTION\n\
  2\n\
BLOCKS\n\
  0\n\
BLOCK\n\
  8\n\
0\n\
  2\n\
$MODEL_SPACE\n\
 70\n\
     0\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  3\n\
$MODEL_SPACE\n\
  1\n\
\n\
  0\n\
ENDBLK\n\
  5\n\
21\n\
  8\n\
0\n\
  0\n\
BLOCK\n\
 67\n\
     1\n\
  8\n\
0\n\
  2\n\
$PAPER_SPACE\n\
 70\n\
     0\n\
 10\n\
0.0\n\
 20\n\
0.0\n\
 30\n\
0.0\n\
  3\n\
$PAPER_SPACE\n\
  1\n\
\n\
  0\n\
ENDBLK\n\
  5\n\
1D\n\
 67\n\
     1\n\
  8\n\
0\n\
  0\n\
ENDSEC\n\
  0\n\
SECTION\n\
  2\n\
ENTITIES\n\
  0\n");
}

void dxfwriter::WriteConstTail()
{
WriteLine("ENDSEC\n\
  0\n\
EOF\n");
}

void dxfwriter::WriteGroup(std::string st, int value)
{
	WriteLine(st.c_str());
	WriteLine("\n");
	WriteSingleInt(value);
	WriteLine("\n");
}

void dxfwriter::WriteGroup(std::string st, double value)
{
	WriteLine(st.c_str());
	WriteLine("\n");
	WriteSingleDouble(value);
	WriteLine("\n");
}

void dxfwriter::WriteGroup(int st, int value)
{
	WriteSingleInt(st);
	WriteLine("\n");
	WriteSingleInt(value);
	WriteLine("\n");
}

void dxfwriter::WriteGroup(int st, double value)
{
	WriteSingleInt(st);
	WriteLine("\n");
	WriteSingleDouble(value);
	WriteLine("\n");
}

void dxfwriter::WriteGroupHex(int st, double value)
{
	WriteSingleInt(st);
	WriteLine("\n");
	WriteSingleIntAsHex((int)value);
	WriteLine("\n");
}

void dxfwriter::WriteSolid(double dx0, double dy0, double dz0,
						   double dx1, double dy1, double dz1,
						   double dx2, double dy2, double dz2,
						   double dx3, double dy3, double dz3)
{
	WriteLine("3DFACE\n");
	WriteGroupHex(5, m_handle);
	m_handle++;
	WriteGroup(8,0);
	WriteGroup(10, dx0);
	WriteGroup(20, dy0);
	WriteGroup(30, dz0);
	WriteGroup(11, dx1);
	WriteGroup(21, dy1);
	WriteGroup(31, dz1);
	WriteGroup(12, dx2);
	WriteGroup(22, dy2);
	WriteGroup(32, dz2);
	WriteGroup(13, dx3);
	WriteGroup(23, dy3);
	WriteGroup(33, dz3);
	WriteLine("0\n");
}

void dxfwriter::WriteSolid(double dx0, double dy0, double dz0,
						   double dx1, double dy1, double dz1,
						   double dx2, double dy2, double dz2)
{
	WriteLine("3DFACE\n");
	WriteGroupHex(5, m_handle);
	m_handle++;
	WriteGroup(8,0);
	WriteGroup(10, dx0);
	WriteGroup(20, dy0);
	WriteGroup(30, dz0);
	WriteGroup(11, dx1);
	WriteGroup(21, dy1);
	WriteGroup(31, dz1);
	WriteGroup(12, dx2);
	WriteGroup(22, dy2);
	WriteGroup(32, dz2);
	WriteGroup(13, dx2);
	WriteGroup(23, dy2);
	WriteGroup(33, dz2);
	WriteLine("0\n");
}

void dxfwriter::WriteBox(double offsetx, double offsety, double offsetz, double sizex, double sizey, double sizez)
{
	double x0 = offsetx;
	double xd = offsetx + sizex;
	double y0 = offsety;
	double yd = offsety + sizey;
	double z0 = offsetz;
	double zd = offsetz + sizez;
	/*Bottom View*/
	WriteSolid(x0,y0,z0,xd,y0,z0,xd,yd,z0,x0,yd,z0);
	WriteSolid(x0,y0,z0,x0,yd,z0,x0,yd,zd,x0,y0,zd);
	WriteSolid(x0,y0,z0,x0,y0,zd,xd,y0,zd,xd,y0,z0);
	/*Top View TODO REmove*/
	//WriteSolid(xd,yd,zd,x0,yd,zd,x0,y0,zd,xd,y0,zd);
	WriteSolid(xd,yd,zd,x0 - 0.1 + sizex / 2,yd,zd,x0 - 0.1+ sizex / 2,y0,zd,xd,y0,zd);
	WriteSolid(xd - sizex / 2,yd,zd,x0,yd,zd,x0,y0,zd,xd - sizex / 2,y0,zd);
	WriteSolid(xd,yd,zd + 0.01,xd - 0.4 - sizex / 4,yd,zd + 0.01,xd,yd - sizey / 2,zd + 0.01,xd,yd,zd + 0.01);

	WriteSolid(xd,yd,zd,xd,y0,zd,xd,y0,z0,xd,yd,z0);
	WriteSolid(xd,yd,zd,x0,yd,zd,x0,yd,z0,xd,yd,z0);

	Write3dLINE(x0,y0,z0,xd,y0,z0);
	Write3dLINE(xd,y0,z0,xd,yd,z0);
	Write3dLINE(xd,yd,z0,x0,yd,z0);
	Write3dLINE(x0,yd,z0,x0,y0,z0);

	Write3dLINE(x0,y0,z0,x0,yd,z0);
	Write3dLINE(x0,yd,z0,x0,yd,zd);
	Write3dLINE(x0,yd,zd,x0,y0,zd);
	Write3dLINE(x0,y0,zd,x0,y0,z0);

	Write3dLINE(x0,y0,z0,x0,y0,zd);
	Write3dLINE(x0,y0,zd,xd,y0,zd);
	Write3dLINE(xd,y0,zd,xd,y0,z0);
	Write3dLINE(xd,y0,z0,x0,y0,z0);

	Write3dLINE(xd,yd,zd,x0,yd,zd);
	Write3dLINE(x0,yd,zd,x0,y0,zd);
	Write3dLINE(x0,y0,zd,xd,y0,zd);
	Write3dLINE(xd,y0,zd,xd,yd,zd);

	Write3dLINE(xd,yd,zd,xd,y0,zd);
	Write3dLINE(xd,y0,zd,xd,y0,z0);
	Write3dLINE(xd,y0,z0,xd,yd,z0);
	Write3dLINE(xd,yd,z0,xd,yd,zd);

	Write3dLINE(xd,yd,zd,x0,yd,zd);
	Write3dLINE(x0,yd,zd,x0,yd,z0);
	Write3dLINE(x0,yd,z0,xd,yd,z0);
	Write3dLINE(xd,yd,z0,xd,yd,zd);
}

#define line(a,b) Write3dLINE(dx##a, dy##a, dz##a, dx##b, dy##b, dz##b)
#define SOLID(a,b,c,e) WriteSolid(dx##a, dy##a, dz##a, dx##b, dy##b, dz##b,dx##c, dy##c, dz##c,dx##e, dy##e, dz##e)

void dxfwriter::WriteBoxFromPoints(double dx0, double dy0, double dz0,
						   double dx1, double dy1, double dz1,
						   double dx2, double dy2, double dz2,
						   double dx3, double dy3, double dz3,
						   double dx4, double dy4, double dz4,
						   double dx5, double dy5, double dz5,
						   double dx6, double dy6, double dz6,
						   double dx7, double dy7, double dz7)
{

	SOLID(0,1,2,3);
	line(0,1);
	line(1,2);
	line(2,3);
	line(3,0);
	SOLID(0,1,5,4);
	line(0,1);
	line(1,5);
	line(5,4);
	line(4,0);
	SOLID(4,5,6,7);
	line(4,5);
	line(5,6);
	line(6,7);
	line(7,4);
	SOLID(6,7,3,2);
	line(6,7);
	line(7,3);
	line(3,2);
	line(2,6);
	SOLID(1,2,6,5);
	line(1,2);
	line(2,6);
	line(6,5);
	line(5,1);
	SOLID(3,0,4,7);
	line(3,0);
	line(0,4);
	line(4,7);
	line(7,3);
}

void dxfwriter::WriteCylinder(double centerx0, double centery0, double centerz0,
						   double radius, double height, double angleX)
{
	int num_faces = 16;
	double step = 2*M_PI / num_faces;
	double facelength = radius * sin(step / 2) + (radius - radius * cos(step / 2)) / cos(step / 2);
	double dx0, dy0, dz0, dx1, dy1, dz1, dx2, dy2, dz2, dx3, dy3, dz3;
	for(int i = 0; i < num_faces; i++)
	{
		double angle = i * step;
		dx0 = dx1 = dx2 = dx3 = centerx0 + radius * cos(angle);
		dx0 -= facelength * sin(angle);
		dx3 -= facelength * sin(angle);
		dx1 += facelength * sin(angle);
		dx2 += facelength * sin(angle);

		dy0 = dy1 = dy2 = dy3 = centery0 + radius * sin(angle) * sin(angleX);
		double temp = facelength * cos(angle) * sin(angleX);
		dy0 += temp ;
		dy3 += temp + height * cos(angleX);
		dy1 -= temp ;
		dy2 -= temp - height * cos(angleX);

		dz0 = dz1 = dz2 = dz3 = centerz0+ radius * sin(angle) * cos(angleX);
		temp = facelength * cos(angle) * cos(angleX);
		dz0 += temp ;
		dz3 += temp - height * sin(angleX);
		dz1 -= temp ;
		dz2 -= temp + height * sin(angleX);

		SOLID(0,1,2,3);
		line(0,1);
		line(1,2);
		line(2,3);
		line(3,0);
	}
}

