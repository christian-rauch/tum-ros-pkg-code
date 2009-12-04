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


#include "RegionOI.h"
#include <sstream>


using namespace Halcon;
using namespace cop;

RegionOI::RegionOI()
{
#ifdef HALCONIMG
  Halcon::gen_empty_region(&m_reg);
#endif
}

RegionOI::RegionOI(RegionRuns row_colStart_colEnd)
{
#ifdef HALCONIMG
  HTuple r, cs, ce;
  RegionRuns::const_iterator it = row_colStart_colEnd.begin();
  for (;it != row_colStart_colEnd.end(); it++)
  {
    r = r.Concat((*it).first);
    cs = cs.Concat((*it).second.first);
    ce = ce.Concat((*it).second.second);
  }
  Halcon::gen_region_runs(&m_reg,r, cs, ce);
#else /*HALCONIMG*/
#endif /*HALCONIMG*/
}

RegionOI::RegionOI(std::string stFilename)
{
#ifdef HALCONIMG
  Halcon::read_region(&m_reg,stFilename.c_str());
#else /*HALCONIMG*/
#endif /*HALCONIMG*/
}

int RegionOI::GetSize()
{
#ifdef HALCONIMG
  HTuple area, row, column;
  Halcon::area_center(m_reg, &area, &row, &column);
  return area[0].I();
#else /*HALCONIMG*/
  return 0;
#endif /*HALCONIMG*/
}

void RegionOI::AddPoint(double Row, double Column)
{
#ifdef HALCONIMG
  Halcon::HTuple r, c;
  Halcon::get_region_points(m_reg, &r, &c);
  tuple_concat(r, Row, &r);
  tuple_concat(c, Column, &c);
  Halcon::gen_region_points(&m_reg, r, c);
#else /*HALCONIMG*/
#endif /*HALCONIMG*/
}

void RegionOI::TransitiveHull()
{
#ifdef HALCONIMG
  Halcon::Hobject reg_struct_cross;
  Halcon::HTuple r_s(5, 1);
  Halcon::HTuple c_s(5,1);
  r_s[0] = 0;
  r_s[4] = 2;
  c_s[1] = 0;
  c_s[3] = 2;
  Halcon::shape_trans(m_reg, &m_reg, "convex");
  Halcon::gen_region_points(&reg_struct_cross, r_s, c_s);
  Halcon::minkowski_add1(m_reg, reg_struct_cross, &m_reg, 60);
  Halcon::HTuple area,r,c;
  Halcon::area_center(m_reg, &area, &r, &c);
  printf("Area of a new Region: %d\n", area[0].I());
  if(area < 500)
     Halcon::dilation_circle(m_reg, &m_reg, 100);
#else /*HALCONIMG*/
#endif /*HALCONIMG*/
}

static unsigned int s_regionCounter = 0;

XMLTag* RegionOI::Save(std::string tagName)
{
 XMLTag* ret = NULL;
  std::ostringstream stFileNameBuf;
  stFileNameBuf << "Region" << s_regionCounter++ << "_" << (unsigned long)time(NULL) << ".dat";
  std::string stFileName(stFileNameBuf.str());
#ifdef HALCONIMG
  try
  {
    Halcon::write_region(m_reg, stFileName.c_str());
  }
  catch(Halcon::HException ex)
  {
    printf("Error writing Region OI to %s: %s\n", stFileName.c_str(), ex.message);
    throw "Tried saving of a bad region";
  }
  ret = new XMLTag(tagName);
  ret ->AddProperty("Filename", stFileName);
/*  Halcon::HTuple r, cs, ce;
  Halcon::get_region_runs(m_reg, &r, &cs, &ce);
  RegionRuns row_colStart_colEnd;
  for (int i = 0;i < r.Num(); i++)
  {
    std::pair< int, std::pair<int, int > >  r_cs_ce;
    r_cs_ce.first = r[i].I();
    r_cs_ce.second.first = cs[i].I();
    r_cs_ce.second.second = ce[i].I();
    row_colStart_colEnd.push_back(r_cs_ce);
  }
  ret = XMLTag::Tag(row_colStart_colEnd, (tagName.length() > 0) ? tagName : XML_NODE_ROI);*/
//#else /*HALCONIMG*/
  #endif /*HALCONIMG*/
  return ret;
}

#ifdef HALCONIMG
Halcon::Hobject& RegionOI::GetRegion(double scale)
{
 if(scale == 1.0)
   return m_reg;
  else
  {
    try
    {
      Halcon::zoom_region(m_reg, &m_regZoomTmp, scale, scale);
    }
    catch(Halcon::HException ex)
    {
      printf("Error in RegionOI Get Region: %s \n", ex.message);
      return m_reg;
    }
    return m_regZoomTmp;
  }
}
#endif
