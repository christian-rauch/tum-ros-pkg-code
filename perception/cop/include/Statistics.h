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

 
/************************************************************************
                        Statistics.h - Copyright klank


**************************************************************************/


#ifndef STATISTICS_H
#define STATISTICS_H

#include <string>
#include <vector>

class XMLTag;

#define XML_NODE_STATISTICS "Statistics"
/**
  * class Statistics
  */
class Statistics
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  Statistics (XMLTag* );

  /**
   * Empty Destructor
   */
  virtual ~Statistics ( );

  // mETHODS
  //  
	XMLTag* Save();
  // Public attributes
  //  


  // Public attribute accessor methods
  //  


  // Public attribute accessor methods
  //  


protected:

  // Static Protected attributes
  //  

  // Protected attributes
  //  


  // Protected attribute accessor methods
  //  


  // Protected attribute accessor methods
  //  


private:

  // Static Private attributes
  //  

  // Private attributes
  //  


  // Private attribute accessor methods
  //  


  // Private attribute accessor methods
  //  



};

#endif // STATISTICS_H
