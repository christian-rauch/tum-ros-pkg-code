/*
 *  Load and export a VTK 3D point cloud
 *  Copywrong (K) 2010 Z.
 *
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: VTKExporter.cc,v 1.0 2010/02/23 12:00:00 zoli Exp $
 */


#include <iostream>
#include <string>
#include <vector>
#include <math.h>

//terminal_tools includes
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>

//local includes
#include "pcl_vtk_tools/misc.h"
#include "pcl_vtk_tools/dxf_writer.h"

using namespace std;
using terminal_tools::print_color;
using terminal_tools::print_error;
using terminal_tools::print_warn;
using terminal_tools::print_info;
using terminal_tools::print_debug;
using terminal_tools::print_value;
using terminal_tools::print_highlight;
using terminal_tools::TT_BRIGHT;
using terminal_tools::TT_RED;
using terminal_tools::TT_GREEN;
using terminal_tools::TT_BLUE;

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    print_error (stderr, "Syntax is: %s input.vtk output.dxf\n", argv[0]);
    return (-1);
  }
  
  // Parse the command line arguments for .vtk or .ply files
  vector<int> p_file_indices_vtk = terminal_tools::parse_file_extension_argument (argc, argv, ".vtk");
  vector<int> p_file_indices_dxf = terminal_tools::parse_file_extension_argument (argc, argv, ".dxf");

  // Loading VTK file
  vtkPolyData* data = reinterpret_cast<vtkPolyData*>(load_poly_data_as_data_set(argv[p_file_indices_vtk.at (0)]));
  //vtkDataSet* data = LoadAsDataSet (argv[pFileIndicesVTK.at(0)]);

  /*vtkPolyDataReader* reader = vtkPolyDataReader::New ();
  reader->SetFileName (argv [pFileIndicesVTK.at (0)]);
  reader->Update ();
  vtkPolyData* data = reader->GetOutput ();*/
  data->Update ();

  // Print info
  print_info (stderr, "Loaded "); print_value (stderr, "%s", argv [p_file_indices_vtk.at (0)]);
  fprintf (stderr, " with "); print_value (stderr, "%d", data->GetNumberOfPoints ()); fprintf (stderr, " points and ");
  print_value (stderr, "%d", data->GetNumberOfPoints ()); fprintf (stderr, " polygons.\n");
  
  // Init mesh
  Mesh_t mesh;
  //mesh.resize (data->GetNumberOfPolys ());

  // Creating mesh
  vtkPoints *points = data->GetPoints ();
  vtkCellArray* polys = data->GetPolys ();
  vtkIdType npts, *pts;
  int n = 0;
  while (polys->GetNextCell (npts, pts))
  {
    if (npts == 3 || npts == 4)
    {
      std::pair<Polygon_t, Polygon_t> polypair; // only first polygon in the pair is used
      polypair.first.resize (npts);
      // Get the points of the polygon
      for (int i = 0; i < npts; i++)
      {
        double p[3];
        points->GetPoint (pts[i], p);
        polypair.first[i].x = p[0];
        polypair.first[i].y = p[1];
        polypair.first[i].z = p[2];
      }
      mesh.push_back (polypair);
    }
    else
      print_warn (stderr, "Skipping cell as it has %d points, and DXF writer can handle only 3 or 4\n", n, npts);
    
  }

  // Writing DXF file
  print_info (stderr, "Writing "); print_value (stderr, "%d", mesh.size ());
  fprintf (stderr, " polygons to "); print_value (stderr, "%s\n", argv [p_file_indices_dxf.at (0)]);
  dxfwriter::WriteMesh (mesh, argv [p_file_indices_dxf.at (0)]);
  fprintf (stderr, "[done]\n");
}
/* ]--- */
