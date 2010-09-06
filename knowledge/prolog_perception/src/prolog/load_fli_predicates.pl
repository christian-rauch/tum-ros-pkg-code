%%
%% Copyright (c) 2009, Dejan Pangercic <dejan.pangercic@cs.tum.edu>
%% All rights reserved.
%%
%% Redistribution and use in source and binary forms, with or without
%% modification, are permitted provided that the following conditions are met:
%%
%%     * Redistributions of source code must retain the above copyright
%%       notice, this list of conditions and the following disclaimer.
%%     * Redistributions in binary form must reproduce the above copyright
%%       notice, this list of conditions and the following disclaimer in the
%%       documentation and/or other materials provided with the distribution.
%%     * Neither the name of Willow Garage, Inc. nor the names of its
%%       contributors may be used to endorse or promote products derived from
%%       this software without specific prior written permission.
%%
%% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%% POSSIBILITY OF SUCH DAMAGE.
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% Set of predicates that load Foreign Language Interface (FLI) Functions
%% operating on the ROS C++ structures. The functions and mappings to prolog
%% predicates are implemented in:
%% cd `rospack find prolog_perception`/src/cpp/*.cpp files.
%%
%% Load as following:
%% cd `rospack find prolog_perception`/src/prolog && prolog
%% consult('load_fli_predicates.pl').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% Disables truncation, prettier printing
:- set_prolog_flag(toplevel_print_anon, false).
:- set_prolog_flag(toplevel_print_options, [quoted(true), portray(true), max_depth(0), attributes(portray)]).
:- set_prolog_flag(float_format, '%.15g').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% queries for and returns a plane with its Id and equation's coefficients
%% in a PointCloud
%% Return type:
%% [table_id, a, b, c, d]
%:- unload_foreign_library('../../lib/libros_query_plane').
%:- load_foreign_library('../../lib/libros_query_plane').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% queries and returns unique identifiers for found tables and clusters located on tables
%% in a PointCloud 
%% Return type:
%% [table_id, [[cl_x, cl_y, cl_z], [], [], ...]
%:- unload_foreign_library('../../lib/libros_query_table_objects').
%:- load_foreign_library('../../lib/libros_query_table_objects').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% queries COP (https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/perception/cop/)
%% to locate clusters in PointCloud data and checks them against given colors (to the time
%% RGB and black and white are supported)
%:- unload_foreign_library('../../lib/libros_query_cluster_color').
%:- load_foreign_library('../../lib/libros_query_cluster_color').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% queries table_memmory node
%% (https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/mapping/cloud_tools/src/table_memory.cpp)
%% for found tables and on them located clusters in PointCloud data.
%% Return type:
%% [[TableId, TimeStamp1, Tx1, Ty1, Tz1, Obj1, ObjID1, Ox1, Oy1, Oz1], ...]
% :- unload_foreign_library('../../lib/libros_query_table_memory').
:- load_foreign_library('../../lib/libros_query_table_memory').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%queries table_memmory node
%%(https://tum-ros-pkg.svn.sourceforge.net/svnroot/tum-ros-pkg/mapping/cloud_tools/src/table_memory.cpp)
%%for found tables and on them located clusters in PointCloud data.
%%Return type:
%%[[TableId, Tx1, Ty1, Tz1, Obj1, Ox1, Oy1, Oz1], ...]
%%The data is used in another pipeline with knowledge processing 
%%and reasoning in order to infer the missing objects on the tabletop.
%:- unload_foreign_library('../../lib/libros_query_tabletop_missing_objects').
%:- load_foreign_library('../../lib/libros_query_tabletop_missing_objects').
