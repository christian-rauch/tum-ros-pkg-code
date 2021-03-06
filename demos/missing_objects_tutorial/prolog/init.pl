%%
%% Copyright (C) 2010 by Moritz Tenorth
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(comp_cop).
:- register_ros_package(comp_missingobj).
% :- register_ros_package(mod_vis).
% :- register_ros_package(mod_srdl).
% :- register_ros_package(ias_semantic_map).
% :- register_ros_package(comp_germandeli).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start listeners

:- print('\n\nstarting listeners...\n\n'), flush.

% :- comp_cop:cop_listener(_).
% :- comp_cop:odufinder_listener(_).
:- comp_cop:missing_obj_listener(_).

% :- visualisation_canvas(_).




