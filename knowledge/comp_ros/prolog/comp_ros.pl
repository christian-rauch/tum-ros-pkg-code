/** <module> comp_ros

  Interface to ROS to retrieve information via the robot middleware

  The computables defined here are stubs that interface with a client program
  written in Java inside the KBClient class

  Copyright (C) 2010 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(comp_ros,
    [
      comp_lo_orientation/2,
      comp_lo_covariance/2,
      query_lo_orientation/5,
      query_lo_covariance/5
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).


%% comp_lo_orientation(+Name, -Orientation) is nondet.
%
% Compute the orientation of object Name by issueing a 'namequery' to the
% lo-service via the ROS middleware.
%
% @param Name Object identifier
% @param Orientation Orientation matrix (4x4) as String rotMat3D-(P00)-(P01)-(P02)-(P03)-(P10)-(P11)-(P12)-(P13)-(P20)-(P21)-(P22)-(P23)-(P30)-(P31)-(P32)-(P33)

    comp_lo_orientation(Name, Orientation) :-
        rdfs_instance_of(Name, knowrob:'EnduringThing-Localized'),!,
        query_lo_orientation('namequery', Name, -1, -1, Orientation).


%% comp_lo_covariance(+Name, -Covariance) is nondet.
%
% Compute the covariance of object Name by issueing a 'namequery' to the
% lo-service via the ROS middleware.
%
% @param Name Object identifier
% @param Covariance Covariance matrix (6x6) as String covMat3D-(P00)-(P01)-(P02)-(P03)-(P04)-(P05)-(P10)-(P11)-(P12)-(P13)-(P14)-(P15)-(P20)-(P21)-(P22)-(P23)-(P24)-(P25)-(P30)-(P31)-(P32)-(P33)-(P34)-(P35)-(P40)-(P41)-(P42)-(P43)-(P44)-(P45)-(P50)-(P51)-(P52)-(P53)-(P54)-(P55)

    comp_lo_covariance(Name, Covariance) :-
        rdfs_instance_of(Name, knowrob:'EnduringThing-Localized'),!,
        query_lo_covariance('namequery', Name, -1, -1, Covariance).



%% query_lo_orientation(+Type, ?Name, ?ObjID, ?ParentID, -Orientation) is nondet.
%
% Low-level interface to the ROS lo-service for computing and converting positions
% and orientations in different coordinate systems
%
% @param Type lo query type, either 'namequery' or 'framequery'
% @param Name lo object name (needs to be bound for 'namequery'
% @param ObjID lo object ID (needs to be bound for 'framequery'
% @param ParentID lo parent object ID (i.e. ID of the reference coordinate frame), can be bound for 'framequery'
% @param Orientation returned orientation matrix (4x4) as String rotMat3D-(P00)-(P01)-(P02)-(P03)-(P10)-(P11)-(P12)-(P13)-(P20)-(P21)-(P22)-(P23)-(P30)-(P31)-(P32)-(P33)

    query_lo_orientation(Type, Name, ObjID, ParentID, Orientation) :-

        % call lo
        jpl_call('KBClient', 'loQuery', [Type, Name, ObjID, ParentID], Answer),
        jpl_get(Answer, 'pose', JPose),
        classifiers:arrays_to_lists(JPose, Pose),

        % read the single pose values from the result, combine to string representation
        nth0(0, Pose, P00),  nth0(1,  Pose, P01), nth0(2,  Pose, P02), nth0(3,  Pose, P03),
        nth0(4, Pose, P10),  nth0(5,  Pose, P11), nth0(6,  Pose, P12), nth0(7,  Pose, P13),
        nth0(8, Pose, P20),  nth0(9,  Pose, P21), nth0(10, Pose, P22), nth0(11, Pose, P23),
        nth0(12, Pose, P30), nth0(13, Pose, P31), nth0(14, Pose, P32), nth0(15, Pose, P33),
        term_to_atom(rotMat3D-(P00)-(P01)-(P02)-(P03)-(P10)-(P11)-(P12)-(P13)-(P20)-(P21)-(P22)-(P23)-(P30)-(P31)-(P32)-(P33), Orientation).




%% query_lo_covariance(+Type, ?Name, ?ObjID, ?ParentID, -Covariance) is nondet.
%
% Low-level interface to the ROS lo-service for computing and converting positions
% and orientations in different coordinate systems
%
% @param Type lo query type, either 'namequery' or 'framequery'
% @param Name lo object name (needs to be bound for 'namequery'
% @param ObjID lo object ID (needs to be bound for 'framequery'
% @param ParentID lo parent object ID (i.e. ID of the reference coordinate frame), can be bound for 'framequery'
% @param Covariance returned covariance matrix (6x6) as String covMat3D-(P00)-(P01)-(P02)-(P03)-(P04)-(P05)-(P10)-(P11)-(P12)-(P13)-(P14)-(P15)-(P20)-(P21)-(P22)-(P23)-(P24)-(P25)-(P30)-(P31)-(P32)-(P33)-(P34)-(P35)-(P40)-(P41)-(P42)-(P43)-(P44)-(P45)-(P50)-(P51)-(P52)-(P53)-(P54)-(P55)

    query_lo_covariance(Type, Name, ObjID, ParentID, Covariance) :-

        % call lo
        jpl_call('KBClient', 'loQuery', [Type, Name, ObjID, ParentID], Answer),
        jpl_get(Answer, 'cov', JCov),
        classifiers:arrays_to_lists(JCov, Cov),

        % read the single cov values from the result, combine to string representation
        nth0(0,  Cov, P00), nth0(1,  Cov, P01), nth0(2,  Cov, P02), nth0(3,  Cov, P03), nth0(4,  Cov, P04), nth0(5,  Cov, P05),
        nth0(6,  Cov, P10), nth0(7,  Cov, P11), nth0(8,  Cov, P12), nth0(9,  Cov, P13), nth0(10, Cov, P14), nth0(11, Cov, P15),
        nth0(12, Cov, P20), nth0(13, Cov, P21), nth0(14, Cov, P22), nth0(15, Cov, P23), nth0(16, Cov, P24), nth0(17, Cov, P25),
        nth0(18, Cov, P30), nth0(19, Cov, P31), nth0(20, Cov, P32), nth0(21, Cov, P33), nth0(22, Cov, P34), nth0(23, Cov, P35),
        nth0(24, Cov, P40), nth0(25, Cov, P41), nth0(26, Cov, P42), nth0(27, Cov, P43), nth0(28, Cov, P44), nth0(29, Cov, P45),
        nth0(30, Cov, P50), nth0(31, Cov, P51), nth0(32, Cov, P52), nth0(33, Cov, P53), nth0(34, Cov, P54), nth0(35, Cov, P55),

        term_to_atom(covMat3D-(P00)-(P01)-(P02)-(P03)-(P04)-(P05)-(P10)-(P11)-(P12)-(P13)-(P14)-(P15)-(P20)-(P21)-(P22)-(P23)-(P24)-(P25)-(P30)-(P31)-(P32)-(P33)-(P34)-(P35)-(P40)-(P41)-(P42)-(P43)-(P44)-(P45)-(P50)-(P51)-(P52)-(P53)-(P54)-(P55), Covariance).

