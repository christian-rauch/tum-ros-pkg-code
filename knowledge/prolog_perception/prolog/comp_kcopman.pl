/** <module> comp_kcopman

  Read object instances from the k-copman system

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

:- module(comp_kcopman,
    [
      objects_on_table/2,
      comp_kcopman/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:-  rdf_meta
    comp_kcopman(r, r).


%% comp_kcopman(-Obj, -Class)
%
% read object information from ROS
%
comp_kcopman(Perception, 'http://ias.cs.tum.edu/kb/knowrob.owl#TableMemory') :-

  % used for the computable that generates perception instances on the fly
  % seems to work more or less, apart from the member() call in the end which
  % returns just one instance

  % query the ROS module
  getTableMemoryObjects(RosObjList),!,

  % create all the object instances
  create_object_instances_from_ros_list(RosObjList, _, Perceptions),!,

  % create all the table instances
  create_table_instances_from_ros_list(RosObjList, _),!,

  member(Perception, Perceptions).



%% objects_on_table(-Obj)
%
% read object information from ROS
%
objects_on_table(Tables, Objs) :-

  % query the ROS module
  getTableMemoryObjects(RosObjList),!,

  % create all the object instances
  create_object_instances_from_ros_list(RosObjList, Objs, _),!,

  % create all the table instances
  create_table_instances_from_ros_list(RosObjList, Tables),!.



%% create_object_instances_from_ros_list(+LongList, -Objs, -Perceptions)
%
% split the long list into short pieces and create the object instances
%
create_object_instances_from_ros_list(LongList, [Obj|Objs], [Perception|Perceptions]) :-

  append([_TableID, Time, _T00, _T01, _T02, _T03, _T10, _T11, _T12, _T13, _T20, _T21, _T22, _T23, _T30, _T31, _T32, _T33, _TWidth, _TDepth, _THeight, Type, _Color, GeometricCategory, KCopManID, CopID, LoID, Xo, Yo, Zo], Rest, LongList),



%   (((float(Xo),float(Yo),float(Zo))) -> (
  (((Type='nn');(Type='Tea-Iced')) -> (
      true
   ) ; (
      % create object instance, using the CoP ID to build the object identifier
      string_to_atom(Type, LocalTypeAtom),
      atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocalTypeAtom, TypeAtom),
      atom_concat(TypeAtom, CopID, Obj),

      (rdf_has(Obj, rdf:type, TypeAtom);
      rdf_assert(Obj, rdf:type, TypeAtom)),

      % create perception using the LoID as identifier (describing single detections of an object)
      %atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#TableMemory', KCopManID, Perception),
      rdf_instance_from_class('http://ias.cs.tum.edu/kb/knowrob.owl#TableMemory', Perception),
      rdf_assert(Perception, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#TableMemory'),
      rdf_assert(Perception, knowrob:objectActedOn, Obj),

      % create detection time point
      atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_', Time, TimePoint),
      rdf_assert(TimePoint, rdf:type, knowrob:'TimePoint'),
      rdf_assert(Perception, knowrob:startTime, TimePoint),

      % object properties (uncomment when available from k-copman)
      % atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', Color, Color1),
      % atom_concat(Color1, 'Color', ObjColor), % build atoms like 'BlackColor'
      % rdf_assert(Obj, knowrob:mainColorOfObject, ObjColor),
      string_to_atom(GeometricCategory, GeometricCategoryAtom),
      rdf_assert(Obj, knowrob:objectShapeType, GeometricCategoryAtom),

      string_to_atom(LoID, LoIDAtom),
      term_to_atom(LoIDTerm, LoIDAtom),
      rdf_assert(Perception, knowrob:loID, literal(type(xsd:int, LoIDTerm))),           % LoID and KCopManID characterize single object detections

      string_to_atom(KCopManID, KCopManIDAtom),
      term_to_atom(KCopManIDTerm, KCopManIDAtom),
      rdf_assert(Perception, knowrob:kcopmanID, literal(type(xsd:int, KCopManIDTerm))),

      string_to_atom(CopID, CopIDAtom),
      term_to_atom(CopIDTerm, CopIDAtom),
      rdf_assert(Obj, knowrob:copID, literal(type(xsd:int, CopIDTerm))),                % CopID is supposed to describe objects (though re-finding an object does not seem to work reliably)

      % set the pose
      atomic_list_concat(['rotMat3D_1_0_0_',Xo,'_0_1_0_',Yo,'_0_0_1_',Zo,'_0_0_0_1'], LocIdentifier),

      atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
      rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),
      rdf_assert(Perception, knowrob:eventOccursAt, Loc)

  )),
  create_object_instances_from_ros_list(Rest, Objs, Perceptions).

create_object_instances_from_ros_list([], [], []).



%% create_table_instances_from_ros_list(+LongList, -Objs, +Index)
%
% split the long list into short pieces and create the table instances
%
create_table_instances_from_ros_list(LongList, [Table|Tables]) :-

%   append([TableID, Time, Tx, Ty, Tz, _Type, _Color, _GeometricCategory, _ObjID1, _Xo, _Yo, _Zo], Rest, LongList),
  append([TableID, Time, T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23, T30, T31, T32, T33, TWidth, TDepth, THeight, _Type, _Color, _GeometricCategory, _KCopManID, _CopID, _LoID, _Xo, _Yo, _Zo], Rest, LongList),

  % create table instance
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable', TableID, Table),

  ((rdf_has(Table, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable')) -> (
    % create perception
    rdf_instance_from_class('http://ias.cs.tum.edu/kb/knowrob.owl#TableMemory', Perception),
    rdf_assert(Perception, knowrob:objectActedOn, Table),

    % create time point
    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_', Time, TimePoint),
    rdf_assert(TimePoint, rdf:type, knowrob:'TimePoint'),
    rdf_assert(Perception, knowrob:startTime, TimePoint),

    % set the pose
    atomic_list_concat(['rotMat3D_',T00,'_',T01,'_',T02,'_',T03,'_',T10,'_',T11,'_',T12,'_',T13,'_',T20,'_',T21,'_',T22,'_',T23,'_',T30,'_',T31,'_',T32,'_',T33], LocIdentifier),
    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),
    rdf_assert(Perception, knowrob:eventOccursAt, Loc),

    create_table_instances_from_ros_list(Rest, Tables)
  ) ; (
    rdf_assert(Table, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable'),
    
    % create perception
    rdf_instance_from_class('http://ias.cs.tum.edu/kb/knowrob.owl#TableMemory', Perception),
    rdf_assert(Perception, knowrob:objectActedOn, Table),

    % create time point
    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_', Time, TimePoint),
    rdf_assert(TimePoint, rdf:type, knowrob:'TimePoint'),
    rdf_assert(Perception, knowrob:startTime, TimePoint),

    % create table instance
    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable', TableID, Table),
    rdf_assert(Table, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable'),
    rdf_assert(Table, knowrob:widthOfObject,  literal(type('http://www.w3.org/2001/XMLSchema#float', TWidth))),
    rdf_assert(Table, knowrob:depthOfObject,  literal(type('http://www.w3.org/2001/XMLSchema#float', TDepth))),
    rdf_assert(Table, knowrob:heightOfObject, literal(type('http://www.w3.org/2001/XMLSchema#float', THeight))),


    % set the pose
    atomic_list_concat(['rotMat3D_',T00,'_',T01,'_',T02,'_',T03,'_',T10,'_',T11,'_',T12,'_',T13,'_',T20,'_',T21,'_',T22,'_',T23,'_',T30,'_',T31,'_',T32,'_',T33], LocIdentifier),
    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),
    rdf_assert(Perception, knowrob:eventOccursAt, Loc),

    create_table_instances_from_ros_list(Rest, Tables)
  )).

create_table_instances_from_ros_list([], []).


% compatibility with Prolog < 5.8
:- if(\+current_predicate(atomic_list_concat, _)).

  atomic_list_concat(List, Atom) :-
    concat_atom(List, Atom).

:- endif.
