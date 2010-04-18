%%
%% Copyright (C) 2009 by Lars Kunze, Lorenz Moesenlechner, Moritz Tenorth
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
:- module(ias_semantic_map,
    [
      readMap/1,
      rdf_atom_no_ns/2,
      objectType/2,
      rootObjects/1,
      rootObject/1,
      objectDimensions/4,
      objectPose/2,
      childObject/2,
      childObjects/2,
      semanticMap/1,
      connectionType/3,
      jointName/3,
      objectAtPoint2D/2,
      objectAtPoint2D/3
    ]).



:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_edit')).
:- use_module(library('semweb/actionmodel')).


:- owl_parser:owl_parse('../owl/ias_semantic_map.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ias_map, 'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#', [keep(true)]).


readMap([R0|EntityList]) :-
    readEntity('http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls', 'null', R0),
    setof(TopLevelObj,
          ( owl_individual_of(TopLevelObj, knowrob:'StorageConstruct');
            owl_individual_of(TopLevelObj, knowrob:'FurniturePiece');
            owl_individual_of(TopLevelObj, knowrob:'CounterTop') ),
          TopLevelObjs),
    readEntities(TopLevelObjs, 'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls', EntityList).
    

readEntities([], _, []).
readEntities([E|Es], P, Res) :-
  ((setof(Prt, rdf_has(E, knowrob:properPhysicalPartTypes, Prt), Prts)) -> (
    readEntity(E, P, R0),
    readEntities(Prts, E, Rs),
    append([R0], Rs, R)
  );(
    readEntity(E, P, R1),
    R=[R1]
  )),
  readEntities(Es, P, Res1),
  append(R, Res1, Res).

  
readEntity(E, Parent, [E_no_ns, Parent_no_ns, Pose]) :-
    rdf_atom_no_ns(E, E_no_ns),
    ((Parent = 'null') ->
     (Parent_no_ns = Parent) ;
     rdf_atom_no_ns(Parent, Parent_no_ns)),
    objectPose( E, Pose ).

rdf_atom_no_ns( Tin, Tout ) :-
    rdf_split_url(_, Tout, Tin ).

objectType(O, T) :-
    rdf_has( O, rdf:type, T ).

semanticMap( Map ) :-
    rdf_has(Map, rdf:type, knowrob:'SemanticEnvironmentMap').

rootObjects( Os ) :-
    setof( O, rootObject(O), Os).

rootObject( O ) :-
    owl_individual_of(O, knowrob:'StorageConstruct');
    owl_individual_of(O, knowrob:'WallOfAConstruction');
    owl_individual_of(O, knowrob:'FurniturePiece');
    owl_individual_of(O, knowrob:'CounterTop').

objectDimensions( O, W, D, H ) :-
    rdf_has( O, knowrob:'widthOfObject', literal(type(_, W_)) ),
    atom_number(W_, W),
    rdf_has( O, knowrob:'depthOfObject', literal(type(_, D_)) ),
    atom_number(D_, D),
    rdf_has( O, knowrob:'heightOfObject', literal(type(_, H_)) ),
    atom_number(H_, H).

objectDimensions( O, W, D, H ) :-
    % The depth of a knob defaults to 3cm here. This information
    % should either be asserted somewhere else or be set as a property
    % when importing the semantic map.
    rdf_has( O, knowrob:'radius', literal(type(_, R_)) ),
    atom_number(R_, R),
    W is 2 * R,
    H is 2 * R,
    D is 0.03.

childObject( P, C ) :-
    rdf_has( P, knowrob:'properPhysicalPartTypes', C ),
    not( owl_individual_of(C, knowrob:'Connection-Physical') ).

connectionType( P, C, 'HingedJoint' ) :-
    rdf_has( P, knowrob:'properPhysicalPartTypes', C ),
    rdf_has( C, knowrob:'properPhysicalPartTypes', J ),
    owl_individual_of(J, knowrob:'HingedJoint'), !.

connectionType( P, P, 'SliderJoint' ) :-
    rdf_has( P, rdf:type, knowrob:'Drawer' ), !.

connectionType( P, C, 'Fixed' ) :-
    rdf_has( P, knowrob:'properPhysicalPartTypes', C ).

jointName( P, C, Name ) :-
    rdf_has( P, knowrob:'properPhysicalPartTypes', C ),
    rdf_has( C, knowrob:'properPhysicalPartTypes', Name ),
    owl_individual_of( Name, knowrob:'Connection-Physical' ).

childObjects( P, Cs ) :-
    setof( C, childObject(P, C), Cs).

objectPose( O, P ) :-
    rdf_triple(knowrob:orientation, O, Pobj ),
    bagof( V,
           X^Y^(member(Y, [0, 1, 2, 3]),
                member(X, [0, 1, 2, 3]),
                matrixValue( Pobj, X, Y, V )),
           P ).

matrixValue( M, X, Y, V ) :-
    concat( m, Y, Tmp ),
    concat( Tmp, X, Name ),
    rdf_global_id(knowrob:Name, FullName),
    rdf_triple( FullName, M, literal(type(_, V_)) ),
    atom_number( V_, V ).

% todo: generalize projection to floor. polygon instead of rectangle.
objectAtPoint2D(Point2D, Obj) :-
    % get coordinates of point of interest
    rdf_triple(knowrob:xCoord, Point2D, literal(type(_,PCx))),atom_to_term(PCx,PX,_),
    rdf_triple(knowrob:yCoord, Point2D, literal(type(_,PCy))),atom_to_term(PCy,PY,_),
    objectAtPoint2D(PX,PY,Obj).
 
objectAtPoint2D(PX,PY,Obj) :-

    % get information of potential objects at positon point2d (x/y)

    rdf_has(Obj, knowrob:widthOfObject, literal(type(_,Ow))),atom_to_term(Ow,OW,_),
    rdf_has(Obj, knowrob:depthOfObject, literal(type(_,Od))),atom_to_term(Od,OD,_),

    rdf_triple(knowrob:orientation, Obj, Mat),
    rdf_triple(knowrob:m03, Mat, literal(type(_,TM03))), atom_to_term(TM03,OX,_),
    rdf_triple(knowrob:m13, Mat, literal(type(_,TM13))), atom_to_term(TM13,OY,_),
    rdf_triple(knowrob:m00, Mat, literal(type(_,TM00))), atom_to_term(TM00,M00,_),
    rdf_triple(knowrob:m01, Mat, literal(type(_,TM01))), atom_to_term(TM01,M01,_),
    rdf_triple(knowrob:m10, Mat, literal(type(_,TM10))), atom_to_term(TM10,M10,_),
    rdf_triple(knowrob:m11, Mat, literal(type(_,TM11))), atom_to_term(TM11,M11,_),

    % object must have an extension
    <(0,OW), <(0,OD),

    % calc corner points of rectangle (consider rectangular objects only!)
    P0X is (OX - 0.5*OW),
    P0Y is (OY + 0.5*OD),
    P1X is (OX + 0.5*OW),
    P1Y is (OY + 0.5*OD),
    P2X is (OX - 0.5*OW),
    P2Y is (OY - 0.5*OD),
    % rotate points
    RP0X is (OX + (P0X - OX) * M00 + (P0Y - OY) * M01), 
    RP0Y is (OY + (P0X - OX) * M10 + (P0Y - OY) * M11), 
    RP1X is (OX + (P1X - OX) * M00 + (P1Y - OY) * M01), 
    RP1Y is (OY + (P1X - OX) * M10 + (P1Y - OY) * M11), 
    RP2X is (OX + (P2X - OX) * M00 + (P2Y - OY) * M01), 
    RP2Y is (OY + (P2X - OX) * M10 + (P2Y - OY) * M11),

    % debug: print rotated points
    %write('P0 X: '), write(P0X), write(' -> '), writeln(RP0X), 
    %write('P0 Y: '), write(P0Y), write(' -> '), writeln(RP0Y), 
    %write('P1 X: '), write(P1X), write(' -> '), writeln(RP1X), 
    %write('P1 Y: '), write(P1Y), write(' -> '), writeln(RP1Y), 
    %write('P2 X: '), write(P2X), write(' -> '), writeln(RP2X), 
    %write('P2 Y: '), write(P2Y), write(' -> '), writeln(RP2Y),

    V1X is (RP1X - RP0X), 
    V1Y is (RP1Y - RP0Y), 

    V2X is (RP2X - RP0X), 
    V2Y is (RP2Y - RP0Y), 

    VPX is (PX - RP0X), 
    VPY is (PY - RP0Y), 

    DOT1 is (VPX * V1X + VPY * V1Y),
    DOT2 is (VPX * V2X + VPY * V2Y),
    DOTV1 is (V1X * V1X + V1Y * V1Y),
    DOTV2 is (V2X * V2X + V2Y * V2Y),

    =<(0,DOT1), =<(DOT1, DOTV1), 
    =<(0,DOT2), =<(DOT2, DOTV2). 
