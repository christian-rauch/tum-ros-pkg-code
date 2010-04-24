/** <module> comp_spatial

  This module contains all computables that calculate qualitative spatial relations
  between objects to allow for spatial reasoning. In addition, there are computables
  to extract components of a matrix or position vector.


  Copyright (C) 2010 by Moritz Tenorth, Lars Kunze

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

@author Moritz Tenorth, Lars Kunze
@license GPL
*/

:- module(comp_spatial,
    [
      holds/2,
      comp_orientation/2,
      on_Physical/2,
      in_ContGeneric/2,
      comp_toTheRightOf/2,
      comp_toTheLeftOf/2,
      comp_toTheSideOf/2,
      comp_inFrontOf/2,
      comp_inCenterOf/2,
      comp_center/2,
      comp_xCoord/2,
      comp_yCoord/2,
      comp_zCoord/2,
      comp_m00/2,
      comp_m01/2,
      comp_m02/2,
      comp_m03/2,
      comp_m04/2,
      comp_m05/2,
      comp_m10/2,
      comp_m11/2,
      comp_m12/2,
      comp_m13/2,
      comp_m14/2,
      comp_m15/2,
      comp_m20/2,
      comp_m21/2,
      comp_m22/2,
      comp_m23/2,
      comp_m23/2,
      comp_m25/2,
      comp_m30/2,
      comp_m31/2,
      comp_m32/2,
      comp_m33/2,
      comp_m34/2,
      comp_m35/2,
      comp_m40/2,
      comp_m41/2,
      comp_m42/2,
      comp_m43/2,
      comp_m44/2,
      comp_m45/2,
      comp_m50/2,
      comp_m51/2,
      comp_m52/2,
      comp_m53/2,
      comp_m54/2,
      comp_m55/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).

:- owl_parser:owl_parse('../owl/comp_spatial.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_spatial, 'http://ias.cs.tum.edu/kb/comp_spatial.owl#', [keep(true)]).



% define holds as meta-predicate and allow the definitions
% to be in different parts of the source file
:- meta_predicate holds(0, ?, ?).
:- discontiguous holds/2.

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    holds(:, r),
    holds_tt(:,t),
    comp_orientation(r, r),
    on_Physical(r, r),
    in_ContGeneric(r, r),
    adjacent_Objects(r, r),
    comp_toTheSideOf(r, r),    comp_toTheRightOf(r, r),    comp_toTheLeftOf(r, r),
    comp_inFrontOf(r, r),
    comp_inCenterOf(r, r),
    comp_center(r, r), comp_xCoord(r, r), comp_yCoord(r, r), comp_zCoord(r, r),
    comp_m00(r, r),    comp_m01(r, r),    comp_m02(r, r),    comp_m03(r, r),    comp_m04(r, r),    comp_m05(r, r),
    comp_m10(r, r),    comp_m11(r, r),    comp_m12(r, r),    comp_m13(r, r),    comp_m14(r, r),    comp_m15(r, r),
    comp_m20(r, r),    comp_m21(r, r),    comp_m22(r, r),    comp_m23(r, r),    comp_m23(r, r),    comp_m25(r, r),
    comp_m30(r, r),    comp_m31(r, r),    comp_m32(r, r),    comp_m33(r, r),    comp_m34(r, r),    comp_m35(r, r),
    comp_m40(r, r),    comp_m41(r, r),    comp_m42(r, r),    comp_m43(r, r),    comp_m44(r, r),    comp_m45(r, r),
    comp_m50(r, r),    comp_m51(r, r),    comp_m52(r, r),    comp_m53(r, r),    comp_m54(r, r),    comp_m55(r, r).




%% on_Physical(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
% 
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%
on_Physical(Top, Bottom) :-
    get_timepoint(T),
    holds(on_Physical(Top, Bottom), T).


%% holds(on_Physical(?Top, ?Bottom), +T) is nondet.
%
% Check if Top has been in the area of and above Bottom at time point T.
%
% Currently does not take the orientation into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%
holds(on_Physical(Top, Bottom), T) :-

    object_perception(Top, T, VPT),
    object_perception(Bottom, T, VPB),

    rdf_triple(knowrob:eventOccursAt, VPT,    TopMatrix),
    rdf_triple(knowrob:eventOccursAt, VPB, BottomMatrix),

    rdf_triple(knowrob:m03, TopMatrix, literal(type(_,TCx))),atom_to_term(TCx,TX,_),
    rdf_triple(knowrob:m13, TopMatrix, literal(type(_,TCy))),atom_to_term(TCy,TY,_),
    rdf_triple(knowrob:m23, TopMatrix, literal(type(_,TCz))),atom_to_term(TCz,TZ,_),

    rdf_triple(knowrob:m03, BottomMatrix, literal(type(_,BCx))),atom_to_term(BCx,BX,_),
    rdf_triple(knowrob:m13, BottomMatrix, literal(type(_,BCy))),atom_to_term(BCy,BY,_),
    rdf_triple(knowrob:m23, BottomMatrix, literal(type(_,BCz))),atom_to_term(BCz,BZ,_),

    % read the dimensions of the bottom entity
    rdf_has(Bottom, knowrob:widthOfObject, literal(type(_,Bw))),atom_to_term(Bw,BW,_),
    rdf_has(Bottom, knowrob:depthOfObject, literal(type(_,Bd))),atom_to_term(Bd,BD,_),

    % the criterion is if the difference between them is less than epsilon=5cm
    =<( BZ, TZ ),

    % additional criterion: center of the top entity has to be inside the
    % area of the bottom entity
    =<( (BX - 0.5*BD), TX ), >=( (BX + 0.5*BD), TX ),
    =<( (BY - 0.5*BW), TY ), >=( (BY + 0.5*BW), TY ),
    Top \= Bottom.




%% comp_toTheLeftOf(?Left, ?Right) is nondet.
%
% Check if Left is to the left of Right.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Left Identifier of the left Object
% @param Right Identifier of the right Object
% 
comp_toTheLeftOf(Left, Right) :-
    get_timepoint(T),
    holds(comp_toTheLeftOf(Left, Right), T).


%% holds(comp_toTheLeftOf(?Left, ?Right), +T) is nondet.
%
% Check if Left is to the left of Right. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%
holds(comp_toTheLeftOf(Left, Right), T) :-
    %
    % TODO: adapt this to take rotations and object dimensions into account
    %

    object_perception(Left, T, VPL),
    object_perception(Right, T, VPR),

    rdf_triple(knowrob:eventOccursAt, VPL, LeftMatrix),
    rdf_triple(knowrob:eventOccursAt, VPR, RightMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, LeftMatrix, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
    rdf_triple(knowrob:m13, LeftMatrix, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
    rdf_triple(knowrob:m23, LeftMatrix, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, RightMatrix, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
    rdf_triple(knowrob:m13, RightMatrix, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
    rdf_triple(knowrob:m23, RightMatrix, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),

    =<( abs( LX - RX), 0.30),  % less than 30cm y diff
    =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
    =<( abs( LZ - RZ), 0.30),  % less than 30cm height diff
    Left \= Right.



%% comp_toTheRightOf(?Right,?Left) is nondet.
%
% Check if Right is to the right of Left.
% 
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @see comp_toTheLeftOf
% 
comp_toTheRightOf(Right, Left) :-
    get_timepoint(T),
    holds(comp_toTheRightOf(Right, Left), T).


%% holds(comp_toTheRightOf(?Right,?Left), +T) is nondet.
%
% Check if Right is to the right of Left.
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @param T      TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
% 
holds(comp_toTheRightOf(Right, Left), T) :-
    holds(comp_toTheLeftOf(Left, Right), T).




%% comp_toTheSideOf(?A, ?B) is nondet.
%
% Check if A is either to the left or the rigth of B.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
% 
% @param A Identifier of Object A
% @param B Identifier of Object B
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
% 
comp_toTheSideOf(A, B) :-
    get_timepoint(T),
    holds(comp_toTheSideOf(A, B), T).

%% holds(comp_toTheSideOf(?A, ?B), +T) is nondet.
%
% Check if A is either to the left or the right of B.
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @param T TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
% 
holds(comp_toTheSideOf(A, B), T) :-
    holds(comp_toTheRightOf(A, B), T);
    holds(comp_toTheLeftOf(A, B), T).




%% comp_inFrontOf(?Front, ?Back) is nondet.
%
% Check if Front is in front of Back. Currently does not take the orientation
% into account, only the position and dimension.
% 
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Front Identifier of the front Object
% @param Back Identifier of the back Object
% 
comp_inFrontOf(Front, Back) :-
    get_timepoint(T),
    holds(comp_inFrontOf(Front, Back), T).

%% holds(comp_inFrontOf(?Front, ?Back), +T) is nondet.
% 
% Check if Front is in front of Back. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Front Identifier of the front Object
% @param Back Identifier of the back Object
% @param T TimePoint or Event for which the relations is supposed to hold
% 
holds(comp_inFrontOf(Front, Back), T) :-
    %
    % TODO: adapt this to take rotations and object dimensions into account
    %

    object_perception(Front, T, VPF),
    object_perception(Back, T, VPB),

    rdf_triple(knowrob:eventOccursAt, VPF, FrontMatrix),
    rdf_triple(knowrob:eventOccursAt, VPB, BackMatrix),

    % read the center coordinates of the front entity
    rdf_triple(knowrob:m03, FrontMatrix, literal(type(_,FCx))),atom_to_term(FCx,FX,_),

    % read the center coordinates of the back entity
    rdf_triple(knowrob:m03, BackMatrix, literal(type(_,BCx))),atom_to_term(BCx,BX,_),

    =<( BX, FX ),      % front obj has a higher x coord
    Front \= Back.




%% comp_inCenterOf(?Inner, ?Outer) is nondet.
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
% 
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
% 
comp_inCenterOf(Inner, Outer) :-
    get_timepoint(T),
    holds(comp_inCenterOf(Inner, Outer), T).

%% holds(comp_inCenterOf(?Inner, ?Outer), +T) is nondet.
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
% @param T TimePoint or Event for which the relations is supposed to hold
% 
holds(comp_inCenterOf(Inner, Outer), T) :-

    object_perception(Inner, T, VPI),
    object_perception(Outer, T, VPO),

    rdf_triple(knowrob:eventOccursAt, VPI, InnerMatrix),
    rdf_triple(knowrob:eventOccursAt, VPO, OuterMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, InnerMatrix, literal(type(_,ICx))),atom_to_term(ICx,IX,_),
    rdf_triple(knowrob:m13, InnerMatrix, literal(type(_,ICy))),atom_to_term(ICy,IY,_),
    rdf_triple(knowrob:m23, InnerMatrix, literal(type(_,ICz))),atom_to_term(ICz,IZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, OuterMatrix, literal(type(_,OCx))),atom_to_term(OCx,OX,_),
    rdf_triple(knowrob:m13, OuterMatrix, literal(type(_,OCy))),atom_to_term(OCy,OY,_),
    rdf_triple(knowrob:m23, OuterMatrix, literal(type(_,OCz))),atom_to_term(OCz,OZ,_),

    =<( abs( IX - OX), 0.20),  % less than 20cm x diff
    =<( abs( IY - OY), 0.20),  % less than 20cm y diff
    =<( abs( IZ - OZ), 0.20),  % less than 20cm z diff
    Inner \= Outer.


%% in_ContGeneric(?InnerObj, ?OuterObj) is nondet.
%
% Check if InnerObj is contained by OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
% 
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
% 
in_ContGeneric(InnerObj, OuterObj) :-
    get_timepoint(T),
    holds(in_ContGeneric(InnerObj, OuterObj), T).


%% holds(in_ContGeneric(?InnerObj, ?OuterObj), +T) is nondet.
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
% @param T TimePoint or Event for which the relations is supposed to hold
% 
holds(in_ContGeneric(InnerObj, OuterObj), T) :-

    object_perception(InnerObj, T, VPI),
    object_perception(OuterObj, T, VPO),

    rdf_triple(knowrob:eventOccursAt, VPI, InnerObjMatrix),
    rdf_triple(knowrob:eventOccursAt, VPO, OuterObjMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, InnerObjMatrix, literal(type(_,ICx))),atom_to_term(ICx,IX,_),
    rdf_triple(knowrob:m13, InnerObjMatrix, literal(type(_,ICy))),atom_to_term(ICy,IY,_),
    rdf_triple(knowrob:m23, InnerObjMatrix, literal(type(_,ICz))),atom_to_term(ICz,IZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, OuterObjMatrix, literal(type(_,OCx))),atom_to_term(OCx,OX,_),
    rdf_triple(knowrob:m13, OuterObjMatrix, literal(type(_,OCy))),atom_to_term(OCy,OY,_),
    rdf_triple(knowrob:m23, OuterObjMatrix, literal(type(_,OCz))),atom_to_term(OCz,OZ,_),

    % read the dimensions of the outer entity
    rdf_has(OuterObj, knowrob:widthOfObject, literal(type(_,Ow))),atom_to_term(Ow,OW,_),
    rdf_has(OuterObj, knowrob:heightOfObject,literal(type(_,Oh))),atom_to_term(Oh,OH,_),
    rdf_has(OuterObj, knowrob:depthOfObject, literal(type(_,Od))),atom_to_term(Od,OD,_),

    % read the dimensions of the inner entity
    rdf_has(InnerObj, knowrob:widthOfObject, literal(type(_,Iw))),atom_to_term(Iw,IW,_),
    rdf_has(InnerObj, knowrob:heightOfObject,literal(type(_,Ih))),atom_to_term(Ih,IH,_),
    rdf_has(InnerObj, knowrob:depthOfObject, literal(type(_,Id))),atom_to_term(Id,ID,_),

    % InnerObj is contained by OuterObj if (center_i+0.5*dim_i)<=(center_o+0.5*dim_o)
    % for all dimensions (x, y, z)
    >=( (IX - 0.5*ID), (OX - 0.5*OD)+0.05 ), =<( (IX + 0.5*ID), (OX + 0.5*OD)-0.05 ),
    >=( (IY - 0.5*IW), (OY - 0.5*OW)+0.05 ), =<( (IY + 0.5*IW), (OY + 0.5*OW)-0.05 ),
    >=( (IZ - 0.5*IH), (OZ - 0.5*OH)+0.05 ), =<( (IZ + 0.5*IH), (OZ + 0.5*OH)-0.05 ),
    InnerObj \= OuterObj.




%% comp_orientation(+Object, -Orientation) is nondet.
%
% Wrapper to compute the (deprecated) object orientation property by
% finding the most current object detection. Intended as convenience
% and compatibility predicate.
% 
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
% 
comp_orientation(Object, Orientation) :-

    % find all perceptions of the object and sort by their start time
    findall([P_i,Object,St], (rdf_has(P_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(P_i,  knowrob:'MentalEvent'),
                              rdf_triple(knowrob:startTime, P_i, StTg),
                              rdf_split_url(_, StTl, StTg),
                              atom_concat('timepoint_', StTa, StTl),
                              term_to_atom(St, StTa)), Perceptions),

    predsort(compare_object_perceptions, Perceptions, Psorted),

    % compute the homography for the newest perception
    nth0(0, Psorted, Newest),
    nth0(0, Newest, NewestPerception),

    rdf_triple(knowrob:eventOccursAt, NewestPerception, Orientation).




% % % % % % % % % % % % % % % % % % % %
% matrix and vector computations (relating the homography-based
% position representation with the old center-point-based one)
%

%% comp_center(+Obj, ?Center) is semidet.
%
% Compute the center point of an object from its homography matrix
%
% @param Obj    The object identifier
% @param Center The center point identifier as a String 'translation_<rotation matrix identifier>'
    comp_center(Obj, Center) :-
      rdf_triple(knowrob:orientation, Obj, Matrix),
      rdf_split_url(G, L, Matrix),
      atom_concat('translation_', L, P),
      rdf_split_url(G, P, Center).


%% comp_xCoord(+Point, ?X) is semidet.
%
% Compute the x-coordinate of the point
%
% @param Obj The point identifier as a String 'translation_<rotation matrix identifier>'
% @param X   The x coordinate value
% @see comp_center
    comp_xCoord(Point, X) :-
      rdf_split_url(G, L, Point),
      atom_concat('translation_', RotMat, L),
      rdf_split_url(G, RotMat, Mat),
      rdf_triple(knowrob:m03, Mat, X0),
      ( (X0=literal(type(_,X)));
         X0=X ).

%% comp_yCoord(+Point, ?Y) is semidet.
%
% Compute the y-coordinate of the point
%
% @param Obj The point identifier as a String 'translation_<rotation matrix identifier>'
% @param Y   The y coordinate value
% @see comp_center
    comp_yCoord(Point, Y) :-
      rdf_split_url(G, L, Point),
      atom_concat('translation_', RotMat, L),
      rdf_split_url(G, RotMat, Mat),
      rdf_triple(knowrob:m13, Mat, Y0),
      ( (Y0=literal(type(_,Y)));
         Y0=Y ).

%% comp_zCoord(+Point, ?Z) is semidet.
%
% Compute the z-coordinate of the point
%
% @param Obj The point identifier as a String 'translation_<rotation matrix identifier>'
% @param Z   The z coordinate value
% @see comp_center
    comp_zCoord(Point, Z) :-
      rdf_split_url(G, L, Point),
      atom_concat('translation_', RotMat, L),
      rdf_split_url(G, RotMat, Mat),
      rdf_triple(knowrob:m23, Mat, Z0),
      ( (Z0=literal(type(_,Z)));
         Z0=Z ).



%% comp_m00(+Matrix, ?M00) is semidet.
%
% Extract component m(0,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M00 Component m(0,0)
    comp_m00(Matrix, M00) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(M00)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m00(Matrix, M00) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(M00)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m01(+Matrix, ?M01) is semidet.
%
% Extract component m(0,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M01 Component m(0,1)
    comp_m01(Matrix, M01) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(M01)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m01(Matrix, M01) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(M01)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m02(+Matrix, ?M02) is semidet.
%
% Extract component m(0,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M02 Component m(0,2)
    comp_m02(Matrix, M02) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(M02)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m02(Matrix, M02) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(M02)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m03(+Matrix, ?M03) is semidet.
%
% Extract component m(0,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M03 Component m(0,3)
    comp_m03(Matrix, M03) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(M03)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m03(Matrix, M03) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(M03)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m04(+Matrix, ?M04) is semidet.
%
% Extract component m(0,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M04 Component m(0,4)
    comp_m04(Matrix, M04) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(M04)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m05(+Matrix, ?M05) is semidet.
%
% Extract component m(0,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M05 Component m(0,5)
    comp_m05(Matrix, M05) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(M05)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).





%% comp_m10(+Matrix, ?M10) is semidet.
%
% Extract component m(1,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M10 Component m(1,0)
    comp_m10(Matrix, M10) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(M10)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m10(Matrix, M10) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(M10)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m11(+Matrix, ?M11) is semidet.
%
% Extract component m(1,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M11 Component m(1,1)
    comp_m11(Matrix, M11) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(M11)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m11(Matrix, M11) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M11)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m12(+Matrix, ?M12) is semidet.
%
% Extract component m(1,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M12 Component m(1,2)
    comp_m12(Matrix, M12) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(M12)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m12(Matrix, M12) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M12)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m13(+Matrix, ?M13) is semidet.
%
% Extract component m(1,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M13 Component m(1,3)
    comp_m13(Matrix, M13) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M13)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m13(Matrix, M13) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M13)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m14(+Matrix, ?M14) is semidet.
%
% Extract component m(1,4) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M14 Component m(1,4)
    comp_m14(Matrix, M14) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M14)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m15(+Matrix, ?M15) is semidet.
%
% Extract component m(1,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M15 Component m(1,5)
    comp_m15(Matrix, M15) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M15)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).




%% comp_m20(+Matrix, ?M20) is semidet.
%
% Extract component m(2,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M20 Component m(2,0)
    comp_m20(Matrix, M20) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M20)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m20(Matrix, M20) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M20)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m21(+Matrix, ?M21) is semidet.
%
% Extract component m(2,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M21 Component m(2,1)
    comp_m21(Matrix, M21) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M21)-(_)-(_)-(_)-(_)-(_)-(_), O).
    comp_m21(Matrix, M21) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M21)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m22(+Matrix, ?M22) is semidet.
%
% Extract component m(2,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M22 Component m(2,2)
    comp_m22(Matrix, M22) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M22)-(_)-(_)-(_)-(_)-(_), O).
    comp_m22(Matrix, M22) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M22)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m23(+Matrix, ?M23) is semidet.
%
% Extract component m(2,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M23 Component m(2,3)
    comp_m23(Matrix, M23) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M23)-(_)-(_)-(_)-(_), O).
    comp_m23(Matrix, M23) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M23)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m24(+Matrix, ?M24) is semidet.
%
% Extract component m(2,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M24 Component m(2,4)
    comp_m24(Matrix, M24) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M24)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m25(+Matrix, ?M25) is semidet.
%
% Extract component m(2,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M25 Component m(2,5)
    comp_m25(Matrix, M25) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M25)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).




%% comp_m30(+Matrix, ?M30) is semidet.
%
% Extract component m(3,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M30 Component m(3,0)
    comp_m30(Matrix, M30) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M30)-(_)-(_)-(_), O).
    comp_m30(Matrix, M30) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M30)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m31(+Matrix, ?M31) is semidet.
%
% Extract component m(3,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M31 Component m(3,1)
    comp_m31(Matrix, M31) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M31)-(_)-(_), O).
    comp_m31(Matrix, M31) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M31)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m32(+Matrix, ?M32) is semidet.
%
% Extract component m(3,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M32 Component m(3,2)
    comp_m32(Matrix, M32) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M32)-(_), O).
    comp_m32(Matrix, M32) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M32)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m33(+Matrix, ?M33) is semidet.
%
% Extract component m(3,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D-(M00)-(M01)-(M02)-(M03)-(M10)-(M11)-(M12)-(M13)-(M20)-(M21)-(M22)-(M23)-(M30)-(M31)-(M32)-(M33) for 4x4 and covMat3D--(M00)-(M01... for 6x6)
% @param M33 Component m(3,3)
    comp_m33(Matrix, M33) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(rotMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M33), O).
    comp_m33(Matrix, M33) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M33)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m34(+Matrix, ?M34) is semidet.
%
% Extract component m(3,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M34 Component m(3,4)
    comp_m34(Matrix, M34) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M34)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m35(+Matrix, ?M35) is semidet.
%
% Extract component m(3,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M35 Component m(3,5)
    comp_m35(Matrix, M35) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M35)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).


%% comp_m40(+Matrix, ?M40) is semidet.
%
% Extract component m(4,0) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M40 Component m(4,0)
    comp_m40(Matrix, M40) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M40)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m41(+Matrix, ?M41) is semidet.
%
% Extract component m(4,1) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M41 Component m(4,1)
    comp_m41(Matrix, M41) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M41)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m42(+Matrix, ?M42) is semidet.
%
% Extract component m(4,2) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M42 Component m(4,2)
    comp_m42(Matrix, M42) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M42)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m43(+Matrix, ?M43) is semidet.
%
% Extract component m(4,3) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M43 Component m(4,3)
    comp_m43(Matrix, M43) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M43)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m44(+Matrix, ?M44) is semidet.
%
% Extract component m(4,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M44 Component m(4,4)
    comp_m44(Matrix, M44) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M44)-(_)-(_)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m45(+Matrix, ?M45) is semidet.
%
% Extract component m(4,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M45 Component m(4,5)
    comp_m45(Matrix, M45) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M45)-(_)-(_)-(_)-(_)-(_)-(_), O).


%% comp_m50(+Matrix, ?M50) is semidet.
%
% Extract component m(5,0) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M50 Component m(5,0)
    comp_m50(Matrix, M50) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M50)-(_)-(_)-(_)-(_)-(_), O).

%% comp_m51(+Matrix, ?M51) is semidet.
%
% Extract component m(5,1) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M51 Component m(5,1)
    comp_m51(Matrix, M51) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M51)-(_)-(_)-(_)-(_), O).

%% comp_m52(+Matrix, ?M52) is semidet.
%
% Extract component m(5,2) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M52 Component m(5,2)
    comp_m52(Matrix, M52) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M52)-(_)-(_)-(_), O).

%% comp_m53(+Matrix, ?M53) is semidet.
%
% Extract component m(5,3) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M53 Component m(5,3)
    comp_m53(Matrix, M53) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M53)-(_)-(_), O).

%% comp_m54(+Matrix, ?M54) is semidet.
%
% Extract component m(5,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M54 Component m(5,4)
    comp_m54(Matrix, M54) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M54)-(_), O).

%% comp_m55(+Matrix, ?M55) is semidet.
%
% Extract component m(5,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D--(M00)-(M01...)
% @param M55 Component m(5,5)
    comp_m55(Matrix, M55) :-
        rdf_split_url(_, O, Matrix),
        term_to_atom(covMat3D-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(M55), O).





