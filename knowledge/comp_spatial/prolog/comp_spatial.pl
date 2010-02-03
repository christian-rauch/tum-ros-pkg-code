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
      on_Physical/2,
      in_ContGeneric/2,
      adjacent_Objects/2,
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

%
% test-comment for svn-ros sync
% 



%% on_Physical(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object

   on_Physical(Top, Bottom) :-
          rdf_triple(knowrob:center, Top, TopCPoint),
          rdf_triple(knowrob:center, Bottom, BottomCPoint),

          % read the center coordinates of the top entity
          rdf_triple(knowrob:xCoord, TopCPoint, literal(type(_,TCx))),atom_to_term(TCx,TX,_),
          rdf_triple(knowrob:yCoord, TopCPoint, literal(type(_,TCy))),atom_to_term(TCy,TY,_),
          rdf_triple(knowrob:zCoord, TopCPoint, literal(type(_,TCz))),atom_to_term(TCz,TZ,_),

          % read the center coordinates of the bottom entity
          rdf_triple(knowrob:xCoord, BottomCPoint, literal(type(_,BCx))),atom_to_term(BCx,BX,_),
          rdf_triple(knowrob:yCoord, BottomCPoint, literal(type(_,BCy))),atom_to_term(BCy,BY,_),
          rdf_triple(knowrob:zCoord, BottomCPoint, literal(type(_,BCz))),atom_to_term(BCz,BZ,_),

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


  


%
% old version: cleaner, but does not work if not both objects have their dimensions specified
%
% 
%     on_Physical(Top, Bottom) :-
%           rdf_triple(knowrob:center, Top, TopCPoint),
%           rdf_triple(knowrob:center, Bottom, BottomCPoint),
% 
%           % read the center coordinates of the top entity
%           rdf_triple(knowrob:xCoord, TopCPoint, literal(type(_,TCx))),atom_to_term(TCx,TX,_),
%           rdf_triple(knowrob:yCoord, TopCPoint, literal(type(_,TCy))),atom_to_term(TCy,TY,_),
%           rdf_triple(knowrob:zCoord, TopCPoint, literal(type(_,TCz))),atom_to_term(TCz,TZ,_),
% 
%           % read the center coordinates of the bottom entity
%           rdf_triple(knowrob:xCoord, BottomCPoint, literal(type(_,BCx))),atom_to_term(BCx,BX,_),
%           rdf_triple(knowrob:yCoord, BottomCPoint, literal(type(_,BCy))),atom_to_term(BCy,BY,_),
%           rdf_triple(knowrob:zCoord, BottomCPoint, literal(type(_,BCz))),atom_to_term(BCz,BZ,_),
% 
%           % read the dimensions of the top entity
%           rdf_has(Top, knowrob:heightOfObject,literal(type(_,Th))),atom_to_term(Th,TH,_),
% 
%           % read the dimensions of the bottom entity
%           rdf_has(Bottom, knowrob:widthOfObject, literal(type(_,Bw))),atom_to_term(Bw,BW,_),
%           rdf_has(Bottom, knowrob:heightOfObject,literal(type(_,Bh))),atom_to_term(Bh,BH,_),
%           rdf_has(Bottom, knowrob:depthOfObject, literal(type(_,Bd))),atom_to_term(Bd,BD,_),
% 
%           % the criterion is if the difference between them is less than epsilon=5cm
%           =<( abs((TZ - 0.5*TH) - (BZ + 0.5*BH)), 0.10),
% 
%           % additional criterion: center of the top entity has to be inside the
%           % area of the bottom entity
%           =<( (BX - 0.5*BD), TX ), >=( (BX + 0.5*BD), TX ),
%           =<( (BY - 0.5*BW), TY ), >=( (BY + 0.5*BW), TY ),
%           Top \= Bottom.


%% comp_toTheLeftOf(?Left, ?Right) is nondet.
%
% Check if Left is to the left of Right. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Left Identifier of the left Object
% @param Right Identifier of the right Object
  comp_toTheLeftOf(Left, Right) :-
  %
  % TODO: adapt this to take rotations and object dimensions into account
  % 
          rdf_triple(knowrob:center, Left, LeftCPoint),
          rdf_triple(knowrob:center, Right, RightCPoint),

          % read the center coordinates of the left entity
          rdf_triple(knowrob:xCoord, LeftCPoint, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
          rdf_triple(knowrob:yCoord, LeftCPoint, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
          rdf_triple(knowrob:zCoord, LeftCPoint, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

          % read the center coordinates of the right entity
          rdf_triple(knowrob:xCoord, RightCPoint, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
          rdf_triple(knowrob:yCoord, RightCPoint, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
          rdf_triple(knowrob:zCoord, RightCPoint, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),

          =<( abs( LX - RX), 0.30),  % less than 30cm y diff
          =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
          =<( abs( LZ - RZ), 0.30),  % less than 30cm height diff
          Left \= Right.


%% comp_toTheRightOf(?Right,?Left) is nondet.
%
% Check if Right is to the right of Left.
% 
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @see comp_toTheLeftOf
  comp_toTheRightOf(Right, Left) :-
          comp_toTheLeftOf(Left, Right).


%% comp_toTheSideOf(?A, ?B) is nondet.
%
% Check if A is either to the left or the rigth of B.
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
  comp_toTheSideOf(A, B) :-
          comp_toTheRightOf(A, B);
          comp_toTheLeftOf(A, B).


%% comp_inFrontOf(?Front, ?Back) is nondet.
%
% Check if Front is in front of Back. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Front Identifier of the front Object
% @param Back Identifier of the back Object
  comp_inFrontOf(Front, Back) :-
  %
  % TODO: adapt this to take rotations and object dimensions into account
  %
          rdf_triple(knowrob:center, Front, FrontCPoint),
          rdf_triple(knowrob:center, Back, BackCPoint),

          % read the center coordinates of the left entity
          rdf_triple(knowrob:xCoord, FrontCPoint, literal(type(_,FCx))),atom_to_term(FCx,FX,_),

          % read the center coordinates of the right entity
          rdf_triple(knowrob:xCoord, BackCPoint, literal(type(_,BCx))),atom_to_term(BCx,BX,_),

          =<( BX, FX ),      % front obj has a higher x coord
          Front \= Back.

%% comp_inCenterOf(?Inner, ?Outer) is nondet.
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
  comp_inCenterOf(Inner, Outer) :-
          rdf_triple(knowrob:center, Inner, InnerCPoint),
          rdf_triple(knowrob:center, Outer, OuterCPoint),

          % read the center coordinates of the left entity
          rdf_triple(knowrob:xCoord, InnerCPoint, literal(type(_,ICx))),atom_to_term(ICx,IX,_),
          rdf_triple(knowrob:yCoord, InnerCPoint, literal(type(_,ICy))),atom_to_term(ICy,IY,_),
          rdf_triple(knowrob:zCoord, InnerCPoint, literal(type(_,ICz))),atom_to_term(ICz,IZ,_),

          % read the center coordinates of the right entity
          rdf_triple(knowrob:xCoord, OuterCPoint, literal(type(_,OCx))),atom_to_term(OCx,OX,_),
          rdf_triple(knowrob:yCoord, OuterCPoint, literal(type(_,OCy))),atom_to_term(OCy,OY,_),
          rdf_triple(knowrob:zCoord, OuterCPoint, literal(type(_,OCz))),atom_to_term(OCz,OZ,_),

          =<( abs( IX - OX), 0.20),  % less than 20cm x diff
          =<( abs( IY - OY), 0.20),  % less than 20cm y diff
          =<( abs( IZ - OZ), 0.20).  % less than 20cm z diff


%% in_ContGeneric(?InnerObj, ?OuterObj) is nondet.
%
% Check if InnerObj is contained by OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
    in_ContGeneric(InnerObj, OuterObj) :-
          rdf_triple(knowrob:center, InnerObj, InnerCPoint),
          rdf_triple(knowrob:center, OuterObj, OuterCPoint),

          % read the center coordinates of the outer entity
          rdf_triple(knowrob:xCoord, OuterCPoint, literal(type(_,OCx))),atom_to_term(OCx,OX,_),
          rdf_triple(knowrob:yCoord, OuterCPoint, literal(type(_,OCy))),atom_to_term(OCy,OY,_),
          rdf_triple(knowrob:zCoord, OuterCPoint, literal(type(_,OCz))),atom_to_term(OCz,OZ,_),

          % read the center coordinates of the inner entity
          rdf_triple(knowrob:xCoord, InnerCPoint, literal(type(_,ICx))),atom_to_term(ICx,IX,_),
          rdf_triple(knowrob:yCoord, InnerCPoint, literal(type(_,ICy))),atom_to_term(ICy,IY,_),
          rdf_triple(knowrob:zCoord, InnerCPoint, literal(type(_,ICz))),atom_to_term(ICz,IZ,_),

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

%% adjacent_Objects(?ObjA, ?ObjB) is nondet.
%
% Check if two objects A, B are adjacent.
%
% @param ObjA Identifier of Object A
% @param ObjB Identifier of Object B
    adjacent_Objects(ObjA, ObjB) :-
          rdf_triple(knowrob:center, ObjA, CenterA),
          rdf_triple(knowrob:center, ObjB, CenterB),
          ObjA \= ObjB,

          % read the center coordinates & dimensions of entityA
          rdf_triple(knowrob:xCoord, CenterA, literal(type(_,ACx))),atom_to_term(ACx,AX,_),
          rdf_triple(knowrob:yCoord, CenterA, literal(type(_,ACy))),atom_to_term(ACy,AY,_),
          rdf_triple(knowrob:zCoord, CenterA, literal(type(_,ACz))),atom_to_term(ACz,AZ,_),
          rdf_has(ObjA, knowrob:widthOfObject, literal(type(_,ATw))),atom_to_term(ATw,AW,_),
          rdf_has(ObjA, knowrob:heightOfObject,literal(type(_,ATh))),atom_to_term(ATh,AH,_),
          rdf_has(ObjA, knowrob:depthOfObject, literal(type(_,ATd))),atom_to_term(ATd,AD,_),


          % read the center coordinates & dimensions of entityB
          rdf_triple(knowrob:xCoord, CenterB, literal(type(_,BCx))),atom_to_term(BCx,BX,_),
          rdf_triple(knowrob:yCoord, CenterB, literal(type(_,BCy))),atom_to_term(BCy,BY,_),
          rdf_triple(knowrob:zCoord, CenterB, literal(type(_,BCz))),atom_to_term(BCz,BZ,_),
          rdf_has(ObjB, knowrob:widthOfObject, literal(type(_,BTw))),atom_to_term(BTw,BW,_),
          rdf_has(ObjB, knowrob:heightOfObject,literal(type(_,BTh))),atom_to_term(BTh,BH,_),
          rdf_has(ObjB, knowrob:depthOfObject, literal(type(_,BTd))),atom_to_term(BTd,BD,_),

          (
            adjacentHelper(AX,AY,AZ,AW,AH,AD, BX,BY,BZ,BW,BH,BD);
            adjacentHelper(AY,AX,AZ,AH,AW,AD, BY,BX,BZ,BH,BW,BD);
            adjacentHelper(AZ,AY,AX,AW,AD,AH, BZ,BY,BX,BW,BD,BH)

          ).

    % takes coordinates & dimensions of two objects, and succeeds if
    % - A is left of B (AX+0.5*AW - BX-0.5*AW <= 0.15)
    % - Center of one Y-Z-Area is inside Other Y-Z-Area
    adjacentHelper(AX,AY,AZ,AW,AH,AD, BX,BY,BZ,BW,BH,BD) :-
          =<( abs((AX + 0.5*AW) - (BX - 0.5*BW)), 0.05),
          (
            (=<( (BZ - 0.5*BD), AZ ), >=( (BZ + 0.5*BD), AZ ),
            =<( (BY - 0.5*BH), AY ), >=( (BY + 0.5*BH), AY ));
            (=<( (AZ - 0.5*AD), BZ ), >=( (AZ + 0.5*AD), BZ ),
            =<( (AY - 0.5*AH), BY ), >=( (AY + 0.5*AH), BY ))
          ),!.


% % % % % % % % % % % % % % % % % % % %
% matrix and vector computations (relating the new homography-based
% position representation with the old center-point-based one)
%

%% comp_center(+Obj, ?Center) is semidet.
%
% Compute the center point of an object using its homography matrix
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


