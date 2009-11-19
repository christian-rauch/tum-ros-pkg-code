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
%% Set of predicates to enable executing first order logic queries on perception
%% data stored as memmory structures. WIP!!!
%% Load as following:
%% cd `rospack find prolog_perception`/src/prolog && prolog
%% consult('passive_perception.pl').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% clear screen in prolog prompt
cls :-  put(27), put("["), put("2"), put("J").

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
assertNewPlane(PL):-
  getPlaneROS(PL),
  [X|Y] = PL,
  assert(perceivedPlane(X, Y)).
retractPlanes:-
  retractall(perceivedPlane(X, Y)).

%stop recursion
%example: splitList([1,2,3,4,5,6],L, 3).
splitList([],[], N).
splitList(F, [L1|Rest], N):-
  split(F, N, L1, L2),
  splitList(L2, Rest, N).

%stop recursion
split(L,0,[],L).
%conditional statement
split([X|Xs],N,[X|Ys],Zs):- N > 0, N1 is N - 1, split(Xs,N1,Ys,Zs).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
assertNewPlaneClusters(PL):-
  getPlaneClustersROS(PL), 
  [X|Y]=PL,
  splitList(Y, R),
  assert(planeClusters(X, R)).

retractPlaneClusters:-
  retractall(planeClusters(X, Y)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
assertDemo(PL):-
    getTableMemoryObjects(PL), 
    splitList(PL, R, 10),
    [X|Y] = R,
    assertTableMemoryObjects(X, Y).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
assertTableMemoryObjects(F, R):-
    [X | Y] = F,
    assert(tableMemoryObject(X, Y)),
    [A | B] = R,
    assertTableMemoryObjects(A, B).
    
retractTableMemoryObjects:-
  retractall(tableMemoryObject(X, Y)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
planeEq(Id, Eq):-
  perceivedPlane([Id | Eq]).

%clustersOnPlane(pl1,[1,2,3]).

%is X cluster on the plane Plane
onPlane(X,Plane) :- planeClusters(Plane,Y), member(X,Y).
%which objects are on table "Table"
objectsOnTable(Table,Objects):- tableMemoryObject(Table, Objects). 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%whichObjectOnTable(Obj, Table):-  tableMemoryObject(Table, Y), member(Obj, Y).  
% getObjAtTime(+Time, -Obj)
getObjAtTime(Time, Obj) :-
 tableMemoryObject(_, [Time, X, Y, Z, Objstr, ID, A, B, C]),
 Obj=[Time, Objstr, ID].

% getObjAtTimeAtTable(+Table, +Time, -Obj)
getObjAtTimeAtTable(Table, Time, Obj) :-
 tableMemoryObject(Table, [Time, X, Y, Z, Objstr, ID, A, B, C]),
 Obj=[Table, Time, Objstr, ID].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%Are there time instances Ti where 2 different objects
%of type X were on the breakfast table?
%twoObjectsOnTable(+Table, -Ti):-
twoObjectsOnTable(TableId, Ti):-
	 tableMemoryObject(TableId, [TimeStamp, Tx1, Ty1, Tz1, Obj1, ObjID1, Ox1, Oy1, Oz1]),
	 tableMemoryObject(TableId, [TimeStamp, Tx2, Ty2, Tz2, Obj2, ObjID2, Ox2, Oy2, Oz2]),
	 ObjID1 \= ObjID2,
	 Ox1 < Ox2,
	 Ti = [TimeStamp, Obj1, Obj2].

oneObjectOnTable(TableId, Ti):-
	 tableMemoryObject(TableId, [TimeStamp1, Tx1, Ty1, Tz1, Obj1, ObjID1, Ox1, Oy1, Oz1]),
	 tableMemoryObject(TableId, [TimeStamp2, Tx2, Ty2, Tz2, Obj2, ObjID2, Ox2, Oy2, Oz2]),
	 ObjID1 \= ObjID2,
	 Ti = [TimeStamp, Obj1, Obj2].
