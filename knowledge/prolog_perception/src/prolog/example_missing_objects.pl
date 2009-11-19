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
%%%neededObject(Obj,Meal) :-
%  subclass(Obj,cup), color(Obj,red).

neededObjectsForMeal(M, NeededObjects):- %returns a list of objects
	queryWikihow(M, NeededObjects).


missingObjects(Missing):-
	perceivedPlane(pl0), %gets plane from ROS(table memory) and asserts it into prolog DB
	partOfPLane(pl0, T),  %checks in ROS whether pl0 is in subset of table T
	instanceOf(T, table), %is T an instance of table class @MORITZ!!!
	primaryFunction(T, 'having-meals'), %is a function of table T to have a meal on? @MORITZ
	perceivedObjects(T, Objects), %gets those objects from ROS(table memory) that lie on table T, Objects shall be a list of tuples [ObjId, ObjClass] where ObjClass is a one of: Mug, Placemat, Plate, Fork, Knife, Mug, Napkin, 
	neededObjectsForMeal('breakfast', NeededObjects), %get missing object from knowledge base
	setOf(Obj, member(Obj, NeededObjects)^not(onTable(Obj, Objects, T)), Missing). %finally retrieve missing objects, onTable checks whether 

%-> picture of semantic map where table is colored
