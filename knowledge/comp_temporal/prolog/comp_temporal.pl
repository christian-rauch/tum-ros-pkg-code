/** <module> comp_ehow

  Description:
    contains all computables that calculate temporal relations between events
    to allow for temporal reasoning

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
:- module(comp_temporal,
    [
      comp_temporallySubsumes/2,
      comp_after/2,
      comp_duration/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).

:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).

%% comp_temporallySubsumes(?Long, ?Short) is nondet.
%
% Check if the time segment Long contains the segment or time point Short.
%
% @param Long Identifier of the longer time segment
% @param Short Identifier of the contained time segment or time point
% 
    comp_temporallySubsumes(Long, Short) :-

      % case: both temporally extended, i.e. start and end set

      rdf_triple(knowrob:startTime, Long, Ls), rdf_split_url(_, LLs, Ls), term_to_atom(Lstart, LLs),
      rdf_triple(knowrob:endTime, Long, Le),   rdf_split_url(_, LLe, Le), term_to_atom(Lend,   LLe),

      rdf_triple(knowrob:startTime, Short, Ss),rdf_split_url(_, LSs, Ss), term_to_atom(Sstart, LSs),
      rdf_triple(knowrob:endTime, Short, Se),  rdf_split_url(_, LSe, Se), term_to_atom(Send,   LSe),

      (Sstart=<Send),

      (Lstart=<Sstart), (Sstart=<Lend),
      (Lstart=<Send),   (Send=<Lend).

    comp_temporallySubsumes(Long, Short) :-

      % case: short is a time point, i.e. end is not set
      rdf_triple(knowrob:startTime, Long, Ls), rdf_split_url(_, LLs, Ls), term_to_atom(Lstart, LLs),
      rdf_triple(knowrob:endTime, Long, Le),   rdf_split_url(_, LLe, Le), term_to_atom(Lend,   LLe),

      rdf_triple(knowrob:startTime, Short, Ss),rdf_split_url(_, LSs, Ss), term_to_atom(Sstart, LSs),
      not(rdf_triple(knowrob:endTime, Short, _)),

      (Lstart=<Sstart), (Sstart=<Lend).


%%  comp_after(Pre, After) is nondet.
%
% Check if the time point Pre is before the time point After
%
% @param Pre Identifier of the earlier time point
% @param After Identifier of the later time point
% 
    comp_after(Pre, After) :-
      rdf_has(Pre,   rdf:type, knowrob:'TimePoint'),
      rdf_has(After, rdf:type, knowrob:'TimePoint'),
      term_to_atom(P, Pre),
      term_to_atom(A, After),
      P<A.


%%  comp_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
% 
    comp_duration(Event, Duration) :-
      rdf_has(Event, knowrob:startTime, Es), rdf_split_url(_, LEs, Es), term_to_atom(Start, LEs),
      rdf_has(Event, knowrob:endTime, Ee),   rdf_split_url(_, LEe, Ee), term_to_atom(End,   LEe),
      Duration is (End-Start).
