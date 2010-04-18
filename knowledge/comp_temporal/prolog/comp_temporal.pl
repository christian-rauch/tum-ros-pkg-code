/** <module> comp_ehow

  Description:
    Contains all computables that calculate temporal relations between events
    to allow for temporal reasoning.

    Now extended with predicates for reasoning on other relations over time (esp.
    holds/holds_tt)

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

:-  rdf_meta
    comp_temporallySubsumes(r, r),
    comp_after(r, r),
    comp_duration(r, r).



:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).

:- owl_parser:owl_parse('../owl/comp_temporal.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_temporal, 'http://ias.cs.tum.edu/kb/comp_temporal.owl#', [keep(true)]).




%% comp_temporallySubsumes(?Long, ?Short) is nondet.
%
% Check if the time segment Long contains the segment or time point Short.
%
% @param Long Identifier of the longer time segment
% @param Short Identifier of the contained time segment or time point
% 
    comp_temporallySubsumes(Long, Short) :-

      % case: both temporally extended, i.e. start and end set

      % read times for the longer event
      rdf_triple(knowrob:startTime, Long, Ls),
      rdf_triple(knowrob:endTime, Long, Le),

      % convert time stamps to numbers
      rdf_split_url(_, LsLocal, Ls),
      atom_concat('timepoint_', LsAtom, LsLocal),
      term_to_atom(Lstart, LsAtom),

      rdf_split_url(_, LeLocal, Le),
      atom_concat('timepoint_', LeAtom, LeLocal),
      term_to_atom(Lend, LeAtom),

      % read times for the shorter event
      rdf_triple(knowrob:startTime, Short, Ss),
      rdf_triple(knowrob:endTime, Short, Se),

      % convert time stamps to numbers
      rdf_split_url(_, SsLocal, Ss),
      atom_concat('timepoint_', SsAtom, SsLocal),
      term_to_atom(Sstart, SsAtom),

      rdf_split_url(_, SeLocal, Se),
      atom_concat('timepoint_', SeAtom, SeLocal),
      term_to_atom(Send, SeAtom),

      % compare the start and end times
      (Sstart=<Send),
      (Lstart=<Sstart), (Sstart=<Lend),
      (Lstart=<Send),   (Send=<Lend).


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

      rdf_split_url(_, PreLocal, Pre),
      atom_concat('timepoint_', PreAtom, PreLocal),
      term_to_atom(P, PreAtom),

      rdf_split_url(_, AfterLocal, After),
      atom_concat('timepoint_', AfterAtom, AfterLocal),
      term_to_atom(A, AfterAtom),

      P<A.

%%  comp_time_point_start_time(Pre, After) is nondet.
%
% Start time of a time point is the time point itself
%
% @param T Identifier of a time point
%
    comp_time_point_start_time(T, T) :-
      rdf_has(T, rdf:type, knowrob:'TimePoint').

%%  comp_time_point_end_time(Pre, After) is nondet.
%
% End time of a time point is the time point itself
%
% @param T Identifier of a time point
%
    comp_time_point_end_time(T, T) :-
      rdf_has(T, rdf:type, knowrob:'TimePoint').

%%  comp_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
    comp_duration(Event, Duration) :-
      rdf_has(Event, knowrob:startTime, Es),
      rdf_has(Event, knowrob:endTime, Ee),

      rdf_split_url(_, StartLocal, Es),
      atom_concat('timepoint_', StartAtom, StartLocal),
      term_to_atom(Start, StartAtom),

      rdf_split_url(_, EndLocal, Ee),
      atom_concat('timepoint_', EndAtom, EndLocal),
      term_to_atom(End, EndAtom),

      Duration is (End-Start).


