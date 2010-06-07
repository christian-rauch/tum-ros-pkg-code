/** <module> mod_vis

  Description:
    Module providing visualisation capabilities


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

:- module(mod_vis,
    [
      vis_all_objects_on_table/1,
      vis_all_objects_inferred/3,
      visualisation_canvas/1,
      clear_canvas/1,
      draw_background/2,
      set_view_parameters/6,
      display_information_for/2,
      display_action/2,
      display_action_fixed/2,
      show_actionseq/4,
      add_object/2,
      add_object_perception/2,
      add_object_with_children/2,
      remove_object/2,
      remove_object_with_children/2,
      highlight_object/2,
      highlight_object/3,
      highlight_object/4,
      highlight_object/6,
      highlight_object/7,
      add_and_highlight_object/2,
      highlight_object_with_children/2,
      highlight_object_with_children/3,
      reset_highlighting/1 
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Visualization canvas management
% 

%% visualisation_canvas(-Canvas) is det.
%
% Launch the visualization canvas
%
:- assert(v_canvas(fail)).
visualisation_canvas(Canvas) :-
    v_canvas(fail),
    jpl_new('javax.swing.JFrame', [], Frame),
    jpl_call(Frame, 'resize', [800, 600], _),
    jpl_new('de.tum.in.fipm.kipm.gui.visualisation.base.PrologVisualizationCanvas', [], Canvas),
    jpl_call(Canvas, 'init', [], _),
    jpl_call(Frame, 'add', [Canvas], _),
    jpl_call(Frame, 'setVisible', [@(true)], _),
    retract(v_canvas(fail)),
    assert(v_canvas(Canvas)).
visualisation_canvas(Canvas) :-
    v_canvas(Canvas).


%% clear_canvas(+Canvas) is det.
% 
% Completely clears the scene
% 
% @param Canvas Visualization canvas
% 
clear_canvas(Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'clear', [], _).


%% draw_background(+Identifier, +Canvas) is det.
%
% Reset the scene and draw the background (i.e. usually the kitchen environment)
%
% same as
%   clear();
%   addObject("http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls");
% 
% @param Identifier Map instance, e.g. 'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls'
% @param Canvas     Visualization canvas
% 
draw_background(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'drawBackground', [Identifier], _).


%% set_view_parameters(+XShift, +YShift, +XRot, +YRot, +Zoom, +Canvas) is det.
%
% Change the display parameters of the visualization applet.
% 
% @param XShift   Shift in x-direction
% @param YShift   Shift in y-direction
% @param XRot     Rotation in x-direction
% @param YRot     Rotation in x-direction
% @param Zoom     Zoom factor
% @param Canvas   Visualization canvas
% 
set_view_parameters(XShift, YShift, XRot, YRot, Zoom, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'setViewParameters', [XShift, YShift, XRot, YRot, Zoom], _).


%% display_information_for(+Identifier, +Canvas) is det.
%
% Show any information that can be retrieved with rdf_has(Identifier, P, O) in the
% external control window of the visualization canvas.
% 
% @param Identifier Identifier of any kind of instance, e.g. an object or an action
% @param Canvas     Visualization canvas
% 
display_information_for(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayInfoFor', [Identifier], _).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Action display predicates
% 

%% display_action(+Identifier, +Canvas) is det.
% 
% Displays an action in the Kitchen environment via its dynamic identifier (created e.g. by CRF segmentation)
% for example "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching50'"
% 
% @param Identifier Action instance, e.g. 'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching50'
% @param Canvas     Visualization canvas
% 
display_action(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayAction', [Identifier], _).


%% display_action_fixed(+Identifier, +Canvas) is det.
% 
% Displays an action in the Kitchen environment via its fixed identifier (including EPISODE_NR and INSTANCE_NR, usually read from the DB)
% for example "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'"
% 
% @param Identifier Action instance, e.g. 'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'
% @param Canvas     Visualization canvas
% 
display_action_fixed(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayFixed', [Identifier], _).
  

%% show_actionseq(+SeqInfos, +Canvas, +Hand, +Level) is det.
%
% Visualize action sequences in the right part of the visualization canvas
%
% @param SeqInfos   Action information of the form [ [ [label],[startTime],[endTime],[[actionType1],...],[objtype]], [...] ], converted to Java array
% @param Hand       Hand, one of 'left' or 'right', corresponding to two stacks of action sequences
% @param Level      Abstraction level (displayed as vertical axis)
% @param Canvas     Visualization canvas
% 
show_actionseq(SeqInfos, Canvas, Hand, Level) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'setActionInformation', [SeqInfos, Hand, Level], _).


%
% Adding/removing objects
% 

%% add_object(+Identifier, +Canvas) is nondet.
%  
% Add object to the scene
% 
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
% 
add_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addObject', [Identifier], _).


%% add_object_with_children(+Identifier, +Canvas)
% 
% Adds objects to the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
% 
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
% 
add_object_with_children(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addObjectWithChildren', [Identifier], _).


%% remove_object(+Identifier, +Canvas) is det.
%  
% Remove object from the scene
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
% 
remove_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'removeObject', [Identifier], _).


%% remove_object_with_children(+Identifier, +Canvas) is det.
% 
% Removes objects from the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
% 
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
% 
remove_object_with_children(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'removeObjectWithChildren', [Identifier], _).


%% add_object_perception(+Identifier, +Canvas) is nondet.
%
% Add an object to the scene and color it based on the kind of process that generated it:
% * Perceived objects are light grey
% * Inferred objects are colored based on their probability
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
add_object_perception(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),

    % add the object based on its latest detection
    add_object(Identifier, Canvas),

    % find all perceptions of the object and sort by their start time
    findall([P_i,Identifier,St], (rdf_has(P_i, knowrob:objectActedOn, Identifier),
                                  rdfs_individual_of(P_i,  knowrob:'MentalEvent'),
                                  rdf_triple(knowrob:startTime, P_i, StTg),
                                  rdf_split_url(_, StTl, StTg),
                                  atom_concat('timepoint_', StTa, StTl),
                                  term_to_atom(St, StTa)), Perceptions),

    predsort(compare_object_perceptions, Perceptions, Psorted),

    % compute the homography for the newest perception
    nth0(0, Psorted, Newest),
    nth0(0, Newest, NewestPerception),

    % highlight based on the source of information
    ((
        % display perceived objects in light grey
        rdfs_instance_of(NewestPerception, knowrob:'Perceiving'),
        highlight_object(Identifier, @(true), 110, 110, 110, Canvas)
    ) ; (
        % display inferred objects by their probability
        rdfs_instance_of(NewestPerception, knowrob:'ProbCogReasoning'),
        rdf_has(NewestPerception, knowrob:probability, Prob),
        highlight_object(Identifier, @(true), 230, 230, 230, Prob, Canvas)
    ) ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Highlighting objects
% 

%% highlight_object(+Identifier, +Canvas) is det.
%% highlight_object(Identifier, Highlight, Canvas) is det.
%% highlight_object(Identifier, Highlight, Color, Canvas) is det.
%% highlight_object(Identifier, Highlight, R, B, G, Canvas) is det.
%% highlight_object(Identifier, Highlight, R, B, G, Prob, Canvas) is det.
% 
% Different methods for highlighting objects. By default, objects are drawn in bright red
% if they are highlighted, but different colors can be specified using either one integer
% value (e.g. #00FF00) or separate values for red, green, and blue.
%
% The parameter Highlight specifies if the highlighting shall be activated or reset; if
% it is missing, a value of @(true) is assumed.
% 
% If the object detection was uncertain, its probability can be visualized using the Prob
% parameter. This is done e.g. using the alpha channel or the hue value in HSV space
% (ignoring, in this case, the parameters R, B, G).
% 
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Highlight  @(true) = highlight; @(false)=remove highlighting
% @param Color      Color value as integer, e.g. #AARRBBGG
% @param R          Red color value
% @param B          Blue color value
% @param G          Green color value
% @param Prob       Object existence probability
% @param Canvas     Visualization canvas
% 
highlight_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, @(true)], _).

highlight_object(Identifier, Highlight, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight], _).

highlight_object(Identifier, Highlight, Color, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, Color], _).

highlight_object(Identifier, Highlight, R, B, G, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, R, B, G], _).

highlight_object(Identifier, Highlight, R, B, G, Prob, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, R, B, G, Prob], _).



%% add_and_highlight_object(+Identifier, +Canvas) is det.
%
% Shortcut to add an object and highlight it at the same time.
% 
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
% 
add_and_highlight_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addObject', [Identifier], _),
    jpl_call(Canvas, 'highlight', [Identifier, @(true)], _).



%% highlight_object_with_children(+Identifier, +Canvas) is det.
%% highlight_object_with_children(+Identifier, +Highlight, +Canvas) is det.
%
% Highlights an object and everything that is reachable from it via knowrob:properPhysicalPartTypes
%
% The parameter Highlight specifies if the highlighting shall be activated or reset; if
% it is missing, a value of @(true) is assumed.
% 
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
highlight_object_with_children(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, @(true)], _).

highlight_object_with_children(Identifier, Highlight, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, Highlight], _).



%% reset_highlighting(+Canvas) is det.
%
% Reset all highlighted objects in the canvas.
% 
% @param Canvas     Visualization canvas
% 
reset_highlighting(Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'clearHighlight', [], _).


vis_all_objects_on_table(Table):- 
  add_object_perception(Table, _),
  current_objects_on_table(Table, O),
  add_object_perception(O, _).

vis_all_objects_inferred(T,O,P):- 
  rdf_has(Inf, rdf:type, knowrob:'TableSettingModelInference'), 
  rdf_has(Inf,knowrob:objectActedOn,O),
  rdf_has(Inf,knowrob:probability,P),
  term_to_atom(N,P),
  N>T,
  add_object_perception(O,  _).

