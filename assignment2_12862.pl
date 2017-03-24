candidate_number(12862).

solve_task(Task,Cost) :-
  ( part(1) -> solve_task_1_3(Task, Cost)
  ; part(3) -> solve_task_1_3(Task, Cost)
  ; part(4) -> solve_task_4(Task, Cost)
  ).

%%%%%%%%%% Part 1 & 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_task_1_3(Task,Cost) :-
  agent_current_position(oscar,P),
  solve_task_astar(Task,[[state(0,0,P),P]],R,Cost,_NewPos),!,  % prune choice point for efficiency
  reverse(R,[_Init|Path]),
  agent_do_moves(oscar,Path).

solve_task_backtrack(Task,Cost) :-
  agent_current_position(oscar,P),
  solve_task_bt(Task,[c(0,P),P],0,R,Cost,_NewPos),!,  % prune choice point for efficiency
  reverse(R,[_Init|Path]),
  agent_do_moves(oscar,Path).
%%%%%%%%%% Part 1 & 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Part 4 (Optional) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_task_4(Task,Cost):-
  my_agent(Agent),
  query_world( agent_current_position, [Agent,P] ),
  solve_task_bt(Task,[c(0,P),P],0,R,Cost,_NewPos),!,  % prune choice point for efficiency
  reverse(R,[_Init|Path]),
  query_world( agent_do_moves, [Agent,Path] ).
%%%%%%%%%% Part 4 (Optional) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%% Useful predicates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% backtracking depth-first search, needs to be changed to agenda-based A*
solve_task_bt(Task,Current,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Current,RPath,Cost,NewPos).
solve_task_bt(Task,Current,D,RR,Cost,NewPos) :-
  Current = [c(F,P)|RPath],
  search(P,P1,R,C),
  \+ memberchk(R,RPath),  % check we have not been here already
  D1 is D+1,
  F1 is F+C,
  solve_task_bt(Task,[c(F1,P1),R|RPath],D1,RR,Cost,NewPos).  % backtrack search

solve_task_astar(Task,Agenda,R,CostList,NewPos) :-
  Agenda = [[state(_,G,Pos)|RPath]|_],
  CostList = [cost(Cost), depth(G)],
  achieved(Task,[c(G,Pos)|RPath],R,Cost,NewPos).

solve_task_astar(Task,Agenda,RR,Cost,NewPos) :-
  Agenda = [ [state(_, G, P) | RPath] | RestAgenda],
  (
    Task = go(Target) -> astar_children(Target, P, G, Children, RPath);
    Task = find(_) -> bf_children(P, G, Children, RPath)
  ),
  add_to_agenda(Children, RestAgenda, NewAgenda),
  solve_task_astar(Task, NewAgenda, RR, Cost, NewPos).

astar_children(Target, P, G, Children, RPath) :-
  setof(State, (Pos,H) ^ (search(P,Pos,Pos,_), map_distance(Pos,Target,H), generate_state(H,G,Pos,RPath,State)), UnfilteredChildren),
  filter_children(UnfilteredChildren, RPath, [], Children).

generate_state(H,G,Pos,RPath,State):-
  NewG is G + 1,
  F is NewG + H,
  State = [state(F,NewG,Pos),Pos|RPath].

bf_children(P, G, Children, RPath) :-
  setof(State, (Pos) ^ (search(P,Pos,Pos,_), generate_state(0,G,Pos,RPath,State)), UnfilteredChildren),
  filter_children(UnfilteredChildren, RPath, [], Children).

filter_children([], _, Children, Children).
filter_children([Child | Rest], RPath, TempChildren, Children) :-
  Child = [state(_,_,ChildPos) | _],
  (
    \+ memberchk(ChildPos,RPath) -> filter_children(Rest, RPath, [Child | TempChildren], Children);
    filter_children(Rest, RPath, TempChildren, Children)
  ).

add_to_agenda([],Agenda,Agenda).
add_to_agenda([Child|Children],OldAgenda,NewAgenda) :-
	add_state(Child,OldAgenda,TmpAgenda),
	add_to_agenda(Children,TmpAgenda,NewAgenda).

add_state(Child,[],[Child]).
add_state(Child,[Node|Rest],[Child,Node|Rest]) :-
	Child = [state(ChildF, _, _) | _],
	Node = [state(NodeF, _, _) | _],
	ChildF < NodeF.

add_state(Child,[Node|Rest],[Node|NewRest]) :-
	Child = [state(ChildF, _, _) | _],
	Node = [state(NodeF, _, _) | _],
	NodeF =< ChildF,
	add_state(Child,Rest,NewRest).


achieved(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [c(Cost,NewPos)|RPath],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).
achieved(find(O),Current,RPath,Cost,NewPos) :-
  Current = [c(Cost,NewPos)|RPath],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,_,O)
  ).

search(F,N,N,1) :-
  map_adjacent(F,N,empty).
