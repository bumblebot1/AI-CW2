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
  Agenda = [[state(F,G,Pos)|RPath]|_],
  CostList = [cost(Cost), depth(G)],
  achieved(Task,[c(G,Pos)|RPath],R,Cost,NewPos).

solve_task_astar(Task,Agenda,RR,Cost,NewPos) :-
  Agenda = [[state(F, G, P) | RPath] | RestAgenda],
  Cur = [state(F, G, P) | RPath],
  children(P, Children, RPath),
  (
    Task = go(Target) -> astar_calculate_costs(Target, Cur, Children, [], ChildCosts);
    Task = find(Target) -> bf_calculate_costs(Target, Cur, Children, [], ChildCosts)
  ),
  append(ChildCosts, RestAgenda, NewAgenda),
  sort(NewAgenda, Sorted),
  solve_task_astar(Task, Sorted, RR, Cost, NewPos).

astar_calculate_costs(_, _, [], ChildCosts, ChildCosts).
astar_calculate_costs(Target, Cur, [Child|Children], TempCosts, ChildCosts) :-
   Cur = [state(_, OldG, _)|RPath],
   G is OldG + 1,
   map_distance(Child, Target, H),
   F is H + G,
   astar_calculate_costs(Target, Cur, Children, [[state(F,G,Child)|[Child|RPath]] | TempCosts], ChildCosts).

bf_calculate_costs(_, _, [], ChildCosts, ChildCosts).
bf_calculate_costs(Target, Cur, [Child | Children], TempCosts, ChildCosts) :-
   Cur = [state(_, OldG, _)|RPath],
   G is OldG + 1,
   bf_calculate_costs(Target, Cur, Children, [[state(G,G,Child)|[Child|RPath]] | TempCosts], ChildCosts).

children(P, Children, RPath) :-
  setof(Pos, search(P,Pos,Pos,_), UnfilteredChildren),
  filter_children(UnfilteredChildren, RPath, [], Children).

filter_children([], _, Children, Children).
filter_children([Child | Rest], RPath, TempChildren, Children) :-
  (
    \+ memberchk(Child,RPath) -> filter_children(Rest, RPath, [Child | TempChildren], Children);
    filter_children(Rest, RPath, TempChildren, Children)
  ).


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
