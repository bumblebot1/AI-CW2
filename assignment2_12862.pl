candidate_number(12862).

solve_task(Task,Cost) :-
  ( part(1) -> solve_task_1_3(Task, Cost)
  ; part(3) -> solve_task_1_3(Task, Cost)
  ; part(4) -> solve_task_4(Task, Cost)
  ).

%%%%%%%%%%%%%%Astar search and its data structures%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%data structure of visited nodes used in astar for efficiency reasons%%%%%%%%
:-dynamic visited/1.

initialise_visited(P):-
  retractall(visited(_)),
  assert(visited(P)).

%%%%%%%%%% Part 1 & 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_task_1_3(Task, RealCost) :-
  agent_current_position(oscar,P),
  agent_current_energy(oscar, Energy),
  initialise_visited(P),
  solve_task_astar(Task,[[state(0,0,P),P]],R,Cost,_NewPos),!,  % prune choice point for efficiency
  Cost = [cost(C)|_],
  writeln(C),
  (
    Energy - C < 20, Task \= c(_) -> topup_energy(P, [cost(Dist)|_]), solve_task_1_3(Task, [cost(TaskDist)|_]), RealCost is Dist + TaskDist;
    otherwise -> reverse(R,[_Init|Path]), agent_do_moves(oscar,Path), RealCost = Cost
  ).

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
  initialise_visited(P),
  solve_task_astar(Task,[[state(0,0,P),P]],R,Cost,_NewPos),!,  % prune choice point for efficiency
  reverse(R,[_Init|Path]),
  (
    query_world(agent_do_moves, [Agent,Path])-> true;
    otherwise->writeln('replanning'),( query_world(agent_current_energy, [Agent, C]), C = 0 -> writeln('can\'t replan, no energy'),!,fail; otherwise -> solve_task_4(Task,_))
  ).
%%%%%%%%%% Part 4 (Optional) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

topup_energy(P, Cost) :-
  writeln('topping up'),
  initialise_visited(P),
  solve_task_astar(find(c(X)),[[state(0,0,P),P]],R,Cost,_NewPos), !,
  reverse(R,[_Init|Path]),
  agent_do_moves(oscar,Path),
  agent_topup_energy(oscar, c(X)).

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
  append(Children,RestAgenda, NewAgenda),
  sort(NewAgenda, SortedAgenda),
  solve_task_astar(Task, SortedAgenda, RR, Cost, NewPos).

%%%%%% Functions generating the children for the astar and breadth-first search%%%%%%%%%%%%%%
%%%%%% The findall + sort idea can be replaced with setof by binding all free variables%%%%%%
astar_children(Target, P, G, Children, RPath) :-
  findall(State, (search(P,Pos,Pos,_), \+memberchk(Pos, RPath), \+visited(Pos), map_distance(Pos,Target,H), generate_state(H,G,Pos,RPath,State)), Children),
  add_adjacents(P).

bf_children(P, G, Children, RPath) :-
  findall(State, (search(P,Pos,Pos,_), \+memberchk(Pos, RPath), \+visited(Pos), generate_state(0,G,Pos,RPath,State)), Children),
  add_adjacents(P).

%%%%%%%%%Generate a state structure for a child position%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%This is a list containing a state functor (F-value, G-value, NewPosition), Path-to-position%%%%
generate_state(H,G,Pos,RPath,State):-
  NewG is G + 1,
  F is NewG + H,
  State = [state(F,NewG,Pos),Pos|RPath],
  assert(visited(Pos)).

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

add_adjacents(P) :-
  findall(N, (map_adjacent(P, N, Type), type_to_pred(Type, N)), _).

type_to_pred(Type, Pos) :-
  (
    Type = o(_) -> retractall(oracle(Type, _)), assert(oracle(Type, Pos));
    Type = c(_) -> retractall(charger(Type, _)), assert(charger(Type, Pos));
    otherwise -> true
  ).
