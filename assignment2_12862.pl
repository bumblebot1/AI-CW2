candidate_number(12862).

solve_task(Task,Cost) :-
  ( part(1) -> solve_task_1_3(Task, Cost)
  ; part(3) -> solve_task_1_3(Task, Cost)
  ; part(4) -> solve_task_4(Task, Cost)
  ).

%%%%%%%%%%%%%%Astar search and its data structures%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% data structure of visited nodes
:-dynamic visited/1.

initialise_visited(P):-
  retractall(visited(_)),
  assert(visited(P)).

:- dynamic 
  used_internal_objects/1, found/1, bound/1, strategy/1, oracle/2, charger/2.

init_state :-
  reset_dynamics,
  assert(strategy(normal)),
  assert(bound(30)).

reset_dynamics :-
  retractall(used_internal_objects(_)),
  retractall(found(_)),
  retractall(bound(_)),
  retractall(strategy(_)),
  retractall(oracle(_,_)),
  retractall(charger(_,_)),
  reset_possible_actors.

reset_bound :-
  retractall(bound(_)),
  assert(bound(30)).

start_solving(A) :-
  reset_dynamics, init_state, find_solution, !, possible_actor(A), say(A).

find_solution :-
  \+agent_current_energy(oscar, 0),
  strategy(S),
  findall(A, possible_actor(A), As),
  length(As, Len),
  (
    Len = 1    -> true;
    S = normal -> normal_strategy, !;
    S = critic -> critical_strategy, !
  ).

close_objects(CurrPos, T, List) :-
  (
    T = 'o'   -> findall(Oracle, (findall(obj(AdjPos, Type), map_adjacent(CurrPos, AdjPos, Type), Adjacents), member(Oracle, Adjacents), Oracle=obj(_,o(X)), \+agent_check_oracle(oscar, o(X))), List);
    T = 'c'   -> findall(Charger, (findall(obj(AdjPos, Type), map_adjacent(CurrPos, AdjPos, Type), Adjacents), member(Charger, Adjacents), Charger=obj(_,c(_))), List);
    otherwise -> List is []
  ).
   
critical_strategy :-
  agent_current_position(oscar, Pos),
  close_objects(Pos, 'c', Chargers),
  length(Chargers, Len),
  (
    Len > 0   -> Chargers = [obj(_, Charger)|_], agent_topup_energy(oscar, Charger), reset_bound;
    otherwise -> solve_task_1_3(find(c(_)),_)
  ),
  check_energy.

best_oracle(Curr, NextOracle) :-
  findall(O, oracle(O,_), KnownOracles),
  findall(obj(D,Oracle), (oracle(Oracle,P), map_distance(Curr, P, D), \+agent_check_oracle(oscar,Oracle)),Oracles),
  sort(Oracles, Sorted),
  length(KnownOracles, Len),
  (
    Len = 0   -> first_oracle(Curr,Fastest), Fastest = [path(_,NextOracle,_)|_];
    otherwise -> Sorted = [obj(_, NextOracle) | _]
  ).

normal_strategy :-
  agent_current_position(oscar, Pos),
  bound(B), 
  agent_current_energy(oscar, E),
  EnergyAfterQuery is E - 10,
  StrongerBound is B + 10,
  close_objects(Pos, 'o', Oracles),
  length(Oracles, Len),
  writeln(Pos),
  (
    EnergyAfterQuery < B            -> retract(bound(_)), assert(bound(StrongerBound)), check_energy;
    Len > 0, EnergyAfterQuery >= B  -> Oracles = [obj(_, Oracle)|_], query_oracle(Oracle), assert(used_internal_objects(Oracle)),check_energy;
    otherwise                       -> best_oracle(Pos, Oracle), solve_task_1_3(find(Oracle), _),!,check_energy
  ).

check_energy :-
  writeln('here'),
  agent_current_energy(oscar, E), 
  strategy(S), 
  bound(B), 
  (
    E < B -> (
      S = normal -> assert(strategy(critic)), retract(strategy(normal)), find_solution;
      otherwise  -> find_solution
    );
    otherwise -> (
      S = normal -> find_solution;
      otherwise  -> assert(strategy(normal)), retract(strategy(critic)), find_solution 
    )
  ).

%%%%%%%%%% Part 1 & 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_task_1_3(Task,Cost) :-
  agent_current_position(oscar,P),
  initialise_visited(P),
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
  append(Children,RestAgenda, NewAgenda),
  sort(NewAgenda, SortedAgenda),
  solve_task_astar(Task, SortedAgenda, RR, Cost, NewPos).

astar_children(Target, P, G, Children, RPath) :-
  findall(State, (search(P,Pos,Pos,_), \+memberchk(Pos, RPath), \+visited(Pos), map_distance(Pos,Target,H), generate_state(H,G,Pos,RPath,State)), Children),
  add_adjacents(P).

generate_state(H,G,Pos,RPath,State):-
  NewG is G + 1,
  F is NewG + H,
  State = [state(F,NewG,Pos),Pos|RPath],
  assert(visited(Pos)).

bf_children(P, G, Children, RPath) :-
  findall(State, (search(P,Pos,Pos,_), \+memberchk(Pos, RPath), \+visited(Pos), generate_state(0,G,Pos,RPath,State)), Children),
  add_adjacents(P).

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
    Type = o(_), \+oracle(Type, _)  -> assert(oracle(Type, Pos));
    Type = c(_), \+charger(Type, _) -> assert(charger(Type, Pos));
    otherwise -> true
  ).

known_oracles(Curr, Sorted) :-
  findall(obj(Dist,OID,Neighbours),(oracle(OID,Pos),findall(El,map_adjacent(Pos,El,empty),Neighbours), map_distance(Pos,Curr,Dist)),List),sort(List,Sorted).

duplicates_find(Res, path(_,X,_), path(_,Y,_)):-
  compare(Res,X,Y).

first_oracle(P,Fastest) :-
  initialise_visited(P),
  findall(path(Cost,o(X),Path),solve_task_astar(find(o(X)),[[state(0,0,P), P]],Path,Cost,_),List),
  sort(List, CostSorted), %sort based on lowest cost
  predsort(duplicates_find,CostSorted,Sorted), %keep only one path obj for each entry (the one kept always has lowest cost)
  sort(Sorted,Fastest). %sort the oracles by lowest cost from start point
