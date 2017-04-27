% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
:-dynamic eliminated_actor/1.

reset_possible_actors :-
  retractall(eliminated_actor(_)).

query_oracle(Oracle) :-
  writeln('Querying'),
  (
    part(3) -> (
        writeln(Oracle),
        agent_ask_oracle(oscar, Oracle, link, RandomLink),
        findall(N, possible_actor(N), Ns),
        maplist(get_wiki(RandomLink), Ns)
      );
    part(4) -> (
        my_agent(Agent),
        query_world(agent_ask_oracle, [Agent, Oracle, link, RandomLink]),
        findall(N, possible_actor(N), Ns),
        maplist(get_wiki(RandomLink), Ns)
      )
  ).

get_wiki(RandomLink, CurrActor):-
  findall(Link, (wp(CurrActor,WT), wt_link(WT,Link)), ListOfLinks),
  (
    \+ memberchk(RandomLink, ListOfLinks) -> assert(eliminated_actor(CurrActor));
    otherwise -> true
  ).

possible_actor(A) :-
  actor(A),
  \+ eliminated_actor(A).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%Predicates to solve the maze and find identity%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

find_identity(A) :-
  (
    part(3) -> reset_dynamics, init_state, find_solution, !, possible_actor(A), say(A);
    part(4) -> reset_dynamics, init_state, find_solution, !, possible_actor(A), my_agent(Agent), oscar_library:say(Agent,A) 
  ).

find_solution :-
  (
    part(3) -> (
        agent_current_energy(oscar, C), 
        C \= 0,
        strategy(S),
        findall(A, possible_actor(A), As),
        length(As, Len),
        (
          Len = 1    -> true;
          otherwise -> ( S = normal -> normal_strategy; S = critic -> critical_strategy)
        ), !
      );
    part(4) -> (
        my_agent(Agent),
        query_world(agent_current_energy, [Agent, C]), 
        C \= 0,
        strategy(S),
        findall(A, possible_actor(A), As),
        length(As, Len),
        (
          Len = 1    -> true;
          otherwise -> ( S = normal -> normal_strategy; S = critic -> critical_strategy)
        ), !
      )
  ).

close_objects(CurrPos, T, List) :-
  (
    part(3) -> (
        T = 'o'   -> findall(Oracle, (findall(obj(AdjPos, Type), map_adjacent(CurrPos, AdjPos, Type), Adjacents), member(Oracle, Adjacents), Oracle=obj(_,o(X)), \+agent_check_oracle(oscar, o(X))), List);
        T = 'c'   -> findall(Charger, (findall(obj(AdjPos, Type), map_adjacent(CurrPos, AdjPos, Type), Adjacents), member(Charger, Adjacents), Charger=obj(_,c(_))), List);
        otherwise -> List = []
      );
    part(4) -> (
        my_agent(Agent),
        (
          T = 'o'   -> findall(Oracle, (findall(obj(AdjPos, Type), map_adjacent(CurrPos, AdjPos, Type), Adjacents), member(Oracle, Adjacents), Oracle=obj(_,o(X)), \+query_world(agent_check_oracle,[Agent, o(X)])), List);
          T = 'c'   -> findall(Charger, (findall(obj(AdjPos, Type), map_adjacent(CurrPos, AdjPos, Type), Adjacents), member(Charger, Adjacents), Charger=obj(_,c(_))), List);
          otherwise -> List = []
        )
      )
  ).   

best_oracle(Curr, NextOracle) :-
  (
    part(3) -> (
        findall(O, oracle(O,_), KnownOracles),
        findall(obj(D,Oracle), (oracle(Oracle,P), map_distance(Curr, P, D), \+agent_check_oracle(oscar,Oracle)),Oracles),
        sort(Oracles, Sorted),
        length(KnownOracles, Len),
        (
          Len = 0   -> first_oracle(Curr,Fastest), Fastest = [path(_,NextOracle,_)|_];
          otherwise -> Sorted = [obj(_, NextOracle) | _]
        )
      );
    part(4) -> (
        my_agent(Agent),
        findall(O, oracle(O,_), KnownOracles),
        findall(obj(D,Oracle), (oracle(Oracle,P), map_distance(Curr, P, D), \+query_world(agent_check_oracle, [Agent,Oracle])),Oracles),
        sort(Oracles, Sorted),
        length(KnownOracles, Len),
        (
          Len = 0   -> first_oracle(Curr,Fastest), Fastest = [path(_,NextOracle,_)|_];
          otherwise -> Sorted = [obj(_, NextOracle) | _]
        )
      )
  ).

check_energy :-
  (
    part(3) -> (
        agent_current_energy(oscar, E), 
        strategy(S), 
        bound(B), 
        (
          E < B -> (
            S = normal -> assert(strategy(critic)), retract(strategy(normal)), retractall(bound(_)), assert(bound(30)), find_solution;
            otherwise  -> find_solution
          );
          otherwise -> (
            S = normal -> find_solution;
            otherwise  -> assert(strategy(normal)), retract(strategy(critic)), find_solution 
          )
        )
      );
    part(4) -> (
        my_agent(Agent),
        query_world(agent_current_energy, [Agent, E]), 
        strategy(S), 
        bound(B), 
        (
          E < B -> (
            S = normal -> assert(strategy(critic)), retract(strategy(normal)), retractall(bound(_)), assert(bound(30)), find_solution;
            otherwise  -> find_solution
          );
          otherwise -> (
            S = normal -> find_solution;
            otherwise  -> assert(strategy(normal)), retract(strategy(critic)), find_solution 
          )
        )
      )
  ).

normal_strategy :-
  (
    part(3) -> (
        agent_current_position(oscar, Pos),
        bound(B), 
        agent_current_energy(oscar, E),
        EnergyAfterQuery is E - 10,
        StrongerBound is B + 10,
        close_objects(Pos, 'o', Oracles),
        length(Oracles, Len),
        writeln(Pos),
        writeln(Len),
        (
          EnergyAfterQuery < B            -> retract(bound(_)), assert(bound(StrongerBound)), check_energy;
          Len > 0, EnergyAfterQuery >= B  -> Oracles = [obj(_, Oracle)|_], query_oracle(Oracle), assert(used_internal_objects(Oracle)), check_energy;
          otherwise                       -> best_oracle(Pos, Oracle), solve_task_1_3(find(Oracle), _),!,check_energy
        )
      );
    part(4) -> (
        my_agent(Agent),
        query_world(agent_current_position, [Agent, Pos]),
        bound(B), 
        query_world(agent_current_energy, [Agent, E]),
        EnergyAfterQuery is E - 10,
        StrongerBound is B + 10,
        close_objects(Pos, 'o', Oracles),
        length(Oracles, Len),
        writeln(Pos),
        (
          EnergyAfterQuery < B            -> retract(bound(_)), assert(bound(StrongerBound)), check_energy;
          Len > 0, EnergyAfterQuery >= B  -> Oracles = [obj(_, Oracle)|_], query_oracle(Oracle), assert(used_internal_objects(Oracle)),check_energy;
          otherwise                       -> best_oracle(Pos, Oracle), solve_task_4(find(Oracle), _),!,check_energy
        )
      )
  ).

critical_strategy :-
  (
    part(3) -> (
        agent_current_position(oscar, Pos),
        close_objects(Pos, 'c', Chargers),
        length(Chargers, Len),
        (
          Len > 0   -> Chargers = [obj(_, Charger)|_], agent_topup_energy(oscar, Charger), reset_bound;
          otherwise -> solve_task_1_3(find(c(X)),_), !, agent_topup_energy(oscar, c(X))
        ),
        check_energy
      );
    part(4) -> (
        my_agent(Agent),
        query_world(agent_current_position, [Agent, Pos]),
        close_objects(Pos, 'c', Chargers),
        length(Chargers, Len),
        (
          Len > 0   -> Chargers = [obj(_, Charger)|_], query_world(agent_topup_energy, [Agent, Charger]), reset_bound;
          otherwise -> solve_task_4(find(c(X)),_), !, query_world(agent_topup_energy, [Agent, c(X)])
        ),
        check_energy
      )
  ).

add_adjacents(P) :-
  findall(N, (map_adjacent(P, N, Type), type_to_pred(Type, N)), _).
  
type_to_pred(Type, Pos) :-
  (
    Type = o(_), \+oracle(Type, _)  -> assert(oracle(Type, Pos));
    Type = c(_), \+charger(Type, _) -> assert(charger(Type, Pos));
    otherwise -> true
  ).

known_oracles(Curr, Sorted) :-
  findall(obj(Dist,OID,Neighbours),(oracle(OID,Pos),findall(El,map_adjacent(Pos,El,empty),Neighbours), map_distance(Pos,Curr,Dist)),List),
  sort(List,Sorted).

duplicates_find(Res, path(_,X,_), path(_,Y,_)):-
  compare(Res,X,Y).

first_oracle(P,Fastest) :-
  initialise_visited(P),
  findall(path(Cost,o(X),Path),solve_task_astar(find(o(X)),[[state(0,0,P), P]],Path,Cost,_),List),
  sort(List, CostSorted), %sort based on lowest cost
  predsort(duplicates_find,CostSorted,Sorted), %keep only one path obj for each entry (the one kept always has lowest cost)
  sort(Sorted,Fastest). %sort the oracles by lowest cost from start point