% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
find_identity(A):-
  findall(A, actor(A), ActorNames),
  create_actor_data(ActorNames, [], Actors),
  search_actors(Actors, Actor),
  Actor = actor_data(A, _),
  !.

search_actors([Actor], Actor).
search_actors(Unfiltered, Actor) :-
  agent_ask_oracle(oscar, o(1), link, Link),
  filter_actors(Unfiltered, Link, [], Filtered),
  search_actors(Filtered, Actor).

create_actor_data([], Actors, Actors).
create_actor_data([Curr|Rest], Processed, Actors) :-
  wp(Curr, WT),
  findall(L, wt_link(WT, L), Links),
  create_actor_data(Rest,[actor_data(Curr,Links)|Processed],Actors).

filter_actors([], _, Filtered, Filtered).
filter_actors([Actor|Unfiltered], Link, Processed, Filtered) :-
  Actor = actor_data(_, Links),
  (
  memberchk(Link, Links) -> filter_actors(Unfiltered,Link,[Actor|Processed],Filtered);
  otherwise -> filter_actors(Unfiltered,Link,Processed,Filtered)
  ).
