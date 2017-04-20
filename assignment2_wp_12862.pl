% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
:-dynamic eliminated_actor/1.

reset_possible_actors :-
  retractall(eliminated_actor(_)).

find_identity(A) :-  
  findall(N, possible_actor(N), Ns),
  length(Ns, Len),
  (
    Len = 1 -> possible_actor(A);
    otherwise -> query_oracle(o(1)), find_identity(A)
  ),!,
  retractall(eliminated_actor(_)).

query_oracle(Oracle) :-
  my_agent(Agent),
  query_world(agent_ask_oracle, [Agent, Oracle, link, RandomLink]),
  findall(N, possible_actor(N), Ns),
  maplist(get_wiki(RandomLink), Ns).

get_wiki(RandomLink, CurrActor):-
  findall(Link, (wp(CurrActor,WT), wt_link(WT,Link)), ListOfLinks),
  (
    \+ memberchk(RandomLink, ListOfLinks) -> assert(eliminated_actor(CurrActor));
    otherwise -> true
  ).

possible_actor(A) :-
  actor(A),
  \+ eliminated_actor(A).