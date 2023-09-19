:- module(plan,
    [
      get_best_fd/2
    ]).

o_type_f(O, F) :- objective(O), triple(O, tomasys:'typeF', F), function(F).
fd_solves_f(FD, F) :- fd(FD), triple(FD, tomasys:'solvesF', F), function(F).

fd_estimated_performance(F, FD, EP) :-
  fd_solves_f(FD, F),
  fd(FD),
  fd_realisability(FD, true),
  triple(FD, tomasys:'hasQAestimation', EQA),
  triple(EQA, tomasys:'isQAtype', mros:'performance'),
  qa_has_value(EQA, EP).

get_best_fd(O, BFD) :-
  o_type_f(O, F),
  findall([FD, EP], fd_estimated_performance(F, FD, EP), LIST_FDS),
  sort(2, @>, LIST_FDS, SORTED_FDS),
  [[BFD, _]|_] = SORTED_FDS.
