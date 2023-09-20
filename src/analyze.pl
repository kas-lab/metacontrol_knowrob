:- module(analyze,
    [
      c_status/2,
      fd_realisability/2,
      fg_measured_qa_violation/2,
      fg_status/2,
      fg_with_estimated_qa_type/2,
      objective_status/2,
      objective_in_error/1,
      qa_violation/3
    ]).

% Component status
c_status(C, S) :- component(C), triple(C, tomasys:'c_status', S).

% FD Realisability (CmpErr)
fd_realisability(FD, S):- fd(FD), triple(FD, tomasys:'fd_realisability', S).

fd_realisability(FD, false) :-
  \+triple(FD, tomasys:'fd_realisability', _),
  c_required_by(C, FD), c_status(C, 'FALSE').

fd_realisability(FD, false) :-
  \+triple(FD, tomasys:'fd_realisability', _),
  fd_estimated_qa(FD, EQA), qa_is_type(EQA, QAT),
  fg(FG), fg_measured_qa(FG, MQA), qa_is_type(MQA, QAT),
  qa_has_value(MQA, MV), qa_has_value(EQA, EV), qa_violation(QAT, MV, EV).

fd_realisability(FD, true):-
  fd(FD), \+triple(FD, tomasys:'fd_realisability', _),
  \+fd_realisability(FD, false).

% Check if the measured QAs violate the FGs(FD) requirement
fg_measured_qa_violation(FG, QAT) :-
  fg_measured_qa(FG, MQA), fg_type(FG, FD), fd_estimated_qa(FD, EQA),
  qa_is_type(MQA, QAT), qa_is_type(EQA, QAT), qa_has_value(MQA, MV),
  qa_has_value(EQA, EV), qa_violation(QAT, MV, EV).

%% FG status
fg_status(FG, S) :- fg(FG), triple(FG, tomasys:'fg_status', S).

fg_status(FG, 'IN_ERROR_COMPONENT') :-
  fg(FG),
  \+triple(FG, tomasys:'fg_status', _), %why this doesnt work?
  fg_type(FG, FD), c_required_by(C, FD), c_status(C, 'FALSE').

fg_status(FG, 'IN_ERROR_FR') :-
  fg(FG),
  \+triple(FG, tomasys:'fg_status', _),
  \+fg_status(FG, 'IN_ERROR_COMPONENT'),
  fg_measured_qa_violation(FG, QAT), qa_critical(QAT, true).

fg_status(FG, 'IN_ERROR_NFR') :-
  fg(FG),
  \+triple(FG, tomasys:'fg_status', _),
  \+fg_status(FG, 'IN_ERROR_COMPONENT'),
  \+fg_status(FG, 'IN_ERROR_FR'),
  fg_measured_qa_violation(FG, QAT), qa_critical(QAT, false).

fg_status(FG, 'OK') :-
  fg(FG),
  \+triple(FG, tomasys:'fg_status', _),
  \+fg_status(FG, 'IN_ERROR_COMPONENT'),
  \+fg_status(FG, 'IN_ERROR_FR'),
  \+fg_status(FG, 'IN_ERROR_NFR'),
  \+fg_measured_qa_violation(FG, _).

% Objective status (S1a,b,c)
objective_status(O, S) :- objective(O), triple(O, tomasys:'o_status', S).
objective_status(O, S) :-
  objective(O), \+triple(O, tomasys:'o_status', S),
  fg_solves_obj(FG, O), fg_status(FG, S).
objective_status(O, 'UNGROUNDED') :-
  objective(O), \+triple(O, tomasys:'o_status', _), \+fg_solves_obj(_, O).
% TODO: infer objective status when objective has a required QA

objective_in_error(O) :- objective_status(O, S), S \= 'OK'.

%% Helper predicates for FG status
% Compare the measured QA with the estimated QA given the comparison_operator
qa_violation(QAT, MV, EV) :-
  qa_comparison_operator(QAT, '<'), !, MV < EV;
  qa_comparison_operator(QAT, '<='), !, MV =< EV;
  qa_comparison_operator(QAT, '>'), !, MV > EV;
  qa_comparison_operator(QAT, '>='), !, MV >= EV.
