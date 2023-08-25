:- module(analyze,
    [
      component/1,
      fd/1,
      fg/1,
      objective/1,
      qa_type/1,
      qa_value/1,
      c_required_by/2,
      fd_realisability/2,
      fg_solves_obj/2,
      fg_type/2,
      fg_measured_qa/2,
      fd_estimated_qa/2,
      qa_is_type/2,
      qa_has_value/2,
      qa_comparison_operator/2,
      qa_critical/2,
      c_status/2,
      fg_status/2,
      objective_status/2
    ]).

% Get instances of TOMASys Classes
component(C) :- instance_of(C, tomasys:'Component').
fd(FD) :- instance_of(FD, tomasys:'FunctionDesign').
fg(FG) :- instance_of(FG, tomasys:'FunctionGrounding').
objective(O) :- instance_of(O, tomasys:'Objective').
qa_type(QA) :- instance_of(QA, tomasys:'QualityAttributeType').
qa_value(QA) :- instance_of(QA, tomasys:'QAvalue').

% Get instances that are in relationships
c_required_by(C, FD) :- component(C), triple(C, mros:'requiredBy', FD), fd(FD).

fg_solves_obj(FG, O) :- fg(FG), triple(FG, tomasys:'solvesO', O), objective(O).
fg_type(FG, FD) :- fg(FG), triple(FG, tomasys:'typeFD', FD), fd(FD).
fg_measured_qa(FG, MQA) :-
  fg(FG), triple(FG, tomasys:'hasQAvalue', MQA), qa_value(MQA).

fd_estimated_qa(FD, EQA) :-
  fd(FD), triple(FD, tomasys:'hasQAestimation', EQA), qa_value(EQA).

qa_is_type(QAV, QAT) :-
  qa_value(QAV), triple(QAV, tomasys:'isQAtype', QAT), qa_type(QAT).
qa_has_value(QA, QAV) :- triple(QA, tomasys:'hasValue', QAV).

% Get property
qa_comparison_operator(QAT, OP) :-
  qa_type(QAT), triple(QAT, tomasys:'qa_comparison_operator', OP).
qa_critical(QAT, CR) :-
  qa_type(QAT), triple(QAT, tomasys:'qa_critical', CR).

c_status(C, S) :- component(C), triple(C, tomasys:'c_status', S).

% S1a,b,c
objective_status(O, S) :- objective(O), triple(O, tomasys:'o_status', S).
objective_status(O, S) :- fg_solves_obj(FG, O), fg_status(FG, S).
% TODO: infer objective UNGROUNDED
% TODO: infer objective OK/IN_PROGRESS
% TODO: infer objective status when objective has a required QA


% helper predicate to get fg error type
qa_error_type(QAT, ERROR) :-
  qa_critical(QAT, true) -> ERROR = 'IN_ERROR_FR'; ERROR = 'IN_ERROR_NFR'.
% qa_error_type(QAT, 'IN_ERROR_NFR') :- qa_critical(QAT, false).

% Compare the measured QA with the estimated QA given the comparison_operator
qa_comparer(QAT, MV, EV) :-
  qa_comparison_operator(QAT, '<'), !, MV < EV;
  qa_comparison_operator(QAT, '<='), !, MV =< EV;
  qa_comparison_operator(QAT, '>'), !, MV > EV;
  qa_comparison_operator(QAT, '>='), !, MV >= EV;
  MV > EV.


% CompErrFg
fg_status(FG, S) :- fg(FG), triple(FG, tomasys:'fg_status', S).
fg_status(FG, 'IN_ERROR_COMPONENT') :-
  fg_type(FG, FD), c_required_by(C, FD), c_status(C, 'FALSE').

fg_status(FG, ERROR) :-
  fg_measured_qa(FG, MQA), fg_type(FG, FD), fd_estimated_qa(FD, EQA),
  qa_is_type(MQA, QAT), qa_is_type(EQA, QAT), qa_has_value(MQA, MV),
  qa_has_value(EQA, EV), qa_comparer(QAT, MV, EV), qa_error_type(QAT, ERROR).
%TODO: infer fg status OK
% fg_status(FG, 'OK') :- fg(FG).

% CompErr
fd_realisability(FD, S):- fd(FD), triple(FD, tomasys:'fd_realisability', S).
fd_realisability(FD, false) :- c_required_by(C, FD), c_status(C, 'FALSE').
% fd_realisability(FD, true):- fd(FD). % TODO: infer realisability OK
