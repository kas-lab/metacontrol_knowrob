:- module(tomasys_query,
    [
      component/1,
      fd/1,
      fg/1,
      function/1,
      objective/1,
      qa_type/1,
      qa_value/1,
      c_required_by/2,
      fd_estimated_qa/2,
      fg_measured_qa/2,
      fd_solves_f/2,
      fg_solves_obj/2,
      fg_type/2,
      fg_with_estimated_qa_type/2,
      o_type_f/2,
      qa_is_type/2,
      qa_has_value/2,
      objective_always_improve/1,
      qa_comparison_operator/2,
      qa_critical/2
    ]).

% Get instances of TOMASys Classes
component(C) :- instance_of(C, tomasys:'Component').
function(F) :- instance_of(F, tomasys:'Function').
fd(FD) :- instance_of(FD, tomasys:'FunctionDesign').
fg(FG) :- instance_of(FG, tomasys:'FunctionGrounding').
objective(O) :- instance_of(O, tomasys:'Objective').
qa_type(QA) :- instance_of(QA, tomasys:'QualityAttributeType').
qa_value(QA) :- instance_of(QA, tomasys:'QAvalue').

% Get instances that are in relationships
c_required_by(C, FD) :- component(C), triple(C, mros:'requiredBy', FD), fd(FD).

fd_estimated_qa(FD, EQA) :-
  fd(FD), triple(FD, tomasys:'hasQAestimation', EQA), qa_value(EQA).

fg_measured_qa(FG, MQA) :-
  fg(FG), triple(FG, tomasys:'hasQAvalue', MQA), qa_value(MQA).
fd_solves_f(FD, F) :- fd(FD), triple(FD, tomasys:'solvesF', F), function(F).
fg_solves_obj(FG, O) :- fg(FG), triple(FG, tomasys:'solvesO', O), objective(O).
fg_type(FG, FD) :- fg(FG), triple(FG, tomasys:'typeFD', FD), fd(FD).
fg_with_estimated_qa_type(FG, QAT) :-
  fg_type(FG, FD), fd_estimated_qa(FD, EQA), qa_is_type(EQA, QAT).

o_type_f(O, F) :- objective(O), triple(O, tomasys:'typeF', F), function(F).

qa_is_type(QAV, QAT) :-
  qa_value(QAV), triple(QAV, tomasys:'isQAtype', QAT), qa_type(QAT).
qa_has_value(QA, QAV) :- triple(QA, tomasys:'hasValue', QAV).

% Get property
objective_always_improve(O) :-
  objective(O), triple(O, tomasys:'o_always_improve', 'true').

qa_comparison_operator(QAT, OP) :-
  qa_type(QAT), triple(QAT, tomasys:'qa_comparison_operator', OP).
qa_comparison_operator(QAT, '>') :-
  qa_type(QAT), \+triple(QAT, tomasys:'qa_comparison_operator', _).

% Check if QA is critical
qa_critical(QAT, CR) :-
  qa_type(QAT), triple(QAT, tomasys:'qa_critical', CR).
qa_critical(QAT, false) :-
  qa_type(QAT), \+triple(QAT, tomasys:'qa_critical', _).
