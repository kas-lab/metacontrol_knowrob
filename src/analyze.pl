:- module(analyze,
    [
      fg/1,
      fg_solves_obj/2,
      objective_status/2
    ]).

fg(FG) :- instance_of(FG, tomasys:'FunctionGrounding').
fg_solves_obj(FG, O) :- triple(FG, tomasys:'solvesO', O).
fg_status(FG, S) :- triple(FG, tomasys:'fg_status', S).
% fg_measured_qa(FG, MQA) :- triple(FG, tomasys:'hasQAvalue', MQA).
% fg_type(FG, FD) :- triple(FG, tomasys:'typeFD', FD).
% fd_estimated_qa(FD, EQA) :- triple(FD, tomasys:'hasQAestimation', EQA).
% qa_type(QA, T) :- triple(QA, tomasys:'isQAtype', T).
% qa_value(QA, QAV) :- triple(QA, tomasys:'hasValue', QAV).

% fg_status(FG, "IN_ERROR_NFR") :-
%   fg(FG), fg_measured_qa(FG, MQA), fg_type(FG, FD),
%   fd_estimated_qa(FD, EQA), qa_type(MQA, T), qa_type(EQA, T),
%   qa_value(MQA, MV), qa_value(EQA, EV), MV < EV.

% S1a,b,c
objective_status(O, S) :- fg(FG), fg_solves_obj(FG, O), fg_status(FG, S).
