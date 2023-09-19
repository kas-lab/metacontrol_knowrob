#!/usr/bin/env python
# import roslib
import rospy

from metacontrol_knowrob.prolog_query import PrologQuery
# import time

# from std_msgs.msg import String


# roslib.load_manifest('rosprolog')
rospy.init_node('manual_tests')

pq = PrologQuery()  # create a prolog query class instance
query = pq.query(
    "load_owl('package://metacontrol_knowrob/test/owl/test.owl',\
     [namespace(metacontrol_test, 'http://www.metacontrol.org/metacontrol_test#')]).")
# pq.load_namespace()

# print("QA:")
# query = pq.query("qa_type(QA).", print_solutions=True)
#
# print("QA critical:")
# query = pq.query("qa_critical(QAT, CR).", print_solutions=True)
# print("QA operator:")
# query = pq.query("qa_comparison_operator(QAT, OP).", print_solutions=True)

# print("FDs:")
# query = pq.query("fd(FD).", print_solutions=True)

print("Objectives status 1:")
query = pq.query("objective_status(O, S).", True)
#
# print("Objectives status 2:")
# query = pq.query(
#     "objective_status(O, 'test'); objective_status(O2, 'test').", True)
#
# print("qa type:")
# query = pq.query("qa_type(QA, T).", True)
#
# print("c status:")
# query = pq.query("c_status(C, S).", True, True)
#
print("fg:")
query = pq.query("fg(FG).", True)
print("fg status:")
query = pq.query("fg_status(FG, S).", True)
# print("fg fg_measured_qa:")
# query = pq.query("fg_measured_qa(FG, QA).", True)
# query = pq.query("fg_status(test:'fg_component_in_error', S).", True)

print("fd_realisability:")
query = pq.query("fd_realisability(Fd, S).", True)
