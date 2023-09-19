#!/usr/bin/env python
import pytest
import rospy

from metacontrol_knowrob.prolog_query import PrologQuery


@pytest.fixture(scope="module")
def prolog_query():
    rospy.init_node('test_rules', anonymous=True)
    pq = PrologQuery()
    query = pq.query(
        "load_owl('package://metacontrol_knowrob/test/owl/test.owl',\
         [namespace(metacontrol_test, \
         'http://www.metacontrol.org/metacontrol_test#')]).")
    return pq


def evaluate_query_results(results, expected_results):
    for result in results:
        for er in expected_results:
            if er[0] in result.values() and er[1] in result.values():
                expected_results[:] = [
                    x for x in expected_results if x is not er]
        if len(expected_results) == 0:
            return True
    print('Expected results not found: ', expected_results)
    return False


def test_objective_in_error(prolog_query):
    results = prolog_query.query(
        "objective_status(O, S).", print_solutions=True)
    expected_results = [
        ('o_fake_infer_in_error_nfr', 'IN_ERROR_NFR'),
        ('o_fake_infer_component_in_error', 'IN_ERROR_COMPONENT'),
        ('o_fake_critical', 'IN_ERROR_FR'),
        ('o_fake_less_operator_error', 'IN_ERROR_NFR'),
        # ('o_fake_less_operator_ok', 'OK'),
        ('o_ungrounded', 'UNGROUNDED'),
    ]
    assert evaluate_query_results(results, expected_results)


def test_objective_status(prolog_query):
    results = prolog_query.query(
        "objective_status(O, S).", print_solutions=True)
    expected_results = [
        ('o_fake_infer_in_error_nfr', 'IN_ERROR_NFR'),
        ('o_fake_infer_component_in_error', 'IN_ERROR_COMPONENT'),
        ('o_fake_critical', 'IN_ERROR_FR'),
        ('o_fake_less_operator_error', 'IN_ERROR_NFR'),
        ('o_fake_less_operator_ok', 'OK'),
        ('o_ungrounded', 'UNGROUNDED'),
    ]
    assert evaluate_query_results(results, expected_results)


# TODO: check if there are extra wrong values,
# e.g., ('fg_less_operator_error', 'OK')
def test_fg_status(prolog_query):
    results = prolog_query.query("fg_status(FG, S).", print_solutions=True)
    expected_results = [
        ('fg_fake_in_error_nfr', 'IN_ERROR_NFR'),
        ('fg_component_in_error', 'IN_ERROR_COMPONENT'),
        ('fg_fake_critical', 'IN_ERROR_FR'),
        ('fg_less_operator_error', 'IN_ERROR_NFR'),
        ('fg_less_operator_ok', 'OK'),
    ]
    assert evaluate_query_results(results, expected_results)


def test_fd_realisability(prolog_query):
    results = prolog_query.query(
        "fd_realisability(FD, S)", print_solutions=True)
    expected_results = [
        ('fd_fake_component', False),
        ('fd_fake1', True),
        ('fd_fake_critical', False),
        ('fd_fake_less_operator', False),
    ]
    assert evaluate_query_results(results, expected_results)


def test_fg_with_estimated_qa_type(prolog_query):
    results = prolog_query.query(
        "fg_with_estimated_qa_type(FG, QAT).", print_solutions=True)
    expected_results = [
        ('fg_fake_critical', 'qa_critical'),
        ('fg_fake_in_error_nfr', 'mockiness'),
        ('fg_fake_in_error_nfr', 'performance'),
        ('fg_less_operator_error', 'qa_less_operator'),
    ]
    assert evaluate_query_results(results, expected_results)
