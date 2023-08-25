#!/usr/bin/env python
import pytest
import rospy

from metacontrol_knowrob.prolog_query import PrologQuery


@pytest.fixture
def node():
    rospy.init_node('test_rules', anonymous=True)


@pytest.fixture
def prolog_query():
    pq = PrologQuery()

    query = pq.query(
        "load_owl('package://metacontrol_knowrob/test/owl/test.owl',\
         [namespace(metacontrol_test, \
         'http://www.metacontrol.org/metacontrol_test#')]).")
    return pq


def test_objective_status(node, prolog_query):
    results = prolog_query.query(
        "objective_status(O, S).", print_solutions=True)
    expected_results = [
        ('o_fake_infer_in_error_nfr', 'IN_ERROR_NFR'),
        ('o_fake_infer_component_in_error', 'IN_ERROR_COMPONENT'),
    ]
    assertion = False
    for result in results:
        for er in expected_results:
            if er[0] in result.values() and er[1] in result.values():
                expected_results[:] = [
                    x for x in expected_results if x is not er]
        if len(expected_results) == 0:
            assertion = True
            break
    print('Expected results not found: ', expected_results)
    assert assertion


def test_fg_in_error_component(node, prolog_query):
    results = prolog_query.query("fg_status(FG, S).")
    assertion = False
    for result in results:
        if result['FG'] == 'fg_component_in_error' \
           and result['S'] == 'IN_ERROR_COMPONENT':
            assertion = True
            break
    assert assertion
