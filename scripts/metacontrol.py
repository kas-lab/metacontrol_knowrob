#!/usr/bin/env python
import math
import re
import time

import rospy

from metacontrol_knowrob.prolog_query import PrologQuery

initial_time = time.time()
water_visibility_period = 80
water_visibility_min = 1.75
water_visibility_max = 3.5
water_visibility_amp = abs(water_visibility_max - water_visibility_min)/2
sec_shift = 0.0

ns = 'http://www.metacontrol.org/suave#'


def calculate_water_visibility():
    current_time = time.time()
    t = current_time - initial_time
    v_delta = water_visibility_amp + water_visibility_min
    water_visibility = water_visibility_amp * math.cos(
        (2*math.pi/water_visibility_period)*(t + sec_shift)) + v_delta
    return water_visibility


def update_measured_water_visibility(pq):
    # Check if meas_water_visibity  exists, in case it doesnt, add it
    q_wv_meas = "instance_of(suave:'meas_water_visibity', tomasys:'QAvalue')"
    result = pq.query(q_wv_meas)
    if result is False:
        add_result = pq.project_query(q_wv_meas)
        istype_result = pq.project_query(
            "triple(suave:'meas_water_visibity',\
                tomasys:'isQAtype', \
                suave:'water_visibility')"
            )
        if add_result is False or istype_result is False:
            return False

    # Delete value from meas_water_visibity and add new one
    value = calculate_water_visibility()
    # print('Measured water_visibility: ', value)
    q_wv_value = "triple(suave:'meas_water_visibity', tomasys:'hasValue', _)"
    meas_water_visibity = pq.query(q_wv_value)
    if meas_water_visibity is not False:
        delete = pq.unproject_query(q_wv_value)
    update_result = pq.project_query(
        "triple(suave:'meas_water_visibity', tomasys:'hasValue', {0})"
        .format(value)
    )

    # attach measured QA with FG
    fgs = pq.query(
        "fg_with_estimated_qa_type(FG, '{}')"
        .format(ns+'water_visibility'), include_ns=True)
    if fgs is not False:
        for fg in fgs:
            pq.project_query(
                "triple(\
                    '{}', tomasys:'hasQAvalue', suave:'meas_water_visibity')"
                .format(fg['FG'])
            )
    return update_result


def add_objective(pq):
    add_obj = pq.project_query(
        "instance_of(suave:'o_search', tomasys:'Objective')")
    add_typef = pq.project_query(
        "triple(suave:'o_search', tomasys:'typeF', \
        suave:'f_generate_search_path')")
    return add_obj and add_typef


def get_objectives_in_error(pq):
    obj_in_error = pq.query("objective_in_error(O).", include_ns=True)
    if obj_in_error is False:
        return []
    return obj_in_error


def get_objectives(pq):
    obj_in_error = pq.query("objective(O).", include_ns=True)
    if obj_in_error is False:
        return []
    return obj_in_error


def select_fds(pq, obj_in_error):
    selected_fds = list()
    for objective in obj_in_error:
        fd = pq.query(
            "get_best_fd('{}', FD).".format(objective['O']),
            include_ns=True
            )
        selected_fds.append({
            'O': objective['O'],
            'FD': fd[0]['FD']
        })
    # print("Selected FDs: ", selected_fds)
    return selected_fds


def ground_fds(pq, selected_fds):
    for selected_fd in selected_fds:
        has_fg = pq.query(
            "fg_solves_obj(FG, '{}')."
            .format(selected_fd['O']), include_ns=True
            )
        if has_fg is not False:
            pq.unproject_query(
                "triple('{}', tomasys:'typeFD', _)"
                .format(has_fg[0]['FG'])
            )
            pq.unproject_query(
                "triple('{}', rdf:'type', tomasys:'FunctionGrounding')"
                .format(has_fg[0]['FG'])
            )
        fg_name = ns + 'fg_' + re.split('#', selected_fd['O'])[-1]
        pq.project_query(
            "triple('{}', rdf:'type', tomasys:'FunctionGrounding')"
            .format(fg_name)
        )
        pq.project_query(
            "triple('{0}', tomasys:'typeFD', '{1}')"
            .format(fg_name, selected_fd['FD'])
        )
        pq.project_query(
            "triple('{0}', tomasys:'solvesO', '{1}')"
            .format(fg_name, selected_fd['O'])
        )


def print_status(pq):
    pq.query("objective_status(O, S)", print_solutions=True)
    pq.query("fg_status(FG, S)", print_solutions=True)
    pq.query("fg_type(FG, FD)", print_solutions=True)
    pq.query(
        "qa_has_value('{}', V)".format(ns+'meas_water_visibity'),
        print_solutions=True
    )


def main():
    rospy.init_node('metacontrol_knowrob')

    pq = PrologQuery()  # create a prolog query class instance

    query = pq.query(
        "load_owl('https://raw.githubusercontent.com/kas-lab/suave/main/suave_metacontrol/config/suave.owl',\
         [namespace(suave, 'http://www.metacontrol.org/suave#')]).")

    add_objective(pq)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        update_measured_water_visibility(pq)
        print_status(pq)
        # objs = get_objectives_in_error(pq)
        objs = get_objectives(pq)
        # print("Objectives in error: ", obj_in_error)
        selected_fds = select_fds(pq, objs)
        ground_fds(pq, selected_fds)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
