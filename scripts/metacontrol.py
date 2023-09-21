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


def update_measured_water_visibility(kb):
    # Check if meas_water_visibity  exists, in case it doesnt, add it
    q_wv_meas = "instance_of(suave:'meas_water_visibity', tomasys:'QAvalue')"
    result = kb.query(q_wv_meas)
    if result is False:
        add_result = kb.project_query(q_wv_meas)
        istype_result = kb.project_query(
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
    meas_water_visibity = kb.query(q_wv_value)
    if meas_water_visibity is not False:
        delete = kb.unproject_query(q_wv_value)
    update_result = kb.project_query(
        "triple(suave:'meas_water_visibity', tomasys:'hasValue', {0})"
        .format(value)
    )

    # attach measured QA with FG
    fgs = kb.query(
        "fg_with_estimated_qa_type(FG, '{}')"
        .format(ns+'water_visibility'), include_ns=True)
    if fgs is not False:
        for fg in fgs:
            kb.project_query(
                "triple(\
                    '{}', tomasys:'hasQAvalue', suave:'meas_water_visibity')"
                .format(fg['FG'])
            )
    return update_result


def add_objective(kb):
    add_obj = kb.project_query(
        "instance_of(suave:'o_search', tomasys:'Objective')")
    add_typef = kb.project_query(
        "triple(suave:'o_search', tomasys:'typeF', \
        suave:'f_generate_search_path')")
    add_always_improve = kb.project_query(
        "triple(suave:'o_search', tomasys:'o_always_improve', 'true')")
    return add_obj and add_typef


def get_objectives_in_error(kb):
    obj_in_error = kb.query("objective_in_error(O).", include_ns=True)
    if obj_in_error is False:
        return []
    return [o['O'] for o in obj_in_error]


def get_objectives_always_improve(kb):
    obj_improve = kb.query("objective_always_improve(O).", include_ns=True)
    if obj_improve is False:
        return []
    return [o['O'] for o in obj_improve]


def get_adaptable_objectives(kb):
    obj_adaptable = get_objectives_in_error(kb)
    obj_improve = get_objectives_always_improve(kb)
    obj_adaptable.extend(o for o in obj_improve if o not in obj_adaptable)
    return obj_adaptable


def select_fds(kb, obj_adaptable):
    selected_fds = list()
    for objective in obj_adaptable:
        fd = kb.query(
            "get_best_fd('{}', FD).".format(objective),
            include_ns=True
            )
        selected_fds.append({
            'O': objective,
            'FD': fd[0]['FD']
        })
    # print("Selected FDs: ", selected_fds)
    return selected_fds


def ground_fds(kb, selected_fds):
    for selected_fd in selected_fds:
        has_fg = kb.query(
            "fg_solves_obj(FG, '{}')."
            .format(selected_fd['O']), include_ns=True
            )
        if has_fg is not False:
            kb.unproject_query(
                "triple('{}', tomasys:'typeFD', _)"
                .format(has_fg[0]['FG'])
            )
            kb.unproject_query(
                "triple('{}', rdf:'type', tomasys:'FunctionGrounding')"
                .format(has_fg[0]['FG'])
            )
        fg_name = ns + 'fg_' + re.split('#', selected_fd['O'])[-1]
        kb.project_query(
            "triple('{}', rdf:'type', tomasys:'FunctionGrounding')"
            .format(fg_name)
        )
        kb.project_query(
            "triple('{0}', tomasys:'typeFD', '{1}')"
            .format(fg_name, selected_fd['FD'])
        )
        kb.project_query(
            "triple('{0}', tomasys:'solvesO', '{1}')"
            .format(fg_name, selected_fd['O'])
        )


def print_status(kb):
    kb.query("objective_status(O, S)", print_solutions=True)
    kb.query("fg_status(FG, S)", print_solutions=True)
    kb.query("fg_type(FG, FD)", print_solutions=True)
    kb.query(
        "qa_has_value('{}', V)".format(ns+'meas_water_visibity'),
        print_solutions=True
    )


def main():
    rospy.init_node('metacontrol_knowrob')

    kb = PrologQuery()  # create a prolog query class instance

    query = kb.query(
        "load_owl('https://raw.githubusercontent.com/kas-lab/suave/main/suave_metacontrol/config/suave.owl',\
         [namespace(suave, 'http://www.metacontrol.org/suave#')]).")

    add_objective(kb)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        update_measured_water_visibility(kb)
        print_status(kb)
        adaptable_objectives = get_adaptable_objectives(kb)
        selected_fds = select_fds(kb, adaptable_objectives)
        ground_fds(kb, selected_fds)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
