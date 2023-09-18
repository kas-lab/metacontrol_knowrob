#!/usr/bin/env python
import math
import time
import rospy

from metacontrol_knowrob.prolog_query import PrologQuery

initial_time = time.time()
water_visibility_period = 80
water_visibility_min = 1.75
water_visibility_max = 3.5
water_visibility_amp = abs(water_visibility_max - water_visibility_min)/2
sec_shift = 0.0


def calculate_water_visibility():
    current_time = time.time()
    t = current_time - initial_time
    v_delta = water_visibility_amp + water_visibility_min
    water_visibility = water_visibility_amp * math.cos(
        (2*math.pi/water_visibility_period)*(t + sec_shift)) + v_delta
    return water_visibility


def update_measured_water_visibility(pq):
    q_wv_meas = "instance_of(suave:'meas_water_visibity', tomasys:'QAvalue')"
    result = pq.query(q_wv_meas)
    if result is False:
        add_result = pq.project_query(q_wv_meas)
        if add_result is False:
            return False

    value = calculate_water_visibility()
    q_wv_value = "triple(suave:'meas_water_visibity', tomasys:'hasValue', _)"
    meas_water_visibity = pq.query(q_wv_value)
    if meas_water_visibity is not False:
        delete = pq.unproject_query(q_wv_value)
    update_result = pq.project_query(
        "triple(suave:'meas_water_visibity', tomasys:'hasValue', {0})"
        .format(value)
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
    print(obj_in_error)
    return obj_in_error


def select_fds(pq, obj_in_error):
    selected_fds = list()
    for objective in obj_in_error:
        selected_fds.append({
            'O': objective['O'],
            'FD': pq.query(
                "get_best_fd('{}', FD).".format(objective['O']),
                include_ns=True
                )
        })
    print(selected_fds)
    return selected_fds


def ground_fds(pq, selected_fds):
    pass


def main():
    rospy.init_node('metacontrol_knowrob')

    pq = PrologQuery()  # create a prolog query class instance

    query = pq.query(
        "load_owl('https://raw.githubusercontent.com/kas-lab/suave/main/suave_metacontrol/config/suave.owl',\
         [namespace(suave, 'http://www.metacontrol.org/suave#')]).")

    add_objective(pq)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        update_measured_water_visibility(pq)
        obj_in_error = get_objectives_in_error(pq)
        selected_fds = select_fds(pq, obj_in_error)
        ground_fds(pq, selected_fds)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
