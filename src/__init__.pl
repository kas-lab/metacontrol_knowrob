:- register_ros_package(knowrob).
:- load_owl('https://raw.githubusercontent.com/meta-control/mc_mdl_tomasys/ros2/owl/tomasys.owl', [namespace(tomasys, 'http://metacontrol.org/tomasys#')]).
:- load_owl('https://raw.githubusercontent.com/meta-control/mros_ontology/main/owl/mros.owl', [namespace(mros, 'http://ros/mros#')]).
:- register_ros_package(metacontrol_knowrob).

:- use_module(library('analyze')).
