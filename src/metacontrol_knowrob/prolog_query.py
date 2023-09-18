""" Module to define a prolog query,
includes inicialization, quering and data extracting methods. """
# !/usr/bin/env python

from rosprolog_client import Prolog
import re


class PrologQuery:
    def __init__(self):

        self.prolog = Prolog()
        self.predicates = []  # initialize list attribute

    def project_query(self, q, print_solutions=False, include_ns=False):
        return self.query("kb_project({})".format(q))

    def unproject_query(self, q, print_solutions=False, include_ns=False):
        return self.query("kb_unproject({})".format(q))

    def query(self, q, print_solutions=False, include_ns=False):
        """ Perform prolog query.
        Takes as argument the query string and returns a list of dictionaries.
        One dictionary per query result. """

        query = self.prolog.query(q)

        solutions = False
        for s in query.solutions():
            if len(s) == 0:
                solutions = True
                break
            else:
                if solutions is False:
                    solutions = list()
                solutions.append(self.format_solution(s, include_ns))

        query.finish()
        if print_solutions:
            self.print_solutions(solutions)
        return solutions

    def print_solutions(self, solutions):
        if type(solutions) is list:
            for solution in solutions:
                for k, v in solution.items():
                    print('{} : {}'.format(k, v))
        else:
            print(solutions)
        print('\n')

    def format_solution(
       self, solution, include_ns=False):
        """ Format result """

        formatted_solution = dict()
        if len(solution) > 0:
            formatted_solution = dict()
            for k, v in solution.items():
                if type(v) is str:
                    f_value = re.split('#', v)
                    if include_ns is False:
                        formatted_solution[k] = f_value[-1]
                    else:
                        formatted_solution[k] = v
        return formatted_solution
