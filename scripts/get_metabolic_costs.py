#!/usr/bin/env python

from os import listdir
import random


def get_random_metab_csv_from_dir(metab_dir):
    possible_files = listdir(metab_dir)
    randint = random.randint(0, len(possible_files) - 1)
    try:
        files = possible_files[randint]
    except IndexError as ex:
        print "Caught IndexError while get file"
        print "len(possible_files): ", len(possible_files)
        print "randint: ", randint
        exit(1)
    return files


def get_random_metab_csv():
    return "../data/" + get_random_metab_csv_from_dir("../data")
