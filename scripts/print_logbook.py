import pickle
import matplotlib.pyplot as plt
from structure_optimization import print_result

def load_and_plot():
    with open("/home/roboy/Documents/NRP/GazeboRosPackages/src/exoskeleton/data/ea_result", "r") as lb_file:
        logbook = pickle.load(lb_file)
        print_result(logbook)

if __name__ == "__main__":
    load_and_plot()
