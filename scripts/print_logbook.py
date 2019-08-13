import pickle
import matplotlib.pyplot as plt
from deap import creator, base, tools, algorithms
import numpy as np

DOCUMENT_PATH = "roboy/Documents"


def load_and_plot():
    #with open("/home/%s/NRP/GazeboRosPackages/src/exoskeleton/data/win_pop" % DOCUMENT_PATH, "r") as pop_file:
    #    pop = pickle.load(pop_file)
    #    for ind in pop:
    #        print ind
    with open("/home/%s/NRP/GazeboRosPackages/src/exoskeleton/data/ea_result" % DOCUMENT_PATH, "r") as lb_file:
        logbook = pickle.load(lb_file)
        print_result(logbook)


def print_result(logbook):
    gen = logbook.select("gen")
    fit_maxs = logbook.select("max")
    fit_avgs = logbook.select("avg")
    fit_mins = logbook.select("min")
    # the second column has the max values, so we only need this one for the max graph
    fit_maxs = np.array(fit_maxs)[:, 0]
    # the first column has the mean values, so we only need this one for the mean graph
    fit_avgs = np.array(fit_avgs)[:, 0]
    fit_mins = np.array(fit_mins)[:, 0]

    # we create subplots with 2 graphs and a legend in the upper right corner
    fig, ax1 = plt.subplots()
    line1 = ax1.plot(gen, fit_maxs, "b-", label="Fitness maximas")
    ax1.set_xlabel("Generation")
    ax1.set_ylabel("Fitness", color="b")
    for tl in ax1.get_yticklabels():
        tl.set_color("b")

    ax2 = ax1.twinx()
    line2 = ax2.plot(gen, fit_avgs, "r-", label="Fitness means")
    ax1.set_ylabel("metabolic costs (J)", color="r")
    for tl in ax2.get_yticklabels():
        tl.set_color("r")

    ax3 = ax1.twinx()
    line3 = ax3.plot(gen, fit_mins, "g-", label="Fitness minima")
    ax3.set_ylabel("metabolic costs (J)", color="g")
    for tl in ax3.get_yticklabels():
        tl.set_color("g")

    lns = line1 + line2 + line3
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc="upper right")

    plt.show()


if __name__ == "__main__":
    load_and_plot()
