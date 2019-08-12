import pickle
import matplotlib.pyplot as plt
from deap import creator, base, tools, algorithms
import numpy as np

def load_and_plot():
    with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/data/pop_5_gen_20_tourn_10", "r") as lb_file:
        logbook = pickle.load(lb_file)
        print_result(logbook)


def print_result(logbook):
    gen = logbook.select("gen")
    fit_maxs = logbook.select("max")
    fit_avgs = logbook.select("avg")
    # the second column has the max values, so we only need this one for the max graph
    fit_maxs = np.array(fit_maxs)[:, 1]
    # the first column has the mean values, so we only need this one for the mean graph
    fit_avgs = np.array(fit_avgs)[:, 0]

    # we create subplots with 2 graphs and a legend in the upper right corner
    fig, ax1 = plt.subplots()
    # line1 = ax1.plot(gen, fit_maxs, "b-", label="Fitness maximas")
    # ax1.set_xlabel("Generation")
    ax1.set_ylabel("Fitness", color="b")
    # for tl in ax1.get_yticklabels():
    #     tl.set_color("b")

    #ax2 = ax1.twinx()
    line2 = ax1.plot(gen, fit_avgs, "b-", label="Fitness means")
    ax1.set_ylabel("metabolic costs (J)", color="b")
    for tl in ax1.get_yticklabels():
        tl.set_color("b")

    lns = line2 # line1 + line2
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc="upper right")

    plt.show()


if __name__ == "__main__":
    load_and_plot()
