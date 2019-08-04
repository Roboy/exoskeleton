import csv
import numpy as np
import matplotlib.pyplot as plt


def get_mean(data):
    """
    builds the mean for every time series in data
    :param data: a list of numpy arrays, containing time series
    :return: a list of means respectively
    """
    mean_list = []
    for i in range(len(data)):
        mean_list.append(np.mean(data[i]))

    return mean_list


def get_max(umberger_total):
    """
    builds the max for every time series in data
    :param data: a list of numpy arrays, containing time series
    :return: a list of max values respectively
    """
    max_list = []
    for i in range(len(umberger_total)):
        max_list.append(np.max(umberger_total[i]))

    return max_list


def plot_costs(file_name, fig_cnt):
    """
    opens the file, gets the data and plots into a figure with the fig_count
    :param file_name:
    :param fig_cnt:
    """
    sim_timestamp, umberger_total = get_metab_data(file_name)

    plt.figure(fig_cnt)
    plt.plot(sim_timestamp, umberger_total[0])
    plt.plot(sim_timestamp, umberger_total[1])
    plt.plot(sim_timestamp, umberger_total[2])
    plt.plot(sim_timestamp, umberger_total[3])
    plt.plot(sim_timestamp, umberger_total[4])
    plt.plot(sim_timestamp, umberger_total[5])


def get_metab_data(file_name):
    """
    gets the metabolic cost data, using a csv reader
    :param file_name: the file containing the data
    :return: an numpy array for the simulation timestamps,
    and a list containing the metabolic cost time series for every muscle
    """
    sim_timestamp = np.array([])
    umberger_total = []
    for i in range(6):
        umberger_total.append(np.array([]))
    with open(file_name, "r") as csv_file:
        data = csv.DictReader(csv_file, delimiter=';')
        for row in data:
            sim_timestamp = np.append(sim_timestamp, [np.float(row["simTimestamp"])])
            umberger_total[0] = np.append(umberger_total[0], [np.float(row[" umbergerTotal_m0"])])
            umberger_total[1] = np.append(umberger_total[1], [np.float(row[" umbergerTotal_m1"])])
            umberger_total[2] = np.append(umberger_total[2], [np.float(row[" umbergerTotal_m2"])])
            umberger_total[3] = np.append(umberger_total[3], [np.float(row[" umbergerTotal_m3"])])
            umberger_total[4] = np.append(umberger_total[4], [np.float(row[" umbergerTotal_m4"])])
            umberger_total[5] = np.append(umberger_total[5], [np.float(row[" umbergerTotal_m5 "])])

    return sim_timestamp, umberger_total


def get_fitness(file_name):
    """
    calculating the mean of all time series means of one time series file
    :param file_name: the file
    :return: the mean of all means
    """
    sim_timestamp, data = get_metab_data(file_name)
    return np.mean(np.mean(data))


if __name__ == "__main__":
    print "absolute mean"
    print "act_3: ", get_fitness("../data/act_3.csv")
    print "act_7: ", get_fitness("../data/act_7.csv")
    print "act_10: ", get_fitness("../data/act_10.csv")

    #plt.show()

