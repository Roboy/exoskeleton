import random
from deap import creator, base, tools, algorithms
from metab_cost_to_fitness import get_fitness, get_fitness_mean
from get_metabolic_costs import get_random_metab_csv
import numpy as np
import matplotlib.pyplot as plt
from metab_to_csv import file_name
import rospy
from roboy_simulation_msgs.srv import FloatToMetab, TestStatus
import pickle
from float_to_vp import distances_to_xml
from print_logbook import print_result
from std_srvs.srv import Trigger
from float_to_osim import update_osim, floats_to_path_points

set_test = None
running_test = None
start_pure_osim_test = None


def evaluate_activation(individual):
    """
    Takes an individual and returns its fitness. Currently the according metabolic cost file is picked random, so the
    fitness value is random too.
    :param individual: a list of floats
    :return: a tuple containing the mean and the max value of the given metabolic costs
    """
    global set_test
    global running_test
    rospy.loginfo("sent test instructions")
    set_test(individual)
    rospy.sleep(1)
    test_flag = running_test()
    while test_flag.running_test:
        rospy.sleep(1)
        test_flag = running_test()
        rospy.loginfo("test not finished yet, test_flag: %s", test_flag)
    rospy.loginfo("test finished, grab fitness now")
    return get_fitness(file_name)


def check_bounds(min, max):
    """
    A decorator for an ea operator. It makes sure that the float values of the individuals stay in the given bounds
    :param min: lower bound
    :param max: upper bound
    :return: the decorator that is carrying about this task
    """

    def decorator(func):
        def wrapper(*args, **kargs):
            offspring = func(*args, **kargs)
            for child in offspring:
                for i in xrange(len(child)):
                    if child[i] > max:
                        child[i] = max
                    elif child[i] < min:
                        child[i] = min
            return offspring

        return wrapper

    return decorator


def muscle_activation_opt():
    """
    The muscle activation optimization using a simple evolutionary algorithm from the DEAP framework.
    :return:
    """
    # number of floats in one individual
    IND_SIZE = 12  # 20
    # bounds of the uniform distribution creating the float values for the individuals
    START = 0.0  # -0.025
    STOP = 1.0  # 0.025
    # number of generations that should be created
    NGEN = 30
    # CXPB  is the probability with which two individuals are crossed
    CXPB = 0.5
    # MUTPB is the probability for mutating an individual
    MUTPB = 0.2

    # first define the problem and the individuals
    # we want to minimize the fitness values
    creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
    # the individuals consist of a list of floats and we give it the above defined minimizer
    creator.create("Individual", list, fitness=creator.FitnessMin)

    # second a toolbox is created that describes the necessary steps of the ea algorithm
    toolbox = base.Toolbox()
    # the attributes of the individuals( a list) are floats and are created by a uniform distribution
    toolbox.register("attr_float", random.uniform, START, STOP)
    # the individual gets defined by adding the float attribute and the number of floats(IND_SIZE)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=IND_SIZE)
    # the populations gets defined as a list of individuals
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # the mate operation is relized by the cxTwoPoint ooperation
    toolbox.register("mate", tools.cxTwoPoint)
    # the mutate operation is defined as a gaussian mutation
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
    # the select operation is a selection tournamend with 3 individuals participating in each tournament
    toolbox.register("select", tools.selTournament, tournsize=3)
    # the evaluate functions gets registered
    toolbox.register("evaluate", evaluate_activation)

    # the genetic operations are "decorated" with a function keeping the float values of the individuals in the defined
    # bounds
    toolbox.decorate("mate", check_bounds(START, STOP))
    toolbox.decorate("mutate", check_bounds(START, STOP))

    # third, statistics get defined that will be used to evaluate the whole algorithm afterwards
    stats = tools.Statistics(key=lambda ind: ind.fitness.values)
    # since our fitness consists out of the mean and the max value, exactly those are the ones we are looking for
    stats.register("avg", np.mean, axis=0)
    stats.register("max", np.max, axis=0)

    # an initial population is defined
    pop = toolbox.population(n=5)

    # and finally a simple ea is used to find the best individual
    pop, logbook = algorithms.eaSimple(pop, toolbox, cxpb=CXPB, mutpb=MUTPB, ngen=NGEN, stats=stats,
                                       verbose=False)
    with open("/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/data/ea_result", "w") as lb_file:
        pickle.dump(logbook, lb_file)
    # Afterwards we wanna plot the statistics, so we get the necessary data out of the logbook
    # print_result(logbook)


def evaluate_structure(individual):
    """
    Takes an individual and returns its fitness. Currently the according metabolic cost file is picked random, so the
    fitness value is random too.
    :param individual: a list of floats
    :return: a tuple containing the mean and the max value of the given metabolic costs
    """
    # generate cardsflow
    distances_to_xml(individual)
    return 0


def structure_optimization():
    """
    The muscle activation optimization using a simple evolutionary algorithm from the DEAP framework.
    :return:
    """
    # number of floats in one individual
    IND_SIZE = 16
    # bounds of the uniform distribution creating the float values for the individuals
    START = -0.025
    STOP = 0.025
    # number of generations that should be created
    NGEN = 30
    # CXPB  is the probability with which two individuals are crossed
    CXPB = 0.5
    # MUTPB is the probability for mutating an individual
    MUTPB = 0.2

    # first define the problem and the individuals
    # we want to minimize the fitness values
    creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
    # the individuals consist of a list of floats and we give it the above defined minimizer
    creator.create("Individual", list, fitness=creator.FitnessMin)

    # second a toolbox is created that describes the necessary steps of the ea algorithm
    toolbox = base.Toolbox()
    # the attributes of the individuals( a list) are floats and are created by a uniform distribution
    toolbox.register("attr_float", random.uniform, START, STOP)
    # the individual gets defined by adding the float attribute and the number of floats(IND_SIZE)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=IND_SIZE)
    # the populations gets defined as a list of individuals
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # the mate operation is relized by the cxTwoPoint ooperation
    toolbox.register("mate", tools.cxTwoPoint)
    # the mutate operation is defined as a gaussian mutation
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
    # the select operation is a selection tournamend with 3 individuals participating in each tournament
    toolbox.register("select", tools.selTournament, tournsize=3)
    # the evaluate functions gets registered
    toolbox.register("evaluate", evaluate_structure)

    # the genetic operations are "decaroated" with a function keeping the float values of the individuals in the defined
    # bounds
    toolbox.decorate("mate", check_bounds(START, STOP))
    toolbox.decorate("mutate", check_bounds(START, STOP))

    # third, statistics get defined that will be used to evaluate the whole algorithm afterwards
    stats = tools.Statistics(key=lambda ind: ind.fitness.values)
    # since our fitness consists out of the mean and the max value, exactly those are the ones we are looking for
    stats.register("avg", np.mean, axis=0)
    stats.register("max", np.max, axis=0)

    # an initial population is defined
    pop = toolbox.population(n=5)

    # and finally a simple ea is used to find the best individual
    pop, logbook = algorithms.eaSimple(pop, toolbox, cxpb=CXPB, mutpb=MUTPB, ngen=NGEN, stats=stats,
                                       verbose=False)
    with open("/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/data/ea_result", "w") as lb_file:
        pickle.dump(logbook, lb_file)
    # Afterwards we wanna plot the statistics, so we get the necessary data out of the logbook
    # print_result(logbook)


def evaluate_path_points(individual):
    """
    Takes an individual and returns its fitness. Currently the according metabolic cost file is picked random, so the
    fitness value is random too.
    :param individual: a list of floats
    :return: a tuple containing the mean and the max value of the given metabolic costs
    """
    global start_pure_osim_test
    global running_test
    update_osim("/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/output/CARDSFlowExo/muscles.osim",
                floats_to_path_points(individual))
    start_pure_osim_test()
    rospy.loginfo("started test")
    rospy.sleep(1.0)
    test_flag = running_test()
    while test_flag.running_test:
        rospy.sleep(0.1)
        test_flag = running_test()
        rospy.loginfo("test not finished yet, test_flag: %s", test_flag)
    rospy.loginfo("test finished, wait until sim_control is finished")
    rospy.sleep(0.5)
    rospy.loginfo("grab fitness now")
    return get_fitness_mean(file_name)


def check_bounds_path_points(shoulder_min, shoulder_max, arm_min, arm_max):
    """
    A decorator for an ea operator. It makes sure that the float values of the individuals stay in the given bounds
    :param min: lower bound
    :param max: upper bound
    :return: the decorator that is carrying about this task
    """

    def decorator(func):
        def wrapper(*args, **kargs):
            offspring = func(*args, **kargs)
            for child in offspring:
                for i in range(4):
                    if child[i] > shoulder_max:
                        child[i] = shoulder_max
                    elif child[i] < shoulder_min:
                        child[i] = shoulder_min
                for i in range(4, 8):
                    if child[i] > arm_max:
                        child[i] = arm_max
                    elif child[i] < arm_min:
                        child[i] = arm_min
            return offspring

        return wrapper

    return decorator


def init_path_points(point, shoulder_min, shoulder_max, arm_min, arm_max):
    path_points = []
    for i in range(4):
        path_points.append(random.uniform(shoulder_min, shoulder_max))
    for i in range(4):
        path_points.append(random.uniform(arm_min, arm_max))
    individual = point(path_points)
    return individual


def osim_path_point_opt():
    """
    The opensim path point optimization using a simple evolutionary algorithm from the DEAP framework.
    :return:
    """
    # bounds of the uniform distribution creating the float values for the individuals
    shoulder_min = 0.0
    shoulder_max = 0.08
    arm_min = 0.0
    arm_max = 0.05
    # number of generations that should be created
    NGEN = 1
    # CXPB  is the probability with which two individuals are crossed
    CXPB = 0.5
    # MUTPB is the probability for mutating an individual
    MUTPB = 0.2

    # first define the problem and the individuals
    # we want to minimize the fitness values
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    # the individuals consist of a list of floats and we give it the above defined minimizer
    creator.create("Individual", list, fitness=creator.FitnessMin)

    # second a toolbox is created that describes the necessary steps of the ea algorithm
    toolbox = base.Toolbox()
    # the individual gets defined by adding the float attribute and the number of floats(IND_SIZE)
    toolbox.register("individual", init_path_points, creator.Individual,
                     shoulder_min=shoulder_min, shoulder_max=shoulder_max, arm_min=arm_min, arm_max=arm_max)
    # the populations gets defined as a list of individuals
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # the mate operation is realized by the cxTwoPoint operation
    toolbox.register("mate", tools.cxTwoPoint)
    # the mutate operation is defined as a gaussian mutation
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
    # the select operation is a selection tournamend with 3 individuals participating in each tournament
    toolbox.register("select", tools.selTournament, tournsize=2)
    # the evaluate functions gets registered
    toolbox.register("evaluate", evaluate_path_points)

    # the genetic operations are "decorated" with a function keeping the float values of the individuals in the defined
    # bounds
    toolbox.decorate("mate", check_bounds_path_points(shoulder_min, shoulder_max, arm_min, arm_max))
    toolbox.decorate("mutate", check_bounds_path_points(shoulder_min, shoulder_max, arm_min, arm_max))

    # third, statistics get defined that will be used to evaluate the whole algorithm afterwards
    stats = tools.Statistics(key=lambda ind: ind.fitness.values)
    # since our fitness consists out of the mean and the max value, exactly those are the ones we are looking for
    stats.register("avg", np.mean, axis=0)
    stats.register("max", np.max, axis=0)

    # an initial population is defined
    pop = toolbox.population(n=2)

    # and finally a simple ea is used to find the best individual
    pop, logbook = algorithms.eaSimple(pop, toolbox, cxpb=CXPB, mutpb=MUTPB, ngen=NGEN, stats=stats,
                                       verbose=False)
    with open("/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/data/ea_result", "w") as lb_file:
        pickle.dump(logbook, lb_file)
    with open("/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/data/win_pop", "w") as pop_file:
        pickle.dump(pop, pop_file)
    # Afterwards we wanna plot the statistics, so we get the necessary data out of the logbook
    # print_result(logbook)


if __name__ == "__main__":
    rospy.init_node("structure_optimization")
    set_test = rospy.ServiceProxy("sim_control/set_test", FloatToMetab)
    start_pure_osim_test = rospy.ServiceProxy("sim_control/start_pure_osim_test", Trigger)
    running_test = rospy.ServiceProxy("sim_control/running_test", TestStatus)
    osim_path_point_opt()
