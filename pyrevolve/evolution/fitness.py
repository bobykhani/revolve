import random as py_random
from pyrevolve.tol.manage import measures


def stupid(_robot_manager, robot):
    return 1.0


def random(_robot_manager, robot):
    return py_random.random()


def displacement(robot_manager, robot):
    displacement_vec = measures.displacement(robot_manager)[0]
    displacement_vec.z = 0
    return displacement_vec.magnitude()


def displacement_velocity(robot_manager, robot):
    return measures.displacement_velocity(robot_manager)


def online_old_revolve(robot_manager):
    """
    Fitness is proportional to both the displacement and absolute
    velocity of the center of mass of the robot, in the formula:

    (1 - d l) * (a dS + b S + c l)

    Where dS is the displacement over a direct line between the
    start and end points of the robot, S is the distance that
    the robot has moved and l is the robot size.

    Since we use an active speed window, we use this formula
    in context of velocities instead. The parameters a, b and c
    are modifyable through config.
    :return: fitness value
    """
    # these parameters used to be command line parameters
    warmup_time = 0.0
    v_fac = 1.0  # fitness_velocity_factor
    d_fac = 5.0  # fitness_displacement_factor
    s_fac = 0.0  # fitness_size_factor
    fitness_size_discount = 0.0
    fitness_limit = 1.0

    age = robot_manager.age()
    if age < (0.25 * robot_manager.conf.evaluation_time) \
            or age < warmup_time:
        # We want at least some data
        return 0.0

    d = 1.0 - (fitness_size_discount * robot_manager.size)
    v = d * (d_fac * measures.displacement_velocity(robot_manager)
             + v_fac * measures.velocity(robot_manager)
             + s_fac * robot_manager.size)
    return v if v <= fitness_limit else 0.0


def size_penalty(robot_manager, robot):
    _size_penalty = 1 / robot.phenotype._morphological_measurements.measurements_to_dict()['absolute_size']

    return _size_penalty

# def displacement_velocity_hill(behavioural_measurements, robot):
#
#     if behavioural_measurements is not None:
#         fitness = behavioural_measurements['displacement_velocity_hill']
#
#         if fitness == 0 or robot.phenotype._morphological_measurements.measurements_to_dict()['hinge_count'] == 0:
#             fitness = -0.1
#
#         elif fitness < 0:
#             fitness /= 10
#
#         return fitness
#     else:
#         return None

def displacement_velocity_hill(robot_manager, robot, cost=False):
    fitness = measures.displacement_velocity_hill(robot_manager)

    if fitness == 0 or robot.phenotype._morphological_measurements.measurements_to_dict()['hinge_count'] == 0:
        fitness = -0.1

    elif fitness < 0:
        fitness /= 10

    if cost and fitness != None:
        _size_penalty = size_penalty(robot_manager, robot) ** 2
        if fitness >= 0:
            fitness = fitness * _size_penalty
        else:
            fitness = fitness / _size_penalty

    return fitness

def displacement_velocity_hill_No_penalty(robot_manager, robot, cost=False):
    fitness = measures.displacement_velocity_hill(robot_manager)
    return fitness


def dvh_c_5050(robot_manager, robot, cost=False):
    fitness = measures.displacement_velocity_hill(robot_manager)
    contact = measures.contacts(robot_manager, robot)
    fitness = 0.5 * fitness + 0.5 * (1/(contact+1))
    if fitness == 0 or robot.phenotype._morphological_measurements.measurements_to_dict()['hinge_count'] == 0:
        fitness = -0.1

    elif fitness < 0:
        fitness /= 10

    if cost and fitness != None:
        _size_penalty = size_penalty(robot_manager, robot) ** 2
        if fitness >= 0:
            fitness = fitness * _size_penalty
        else:
            fitness = fitness / _size_penalty

    return fitness

def dvh_c_b_33(robot_manager, robot, cost=False):
    fitness = measures.displacement_velocity_hill(robot_manager)
    contact = measures.contacts(robot_manager, robot)
    balance = measures.head_balance(robot_manager)
    fitness = 0.33 * fitness + 0.33 * (1/(contact+1)) + (0.33 * balance)
    if fitness == 0 or robot.phenotype._morphological_measurements.measurements_to_dict()['hinge_count'] == 0:
        fitness = -0.1

    elif fitness < 0:
        fitness /= 10

    if cost and fitness != None:
        _size_penalty = size_penalty(robot_manager, robot) ** 2
        if fitness >= 0:
            fitness = fitness * _size_penalty
        else:
            fitness = fitness / _size_penalty

    return fitness

def dvh_c_b_701515(robot_manager, robot, cost=False):
    fitness = measures.displacement_velocity_hill(robot_manager)
    contact = measures.contacts(robot_manager, robot)
    balance = measures.head_balance(robot_manager)
    fitness = 0.7 * fitness + 0.15 * (1/(contact+1)) + (0.15 * balance)
    if fitness == 0 or robot.phenotype._morphological_measurements.measurements_to_dict()['hinge_count'] == 0:
        fitness = -0.1

    elif fitness < 0:
        fitness /= 10

    if cost and fitness != None:
        _size_penalty = size_penalty(robot_manager, robot) ** 2
        if fitness >= 0:
            fitness = fitness * _size_penalty
        else:
            fitness = fitness / _size_penalty

    return fitness


def floor_is_lava(robot_manager, robot, cost=False):
    _displacement_velocity_hill = displacement_velocity_hill(robot_manager, robot, cost)
    _contacts = measures.contacts(robot_manager, robot)

    _contacts = max(_contacts, 0.0001)
    if _displacement_velocity_hill >= 0:
        fitness = _displacement_velocity_hill / _contacts
    else:
        fitness = _displacement_velocity_hill * _contacts

    return fitness

def standingHeight(robot_manager, robot, cost=False):
    fitness = measures.standingHeight(robot_manager)
    return fitness
