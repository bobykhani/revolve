#!/usr/bin/env python3
import asyncio
from pygazebo.pygazebo import DisconnectError

from pyrevolve import parser
from pyrevolve.evolution import fitness
from pyrevolve.evolution.selection import multiple_selection, tournament_selection
from pyrevolve.evolution.population import Population, PopulationConfig
from pyrevolve.evolution.learning import Learning
from pyrevolve.evolution.pop_management.steady_state import steady_state_population_management
from pyrevolve.experiment_management import ExperimentManagement
from pyrevolve.genotype.plasticoding.crossover.crossover import CrossoverConfig
from pyrevolve.genotype.plasticoding.crossover.standard_crossover import standard_crossover
from pyrevolve.genotype.plasticoding.initialization import random_initialization
from pyrevolve.genotype.plasticoding.mutation.mutation import MutationConfig
from pyrevolve.genotype.plasticoding.mutation.standard_mutation import standard_mutation
from pyrevolve.genotype.plasticoding.plasticoding import PlasticodingConfig
from pyrevolve.util.supervisor.simulator_simple_queue import SimulatorSimpleQueue
from pyrevolve.custom_logging.logger import logger


async def run():
    """
    The main coroutine, which is started below.
    """

    # experiment params #
    num_generations = 100
    population_size = 100
    offspring_size = 50

    genotype_conf = PlasticodingConfig(
        max_structural_modules=100,
    )

    mutation_conf = MutationConfig(
        mutation_prob=0.8,
        genotype_conf=genotype_conf,
    )

    crossover_conf = CrossoverConfig(
        crossover_prob=0.8,
    )
    # experiment params #

    # Parse command line / file input arguments
    settings = parser.parse_args()
    experiment_management = ExperimentManagement(settings)
    do_recovery = settings.recovery_enabled and not experiment_management.experiment_is_new()

    if do_recovery:
        gen_num, has_offspring, next_robot_id = experiment_management.read_recovery_state(population_size, offspring_size)

        if gen_num == num_generations-1:
            logger.info('Experiment is already complete.')
            return
    else:
        gen_num = 0
        next_robot_id = 1

    population_conf = PopulationConfig(
        population_size=population_size,
        genotype_constructor=random_initialization,
        genotype_conf=genotype_conf,
        fitness_function=fitness.displacement_velocity_hill,
        mutation_operator=standard_mutation,
        mutation_conf=mutation_conf,
        crossover_operator=standard_crossover,
        crossover_conf=crossover_conf,
        selection=lambda individuals: tournament_selection(individuals, 2),
        parent_selection=lambda individuals: multiple_selection(individuals, 2, tournament_selection),
        population_management=steady_state_population_management,
        population_management_selector=tournament_selection,
        evaluation_time=settings.evaluation_time,
        offspring_size=offspring_size,
        experiment_name=settings.experiment_name,
        experiment_management=experiment_management,
        measure_individuals=settings.measure_individuals,
        perform_learning=True,
        max_learn_evals=500
    )

    settings = parser.parse_args()
    simulator_queue = SimulatorSimpleQueue(settings.n_cores, settings, settings.port_start)
    await simulator_queue.start()

    population = Population(population_conf, simulator_queue, next_robot_id)

    if do_recovery:
        # loading a previous state of the experiment
        await population.load_snapshot(gen_num)
        logger.info('Recovered snapshot '+str(gen_num))
        if has_offspring:
            individuals = await population.load_offspring(gen_num, population_size, offspring_size, next_robot_id)
            gen_num += 1
            logger.info('Recovered unfinished offspring '+str(gen_num))

            # delete existing files for unevaluated robots
            experiment_management.delete_robot_learn_files(next_robot_id)

            if gen_num == 0:
                await population.init_pop(individuals)
            else:
                population = await population.next_gen(gen_num, individuals)

            experiment_management.export_snapshots(population.individuals, gen_num)
    else:
        # starting a new experiment
        experiment_management.create_exp_folders()
        await population.init_pop()
        experiment_management.export_snapshots(population.individuals, gen_num)

    while gen_num < num_generations-1:
        gen_num += 1
        population = await population.next_gen(gen_num)
        experiment_management.export_snapshots(population.individuals, gen_num)

    # output result after completing all generations...
