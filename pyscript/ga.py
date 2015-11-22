#!/usr/bin/env python

import argos_util
import subprocess
import csv
import tempfile
import os
import numpy as np
import time
import argparse
import errno
import copy
from lxml import etree
import logging

# http://stackoverflow.com/questions/600268/mkdir-p-functionality-in-python
def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

class ArgosRunException(Exception):
    pass


class iAntGA(object):
    def __init__(self, pop_size=50, gens=20, elites=3,
                 mut_rate=0.1, robots=20, length=300,
                 system="linux", tests_per_gen=10):
        self.system = system
        self.pop_size = pop_size
        self.gens = gens
        self.elites = elites
        self.mut_rate = mut_rate
        self.current_gen = 0
        # Initialize population
        self.population = []
        self.prev_population = None
        self.system = system
        self.fitness = np.zeros(pop_size)
        self.starttime = int(time.time())
        self.length = length
        self.tests_per_gen = tests_per_gen
        dirstring = str(self.starttime) + "_e_" + str(elites) + "_p_" + str(pop_size) + "_r_" + str(robots) + "_t_" + \
                    str(length) + "_k_" + str(tests_per_gen)
        self.save_dir = os.path.join("gapy_saves", dirstring)
        mkdir_p(self.save_dir)
        logging.basicConfig(filename=os.path.join(self.save_dir,'iAntGA.log'),level=logging.DEBUG)

        for _ in xrange(pop_size):
            self.population.append(argos_util.uniform_rand_argos_xml(robots, length, system))

    def test_fitness(self, argos_xml, seed):
        argos_util.set_seed(argos_xml, seed)
        xml_str = etree.tostring(argos_xml)
        cwd = os.getcwd()
        tmpf = tempfile.NamedTemporaryFile('w', suffix=".argos", prefix="gatmp",
                                           dir=os.path.join(cwd, "experiments"),
                                           delete=False)
        tmpf.write(xml_str)
        tmpf.close()
        argos_args = ["argos3", "-n", "-c", tmpf.name]
        argos_run = subprocess.Popen(argos_args, stdout=subprocess.PIPE)
        # Wait until argos is finished
        while argos_run.poll() is None:
            time.sleep(0.5)
        if argos_run.returncode != 0:
            logging.error("Argos failed test")
            # when argos fails just return fitness 0
            return 0
        lines = argos_run.stdout.readlines()
        os.unlink(tmpf.name)
        print lines[-1]
        logging.info("partial fitness = %d", int(lines[-1].strip().split(",")[0]))
        return int(lines[-1].strip().split(",")[0])

    def run_ga(self):
        while self.current_gen <= self.gens:
            self.run_generation()

    def run_generation(self):
        logging.info("Starting generation: " + str(self.current_gen))
        self.fitness = np.zeros(pop_size)
        seeds = [np.random.randint(2 ** 32) for _ in range(self.tests_per_gen)]
        logging.info("Seeds for generation: " + str(seeds))

        for i, p in enumerate(self.population):
            for test_id in xrange(self.tests_per_gen):
                seed = seeds[test_id]
                logging.info("pop %d at test %d with seed %d", i, test_id, seed)
                self.fitness[i] += self.test_fitness(p, seed)
        # use average fitness as fitness
        for i in xrange(len(self.fitness)):
            logging.info("pop %d total fitness = %g", i, self.fitness[i])
            self.fitness[i] /= self.tests_per_gen
            logging.info("pop %d avg fitness = %g", i, self.fitness[i])

        # sort fitness and population
        fitpop = sorted(zip(self.fitness, self.population), reverse=True)
        self.fitness, self.population = map(list, zip(*fitpop))

        self.save_population(seed)

        self.prev_population = self.population

        self.population = []

        # Add elites
        for i in xrange(self.elites):
            # reverse order from sort
            self.population.append(self.prev_population[i])

        # Now do crossover and mutation until population is full
        for i in xrange(self.pop_size - self.elites):
            p1c = np.random.choice(len(self.prev_population), 2)
            p2c = np.random.choice(len(self.prev_population), 2)

            parent1 = self.prev_population[p1c[1]]
            if self.fitness[p1c[0]] > self.fitness[p1c[1]]:
                parent1 = self.prev_population[p1c[0]]

            parent2 = self.prev_population[p2c[1]]
            if self.fitness[p2c[0]] > self.fitness[p2c[1]]:
                parent1 = self.prev_population[p2c[0]]

            child = argos_util.uniform_crossover(parent1, parent2, self.system)

            argos_util.mutate_cpfa(child, self.mut_rate)
            self.population.append(child)

        self.current_gen += 1

    def save_population(self, seed):
        save_dir = self.save_dir
        mkdir_p(save_dir)
        filename = "gen_%d.gapy" % self.current_gen
        population_data = []
        for f, p in zip(self.fitness, self.population):
            data = copy.deepcopy(argos_util.get_cpfa(p))
            data["fitness"] = f
            data["seed"] = seed
            population_data.append(data)
            print data
        data_keys = argos_util.CPFA_LIMITS.keys()
        data_keys.append("fitness")
        data_keys.append("seed")
        data_keys.sort()
        with open(os.path.join(save_dir, filename), 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=data_keys, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(population_data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='GA for argos')
    parser.add_argument('-s', '--system', action='store', dest='system')
    parser.add_argument('-r', '--robots', action="store", dest="robots", type=int)
    parser.add_argument('-m', '--mut_rate', action='store', dest='mut_rate', type=float)
    parser.add_argument('-e', '--elites', action='store', dest='elites', type=int)
    parser.add_argument('-g', '--gens', action='store', dest='gens', type=int)
    parser.add_argument('-p', '--pop_size', action='store', dest='pop_size', type=int)
    parser.add_argument('-t', '--time', action='store', dest='time', type=int)
    parser.add_argument('-k', '--tests_per_gen', action='store', dest='tests_per_gen', type=int)


    pop_size = 50
    gens = 20
    elites = 1
    mut_rate = 0.1
    robots = 10
    system = "linux"
    length = 3600
    tests_per_gen=10

    args = parser.parse_args()

    if args.pop_size:
        pop_size = args.pop_size

    if args.gens:
        gens = args.gens

    if args.elites:
        elites = args.elites

    if args.mut_rate:
        mut_rate = args.mut_rate

    if args.robots:
        robots = args.robots

    if args.system:
        system = args.system

    if args.time:
        length = args.time

    if args.tests_per_gen:
        tests_per_gen = args.tests_per_gen

    ga = iAntGA(pop_size=pop_size, gens=gens, elites=elites, mut_rate=mut_rate,
                robots=robots, length=length, system=system, tests_per_gen=tests_per_gen)

    ga.run_ga()