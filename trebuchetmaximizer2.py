import random

import genetic
import treblink
import numpy as np

f = open("output.data", 'a', buffering = 0)

class TrebMax(genetic.Individual):
    optimization = genetic.MAXIMIZE
    def __init__(self, chromosome=None):
        self.chromosome = chromosome or self._makechromosome()
        self.chromosome.mutate()
        self.score = None  # set during evaluation
    def _makechromosome(self):
        return treblink.LinkTrebuchet([treblink.TrebLink(random.random()*10, 1+random.random()*9, []).mutate() for a in range(5)]) 
    def evaluate(self, optimum=None):
        print self.chromosome
        self.score = self.chromosome.evaluate()
        if self.score > .0001:
            f.write("\n" + str(self.chromosome) + '\n' + str(self.score))
                     
    def mutate(self, gene):
        self.chromosome = self.chromosome.mutate()
    def crossover(self, other):
        return [TrebMax(a) for a in self.chromosome.crossover(other.chromosome)]
    def copy(self):
        twin = TrebMax(self.chromosome)
        twin.score = self.score
        return twin 
    def __repr__(self):
        return self.chromosome.__repr__()
   
if __name__ == "__main__":
    env = genetic.Environment(TrebMax, maxgenerations=4000, optimum=1.0, mutation_rate=.017, size = 100)
    env.run()
