import numpy as np
import random as rd

class Individual:
    def __init__(self,genes,nb_genes):
        self.genes=genes
        self.nb_genes=nb_genes
        self.fitness=-1
    def random_init(self):
        self.genes=np.random.uniform(-5,5,self.nb_genes)

    def mutation(self,mutation_rate,mutation_scope):
        for i in range(self.nb_genes):
            if rd.uniform(0,1)>mutation_rate:
                self.genes[i]+=rd.uniform(-mutation_scope,mutation_scope)

    def get_fitness(self):#put the fitness function
        
        if self.fitness==-1:
            self.fitness=abs(np.prod(self.genes)+100)

class Generation:
    def __init__(self,individuals,nb_indi,mutation_rate,mutation_scope,nb_genes_indi):
        self.nb_indi=nb_indi
        self.individuals=individuals
        self.mutation_rate=mutation_rate
        self.mutation_scope=mutation_scope
        self.nb_genes_indi=nb_genes_indi
    
    def initial_gen(self,):
        for i in range(self.nb_indi):
            new_indi=Individual(np.array([]),self.nb_genes_indi)
            new_indi.random_init()
            self.individuals.append(new_indi)
        
    def cross_over(self,elite_rate):
        new_childs=[]
        elites=self.elitism(elite_rate)
        nb_parent=int(self.nb_indi-len(elites))
        for i in range(nb_parent):
            a=rd.randint(0,len(elites))
            b=rd.randint(0,len(elites))
            slice_a=rd.randint(1,self.nb_genes_indi-1)
            new_child_genes=np.concatenate((self.individuals[a].genes[0:slice_a],self.individuals[a].genes[slice_a:self.nb_genes_indi]),axis=None)

            child=Individual(new_child_genes,self.nb_genes_indi)
            child.mutation(self.mutation_rate,self.mutation_scope)
            new_childs.append(child)
            new_childs
        return new_childs

    def elitism(self,elite_rate):
        self.individuals.sort(key=lambda x: x.fitness, reverse=False)
        elites=self.individuals[0:(int(elite_rate*self.nb_indi))]
        for i in range(int(elite_rate*self.nb_indi)):
            elites
        return elites

    def get_fitness_gen(self):
        for i in range(self.nb_indi):
            self.individuals[i].get_fitness()


class Training:
    def __init__(self):
        self.old_generations=[]
        self.nb_indi_gen=10000
        self.nb_genes_indi=10
        self.current_gen_num=1
        self.mutation_rate=0.01
        self.mutation_scope=0.01
        self.elite_rate=0.1
        self.current_gen=Generation([],self.nb_indi_gen,self.mutation_rate,self.mutation_scope,self.nb_genes_indi)

    def nextGeneration(self):
        new_childs=self.current_gen.cross_over(self.elite_rate)
        self.old_generations.append(self.current_gen)
        elites=self.current_gen.elitism(self.elite_rate)

        new_pop=elites+new_childs
        
        print("best",new_pop[0].fitness)
        #for i in range(self.nb_indi_gen):
        print("best",new_pop[0].genes)
        new_gen=Generation(new_pop,self.nb_indi_gen,self.mutation_rate,self.mutation_scope,self.nb_genes_indi)
        
        self.current_gen=new_gen
        self.current_gen_num+=1

    def stop_condition(self):
        # to modify until it I settle it 
        return self.current_gen_num<100


    def doTraining(self):
        self.current_gen.initial_gen()
        while(self.stop_condition()):
            print(self.current_gen_num)
            self.current_gen.get_fitness_gen()
            self.nextGeneration()
            
            
            

train=Training()
train.doTraining()