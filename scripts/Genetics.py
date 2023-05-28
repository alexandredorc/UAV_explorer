import numpy as np
import random as rd
import json
import os

class Individual:
    def __init__(self,genes,nb_genes,generation_nb):
        self.genes=genes
        self.nb_genes=nb_genes
        self.fitness=-1
        self.generation_nb=generation_nb
        self.step_count=0.111
        self.time=0.222
        self.explo_rate=0.333

    def random_init(self):
        self.genes=np.random.uniform(-1,1,self.nb_genes)

    def mutation(self,mutation_rate,mutation_scope):
        for i in range(self.nb_genes):
            if rd.uniform(0,1)>mutation_rate:
                self.genes[i]+=rd.uniform(-mutation_scope,mutation_scope)

    def get_fitness(self):#put the fitness function
        
        if self.fitness==-1:
            self.fitness=1/abs(np.prod(self.genes)+100)

class Generation:
    def __init__(self,individuals,nb_indi,mutation_rate,mutation_scope,nb_genes_indi,gen_nb):
        self.nb_indi=nb_indi
        self.individuals=individuals
        self.mutation_rate=mutation_rate
        self.mutation_scope=mutation_scope
        self.nb_genes_indi=nb_genes_indi
        self.gen_nb=gen_nb
        self.global_fitness=-1
    
    def initial_gen(self,):
        for i in range(self.nb_indi):
            new_indi=Individual(np.array([]),self.nb_genes_indi,self.gen_nb)
            new_indi.random_init()
            self.individuals.append(new_indi)
        
    def selection(self):
        idx=-1
        pop_fitness=[]
        for indi in self.individuals:
            pop_fitness.append(indi.fitness)
        pop_fitness/=np.sum(np.array(pop_fitness))
        cumul_prob=np.ndarray.cumsum(pop_fitness)
        random_num=rd.random()
        for i in range(self.nb_indi):
            if cumul_prob[i]>random_num:
                idx=i
                break
        return self.individuals[idx]
    
    def cross_over(self,elite_rate):
        new_childs=[]
        for i in range(int(self.nb_indi*(1-elite_rate))):
            a=self.selection()
            b=self.selection()
            new_child_genes=[]
            for g in range(self.nb_genes_indi):
                if rd.random()<0.5:
                    new_child_genes.append(a.genes[g])
                else:
                    new_child_genes.append(b.genes[g])

            child=Individual(np.array(new_child_genes),self.nb_genes_indi,self.gen_nb)
            child.mutation(self.mutation_rate,self.mutation_scope)
            new_childs.append(child)
            new_childs
        return new_childs

    def elitism(self,elite_rate):
        self.individuals.sort(key=lambda x: x.fitness, reverse=True)
        elites=self.individuals[0:(int(elite_rate*self.nb_indi))]
        for i in range(int(elite_rate*self.nb_indi)):
            elites
        return elites

    def get_fitness_gen(self):
        self.global_fitness=0
        for i in range(self.nb_indi):
            self.individuals[i].get_fitness()
            self.global_fitness+= self.individuals[i].fitness
        self.global_fitness/=self.nb_indi


class Training:
    def __init__(self):
        self.max_steps=500
        self.stag_threshold=0.1
        self.stag_generation_nb=5
        self.stage_gen_count=0
        self.old_generations=[]
        self.nb_indi_gen=50
        self.nb_genes_indi=10
        self.current_gen_num=1
        self.mutation_rate=0.01
        self.mutation_scope=1
        self.elite_rate=0.1
        self.prev_pop_fitness=-1
        self.current_gen=Generation([],self.nb_indi_gen,self.mutation_rate,self.mutation_scope,self.nb_genes_indi,self.current_gen_num)

    def nextGeneration(self):
        new_childs=self.current_gen.cross_over(self.elite_rate)
        self.prev_pop_fitness=self.current_gen.global_fitness
        self.old_generations.append(self.current_gen)
        elites=self.current_gen.elitism(self.elite_rate)

        new_pop=elites+new_childs
        
        print("best fit",new_pop[0].fitness)
        #for i in range(self.nb_indi_gen):
        print("best",new_pop[0].genes)
        new_gen=Generation(new_pop,self.nb_indi_gen,self.mutation_rate,self.mutation_scope,self.nb_genes_indi,self.current_gen_num)
        
        print("best", abs(np.prod(new_pop[0].genes)))
        self.current_gen=new_gen
        self.current_gen_num+=1

    def stop_condition(self):
        if self.current_gen.global_fitness-self.prev_pop_fitness<self.stag_threshold and self.current_gen.global_fitness>self.stag_threshold:
            self.stage_gen_count+=1
        else:
            self.stage_gen_count=0
        print(self.current_gen.global_fitness)
        if self.stage_gen_count>=self.stag_generation_nb:
            return True
        
        return False


    def doTraining(self):
        self.current_gen.initial_gen()
        self.start_session_to_json()
        while(True):
            print(self.current_gen_num)
            self.current_gen.get_fitness_gen()
            self.write_data_to_json()
            if self.stop_condition():
                break
            self.nextGeneration()
            
            
    def start_session_to_json(self):
        file_path ="data_UAV.json"
        print(file_path)
        if not os.path.isfile(file_path):
            #Create an empty file if it doesn't exist
            with open(file_path, 'w') as file:
                json.dump({"sessions": []}, file)
            
        
        with open("data_UAV.json") as file:
            data = json.load(file)
        
        cur_session_idx=len(data["sessions"])+1

      
        cur_session={
            "session_idx": cur_session_idx,
            "map":[
                {
                    "map_file": "map_example.dae",
                    "start_gen": 0
                }
            ],
            "params": {
                "individuals_nb": self.nb_indi_gen,
                "max_steps": self.max_steps,
                "mutation_rate": self.mutation_rate,
                "stag_threshold": self.stag_threshold,
                "stag_generation_nb": self.stag_generation_nb,
                "elitism_rate": self.elite_rate
            },
            "generations": []
        }

        data["sessions"].append(cur_session)
        with open(file_path, 'w') as file:
            json.dump(data, file, indent=4)
            
    def write_data_to_json(self):
        file_path = "data_UAV.json"
        with open(file_path, 'r') as file:
            data = json.load(file)
        gen= self.current_gen
        pop_json=[]
        for indi in gen.individuals:
            indi_json={
                "indi_idx": {
                    "genes": list(indi.genes),
                    "fitness": indi.fitness,
                    "step_count": indi.step_count,
                    "time": indi.step_count,
                    "exploration_rate": indi.explo_rate,
                    "generation_create": indi.generation_nb
                }
            }
            pop_json.append(indi_json)
                
            
        gen_json = {
            "gen_nb": self.current_gen_num,
            "individuals": pop_json
        }
        
        
        data["sessions"][-1]["generations"].append(gen_json)
        with open(file_path, 'w') as file:
            json.dump(data, file, indent=4)
        
        print(f"Data successfully written to file")

