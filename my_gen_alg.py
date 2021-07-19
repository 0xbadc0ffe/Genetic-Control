import random
import string

class genes:
    def __init__(self, genes):
        self.genes = genes
        
    def __str__(self):
        return " ".join(str(gene) for gene in self.genes)


class bleb:

    def __init__(self, genes, generation=None, fitness=None, name="generic bleb", data=None, str_type=1):
        self.genes = genes
        self.generation = generation
        self.fitness = fitness
        self.name = name
        self.data = data
        self.str_type = str_type
        
    def fit(self, fitfunc):
        self.fitness = fitfunc(self) # self.genes
    
    def mutate(self, mutation):
        new_genes = []
        for gene in self.genes:
            gene = mutation(gene)
            new_genes.append(gene)
        self.genes = new_genes
            
    def mutate_cust(self, mutation):    
        self.genes = mutation(self.genes)
    
    def __str__(self):
        if self.fitness is None:
            fitt = "None"
           # return self.name+"| Fitness:"+" "*(25-len(str(fitt)))+fitt+ "| " + " ".join(str(gene) for gene in self.genes)
        else:
            fitt = str(self.fitness)
        if self.str_type == 1:
            return self.name+"| Fitness:"+" "*(25-len(str(fitt)))+fitt+ "| " + " ".join(str(gene)[:min(8,len(str(gene)))] for gene in self.genes)
        elif self.str_type == 2:
            zstr = "["+" ".join(str(gene)[:min(8,len(str(gene)))] for gene in self.genes[0])+"]"
            pstr = "["+" ".join(str(gene)[:min(8,len(str(gene)))] for gene in self.genes[1])+"]"
            return self.name+"| Fitness:"+" "*(25-len(str(fitt)))+fitt+ "| " + zstr+", "+pstr
        #return self.name+"| Fitness:"+" "*(25-len(str(fitt)))+fitt+ "| "+str(len(self.genes))
        #return self.name+"| Fitness:"+" "*(25-len(str(fitt)))+fitt+ "| " + " ".join(str(self.genes[key]) for key in self.genes)
        
        
class population:

    def __init__(self, pop_number, spawn_genes, mating, selection, generation=0, st_population=None, str_type=1):
        self.population = []
        self.generation = generation
        self.pop_number = pop_number
        self.spawn_genes = spawn_genes
        self.str_type=str_type
        if st_population is None:
            for i in range(self.pop_number):
                self.population.append(bleb(spawn_genes(), generation=self.generation, name="bleb"+" "*(8-len(hex(i)[2:])-len(str(self.generation)))+hex(i)[2:]+"or-"+str(self.generation)+" "*6, str_type=self.str_type))            
        else:
            self.population = st_population
        
        self.selection = selection
        self.mating = mating
    
    def fit_all(self, fitfunc):
        for bleb in self.population:
            bleb.fit(fitfunc)
        
    def evolve(self, fitfunc, mutation):
        self.fit_all(fitfunc)
        self.population = self.selection(self.population)
        self.mating(self.population, 2, self.generation)
        self.generation += 1
        for bleb in self.population:
            #bleb.mutate(mutation)
            bleb.mutate_cust(mutation)
            bleb.generation += 1
        
            
    def fill(self):
        to_fill = self.pop_number - len(self.population)
        if to_fill < 0:
            raise Exception("popultation outgrowing")
        elif to_fill > 0:
            for i in range(to_fill):
                self.population.append(bleb(self.spawn_genes(), generation=self.generation, name="bleb"+" "*(8-len(hex(self.pop_number-i)[2:])-len(str(self.generation)))+str(hex(self.pop_number-i)[2:])+"sp-"+str(self.generation)+" "*6, str_type=self.str_type))
                # name =bleb/r{f} hex(i){s/o}-gen(gen_f_b64)
    
    def pickbest(self):
        return selection(self.population, 1)[0]
        
    
    def print(self):
        print("Generation: "+str(self.generation)+"\n")
        for b in self.population:
            print(str(b))


    
                    
                    
def selection(population, selected_numb):
    pop = population.copy()
    def fitness(elem):
        return elem.fitness
        

    # 1
    #list.sort(pop, key=fitness, reverse=True)
    # 2
    list.sort(pop, key=fitness)
    if selected_numb < len(pop):
        pop = pop[:selected_numb]
    return pop
    

def mating(population, parents_num):
    '''
    if parents_num%2 !=0:
        parents_num = parents_num-1
    parents_list = population[:parents_num]
    for i in range(0,len(parent_list),2):
    ''' 
        
    pass
            

def spawn_genes():
        return [random.choice(string.printable), random.choice(string.printable)]
        #return genes([random.choice(string.printable), random.choice(string.printable)])
        
def fitfunc(bleb):
    # 1
    #return ord(bleb.genes[0]) + ord(bleb.genes[1])
    # 2
    return abs(ord(bleb.genes[0]) - ord(bleb.genes[1]))
                

def mutation(genes):
    new_genes = []
    for gene in genes:
        new_genes.append(chr((ord(gene)+random.randint(-10, 10))%128))
    return new_genes

    

if __name__=="__main__":
    pop = population(10, spawn_genes, mating, lambda x: selection(x,5))
    pop.fit_all(fitfunc)
    pop.print()
    best_fit = 1
    while best_fit != 0:
        print("\n\n-------------------\n\n")
        pop.evolve(fitfunc, mutation)
        pop.fill()
        pop.fit_all(fitfunc)
        pop.print()
        print("\n")
        best = pop.pickbest()
        print(best)
        best_fit = best.fitness
    print()
    




        
        
        