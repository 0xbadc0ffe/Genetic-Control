import matlab.engine
import numpy as np
import random as rnd
import os
from my_gen_alg import population, bleb
import pyperclip
import copy
import platform




if platform.system() == 'Windows':
    CLEAR_STR = "cls" 
else:
    CLEAR_STR = "clear"
os.system("CLEAR_STR")

eng = matlab.engine.start_matlab()

#mat = matlab.double([[1,2,1],[1,2,3],[2,6,0]])
#tf = eng.det(mat)
#print(tf)

#a, b = eng.margin(eng.tf(matlab.double([1,1]),matlab.double([2,1,1])), nargout=2)
#print(a,b)


time_str = "t=0:0.04:20;"
eng.eval(time_str, nargout=0)

ref = matlab.double([1])


# DRONE Linearized Dynamics

Astr = 'A = [0 1 0 0 0 0; 0 0 -0.0106 0.0106 -0.0106 0.0106; 0 0 -10 0 0 0; 0 0 0 -10 0 0; 0 0 0 0 -10 0; 0 0 0 0 0 -10];'
Bstr = 'B = [0 ; 0 ; 1 ; -1 ; 1 ;-1];'
Cstr = 'C = [0 1 0 0 0 0];'



eng.eval(Astr,nargout=0)
A = np.matrix(eng.workspace['A'])

eng.eval(Bstr,nargout=0)
B = np.matrix(eng.workspace['B'])

eng.eval(Cstr,nargout=0)
C = np.matrix(eng.workspace['C'])

#print(A.size, B.size, C.size)

#print(eng.eval("size(B)"))
#print(eng.eval("size(C)"))

#print(eng.eval('C*A*B;'))
#print(C@A@B)

#print(eng.eval('step(tf([1],[1 1 1]),t)'))


eng.eval("[num, den] = ss2tf(A,B,C,0);", nargout=0)
eng.eval("proc = tf(num,den);", nargout=0)


print("\n## Initializing Variables:\n\n")
print(Astr)
print(Bstr)
print(Cstr)
print("[num, den] = ss2tf(A,B,C,0);")
print("proc = tf(num,den);")
print(time_str)
print()

pyperclip.copy(Astr+Bstr+Cstr+"[num, den] = ss2tf(A,B,C,0);"+"proc = tf(num,den);"+time_str)
print("Copied on clipboard\n\n")




def test_controller(G):
    eng.workspace['G'] = G
    #test = eng.eval("1-step(feedback(tf([20],[1 1]),1), t)")
    #print(eng.eval("tfdata(G, 'v');",nargout=2))
    test = eng.eval("1-step(feedback(G*proc, 1), t)")
    #print(test)
    sum = 0
    cnt = 0
    max_e = 0
    for t in test:
        cnt += 1
        sum += abs(t[0])     # Squared?
        if abs(t[0]) > max_e:
            max_e = t[0]
    
    res = sum/cnt + abs(max_e)     # maybe it is better to round a bit

    #print(sum,cnt,max_e)
    #print(f"RES: {res}")

    return res


def test_controller_2(G, ref=1):
    eng.workspace['G'] = G
    test = eng.eval(f"{ref}-step(feedback(G*proc, 1), t)")
    #print(test)
    sum = 0
    cnt = 0
    max_e = 0
    os_time = len(test)
    sw = False                  # True after  x(tk) > 0.95*ref
    for t in test:              # t[0] = ref - x(tk)
        cnt += 1
        mol = 2*cnt/(cnt+1)
        sum += mol*(t[0])**2 
        if t[0] < 0.05*ref:
            sw = True
            os_time = cnt
        if sw and abs(t[0]) > max_e:
            max_e = t[0]  
    N = 3
    Z = 1
    BpWeight = 2
    OvWeight = 2

    res = sum/cnt*(N+0.5*Z) + abs(max_e)*OvWeight + os_time/cnt*BpWeight

    #print(sum,cnt,max_e)
    #print(f"RES: {res}")

    return res

print("## Starting Test phase\n")
#test_controller(eng.eval("tf([1],[1 2 0]);"))


#choices = [ 0 for i in range(7)] + [j for j in range(100)]
choices = []
top = 100
for i in range(top):
    choices += [i for k in range(top-i)]
#print(choices)

#comm = f"tf([{rnd.randint(1,40)}],[1 {rnd.randint(1,40)} {rnd.randint(1,40)}]);"
comm = f"tf([{rnd.choice(choices)+0.5}],[1 {rnd.choice(choices)} {rnd.choice(choices)}]);"
print(comm)
test_controller(eng.eval(comm))

comm = f"tf([{rnd.choice(choices)+0.5}],[1 {rnd.choice(choices)} {rnd.choice(choices)}]);"
print(comm)
test_controller(eng.eval(comm))

comm = f"tf([{rnd.choice(choices)+0.5}],[1 {rnd.choice(choices)} {rnd.choice(choices)}]);"
print(comm)
test_controller(eng.eval(comm))


print("\nPress Enter to start\n")
input()



def selection(population, selected_numb):
    pop = population.copy()
    def fitness(elem):
        return elem.fitness
    
    list.sort(pop, key=fitness)
    #pop = [copy.deepcopy(pop[0])] + pop   # double the first bleb, better exploration of local minima
    #pop = [copy.deepcopy(pop[0])] + [pop[0]] + [copy.deepcopy(pop[1])] + pop[1:] # double the first 2 bleb , it is easier to stuck in local minima for lots of generations

    # subsitute only 2nd to avoid massive reproduction of best bleb (and so to be stuck in a local minima)
    #pop = [pop[0]] + [copy.deepcopy(pop[0])] + pop[2:]

    # double the first bleb and check if too much copies of best bleb are already present
    pop = [copy.deepcopy(pop[0])] + pop 
    c = 0
    cap = 1
    for b in pop[1:]:
        if b.name == pop[0].name:
            c += 1
        else:
            break # non consecutive same bleb is enough
    
    if c >= cap:
        pop = pop[:cap+1]+pop[c+1:]

 
    if selected_numb < len(pop):
        pop = pop[:selected_numb]
    return pop
    

def spawn_genes():
        return [rnd.choice(choices)+0.5, rnd.choice(choices), 0] #rnd.choice(choices)]                         # a/(s^2+bs +c)         a>0
        #return [rnd.choice(choices), rnd.choice(choices)+0.5, rnd.choice(choices), rnd.choice(choices)]     # (as +b)/(s^2 +cs +d)  b>0


def fitfunc(bleb):
    comm = f"tf([{bleb.genes[0]}],[1 {bleb.genes[1]} {bleb.genes[2]}]);"
    #comm = f"tf([{bleb.genes[0]}, {bleb.genes[1]}],[1 {bleb.genes[2]} {bleb.genes[3]}]);"
    return test_controller_2(eng.eval(comm))
                

def mutation(genes):
    new_genes = []
    for gene in genes:
        #new_genes.append(gene + rnd.uniform(-1,1))
        new_genes.append(gene + np.random.normal(0, 0.6))        # maybe is better to round a bit
    return new_genes


# crossover
def mix(gene1,gene2):
    genes = [gene1, gene2]
    new_gene = []
    mix_ind = [ rnd.randint(0,1) for i in range(len(gene2))]
    l2 = len(gene2)
    l1 = len(gene1)
    if l2<l1:
        mix_ind = mix_ind + [0]*(l1-l2)
    for i in range(len(gene1)):
        new_gene.append(genes[mix_ind[i]][i])
    return new_gene
    
def mating(population, parents_num):
    moth = population[0]
    fath = population[1]
    baby_bleb = bleb(mix(moth.genes, fath.genes), generation=moth.generation +1, name=moth.name[:4]+"f"+moth.name[5:]) 
    population.append(baby_bleb)        
            




pop = population(10, spawn_genes, mating, lambda x: selection(x,5))
pop.fit_all(fitfunc)
pop.print()
best_fit = 100
abs_best_fit = -1
abs_best = None
threshold = 0.01
gencap = 100000   # At most 50 generations
genmin = 500     # At least 50 generations
gen = 0
try:
    while abs_best_fit > threshold or gen < genmin:     
        print("\n\n-------------------\n\n")
        pop.evolve(fitfunc, mutation)
        if rnd.choice([0]*99+[1]) and abs_best: # 1% prob to resurrect abs_best
            res = copy.deepcopy(abs_best)
            res.name = res.name[:3]+"rb"+res.name[5:]
            pop.population.append(res)
        pop.fill()
        pop.fit_all(fitfunc)
        pop.print()
        print("\n")
        best = pop.pickbest()
        print(best)
        best_fit = best.fitness
        if best_fit < abs_best_fit or abs_best_fit < 0:
            abs_best = copy.deepcopy(best)
            abs_best_fit = best_fit
            # save top 10 ?
        print(abs_best)
        gen += 1 
        if gen > gencap: break
except KeyboardInterrupt:
    pass
except TypeError:
    pass

print("\n\nAbsolute best:\n")
print(abs_best)
#print(abs_best_fit)

bleb = abs_best
comm = f"tf([{bleb.genes[0]}],[1 {bleb.genes[1]} {bleb.genes[2]}]);"
#comm = f"tf([{bleb.genes[0]}, {bleb.genes[1]}],[1 {bleb.genes[2]} {bleb.genes[3]}]);"
show_comm = f"clf;step(feedback({comm[:-1]}*proc,1),t);hold on;fplot(1)"
print("\n" + show_comm)
#print(f"tf([{abs_best.genes[0]}],[1 {abs_best.genes[1]} {abs_best.genes[2]}])")

# figure(2)
# step(proc,t)
# figure(3)
# step(feedback(proc,1),t)

pyperclip.copy(show_comm)
print("\nCommand printed on clipboard!")

input()





### Saved

# bleb    7sp-69| Fitness:       1.1709484411176119| -78.46184454623621 1.249199967416723 0.9606306034872599
# clf;step(feedback(tf([-78.46184454623621],[1 1.249199967416723 0.9606306034872599])*proc,1),t);hold on;fplot(1)


# bleb   8sp-482| Fitness:       0.6707799334163863| -70.48672859950968 5.26519012108588 0.0423385412829278
# clf;step(feedback(tf([-70.48672859950968],[1 5.26519012108588 0.0423385412829278])*proc,1),t);hold on;fplot(1)


# bleb     9sp-5| Fitness:       1.1613309084650867| -98.38439609956664 1.3401518821750429 1.3403834602854936
# clf;step(feedback(tf([-98.38439609956664],[1 1.3401518821750429 1.3403834602854936])*proc,1),t);hold on;fplot(1)

#  FIT 2
# bleb  8sp-8244| Fitness:       0.7599189712932578| -122.95195182628723 1.8895051728276868 1.4402154698279206
# clf;step(feedback(tf([-122.95195182628723],[1 1.8895051728276868 1.4402154698279206])*proc,1),t);hold on;fplot(1)

# bleb  10sp-127| Fitness:       1.0740323395454352| -53.03561090509422 1.3047681234618307 0.9852505714946056
# clf;step(feedback(tf([-53.03561090509422],[1 1.3047681234618307 0.9852505714946056])*proc,1),t);hold on;fplot(1)

# blerf    9sp-1| Fitness:       0.4039846305311866| -95.00068180568024 1.520897151028731 1.1905035020891612
# clf;step(feedback(tf([-95.00068180568024],[1 1.520897151028731 1.1905035020891612])*proc,1),t);hold on;fplot(1)

#  FIT 3

# blerf    0or-0| Fitness:       1.3605484730260946| -132.46525976439324 1.5140943813292274 1.457086057683451
# clf;step(feedback(tf([-132.46525976439324],[1 1.5140943813292274 1.457086057683451])*proc,1),t);hold on;fplot(1)

# FIT 4

# blerf   10sp-2| Fitness:        2.665612491002541| -172.51216655986872 1.838478458712134 1.7589848851844672
# clf;step(feedback(tf([-172.51216655986872],[1 1.838478458712134 1.7589848851844672])*proc,1),t);hold on;fplot(1)




'''
A = [0 1 0 0 0 0; 0 0 -0.0106 0.0106 -0.0106 0.0106; 0 0 -10 0 0 0; 0 0 0 -10 0 0; 0 0 0 0 -10 0; 0 0 0 0 0 -10];
B = [0 ; 0 ; 1 ; -1 ; 1 ;-1];
C = [0 1 0 0 0 0];
[num, den] = ss2tf(A,B,C,0);
proc = tf(num,den);
t=0:0.04:20;
'''

# Fitness:

# FIT 1:
# Sum(|ref - x(t)|)/DelT + |max(ref-x(t))|
# Sum(t*(ref-x(t)))/[DelT*(sum(t))]*(N +1/2*Z) + |max(ref-x(t))|,  N num poles, Z num zeros
# FIT 2:
# Sum([2t/(t+1)]*(ref-x(t))^2)/[DelT*(sum(2t/(t+1))]*(N +1/2*Z) + |max(ref-x°(t))|,  N num poles, Z num zeros, |max(ref-x°(t))| defined only after x(tk)=ref for some tk (so it's the overshooting)
# FIT 3:
# Sum([2t/(t+1)]*(ref-x(t))^2)/[DelT*(sum(2t/(t+1))]*(N +1/2*Z) + |max(ref-x°(t))|+overshoot_time/DelT,  N num poles, Z num zeros, |max(ref-x°(t))| defined only after x(tk)=ref for some tk (so it's the overshooting)






