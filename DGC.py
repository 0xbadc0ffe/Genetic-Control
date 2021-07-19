import matlab.engine
import numpy as np
import random as rnd
import os
from my_gen_alg import population, bleb
import pyperclip
import copy
import platform
import base64
    

if platform.system() == 'Windows':
    CLEAR_STR = "cls" 
else:
    CLEAR_STR = "clear"
os.system(CLEAR_STR)

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
Cstr = 'C = [1 0 0 0 0 0];'



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


#val_choices = [ 0 for i in range(7)] + [j for j in range(100)]
val_choices = []
top = 100
for i in range(top):
    val_choices += [i for k in range(top-i)]
    #val_choices += [-i for k in range(top-i)] # neg values





class Controller:
    
    def __init__(self, zeros, poles, fixpoles=True):
        self.poles = poles
        self.zeros = zeros
        self.fixpoles = fixpoles
        
       
    def numpoles(self):
        if self.fixpoles:
            return len(self.poles)
        else:
            return len(self.poles)-1
        
    def numzeros(self):
        return len(self.zeros)-1
        
    def matstr(self):
        zerostr = "[" + " ".join(str(z) for z in self.zeros)+"], "
        if self.fixpoles:
            polestr = "[ 1 " + " ".join(str(p) for p in self.poles)+"]"
        else:
            polestr = "[" + " ".join(str(p) for p in self.poles)+"]"
        return zerostr+polestr
        
    def __str__(self):
        return "Z:["+" ".join(str(z) for z in self.zeros)+"] P:["+" ".join(str(p) for p in self.poles)+"]"
    
    def tf_str(self):
        zerostr = ""
        cnt = len(self.zeros)-1
        # if cnt==0 remove s^0?
        for z in self.zeros:
            if z > 0:
                zerostr += f" +{z}*s^{cnt}"
            else:
                zerostr += f" {z}*s^{cnt}"
            cnt -= 1
        cnt = len(self.poles)-1
        if self.fixpoles:
            polestr = f"s^{cnt+1}"
        else:
            polestr = ""
        for p in self.poles:
            if p > 0:
                polestr += f" +{p}*s^{cnt}"
            else:
                polestr += f" {p}*s^{cnt}"
            cnt -= 1
        return zerostr + "\n"+"-"*max(len(zerostr),len(polestr)) +"\n" +polestr
   
def randController(dim_range=7, fix=False, N=0, P=1, val_range=100, strict=False):

    #types = ["s+{}"]*20 + ["(s+{})^2 +{}^2"]*2+["s"]*3   # 80% binomies, 8% complex trinomies, 12% zero-monomies
    types = ["bin"]*20 +["s"]*3 + ["comp"]*2
    zeros = [1]
    poles = [1]
    if not fix:
        if(dim_range < 1):
            return Controller([1],[np.random.normal(1, val_range)], fixpoles=False)
        #P = rnd.randint(0,dim_range)
        P = int(np.random.normal(0,dim_range)%dim_range)
        if strict:
            N = rnd.randint(0,P-1)
        else:
            N = rnd.randint(0,P)
    if dim_range<0 or N<0 or P<0:
        raise Exception("Negative values for Controller dimension")
    if N>P:
        raise Exception("Non causal controller")
    #print(P,N)
    z = 0
    while z < N:
        if N-z > 1:
            ch = rnd.choice(types)
        else:
            ch = rnd.choice(types[:-2])
        if ch == "s":
            pol=[1, 0]
        elif ch == "bin":
            pol=[1, rnd.choice([i for i in range(-val_range,val_range)])]
        elif ch == "comp":
            z += 1
            a = rnd.choice([i for i in range(-val_range,val_range)])
            w = rnd.choice([i for i in range(-val_range,val_range)])
            pol=[1, 2*a, a*a+w*w]
        z += 1
        zeros = np.convolve(zeros, pol)
    zeros = np.convolve(zeros, [np.random.normal(1, val_range)]) # Gain
    p = 0
    while p < P:
        if P-p > 1:
            ch = rnd.choice(types)
        else:
            ch = rnd.choice(types[:-2])
        if ch == "s":
            pol=[1, 0]
        elif ch == "bin":
            pol=[1, rnd.choice([i for i in range(-val_range,val_range)])]
        elif ch == "comp":
            p += 1
            a = rnd.choice([i for i in range(-val_range,val_range)])
            w = rnd.choice([i for i in range(-val_range,val_range)])
            pol=[1, 2*a, a*a+w*w]
        p += 1
        poles = np.convolve(poles, pol)
    return Controller(zeros, poles, fixpoles=False)
        


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


def test_controller_2(G, ref=1, Z=1, N=1):

    # for >= 1 dimensional controllers only
    #if N == 0:
    #    return 10000

    eng.workspace['G'] = G
    test = eng.eval(f"{ref}-step(feedback(G*proc, 1), t)")
    #print(test)
    sum = 0
    cnt = 0
    max_e = 0
    os_time = len(test)
    sw = False                  # True after  x(tk) > 0.95*ref
    for t in test:              # t[0] = ref - x(tk)
        try:
            cnt += 1
            mol = 2*cnt/(cnt+1)
            sum += mol*(t[0])**2 
            if t[0] < 0.05*ref:
                sw = True
                os_time = cnt
            if sw and abs(t[0]) > max_e:
                max_e = t[0] 
        except OverflowError:
            return 1000000000

    Zp = 0.5
    Np = 1
    BpWeight = 2
    OvWeight = 16
    IntWeigth = 1
    #if os_time/cnt < 0.25: 
    #    os_time=0.25*cnt + os_time
    if not sw:
        max_e = ref
    res = IntWeigth*sum/cnt*(Np*(N+1)+Zp*(Z+1)) + abs(max_e)*OvWeight + os_time/cnt*BpWeight   

    #print(sum,cnt,max_e)
    #print(f"RES: {res}")

    return res
    

def TestingPhase():

    print("## Starting Test phase\n")
    #test_controller(eng.eval("tf([1],[1 2 0]);"))


    #comm = f"tf([{rnd.randint(1,40)}],[1 {rnd.randint(1,40)} {rnd.randint(1,40)}]);"
    comm = f"tf([{rnd.choice(val_choices)+0.5}],[1 {rnd.choice(val_choices)} {rnd.choice(val_choices)}]);"
    print(comm)
    test_controller(eng.eval(comm))

    comm = f"tf([{rnd.choice(val_choices)+0.5}],[1 {rnd.choice(val_choices)} {rnd.choice(val_choices)}]);"
    print(comm)
    test_controller(eng.eval(comm))

    comm = f"tf([{rnd.choice(val_choices)+0.5}],[1 {rnd.choice(val_choices)} {rnd.choice(val_choices)}]);"
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
    #return [rnd.choice(val_choices)+0.5, rnd.choice(val_choices), 0] #rnd.choice(val_choices)]                         # a/(s^2+bs +c)         a>0
    #return [rnd.choice(val_choices), rnd.choice(val_choices)+0.5, rnd.choice(val_choices), rnd.choice(val_choices)]     # (as +b)/(s^2 +cs +d)  b>0
    #return [randController()] # ones should implement mutation and crossover inside the Controller class
    rc = randController()
    return [rc.zeros, rc.poles]

def fitfunc(bleb):
    #comm = f"tf([{bleb.genes[0]}],[1 {bleb.genes[1]} {bleb.genes[2]}]);"
    #comm = f"tf([{bleb.genes[0]}, {bleb.genes[1]}],[1 {bleb.genes[2]} {bleb.genes[3]}]);"
    #comm = f"tf({bleb.genes[0].matstr()})"
    comm = f"tf({bleb.genes[0]},{bleb.genes[1]});".replace("\n","")
    Z = len(bleb.genes[0])-1
    N = len(bleb.genes[1])-1
    return test_controller_2(eng.eval(comm),Z=Z,N=N)
                

def mutation(genes):
    mut_rate = 0.1
    gene1 = []
    for gene in genes[0]:
        #new_genes.append(gene + rnd.uniform(-1,1))
        gene1.append(gene + np.random.normal(0, abs(gene*mut_rate)))   
    gene2 = []
    for gene in genes[1]:
        #new_genes.append(gene + rnd.uniform(-1,1))
        gene2.append(gene + np.random.normal(0, abs(gene*mut_rate))) 
    return [gene1,gene2]


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
  
# crossover zero-poles
def mix_zp(gene1,gene2):
    if len(gene2[0]) > len(gene1[1]):
        return [gene1[0],gene2[1]]
    else:
        return [gene2[0],gene1[1]]
    
  
def mating(population, parents_num, gen, inc=False):
    moth = population[0]
    fath = population[1]
    if not inc:
        i = 2
        try:
            while moth.name[:-5] == fath.name[:-5]:
                fath = population[i]               
                i += 1
        except IndexError:
            return
            #fath = population[2]
    numgen = base64.b64encode(gen.to_bytes(3,"big")).decode("ascii")
    # recover generation from numgen ---- => int.from_bytes(base64.b64decode(----), "big")
    baby_bleb = bleb(mix_zp(moth.genes, fath.genes), generation=moth.generation +1, name=moth.name[:4]+"f"+moth.name[5:12]+fath.name[10]+moth.name[13:-6]+f"({numgen})", str_type=2) 
    population.append(baby_bleb)        
            



if __name__ == "__main__":

    TestingPhase()

    pop = population(10, spawn_genes, mating, lambda x: selection(x,5), str_type=2)
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
    

    #c = Controller([abs_best.genes[1], abs_best.genes[2]], [abs_best.genes[0]])

    #print(f"\n\n{c}\n\n")
    #print(c.matstr())

    bleb = abs_best
    comm = f"tf({bleb.genes[0]},{bleb.genes[1]});".replace("\n","")
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

    input("\n\nPress Enter to exit")





### Saved

#Fitness 1

# blerb     8sp-3(AAAM)| Fitness:       2.0454716771708905| [-439.42 [0.36791
# clf;step(feedback(tf([-439.4218320919557],[0.3679111604494309])*proc,1),t);hold on;fplot(1)

# blerf     9or-0(AACZ)| Fitness:        2.021098655104519| [-1192.31], [1.105073]
# clf;step(feedback(tf([-1192.3115814311743],[1.1050730883435729])*proc,1),t);hold on;fplot(1)

# Fitness 2 (Ov 2=>16)

# LOL
# clf;step(feedback(tf([-31.533497578834638],[0.9029494515952657])*proc,1),t);hold on;fplot(1)

# blerf     asp-1(AAFV)| Fitness:        2.029994287818819| [-76.0012], [0.074651]
# clf;step(feedback(tf([-76.00129263889227],[0.07465139047644682])*proc,1),t);hold on;fplot(1)

# blerf     8sp-5(ABD+)| Fitness:        2.029992693353196| [-517.331], [0.508154]
# clf;step(feedback(tf([-517.3313907918358],[0.5081545262387778])*proc,1),t);hold on;fplot(1)



# blerf  asp-4410(AD+t)| Fitness:       2.0333135171088372| [0.139576 -42611.8 0.0], [0.001301 0.000447 32.21256]
# clf;step(feedback(tf([0.13957641106017923, -42611.80143019388, 0.0],[0.0013015296065708815, 0.00044755265317006636, 32.21256871786219])*proc,1),t);hold on;fplot(1)

# blerf   asp-465(AAWH)| Fitness:        2.048705498303066| [7.021110 -210068. 0.0], [0.008269 0.018023 193.6391]
# clf;step(feedback(tf([7.021110833776133, -210068.94898172544, 0.0],[0.008269060180778717, 0.018023376104508552, 193.63912813564187])*proc,1),t);hold on;fplot(1)

# blebf    7sps52(AACI)| Fitness:        2.065746139582047| [-271.75 [0.04878
# clf;step(feedback(tf([-271.7540446679815, 0.0],[0.048785511942663506, 0.48311952389684015])*proc,1),t);hold on;fplot(1)


# clf;step(feedback(tf([-184.8316557833927, 0.0],[2.0917875734202063e-05, 0.18158889952130533])*proc,1),t);hold on;fplot(1)

# TOO much overshoot
# blerb 8sp-l1549(AD5U)| Fitness:        2.010624426829117| [-71.4781876661425, -41.35771689361941], [1.327823138606177e-08, 0.02011841435610023]
# clf;step(feedback(tf([-71.4781876661425, -41.35771689361941],[1.327823138606177e-08, 0.02011841435610023])*proc,1),t);hold on;fplot(1)


# RECT
# blerf  9sp-6-84(ACis)| Fitness:       2.0059880239520957| [-37.5995 -11242.0 0.0], [2.001352 1.120090 0.001554]
# clf;step(feedback(tf([-37.59957355009163, -11242.094841941744, 0.0],[2.001352884924425e-10, 1.120090321150361e-06, 0.0015540378050836695])*proc,1),t);hold on;fplot(1)

#clf;step(feedback(tf([-4.432456744193351, -961.7650308903202, 0.0],[6.791401104170124e-15, 8.980991723133649e-11, 2.680201467509264e-07, 0.000353065248027001])*proc,1),t);hold on;fplot(1)

#clf;step(feedback(tf([-5466, 0.0],[1, 13])*proc,1),t);hold on;fplot(1)




# The new B is 7 times the older, divide by 7 the previous solutions


