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

dim_state = 6
Astr = 'A = [0 1 0 0 0 0; 0 0 -0.0106 0.0106 -0.0106 0.0106; 0 0 -10 0 0 0; 0 0 0 -10 0 0; 0 0 0 0 -10 0; 0 0 0 0 0 -10];'
Bstr = 'B = 7*[0 ; 0 ; 1 ; -1 ; 1 ;-1];'
Cstr = 'C = [1 0 0 0 0 0];'
Systr = 'sys = ss(A,B,C,0);'



eng.eval(Astr,nargout=0)
A = np.matrix(eng.workspace['A'])

eng.eval(Bstr,nargout=0)
B = np.matrix(eng.workspace['B'])

eng.eval(Cstr,nargout=0)
C = np.matrix(eng.workspace['C'])

eng.eval(Systr,nargout=0)


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
print(Systr)
print()

pyperclip.copy(Astr+Bstr+Cstr+"[num, den] = ss2tf(A,B,C,0);"+"proc = tf(num,den);"+time_str+Systr)
print("Copied on clipboard\n\n")


#val_choices = [ 0 for i in range(7)] + [j for j in range(100)]
val_choices = []
top = 1000
for i in range(top):
    val_choices += [i for k in range(top-i)]
    #val_choices += [-i for k in range(top-i)] # neg values



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
    Zp = 0.5
    Np = 1
    BpWeight = 2
    OvWeight = 2
    IntWeigth = 1

    res = IntWeigth*sum/cnt*(Np*N+Zp**Z) + abs(max_e)*OvWeight + os_time/cnt*BpWeight

    #print(sum,cnt,max_e)
    #print(f"RES: {res}")

    return res
    

def test_controller_3(Q,R, ref=1):
    eng.eval(f"Q = {Q};",nargout=0) 
    eng.eval(f"R = {R};",nargout=0) 
    #print(Q,R)
    try:
        #eng.eval("K = lqi(sys,Q,R);", nargout=0)   
        zer = ' '.join(str(0) for i in range(dim_state+1))
        eng.eval(f"try K = lqi(sys,Q,R); catch K = [{zer}]; end", nargout=0)  # error catched internally to engine
    except:
        return 1000000000  # Cannot LQI
    
    eng.eval(f"[num, den] = ss2tf(A-B*K(1:{dim_state}),B,C,0);", nargout=0)
    eng.eval(f"proc = tf(num,den)*tf([-K({dim_state+1})],[1 0]);", nargout=0)
          
    test = eng.eval(f"{ref}-step(feedback(proc,1),t)")
    #print(test)
    sum = 0
    cnt = 0
    max_e = 0
    os_time = len(test)
    sw = False                  # True after  x(tk) > 0.95*ref
    for t in test:              # t[0] = ref - x(tk)
        cnt += 1
        mol = 3*cnt/(cnt+1)
        sum += mol*(t[0])**2 
        if t[0] < 0.05*ref:
            sw = True
            os_time = cnt
        if sw and abs(t[0]) > max_e:
            max_e = t[0]  

    if not sw:
        max_e = ref

    BpWeight = 3    # Bandwidth
    OvWeight = 4    # Overshoot
    IntWeight = 3   # Tracking
    CeWeight = 0.2  # Control Effort

    res = IntWeight*sum/cnt + abs(max_e)*OvWeight + os_time/cnt*BpWeight # + CeWeight*1/(float(R[1:-1]))


    #print(sum,cnt,max_e)
    #print(f"RES: {res}")

    return res


def TestingPhase():

    print("## Starting Test phase\n")
    
    comm = f"tf([{rnd.choice(val_choices)+0.5}],[1 {rnd.choice(val_choices)} {rnd.choice(val_choices)}]);"
    print("TEST 1   LC1  "+comm)  # linear controller 1
    test_controller(eng.eval(comm))

    comm = f"tf([{rnd.choice(val_choices)+0.5}],[1 {rnd.choice(val_choices)} {rnd.choice(val_choices)}]);"
    print("TEST 2   LC2  "+comm)
    test_controller_2(eng.eval(comm))

    Q = [rnd.choice(val_choices),rnd.choice(val_choices), abs(np.random.normal(0, 6)), abs(np.random.normal(0, 6)), abs(np.random.normal(0, 6)),  abs(np.random.normal(0, 6)), rnd.choice(val_choices)]
    R = [abs(np.random.normal(1, 10))]
    Q = f"diag([" +" ".join( str(Q[i]) for i in range(len(Q)-1)) +"])"
    R = f"{R}"
    print("TEST 3   LQI  "+Q+" "+R )
    test_controller_3(Q,R)


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
    #Q = [rnd.choice(val_choices),rnd.choice(val_choices), abs(np.random.normal(0, 6)), abs(np.random.normal(0, 6)), abs(np.random.normal(0, 6)),  abs(np.random.normal(0, 6)), rnd.choice(val_choices)]
    Q = [abs(np.random.normal(1, 10000)), abs(np.random.normal(1, 1000)), abs(np.random.normal(0, 6)), abs(np.random.normal(0, 6)),  abs(np.random.normal(0, 6)),abs(np.random.normal(0, 6)), abs(np.random.normal(1, 10000))]
    R = [abs(np.random.normal(1, 0.1))]
    return Q+R

# feedback variance spawn 
def spawn_genes_fb():
    global gen
    if gen > FB_SPAWN:
        b = abs_best
        Q = [abs(np.random.normal(1, b.genes[0])), abs(np.random.normal(1, b.genes[1])), abs(np.random.normal(0, b.genes[2])), abs(np.random.normal(0, b.genes[3])),  abs(np.random.normal(0, b.genes[4])),abs(np.random.normal(0, b.genes[5])), abs(np.random.normal(1, b.genes[6]))]
        R = [abs(np.random.normal(1, b.genes[7]))]
    else:
        Q = [abs(np.random.normal(1, 10000)), abs(np.random.normal(1, 1000)), abs(np.random.normal(0, 6)), abs(np.random.normal(0, 6)),  abs(np.random.normal(0, 6)),abs(np.random.normal(0, 6)), abs(np.random.normal(1, 10000))]
        R = [abs(np.random.normal(1, 0.1))]
    return Q+R


def fitfunc(bleb):
    Q = f"diag([" +" ".join( str(bleb.genes[i]) for i in range(len(bleb.genes)-1)) +"])"
    R = f"{bleb.genes[6]}"
    return test_controller_3(Q,R)
                

def mutation(genes):
    new_genes = []
    i = 0
    for gene in genes:
        if i==0 or i==1 or i==6:
            new_genes.append(abs(gene + np.random.normal(0, 60)))
        elif i==7:
            new_genes.append(abs(gene + np.random.normal(0, 0.6)))
        else:
            new_genes.append(abs(gene + np.random.normal(0, 0.6)))
    return new_genes
    
# feedback variance mutation
# maybe based on best instead of abs_best?
def mutation_fb(genes):
    new_genes = []
    i = 0 
    global gen
    if gen > FB_SPAWN:
        b = abs_best
        min_mut = 0.6
        mut_percent_variance = 0.1
        for gene in genes:
            new_genes.append(abs(gene + np.random.normal(0, max(min_mut,abs_best.genes[i]*mut_percent_variance))))
            #print(i,  max(min_mut,abs_best.genes[i]*mut_percent_variance))
            i+=1
        return new_genes
    else:
        for gene in genes:
            if i==0 or i==1 or i==6:
                new_genes.append(abs(gene + np.random.normal(0, 60)))
            elif i==7:
                new_genes.append(abs(gene + np.random.normal(0, 0.6)))
            else:
                new_genes.append(abs(gene + np.random.normal(0, 0.6)))
            i+= 1
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
    
def mating(population, parents_num, gen, inc=False):
    moth = population[0]
    fath = population[1]
    #avoid incest
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
    baby_bleb = bleb(mix(moth.genes, fath.genes), generation=moth.generation +1, name=moth.name[:4]+"f"+moth.name[5:12]+fath.name[10]+moth.name[13:-6]+f"({numgen})") 
    population.append(baby_bleb)        
            

gen = 0
abs_best = None
FB_SPAWN = 1000


if __name__ == "__main__":

    TestingPhase()

    pop = population(10, spawn_genes_fb, mating, lambda x: selection(x,5))
    pop.fit_all(fitfunc)
    pop.print()
    best_fit = 100
    abs_best_fit = -1
    abs_best = None
    threshold = 0.1
    gencap = 1000000  # At most 1000000 generations
    genmin = 500      # At least 500 generations
    gen = 0
    try:
        while abs_best_fit > threshold or gen < genmin:     
            print("\n\n-------------------\n\n")
            if gen > FB_SPAWN:
                print("*Feedback Variance Spawn Activated*")
            pop.evolve(fitfunc, mutation_fb)
            if rnd.choice([0]*499+[1]) and abs_best: # 0.2% prob to resurrect abs_best
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

            print(abs_best)
            gen += 1 
            if gen > gencap: break
    except KeyboardInterrupt:
        pass
    except TypeError:
        pass
    #except:
    #    pass

    print("\n\nAbsolute best:\n")
    print(abs_best)


    bleb = abs_best
    show_comm = f"K = lqi(sys,diag(["+" ".join( str(bleb.genes[i]) for i in range(len(bleb.genes)-1)) +f"]),{bleb.genes[-1]});[num, den] = ss2tf(A-B*K(1:{dim_state}),B,C,0);proc = tf(num,den)*tf([-K({dim_state+1})],[1 0]);clf;step(feedback(proc,1),t)"
    print("\n\nK:", eng.eval("lqi(sys,diag(["+" ".join( str(bleb.genes[i]) for i in range(len(bleb.genes)-1)) +f"]),{bleb.genes[-1]})"))
    print("\n\n\n" + show_comm)

    pyperclip.copy(show_comm)
    print("\nCommand printed on clipboard!")

    input("\n\nPress Enter to exit")





### Saved

# Fitness 1 (original)

#
# K = lqi(sys,diag([836.6034687497711 9.705821452992325 0.05184362591561706 0.2732256468188451 0.056052154155068076 0.5386734043920408 148.0770731030816]),20.882468633949436);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# blerf    8sp-18| Fitness:       3.9422583551741477| 34.91183 19.42091 0.017529 0.005037 0.010729 0.341802 94.86632 0.177782
# K = lqi(sys,diag([34.911837399866045 19.42091898637313 0.0175295229137451 0.005037857978450466 0.010729777210092939 0.34180297283471306 94.86632268685638]),0.1777827777179728);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# Fitness 2 (Bandw. 2 => 3, Int 3.5 => 3)

# blerf   7sp-314| Fitness:        4.675153627556849| 640.6768 15.69261 0.009376 0.185520 0.003770 0.003406 608.1705 10.22542
# K = lqi(sys,diag([640.6768127892254 15.692610867274906 0.009376923646392775 0.1855208343503294 0.0037706300347579713 0.003406496821948407 608.1705578631747]),10.225429138249858);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

#blebf   8sp-577| Fitness:         4.67539223500496| 447.5928 52.27077 0.106317 0.160042 0.047253 0.046745 729.2114 16.31509
#K = lqi(sys,diag([447.59282554908754 52.27077860624034 0.10631773954219093 0.1600426021486335 0.04725317806633239 0.046745199970459894 729.2114570937557]),16.31509622733931);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)


# Fitness 3 (Ov 2 => 8)

# blerf     asp-9| Fitness:        4.850366872635833| 831.5473 159.1557 0.373459 0.078871 0.080788 0.156191 203.4844 13.48719
# K = lqi(sys,diag([831.5473110959338 159.15579879295836 0.37345909956095025 0.07887134262525952 0.08078826152687185 0.1561919853638108 203.4844921716184]),13.487191925990853);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# bleb    asp-636| Fitness:        4.855514263978337| 805 57 1.098009 1.380615 3.709826 0.594059 192 12.75584
# K = lqi(sys,diag([805 57 1.098009556797321 1.3806150029082471 3.709826389349883 0.5940596707168857 192]),12.75584370013418);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# bleb  8sp-32670      | Fitness:        4.849717724495378| 840 4 1.072882 1.985318 3.404430 0.412082 25 2.293753
# K = lqi(sys,diag([840 4 1.072882 1.985318 3.404430 0.412082 25]),2.293753);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# blerf 8sp-32670(AKTH)| Fitness:        4.834964151871863| 847.7394 1.001743 1.331663 0.235119 0.425400 1.774478 25.50051 5.180568
# K = lqi(sys,diag([847.7394921978885 1.0017430169001922 1.3316636304641771 0.23511960140436217 0.4254002293795236 1.7744787181482273 25.50051482252991]),5.180568165696581);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# K = lqi(sys,diag([849.5364 1.722395 2.925471 2.155944 0.948181 1.440402 26.07072]), 5.343997);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)

# K = lqi(sys,diag([849.5364 1.722395 2.925471 2.155944 0.948181 1.440402 260.07072]), 1.343997);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)


# Fitness 4 (Ov 8 => 4, mol 2 => 3)

# blerf     9sp-9(AABR)| Fitness:        5.552357883330344| 807.3862 246.4701 1.824170 1.464209 1.436379 0.296976 565.0356 10.78588
# K = lqi(sys,diag([807.3862089302453 246.47015428394718 1.8241706837027678 1.4642099900728924 1.4363796065781058 0.29697666946365153 565.0356749682998]),10.785881727955166);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)


# K = lqi(sys,diag([807.3862089302453 246.47015428394718 1.8241706837027678 1.4642099900728924 1.4363796065781058 0.29697666946365153 565.0356749682998]),0.285881727955166);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)


# K = lqi(sys,diag([849.5364 20.722395 1.925471 1.155944 0.948181 1.440402 304.07072]), 0.0143997);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)
# LQRG.test_controller_3("diag([849.5364 20.722395 1.925471 1.155944 0.948181 1.440402 304.07072])", "0.0143997")
# 4.305360905713037


# K = lqi(sys,diag([3229.5364 15.722395 0.925471 1.155944 0.948181 0.440402 3054.07072]), 0.0043997);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)
# K = lqi(sys,diag([5418.7438 30.203217 0.820723 1.374551 1.291294 0.412039 5654.07072]), 0.0209932);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)


# blerf 6sp-732(AKBR)| Fitness:        3.666169049000805| 390418.04 30000.987 0.902951 0.877231 1.150631 0.653851 2300654.7 0.0209932
# K = lqi(sys,diag([390418.04 30000.987 0.902951 0.877231 1.150631 0.653851 2300654.7]), 0.0209932);[num, den] = ss2tf(A-B*K(1:6),B,C,0);proc = tf(num,den)*tf([-K(7)],[1 0]);clf;step(feedback(proc,1),t)
# LQRG.test_controller_3("diag([390418.04 30000.987 0.902951 0.877231 1.150631 0.653851 2300654.7])", "0.0209932")
# gen 41041








