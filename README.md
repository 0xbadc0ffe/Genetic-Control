# Genetic-Control
Generation of a linear controller vs the tuning of the cost functional of a LQR controller for an UAV heigth control, both made via a genetic algorithm.

The aim is to make the choise of the LQR functional less heuristic and optimize its choice (its weights), thanks to the aforementioned alogrithm, 
with respect to more esplicit parameters like the raising time, bandwidth and control effort. 
<br /><br />
The LQR "genetically" tuned is compared to the generated linear controller and with a classical weights choice approach for a LQR.

See more into our paper [LQR_with_GA.pdf](https://github.com/0xbadc0ffe/Genetic-Control/blob/main/LQR_with_GA.pdf).

<br /><br />
Soon i'll beautify this repo.

![](/media/genetic-alg.GIF)

# TODO

Feedback Mean GA - how well does it works?
<br />
Hybrid random parameters generation scheme: FMGA, FVGA and RNG.

