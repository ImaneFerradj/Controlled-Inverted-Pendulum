#!/usr/bin/env python3
"""
Simulation Physique MU4RBR05
Author: Imane_F

la classe Lineariser_Le_Systeme est appelee pour pour l'application de la methode de LQR
le but de cette clase est de renvoyer la valeur optimale du gain K pour la commande optimale: u = Kx
ou u: est la force requis pour deplacer la barre afin de maintenir le pendule a la position souhaitee (90 vers la haut)

"""

import numpy as np
import cv2
from Pendule import Pendule
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import control


syst = Pendule()

class Lineariser_Le_Systeme:
    def __init__(self):
        global syst
                                              
        self.A = np.array([[0,1,0,0], [0,-syst.d1 , -syst.g * syst.m / syst.M , 0], [0,0,0,1.], [0,syst.d1/syst.L, (syst.m + syst.M) * syst.g / (syst.M * syst.L) ,-syst.d2] ] )  # 4x4
        self.B = np.expand_dims( np.array( [0, 1.0/syst.M, 0., -1/(syst.M*syst.L)] ) , 1 ) # 4x1

    def compute_K(self, valeurs_propres_souhaites = [-0.1, -0.2, -0.3, -0.4] ):
        print ('[compute_K] valeurs_propres_souhaites=', valeurs_propres_souhaites)
        self.K = control.place( self.A, self.B,  valeurs_propres_souhaites)
