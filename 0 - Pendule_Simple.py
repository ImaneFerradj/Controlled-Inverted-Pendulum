#!/usr/bin/env python3


"""
Simulation Physique MU4RBR05
Author: Imane_F



Simulation d'un pendule simple [la barre est completement ignoree et non etudiee]

"""
# Y : [ x, x_dot, theta, theta_dot]
# x: translation de la barre
# x_dot : la vitesse de la barre 
# theta : la position angulare du pendule 
# theta_dot: la vitesse angulaire du pendule 
 
import numpy as np
import cv2
from Pendule import Pendule
from scipy.integrate import solve_ivp


syst = Pendule()

def Pendule_Simple( t, y ):
    global syst
    damping =  - syst.d2*y[1]
    
    # la fonction renvoie le vecteur d'etat Y
    return [ y[1], - syst.g/syst.L * np.cos( y[0] )  + damping]


def main():

    time = int(input("temps de simulation = "))
    iteration = 20*time
    sol = solve_ivp(Pendule_Simple, [0, iteration], [ np.pi/2 + 0.1, 0 ],  t_eval=np.linspace(0, iteration, 30*time)  )
    
    for i, t in enumerate(sol.t):
        rendered = syst.Pas( [0,1, sol.y[0,i], sol.y[1,i] ], t )
        cv2.imshow( 'im', rendered )

        if cv2.waitKey(30) == ord('q'):
            break

main()
