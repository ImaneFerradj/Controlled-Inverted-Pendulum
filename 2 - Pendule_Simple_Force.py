#!/usr/bin/env python3


"""
Simulation Physique MU4RBR05
Author: Imane_F



Simulation d'un pendule simple attache a une barre avec la presence d'un force externe pour deplacer la barre

"""

# Y : [ x, x_dot, theta, theta_dot]
# x: translation de la barre
# x_dot : la vitesse de la barre 
# theta : la position angulare du pendule 
# theta_dot: la vitesse angulaire du pendule 



import numpy as np
import cv2
from scipy.integrate import solve_ivp
from Pendule import Pendule



sig = int(input(" forward(1) or backward(2) ? = "))
syst = Pendule()
def u( t ):

    t1 = 1*20. #on force a partir de 3 sec du debut de la simulation 
    t2 = 7*20. #et on n'arrete de force apres 5 sec du debut de la simulation
    global sig
    if( t > t1 and t < t2 ):
        if sig == 1: 
           return 0.5  # actionner les moteurs: a step of 0.5 m forward 
        if sig == 2: 
           return -0.5  #actionner les moteurs: a step of 0.5 m backward 
        else:
           print("erreur de saisie")
           exit(0)
    else:
        return 0.


def Force_le_pendule( t, y ):
    global syst
    # les equations de mouvements retrouvee par Euleur-Lagrange de: 
    
    # x_ddot (acceleration lineaire de la barre)
    x_ddot = u(t) - syst.m * syst.L * y[3]*y[3] * np.cos( y[2] ) + syst.m * syst.g * np.cos(y[2]) *  np.sin(y[2])
    x_ddot = x_ddot / ( syst.M + syst.m - syst.m * np.sin(y[2])* np.sin(y[2]) )
    
    # theta_ddot (acceleration angulaire du pendule)
    theta_ddot = -syst.g /syst.L * np.cos( y[2] ) - 1./syst.L * np.sin( y[2] ) * x_ddot
    
    # les differents imperfections de l'environement de simulation: 
    # [les coefficients d1 et d2 pourront etre modifies dans la classe Pendule]
    
    # la viscosite de l'air qui joue le role d'amortir les mouvements angulaires du pendule 
    damping_theta =  - syst.d2*y[3]
    
    # les frottements pour amortir le mouvement de la barre (friction entre la barre et le sol)
    damping_x =  - syst.d1*y[1]
    
    
    # la fonction renvoie le vecteur d'etat Y 
    return [ y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta ]


def main():
    # saisir la duree de la simulation 
    time = int(input("temps de simulation = "))
    
    # convertir le temps de simulation qui est saisis en sec en: un nombre d'iterations
    iteration = 20*time
    
    # la position initiale de la barre
    x= 0.
    
    # definition initiale du solveur a un etat Y initiale
    sol = solve_ivp(Force_le_pendule, [0, iteration], [ x, 0., np.pi/2 - 0.1, 0. ],   t_eval=np.linspace( 0, iteration, 30*time))
    
    # pour chaque iteration:  resoudre les equations differentiels 
    # chaque solution represente l'etat du pendule 
    # qui est ensuite affichee 
    for i, t in enumerate(sol.t):
        rendered = syst.Pas( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        cv2.imshow( 'im', rendered )
        cv2.moveWindow( 'im', 100, 100 )

        if cv2.waitKey(100) == ord('q'):
            break
            
            
main()
