#!/usr/bin/env python3


"""
Simulation Physique MU4RBR05
Author: Imane_F



Simulation du pendule inverse: le but du projet => le maintient du pendule a une posion instable

"""

# Y : [ x, x_dot, theta, theta_dot]
# x: translation de la barre
# x_dot : la vitesse de la barre 
# theta : la position angulare du pendule 
# theta_dot: la vitesse angulaire du pendule 


import numpy as np
import cv2
from Pendule import Pendule
from Lineariser_Le_Systeme import Lineariser_Le_Systeme
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import control

syst = Pendule()               # le systeme etudie
sys = Lineariser_Le_Systeme()  # renvoie K (pour appliquer le LQR)


# Q et R : des matrices symetriques positives
Q = np.diag( [1,1,1,1.] )
R = np.diag( [1.] )

# K : [State feedback for stavility]
# S : Solution de l'equation de Ricatti
# E : les valeurs propres pour un systeme a boucle ferme 

K, S, E = control.lqr( sys.A, sys.B, Q, R )
sys.compute_K(valeurs_propres_souhaites = E )


# fonction pour definir la valeur du deplacement u requis pour maintenir le pendule inverse a 90 degs 
def u( t , y ):
    u_ = -np.matmul( K , y - np.array([0,0,np.pi/2.,0]) ) # au lieux de y = Ax + Bu => y = Ax - B(Kx) = x(A - BK)
    print ('t=',t, 'u_=', u_)
    return u_[0]


def Pendule_Inverse( t, y ):
    global syst
    
    # les equations de mouvements retrouvee par Euleur-Lagrange de:
    
    # x_ddot (acceleration lineaire de la barre) 
    x_ddot = u(t, y) - syst.m * syst.L * y[3] * y[3] * np.cos( y[2] ) + syst.m * syst.g * np.cos(y[2]) *  np.sin(y[2])
    x_ddot = x_ddot / ( syst.M + syst.m - syst.m * np.sin(y[2]) * np.sin(y[2]) )
    
    # theta_ddot (acceleration angulaire du pendule)
    theta_ddot = -syst.g / syst.L * np.cos( y[2] ) -  np.sin( y[2] ) / syst.L * x_ddot

    # les differents imperfections de l'environement de simulation: 
    # [les coefficients d1 et d2 pourront etre modifies dans la classe Pendule]
    
    # la viscosite de l'air qui joue le role d'amortir le mouvement rotatif du pendule 
    damping_x =  - syst.d1*y[1]
    
    
    # les frottements pour amortir le mouvement de la barre (friction entre la barre et le sol)
    damping_theta =  - syst.d2*y[3]


    # la fonction renvoie le vecteur d'etat Y 
    # Y : [ x, x_dot, theta, theta_dot]
    return [ y[1], x_ddot + damping_x, y[3], theta_ddot + damping_theta ]
    
    
def main():

    # saisir la duree de la simulation 
    time = int(input("temps de simulation = "))
    
    # convertir le temps de simulation qui est saisis en sec en: un nombre d'iterations
    iteration = 20*time
    
    # la position initiale de la barre
    x= 0.
    
    # definition initiale du solveur a un etat Y initiale
    sol = solve_ivp(Pendule_Inverse, [0, iteration], [ x, 0., np.pi/2 + 0.01, 0. ],   t_eval=np.linspace( 0, iteration, time*30)  )
    
    
    # pour chaque iteration:  resoudre les equations differentiels 
    # chaque solution represente l'etat du pendule 
    # qui est ensuite affichee 
    for i, t in enumerate(sol.t):
        rendered = syst.Pas( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        theta = sol.y[2,i]
        
        if cv2.waitKey(30) == ord('r'):  # cliquer sur l pour translater la barre a gauche tout en maintenant le pendule inverse
            x += 0.5
            sol = solve_ivp(Pendule_Inverse, [0, 3], [ x, 0., np.pi/2 + 0.01, 0. ],   t_eval=np.linspace( 0, 3, time*30))
            rendered = syst.Pas( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
            
        if cv2.waitKey(30) == ord('l'):  # cliquer sur r pour translater la barre a droite tout en maintenant le pendule inverse
            x += -0.5
            sol = solve_ivp(Pendule_Inverse, [0, 3], [ x, 0., np.pi/2 + 0.01, 0. ],   t_eval=np.linspace( 0, 3, time*30))
            rendered = syst.Pas( [sol.y[0,i], sol.y[1,i], sol.y[2,i], sol.y[3,i] ], t )
        
        print("Translater le systeme : a gauche (l), a droite(r) ")   
        cv2.imshow( 'im', rendered )
        cv2.moveWindow( 'im', 100, 100 )

        if cv2.waitKey(100) == ord('q'):
            break
main()
