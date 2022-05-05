#!/usr/bin/env python3
"""
Simulation Physique MU4RBR05
Author: Imane_F



la classe Pendule est appelee pour l'affichage du systeme et pour saisir les diffrents parametres des simulations



"""

import numpy as np
import cv2
import sys





class Pendule:
    def __init__(self, g = 9.8 , L = float(input("La longueure de la tige (entre 0.5 et 3 m) = ")) , M = 10.0, m = float(input("La masse du pendule (entre 5 et 30 kg)= ")), d1 = 1.0, d2 = 0.1):
        self.g = g       # gravite
        self.L = L       # la longueur de la tige
        self.m = m       # la masse du pendule 
        self.M = M       # la masse de la barre
        self.d1 = d1     # coefficient de viscosite de l'air (amortir le mouvement du pendule angulaire) 
        self.d2 = d2     # coefficient de la friction (amortir le movement de la barre dans la direction x)
        
    def Pas( self, vecteur_etat, t = None):
                                                                 # vecteur_etat[1]: represente la vitesse lineaire de la barre selon x (x_dot)
                                                                 # vecteur_etat[3]: represente la vitesse angulaire du pendule (theta_dot)
        Barre_Translate = vecteur_etat[0]                        # la position de la barre (x)
        Angle_Pendule = vecteur_etat[2]*180./np.pi               # la position angulaire du pendule (theta) en degree
        Longueure_Pendule = 55*self.L                            # Affichage de la longueure de la tige                             
        Affichage = np.zeros((520, 520 ,3), dtype='uint8')       # l'ecran de l'affichage 
        
        # le sol (ground)
        cv2.line(Affichage, (0, 400), (Affichage.shape[1], 400), (19,69,139), 40 )  
        
        # les mouvements de la barre se font uniquement entre X_begin et X_End
        X_Begin = -5.
        X_End = 5.
        

        # Barre de translation
        Barre_Translate_Begin = int((Barre_Translate - 2.5 - X_Begin) / (X_End - X_Begin) * Affichage.shape[0])
        Barre_Translate_End   = int((Barre_Translate + 2.5 - X_Begin) / (X_End - X_Begin) * Affichage.shape[0])
        cv2.line(Affichage,(Barre_Translate_Begin , 380),(Barre_Translate_End , 380),(255,255,255),6)
        cv2.line(Affichage,(Barre_Translate_Begin + 100 , 380),(Barre_Translate_End - 100 , 380),(200,200,200),16)
        
        # Liaison_Pivot
        Liaison_Pivot_x = int((Barre_Translate - X_Begin) / (X_End - X_Begin) * Affichage.shape[0])
        Liaison_Pivot_y = 380
        cv2.circle(Affichage, (Liaison_Pivot_x, Liaison_Pivot_y), 10, (255,255,0), -1)
       
        
        # Pendule 
        Pendule_x = int(Longueure_Pendule * np.cos(Angle_Pendule/ 180. * np.pi))
        Pendule_y = int(Longueure_Pendule * np.sin(Angle_Pendule/ 180. * np.pi))
        cv2.circle(Affichage, (Liaison_Pivot_x + Pendule_x, Liaison_Pivot_y - Pendule_y), int(self.m) , (255,255,0), -1)
        cv2.line(Affichage, (Liaison_Pivot_x, Liaison_Pivot_y), (Liaison_Pivot_x + Pendule_x, Liaison_Pivot_y - Pendule_y), (255,255,255), 3)
        
        #Mettre a jour la valeur de l'angle
        Angle_Updated= Angle_Pendule % 360
        if( Angle_Updated > 180 ):
            Angle_Updated = -360 + Angle_Updated
        cv2.putText(Affichage, "Angle ="+str( np.round(Angle_Updated,2) )+ " deg", (Liaison_Pivot_x - 15, Liaison_Pivot_y - 15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (200,200,250), 1);
        
        # Lors d'une simulation en temps reel: on affiche les differents details de la simulation en haut de l'ecran
        if t is not None:
            cv2.putText(Affichage, "temps de simulation ="+str(np.round(t,4)/20)+" seconds", (15, 15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.7, (255,255,255), 1);
            cv2.putText(Affichage, "Angle du pendule ="+str(np.round(Angle_Pendule,4))+" deg", (15, 35), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.7, (255,255,255), 1);
            cv2.putText(Affichage, "Position de la barre x ="+str(np.round(Barre_Translate,4))+" m", (15, 55), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.7, (255,255,255), 1);
        cv2.putText(Affichage, "Simulation [MU4RBR05]", (280, 510), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.75, (255,255,255), 1);

       
        return Affichage

        
        
if __name__=="__main__":
    # simulation non-reel 
    
    Pendule = Pendule()

    # les parametres initiaux
    x = 0.
    s_x = 1.
    theta = np.pi/3
    s_theta = np.pi/3
    t = 0.
    print("Translater le systeme : a gauche (l), a droite(r) ")
    while True:
        # la vitesse de translation 
        x += s_x*0.2
        
        # la vitesse de rotation du pendule
        theta += s_theta*1.
        
        # si x < 5 la translation se fait a droite
        if( x > 5 ):
            s_x = -1. 
        
        # si x < -5 la translation se fait a gauche                   
        if( x < -5 ):
            s_x = 1.0
            
        if cv2.waitKey(30) == ord('r'):  # cliquer sur r pour translater la barre a droite
            s_x = 1
        if cv2.waitKey(30) == ord('l'):  # cliquer sur l pour translater la barre a gauche
            s_x = -1
            
        theta = -9.8 / 1.5 * np.cos( t ) # equation de mouvement de rotation
        rendered = Pendule.Pas( [x,0,theta,0] )  
        cv2.imshow( 'Pendule_Affichage', rendered )
        if cv2.waitKey(30) == ord('q'):
            break

        t += 30./1000.        
