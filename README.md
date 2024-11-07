Le robot est piloté par une carte NUCLEO-L476RG et comporte les composants suivants :

- Sonar : Pour détecter les obstacles et guider l’évitement.
- Servomoteurs : Quatre servos assurent la mobilité des "chevilles" gauche et droite.
- Module Bluetooth : Permet de contrôler le robot à distance.

Voici les objectifs principaux du projet :

1) Contrôle par Bluetooth : Le robot peut avancer, reculer, aller à gauche et à droite.
2)Vitesse Fixe : La vitesse du robot est réglée à 20 cm/s.
3) Évitement d'Obstacles : Le robot utilise un sonar pour détecter les obstacles, adapte sa trajectoire en fonction de l'angle d'impact, et longe les obstacles.
4) Arrêt d'Urgence : Un bouton permet d’arrêter ou démarrer le robot à tout moment pour des raisons de sécurité.
5) Vérification du Niveau de Batterie : Une LED s’allume si la tension de la carte de contrôle tombe sous 3V.
6) Debugging via UART : Des messages de débogage sont envoyés pour surveiller l’état du robot.

Fonctionnement Général

Après une phase d’initialisation, le robot entre dans une boucle infinie, dans laquelle il :

- Active/Désactive les Roues : En fonction de l'état du bouton B1 (arrêt d’urgence).
- Détecte et Évite les Obstacles : En continue, le sonar détecte les obstacles et une fonction ajuste la trajectoire.
- Réagit aux Interruptions : Pour le contrôle par Bluetooth.
- Vérifie la Batterie : Une surveillance de la tension est effectuée en permanence. En dessous de 3V, une LED est activée pour signaler un niveau critique.

Ce système assure une navigation autonome et un contrôle sécurisé du robot, avec un retour d’information pour un diagnostic efficace.
