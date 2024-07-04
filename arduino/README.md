

# Boucle principale

On
* Envoie des donnes si le delais de communication est atteint
* Mets a jour la machine a etat
* Agit en concordance avec la machine a etat
* Mets a jour les wrappers de capteur
* Mets a jour le pid 

# Communication

On cree un objet JSON qu'on serialise et envois via le port serie
Presentement on mets toujours tout (la meme chose) dans l'objet JSON, on pourrait etre dynamique et changer ce qu'on envoit selon
l'etat, ensuite le code du GUI doit etre un peu solide et ne pas s'attendre a quoi que ce soit

# Controle

## Wrappers

Les capteurs ont des wrapers qui pemettent d'obtenir des valeurs d'ingenerie (+derivees)

* Tickswrapper permet de gerer l'encodeur du moteur
* Potwrapper permet de gerer le potentiometre


## Simulation

Le controle se fait avec un pid qui tente d'optimiser l'acceleration
On recoit 10 samples sur 100ms de la simulation
A chaque instant (millis()), on fait une interpolation lineaires entre les deux samples les plus pres temporelement
La logique d'interpolation est definie dans Command.hpp

## Proprioception

Le robot connait sa position, ses dimensions et l'angle du pendule
la position de l'effecteur est recalculee dans la fonction update_eot()

# Machine a etat

La machine a etat a plusieurs etat qui se suivent pour faire la sequence de jeu
et des etats isoles pour des tests par exemples

Les etats sont definis dans common_rpiarduino/Common.hpp

* Ready [sequence] => Le robot est immobile
* Stabilize [sequence] => Le robot se bat contre le pendule pour le stabiliser
* ReturnHome [sequence] => Le robot retourne tranquillement (sans pid) a sa position de depart
* TakingTree [sequence] => Le robot ferme sa pince
* Swinging [sequence] => Le robot est dans sa phase principale et fait osciller le pendule
* JustGonnaSendIt [sequence] => Le robot va full pin pour passer par dessus l'obstacle
* Drop [sequence] => Le robot lache l'arbre
* ShortCircuitForward [test] => Le robot avance (active par bouton)
* ShortCircuitBackward [test] => Le robot recule (active par bouton)

## Boundingbox

Une class boundingbox est utilisee pour faire changer d'etat
Par exemple si le robot est en etat JustGonnaSendIt et que les arbres sont dans la boite definie
de la zone de lachage, on change d'etat vers Drop