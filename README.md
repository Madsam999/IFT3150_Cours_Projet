# IFT3150 Cours Projet

# Description:
Le projet consiste à augmenter le programme de lancer de rayons développé dans 
le cours IFT3355 pour qu'il génère des images de volumes participatifs,
intégrés dans la scène 3D courante comme une primitive régulière.  A noter que 
deux volumes en superposition (problème mal posé) n'ont pas à être gérés 
correctement.

Les étapes suivantes seront intégrées:

- génération procédurale et lecture de volumes participatifs sous forme de 
  grilles de valeurs dans une boîte (extension possible à OpenVDB en bonus)
- implémentation du 'ray marching' dans la grille (pas réguliers, pas réguliers 
  avec jitter, milieu du rayon dans chaque voxel, milieu avec jitter)
- évaluation de l'atténuation le long d'un rayon (caméra, ombre, autre)
- évaluation de 'single scattering' avec fonction de phase (uniforme, Mie,
  Rayleigh, HG)

Toute extension supplémentaire sera considérée comme un bonus (intégration 
analytique, next-event, tracé de chemins, bidirectionnel, etc.).

# Plan de Développement:
- Date de début: Semaine du 23 septembre 2024
- Date de fin: Autour de la fin de la session (À être déterminé)

- Implementation de l'interpolation Trilinéaire: Semaine du 7 octobre 2024
- Implémentation du ray marcher: Semaine du 7 octobre (potentielement semaine du 14 octobre si c'est plus compliqué qu'anticipé)

#Informations:
- Professeur responsable: Pierre Poulin

- Éleve: Samuel Fournier

- Matricule: 20218212
