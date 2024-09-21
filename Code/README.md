# Comment faire rouler le code
1. Clonez le projet avec GitHub
```
git clone https://github.com/Madsam999/IFT3150_Cours_Projet.git
```
2. Ouvrez le projet avec votre IDE préféré
## CLion
De mon expérience, CLion devrait automatiquement détecter le CMakeLists.txt.
Il est important de changer quelques paramètres de l'exécutable pour que ça fonctionne.
1. Allez dans Run -> Edit Configurations.
2. Changer le champ "Program arguments" pour le fichier .ray que vous voulez lire.
3. Modifier le champ "Working directory" pour C:\Users\{votre nom d'utilisateur}\...\IFT3150_Cours_Projet\Code (si vous utilisez MacOS ou Linux, changez le chemin en conséquence).
4. Appuyez sur OK.
Avec ces paramètres, le projet devrait rouler sans problème.
## Visual Studio
Voir le fichier "Instruction_TP2_IFT3355_H24.pdf" que Caio a écrit. (Ce n'est pas le IDE que j'utilise, alors je ne peux pas vraiment aider...)

3. Une fois que vous avez configuré votre IDE, vous pouvez compiler et exécuter le projet.
4. Le rendu de la scène devrait être sauvegarder dans le dossier 'data/output/{nom du fichier .ray choisi}'

Il y aura 2 images. Color.bmp est l'image avec les couleurs (et la plus intéressante à regarder), et Depth.bmp est un grayscale de la profondeur de chaque pixel.