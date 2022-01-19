# Ned Doc
## Pré-requis

1. (Vous pouvez passez cette partie si vous utilisez Docker).

    Pour modifier la doc, vous aurez tout d'abord besoin de certaines librairies python,  :

    La documentation est compilée avec sphinx et python3. Il faut installer au préalable des librairies python
    en utilisant le fichiers de requirements 'requirements_docs.txt'

    `pip3 install -r requirements_docs.txt'

2. Le dossier front_end est un [sous-module git](https://git-scm.com/book/fr/v2/Utilitaires-Git-Sous-modules),
    il peux donc être nécessaire de le mettre à jour régulièrement.
    Pour cloner / update le sous module front-end, veuillez effectuer la commande suivante:

    `git submodule update --init relative_path_to/front_end`

## Génération
- Placez vous à la racine de votre dossier sphinx

- Pour générer la documentation complete en html, utilisez : 

    `make all`

- Pour mettre à jour les fichiers de traduction, utiliser :

    `make update`

- Pour mettre à jour la documentation en anglais uniquement, utilisez :

    `make en`

- Pour mettre à jour la documentation en français uniquement, utilisez :

    `make fr`

- Le fichier à ouvrir est : **_build/fr/index.html** pour la documentation française, **_build/en/index.html** pour la documentation anglaise

    Command clean + make + open : 

    ``make clean; make all ; xdg-open _build/fr/index.html``

- Pour les paquets concernant le ros, vous pouvez générer une template pour le fichier .rst. Il récupéra automatiquement les services, messages, config et créer le template pour les services et topics. Allez dans le directorie `scripts`

    ```./generateDocRst.sh -p [path of doc rst from source]```

    example:
    `./generateDocRst.sh -p ros/stack/low_level/common`. Cette commande génère un fichier doc common.rst dans le `sphinx/source/ros/stack/low_level/common`.

## Ressources
### Spinxcontrib

http://otamachan.github.io/sphinxcontrib-ros/index.html
