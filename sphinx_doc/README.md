# Ned Doc
## Pré-requis

1. (Vous pouvez passez cette partie si vous utilisez Docker).

    Pour modifier la doc, vous aurez tout d'abord besoin de Sphinx :

    `pip install Sphinx sphinx_rtd_theme`

2. Le dossier front_end est un [sous-module git](https://git-scm.com/book/fr/v2/Utilitaires-Git-Sous-modules),
    il peux donc être nécessaire de le mettre à jour régulièrement.
    Pour cloner / update le sous module front-end, veuillez effectuer la commande suivante:

    `git submodule update --init relative_path_to/front_end`

## Génération
Placez vous à la racine de votre dossier sphinx

Pour générer la documentation complete en html, utilisez : 

`make all`

Pour mettre à jour les fichiers de traduction, utiliser :

`make update`

Pour mettre à jour la documentation en anglais uniquement, utilisez :

`make en`

Pour mettre à jour la documentation en français uniquement, utilisez :

`make fr`

Le fichier à ouvrir est : **_build/fr/index.html** pour la documentation française, **_build/en/index.html** pour la documentation anglaise

Command clean + make + open : 

`make clean; make all ; xdg-open _build/fr/index.html`

## Ressources
### Spinxcontrib

http://otamachan.github.io/sphinxcontrib-ros/index.html
