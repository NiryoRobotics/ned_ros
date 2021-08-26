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

Pour générer le html, utilisez : 

`make html`

Le fichier à ouvrir est : **_build/html/index.html**

Command clean + make + open : 

`make clean; make html ; google-chrome _build/html/index.html`

## Ressources
### Spinxcontrib

http://otamachan.github.io/sphinxcontrib-ros/index.html
