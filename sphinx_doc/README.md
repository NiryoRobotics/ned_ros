# Ned Doc
## Préambule
Pour modifier la doc, vous aurez tout d'abord besoin de Sphinx, de la template d'affichage 
et d'une librairie Python pour la génération des messages :

`pip install -r requirements_docs.txt`

ou si ça ne fonctionne pas

`pip install Sphinx sphinx_rtd_theme sphinxcontrib-ros`

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
