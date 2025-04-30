# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

import os
import sys
import shutil

sys.path.insert(0, os.path.abspath("./rosdoc_extension"))
sys.path.insert(0, os.path.abspath('./rosdoc_extension/src'))

# -- Project information -----------------------------------------------------

project = u'Ned ROS Stack'
copyright = u'2025, Niryo'
author = u'Niryo'

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx_copybutton',
    'sphinx.ext.extlinks',
    "sphinx_multiversion",
    "niryo_rosdoc_directive",
    "niryo_standalone_interface_package_directive",
]

# Sphinx-multiversion configuration
# TODO(thomas): configure correctly once multiversion works with gitlab CI
smv_tag_whitelist = r'^.*$'
smv_branch_whitelist =  r'^.*$'
smv_remote_whitelist = r'^(origin)$'
smv_outputdir_format = '{ref.name}'
smv_prefer_remote_refs = False

# Add definitions of interpreted text roles (classes) for S5/HTML data.
rst_prolog = """
 .. include:: <s5defs.txt>
 """

templates_path = ['.templates']

source_suffix = '.rst'

master_doc = 'index'

language = 'en'

exclude_patterns = ['.build', 'Thumbs.db', '.DS_Store', 'README.md', 'venv']

pygments_style = None

ros_distro = 'noetic'

extlinks = {
    'wiki_ros': ('http://wiki.ros.org/%s', ''),
    'ros_multimachine': ('https://wiki.ros.org/ROS/Tutorials/MultipleMachines%s', ''),
    'doc_niryo_studio': ('https://docs.niryo.com/niryostudio/%s', ''),
    'niryo_studio_simulation': ('https://docs.niryo.com/niryostudio/connecting-to-your-robot/#section-header-two-9qfpm%s', None),
    'niryo_studio_scan_equipment': ('https://docs.niryo.com/niryostudio/getting-started/startup-configuration/#section-header-two-64s42%s', None),
    'modbus': ('https://docs.niryo.com/api/modbus%s', None),
    'pyniyro': ('https://docs.niryo.com/api/pyniryo%s', None),
    'blockly': ('https://docs.niryo.com/api/blockly%s', None),
}

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'

html_theme_options = {"collapse_navigation": False}

html_static_path = ['.static']

html_logo = '.static/images/logo_niryo-nobg.png'

html_css_files = ["css/custom.css"]

html_sidebars = {
    '**': [
        'versioning.html',
    ],
}

def copy_sound_files(app, exception):
    """Copy sound files to the _static directory during the build."""
    if exception is None:  # Ensure the build wasn't interrupted
        source_sounds = os.path.abspath(os.path.join(app.confdir, "..", "niryo_robot_sound/niryo_robot_state_sounds/"))
        target_static = os.path.join(app.outdir, '_static/audio')

        if not os.path.exists(target_static):
            os.makedirs(target_static)

        for file in os.listdir(source_sounds):
            if file.endswith('.wav'):
                shutil.copy(os.path.join(source_sounds, file), target_static)

def setup(app):
    # Make ros_distro variable accessible for extensions
    app.add_config_value("ros_distro", ros_distro, "env")
    app.connect("build-finished", copy_sound_files)