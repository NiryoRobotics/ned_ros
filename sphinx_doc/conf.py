import sys
from sphinx.builders.html import StandaloneHTMLBuilder

import os

# Import ROS Wrapper
sys.path.append(os.path.abspath('../niryo_robot_python_ros_wrapper/src/'))

# Kindda hack the import to import shared config file
sys.path.append(os.path.abspath('.'))
from front_end.config import shared_conf

# -- Project information -----------------------------------------------------

project = u'Ned ROS Documentation'
copyright = shared_conf.copyright
author = shared_conf.author

# The short X.Y version
version = u'v3.2'
# The full version, including alpha/beta/rc tags
release = u'v3.2.0'

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx.ext.extlinks',
    'sphinx.ext.todo',
    'sphinx_togglebutton'
]

StandaloneHTMLBuilder.supported_image_types = [
    'image/svg+xml',
    'image/gif',
    'image/png',
    'image/jpeg'
]

# Avoid autosection label to trigger warning on low level titles
autosectionlabel_maxdepth = 3
# Avoid clash between same label in different document
autosectionlabel_prefix_document = True

# Todo_extension
todo_include_todos = True
todo_emit_warnings = True

# Documentation infos
source_suffix = '.rst'
master_doc = 'index'

# Toggle button text
togglebutton_hint = ""

for arg in sys.argv:
    if not arg.startswith("language="):
        continue
    else:
        language = arg.replace("language=", "")
        break
else:
    language = None

translation_object = {}
translation_object["fr"] = {}
translation_object["fr"]["PROJECT_NAME"] = "Documentation de ROS pour le Ned"

translation_object["en"] = {}
translation_object["en"]["PROJECT_NAME"] = "Ned ROS documentation"

html_context = {}

# Only for testing purpose
html_context["BASE_FOLDER_URL"] = "https://docs.niryo.com/dev/ros"


html_context["TRANSLATION"] = translation_object[language if language is not None else 'en']

exclude_patterns = [u'_build', 'Thumbs.db', '.DS_Store']

pygments_style = None

add_module_names = False


# -- Options for HTML output -------------------------------------------------
html_theme = shared_conf.html_theme

templates_path = shared_conf.templates_path
html_static_path = shared_conf.html_static_path

html_logo = shared_conf.html_logo
html_favicon = shared_conf.html_favicon

html_css_files = shared_conf.html_css_files

html_js_files = shared_conf.html_js_files

html_theme_options = shared_conf.html_theme_options

html_show_sphinx = shared_conf.html_show_sphinx

# -- Internationalization --
locale_dirs = ['locale/']  # path is example but recommended.
gettext_compact = False  # optional.

# -- Options for intersphinx extension ---------------------------------------

extensions += ['sphinxcontrib.ros']
ros_base_path = ['../']

# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'https://docs.python.org/': None}

# Links
ros_distro = 'melodic'
extlinks = {
    'rosdocs': ('http://docs.ros.org/' + ros_distro + '/api/%s', ''),
    'tf2': ('http://docs.ros.org/' + ros_distro + '/api/tf2_ros/html/c++/classtf2__ros_1_1%s.html', ''),
    'msgs_index': ('http://docs.ros.org/' + ros_distro + '/api/%s/html/index-msg.html', ''),
    'moveit_msgs': ('http://docs.ros.org/' + ros_distro + '/api/moveit_msgs/html/msg/%s.html', 'moveit_msgs/'),
    'sensor_msgs': ('http://docs.ros.org/' + ros_distro + '/api/sensor_msgs/html/msg/%s.html', 'sensor_msgs/'),
    'std_msgs': ('http://docs.ros.org/' + ros_distro + '/api/std_msgs/html/msg/%s.html', 'std_msgs/'),
    'std_srvs': ('http://docs.ros.org/' + ros_distro + '/api/std_srvs/html/srv/%s.html', 'std_srvs/'),
    'wiki_ros': ('http://wiki.ros.org/%s', ''),
    'niryo_studio_simulation': (
        'https://docs.niryo.com/product/ned/source/software/niryo_studio.html#connecting-simulation-to-niryo-studio/%s',
        None),
}

TRANSLATE_CAPTIONS_JS="""
window.onload = function ()
{
    const matches = document.querySelectorAll(".wy-menu-vertical p span");
    // Due to sphinx version problem (sphinxcontribros)
    // Change dynamically titles in left menu
    if (document.documentElement.lang == 'fr') {
        matches[0].innerText = "Introduction"
        matches[1].innerText = "Packages"
        matches[2].innerText = "Pour aller plus loin..."
    }
}
"""
# Add custom JS that translate captions in left menu bar
def setup(app):
    # 3. Tell Sphinx to add your JS code. Sphinx will insert
    #    the `body` into the html inside a <script> tag:
    app.add_js_file(None, body=TRANSLATE_CAPTIONS_JS)