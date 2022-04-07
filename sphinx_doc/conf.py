import sys
from sphinx.builders.html import StandaloneHTMLBuilder

import os

# Import ROS Wrapper
sys.path.append(os.path.abspath('../niryo_robot_python_ros_wrapper/src/niryo_robot_python_ros_wrapper/'))
sys.path.append(os.path.abspath('../niryo_robot_led_ring/src/niryo_robot_led_ring/api/'))
sys.path.append(os.path.abspath('../niryo_robot_led_ring/src/niryo_robot_sound/api/'))

# Kindda hack the import to import shared config file
sys.path.append(os.path.abspath('.'))
from front_end.config import shared_conf
from front_end.config import base_conf

# -- Project information -----------------------------------------------------

project = u'Ned ROS Documentation'
copyright = shared_conf.copyright
author = shared_conf.author

# The short X.Y version
version = u'v4.0'
# The full version, including alpha/beta/rc tags
release = u'v4.1.0'

# -- General configuration ---------------------------------------------------

extensions = base_conf.extensions

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
html_static_path = shared_conf.html_static_path[:] + ['static/']

html_logo = shared_conf.html_logo
html_favicon = shared_conf.html_favicon

html_css_files =  ['config.css'] + shared_conf.html_css_files[:]

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
    'rosgraph_msgs': ('http://docs.ros.org/' + ros_distro + '/api/rosgraph_msgs/html/msg/%s.html', 'rosgraph_msgs/'),
    'sensor_msgs': ('http://docs.ros.org/' + ros_distro + '/api/sensor_msgs/html/msg/%s.html', 'sensor_msgs/'),
    'control_msgs': ('http://docs.ros.org/' + ros_distro + '/api/control_msgs/html/msg/%s.html', 'control_msgs/'),
    'visualization_msgs': (
    'http://docs.ros.org/' + ros_distro + '/api/visualization_msgs/html/msg/%s.html', 'visualization_msgs/'),
    'std_msgs': ('http://docs.ros.org/' + ros_distro + '/api/std_msgs/html/msg/%s.html', 'std_msgs/'),
    'std_srvs': ('http://docs.ros.org/' + ros_distro + '/api/std_srvs/html/srv/%s.html', 'std_srvs/'),
    'visualization_msgs': (
    'http://docs.ros.org/' + ros_distro + '/api/visualization_msgs/html/msg/%s.html', 'visualization_msgs/'),
    'wiki_ros': ('http://wiki.ros.org/%s', ''),
    'rosconsole': ('http://wiki.ros.org/rosconsole%s', ''),
    'log4cxx': ('https://logging.apache.org/log4cxx/latest_stable/index.html%s', ''),
    'ros_multimachine': ('https://wiki.ros.org/ROS/Tutorials/MultipleMachines%s', ''),
    'doc_niryo_studio': ('https://docs.niryo.com/product/niryo-studio/v3.2.1/en/index.html%s', ''),
    'niryo_studio_simulation': (
        'https://docs.niryo.com/product/niryo-studio/source/connection.html#using-ned-in-simulation-with-niryo-studio/%s',
        None),
    'pymodbus': ('https://pymodbus.readthedocs.io/en/latest/index.html%s', ''),
    'python_website': ('https://www.python.org/%s', None),
    'python_installation': ('https://realpython.com/installing-python/%s', None),
    'pip_website': ('https://pypi.org/project/pip/%s', None),
    'pip_installation': ('https://pip.pypa.io/en/stable/installing/%s', None),
    'ned_ros_stack': ('https://github.com/NiryoRobotics/niryo_one_ros/%s', None),
    'api_modbus_readme': ('https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_modbus/%s', None),
    'modbus_examples': ('https://github.com/NiryoRobotics/ned_ros/tree/master/niryo_robot_modbus/examples/%s', None)
}

TRANSLATE_CAPTIONS_JS = """
window.onload = function ()
{
    const matches = document.querySelectorAll(".wy-menu-vertical p span");
    // Due to sphinx version problem (sphinxcontribros)
    // Change dynamically titles in left menu
    if (document.documentElement.lang == 'fr') {
        matches[0].innerText = "Introduction"
        matches[1].innerText = "Installation"
        matches[2].innerText = "Packages"
        matches[3].innerText = "Pour aller plus loin..."
    }
}
"""


# Add custom JS that translate captions in left menu bar
def setup(app):
    # 3. Tell Sphinx to add your JS code. Sphinx will insert
    #    the `body` into the html inside a <script> tag:
    app.add_js_file(None, body=TRANSLATE_CAPTIONS_JS)
