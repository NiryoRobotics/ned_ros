import os
from typing import List
import subprocess
import abc

from sphinx.util.docutils import SphinxDirective
from sphinx.util.nodes import nested_parse_with_titles
from docutils.statemachine import StringList
from docutils import nodes

from config import ROS_INTERFACE_PACKAGES

from jinja2 import Environment, FileSystemLoader


class AbstractRosdocDirective(SphinxDirective, abc.ABC):

    @abc.abstractmethod
    def prepare_data(self, *args, **kwargs) -> dict:
        """
        Abstract method to prepare data. Subclasses must implement this method.
        """
        pass

    def run_rosdoc_lite(self, package_path: str):
        """ Run rosdoc_lite on a given package to generate html files for messages """
        output_path = os.path.join(self.env.app.outdir, 'rosdoc_lite', os.path.basename(package_path))
        subprocess.run(["rosdoc_lite", package_path, "-o", output_path], check=True, capture_output=True)

    def resolve_relative_path(self, relative_path: str) -> str:
        """
        Resolves a relative path to the source directory of the Sphinx documentation.

        :param relative_path: The relative path to resolve.
        :param app: The Sphinx application object.
        :return: An absolute path resolved to the source directory.
        """
        return os.path.join(self.env.srcdir, relative_path)

    def resolve_file_path(self, relative_path: str) -> str:
        """
        Resolve the file path relative to the Sphinx source directory.

        This method takes a relative path and resolves it to an absolute path
        based on the Sphinx source directory and the current document's directory.
        It ensures that the resolved path exists and is a directory.

        Args:
            relative_path (str): The relative path to resolve.

        Returns:
            str: The resolved absolute path.

        Raises:
            self.error: If the resolved path does not exist or is not a file.
        """

        file_path = os.path.abspath(os.path.join(self.env.srcdir, relative_path))

        if not os.path.isfile(file_path):
            raise self.error(f"The path '{relative_path}' (resolved to '{file_path}') does not exist or is not a file.")
        return file_path

    def resolve_directory_path(self, relative_path: str) -> str:
        """
        Resolve the directory path relative to the Sphinx source directory.

        This method takes a relative path and resolves it to an absolute path
        based on the Sphinx source directory and the current document's directory.
        It ensures that the resolved path exists and is a directory.

        Args:
            relative_path (str): The relative path to resolve.

        Returns:
            str: The resolved absolute package path.

        Raises:
            self.error: If the resolved path does not exist or is not a directory.
        """

        dir_path = os.path.abspath(os.path.join(self.env.srcdir, relative_path))

        if not os.path.isdir(dir_path):
            raise self.error(
                f"The path '{relative_path}' (resolved to '{dir_path}') does not exist or is not a directory.")
        return dir_path

    def add_link_to_type_documentation(self, interface_type: str, interface_category: str = "msg") -> str:
        """
        Generate a link to the documentation for a ROS message type.

        Args:
            interface_type (str): The type of the ROS interface (e.g., "std_msgs/String").
            interface_category (str): The category of the ROS interface, which can be "msg", "srv", or "action".
                                    Defaults to "msg".

        Returns:
            str: A URL linking to the documentation for the specified ROS interface type.

        Raises:
            ValueError: If the interface category is not one of "msg", "srv", or "action".
        """

        if interface_category not in ["msg", "srv", "action"]:
            raise ValueError(
                f"Invalid interface category '{interface_category}'. Must be one of 'msg', 'srv', or 'action'.")

        package, interface_name = interface_type.split("/")

        # Handle special case for action file name which does not match the interface type
        if interface_category == "action" or "Action" in interface_name:
            interface_name = interface_name.split("Action")[0]

        # Check if the type is a standard or a custom ROS interface
        if package in ROS_INTERFACE_PACKAGES:
            return f"`{interface_type} <https://docs.ros.org/en/{self.env.config.ros_distro}/api/{package}/html/{interface_category}/{interface_name}.html>`_"
        else:
            for folder in ["msg", "srv", "action"]:
                relative_file_path = f"rosdoc_lite/{package}/html/{folder}/{interface_name}.html"
                abs_file_path = os.path.join(self.env.app.outdir, relative_file_path)
                if os.path.exists(abs_file_path):
                    return f"`{interface_type} <{os.path.join('/',relative_file_path)}>`_"
        return interface_type

    def render_template(self, data: dict, template_dir: str, template_file: str, helper_functions: dict = None) -> str:
        """
        Render the Jinja2 template with the provided data.

        This method sets up a Jinja2 environment with a template directory
        located in the same directory as this script, and renders the 
        'package_template.jinja' template with the given data.

        Args:
            data (dict): A dictionary containing the data to be rendered 
                    into the template.
            template_file (str): The name of the template file to render.
            helper_functions (dict): Optional dictionary of helper functions 
                                    to add to the Jinja2 environment.

        Returns:
            str: The rendered template as a string.
        """
        env = Environment(loader=FileSystemLoader(template_dir), autoescape=False)

        # Add helper functions to the Jinja environment
        if helper_functions:
            for name, function in helper_functions.items():
                env.globals[name] = function

        template = env.get_template(template_file)
        return template.render(data)

    def parse_rst_content(self, content: str) -> List[nodes.Node]:
        """
        Parse the rendered content as reStructuredText (RST) and return the resulting nodes.

        Args:
            content (str): The content to be parsed as RST.

        Returns:
            List[nodes.Node]: A list of parsed nodes from the RST content.
        """
        node = nodes.section()
        node.document = self.state.document

        nested_parse_with_titles(self.state, StringList(content.splitlines()), node)
        return node.children
