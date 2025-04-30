import os
from typing import List

from sphinx.application import Sphinx
from docutils import nodes

from src.abstract_rosdoc_directive import AbstractRosdocDirective

from sphinx.util import logging

logger = logging.getLogger(__name__)


class StandaloneInterfacePackageDirective(AbstractRosdocDirective):
    """
    Sphinx directive to scan a ROS package for .msg and .srv files and list them in a table using Jinja2 template.
    """
    has_content = True
    required_arguments = 1  # Require exactly one argument (The package relative path)

    def run(self) -> List[nodes.Node]:

        if not self.arguments:
            return [
                self.state_machine.reporter.error(
                    'No package path specified for the rosdoc_standalone_interface_package directive.',
                    line=self.lineno)
            ]

        package_path = self.arguments[0]
        package_abs_path = self.resolve_directory_path(package_path)
        if not os.path.isdir(package_abs_path):
            return [
                self.state_machine.reporter.error(
                    f'The specified package path does not exist: {package_path}',
                    line=self.lineno)
            ]

        # Find all .msg and .srv files
        msg_files = self.find_files(package_abs_path, '.msg')
        srv_files = self.find_files(package_abs_path, '.srv')

        # Prepare data for template rendering
        data = self.prepare_data(msg_files, srv_files, package_path)

        # Run rosdoc_lite on the package
        self.run_rosdoc_lite(package_abs_path)

        # Render the template
        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        rendered_content = self.render_template(
            data,
            template_dir,
            "standalone_interface_package_template.jinja",
            helper_functions={
                "add_link_to_type_documentation":
                self.add_link_to_type_documentation
            })

        # Parse and return the rendered content
        return self.parse_rst_content(rendered_content)
    
    def prepare_data(self, msg_files: List[str], srv_files: List[str],
                     package_path: str) -> dict:
        """
        Prepare the data dictionary to be passed to the Jinja2 template.

        Args:
            msg_files (List[str]): List of .msg files in the package.
            srv_files (List[str]): List of .srv files in the package.
            package_path (str): The relative path to the package.

        Returns:
            dict: A dictionary containing the data to be rendered into the template.
        """
        msg_files_info = [{
            "type":
            os.path.basename(package_path) + '/' + os.path.splitext(file)[0],
            "full_path":
            os.path.join('/', package_path, "msg", file)
        } for file in msg_files]

        srv_files_info = [{
            "type":
            os.path.basename(package_path) + '/' + os.path.splitext(file)[0],
            "full_path":
            os.path.join('/', package_path, "srv", file)
        } for file in srv_files]
        return {
            "messages": msg_files_info,
            "services": srv_files_info,
        }


    def find_files(self, directory_path: str, extension: str) -> List[str]:
        """
        Find all files with the given extension in the package directory.

        Args:
            directory_path (str): The path to the directory.
            extension (str): The file extension to search for (e.g., ".msg").

        Returns:
            List[str]: A list of file paths with the given extension.
        """
        files = []
        for root, dirs, filenames in os.walk(directory_path):
            for filename in filenames:
                if filename.endswith(extension):
                    files.append(filename)
        return files

def setup(app: Sphinx):
    app.add_directive("rosdoc_standalone_interface_package",
                      StandaloneInterfacePackageDirective)

    return {
        'version': '0.1',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
