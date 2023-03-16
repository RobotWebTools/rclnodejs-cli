# Copyright 2020 Wayne Parrott
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from io import StringIO
import getpass
import os
import shutil
import subprocess
import sys
import re

import em

from ros2pkg.verb import VerbExtension
from catkin_pkg.package import package_exists_at, parse_package, PACKAGE_MANIFEST_FILENAME
try:
    import importlib.resources as importlib_resources
except ModuleNotFoundError:
    import importlib_resources

ERROR_RETURN = 1
SUCCESS_RETURN = 0

CMAKELISTS_FILENAME = 'CMakeLists.txt'


class CreateROS2NodeJsPkgVerb(VerbExtension):
    """Create a ROS2 package for Nodejs development."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'package_name',
            help='The package name')
        parser.add_argument(
            '--description',
            default='TODO: Package description',
            help='The description given in the package.xml')
        parser.add_argument(
            '--license',
            default='TODO: License declaration',
            help='The license attached to this package')
        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the package directory')
        parser.add_argument(
            '--dependencies',
            nargs='+',
            default=[],
            help='list of dependencies')
        parser.add_argument(
            '--maintainer-email',
            help='Email address of the maintainer of this package'),
        parser.add_argument(
            '--maintainer-name',
            default=getpass.getuser(),
            help='Name of the maintainer of this package'),
        parser.add_argument(
            '--no-npm-init',
            action='store_true',
            default=False,
            help='Do not run \'npm init\' on newly created package')
        parser.add_argument(
            '--template-location',
            help='The path to templates directory')
        parser.add_argument(
            '--typescript',
            action='store_true',
            default=False,
            help='Configure as a TypeScript Node.js project')
        parser.add_argument(
            '--no-ros-pkg',
            action='store_true',
            default=False,
            help='Limit pkg creation to only the Nodejs package structure')
        parser.add_argument(
            '--rclnodejs-version',
            help='Version of rclnodejs client library to use, x.y.z format')

    def main(self, *, args):
        if not args.no_ros_pkg:
          # run 'ros2 pkg create <package_name>'
          ros2pkg_args = [
              'ros2',
              'pkg',
              'create',
              args.package_name,
              '--description',
              args.description,
              '--license',
              args.license,
              '--destination-directory',
              args.destination_directory,
            ]

          if args.maintainer_email:
            ros2pkg_args.append('--maintainer-email')
            ros2pkg_args.append(args.maintainer_email)
          if args.dependencies:
            ros2pkg_args.append('--dependencies')
            ros2pkg_args.append(args.dependencies)

          proc_result = subprocess.run(ros2pkg_args)
          if proc_result.returncode != 0:
            print('Unable to create package')
            return ERROR_RETURN

          # verify cwd contains package.xml file
          os.chdir(os.path.join(args.destination_directory, args.package_name))
          cwd = os.getcwd()
          if not package_exists_at(cwd):
              print(f'  Current directory {cwd} does not contain a ROS {PACKAGE_MANIFEST_FILENAME} manifest file.')
              return ERROR_RETURN

          # parse package.xml
          pkg = parse_package(cwd)

        else:
          pkgDir = os.path.join(args.destination_directory, args.package_name)

          if os.path.exists(pkgDir):
            print(f'  Package directory {cwd} already exists.')
            return ERROR_RETURN

          #manually create pkg folder
          os.mkdir(pkgDir)
          os.chdir(pkgDir)
          cwd = os.getcwd()

        if args.typescript:
            print('Setting up Node.js and TypeScript.')
        else:
            print('Setting up Node.js.')

        template_dir_path = _get_template_dir_path(args)
        if _validate_template_dir(template_dir_path) == ERROR_RETURN:
            return ERROR_RETURN

        # copy all non-template files
        print('  Copy core Node.js files into package')
        ignore_patterns = shutil.ignore_patterns(
            '__*',
            '*.em',
        )
        shutil.copytree(template_dir_path, cwd, ignore=ignore_patterns, dirs_exist_ok=True)
        
        if args.no_ros_pkg:
          pkgjson_data = {
              'name': args.package_name,
              'version': "0.0.0",
              'license': " ".join(args.license),
              'description': args.description
          }
        else:
          # create example launch file
          print('  Creating example launch file, example.launch.py')
          launch_data = {
              'name': pkg.name
          }
          _expand_template(
              os.path.join(template_dir_path, 'example.launch.em'),
              launch_data,
              os.path.join(cwd, 'launch', 'example.launch.py')
          )

          # update CMakeLists.txt with install() rules 
          print('  Adding \'install()\' rules to CMakeLists.txt')
          cmakelist_exists = os.path.exists(os.path.join(cwd, CMAKELISTS_FILENAME))
          if not cmakelist_exists:
              print(f'  Unable to setup cmake install rules. '
                  'Current directory {cwd} does not contain a {CMAKELISTS_FILENAME} file.')
          else:
              _updateCMakeListsFile(
                  os.path.join(template_dir_path, 'CMakeLists.install.em'),
                  os.path.join(cwd, CMAKELISTS_FILENAME))
          pkgjson_data = {
              'name': pkg.name,
              'version': pkg.version,
              'license': " ".join(pkg.licenses),
              'description': pkg.description
          }

        # create package.json
        print('  Configuring package.json')

        _expand_template(
            os.path.join(template_dir_path, 'package.json.em'),
            pkgjson_data,
            os.path.join(cwd, 'package.json')
        )

        if not args.no_npm_init:
            # install all node.js dependencies in package.json
            _run_npm_install(args)

        print('Node.js setup complete.')
        return SUCCESS_RETURN


def _updateCMakeListsFile(cmakelist_template_path, cmakelist_path):
    if not os.path.exists(cmakelist_template_path):
        print('Skipping install rule creation. \n',
              'Template directory does not contain a CMakeList.install.emfile.')
        return

    # look for install() rules, if present stop now
    install_rule_re = re.compile(r"^\s*install\s*\(")
    with open(cmakelist_path, 'r') as pkg_cmakelist_file:
        for line in pkg_cmakelist_file:
            if install_rule_re.match(line):
                print('  Skipping install rule creation. CMakeList.txt already contains install() rules.')
                return ERROR_RETURN

    with open(cmakelist_template_path) as f:
        cmakelist_template_lines = f.readlines()

    pkg_cmakelist_file = open(cmakelist_path, 'a')
    print('', file=pkg_cmakelist_file)
    for line in cmakelist_template_lines:
        print(line.rstrip('\n'), file=pkg_cmakelist_file)
    pkg_cmakelist_file.close()

    return 0


def _get_template_dir_path(args):
    if args.template_location:
        template_dir_path = args.template_location
        print(f'  Using custom template location {template_dir_path}')
    else:
        language_folder = 'javascript'
        if args.typescript:
            language_folder = 'typescript'

        full_package = 'rclnodejs_pkg_creation_tool.resource.templates' #+ language_folder
        template_dir_path = importlib_resources.path(full_package, 'readme.txt')
        with template_dir_path as seg:
            template_dir_path = seg.parent
        if args.typescript:
            template_dir_path = os.path.join(template_dir_path, 'typescript')
        else:
            template_dir_path = os.path.join(template_dir_path, 'javascript')

    return template_dir_path


def _validate_template_dir(template_dir_path):
    if not os.path.exists(template_dir_path):
        print(f'  Template directory {template_dir_path} does not exist.')
        return ERROR_RETURN

    if not os.path.exists(os.path.join(template_dir_path, 'package.json.em')):
        print(f'  Template directory {template_dir_path} does not contain a package.json.em file.')
        return ERROR_RETURN

    return SUCCESS_RETURN


# from https://github.com/ros2/ros2cli/blob/master/ros2pkg/ros2pkg/api/create.py
def _expand_template(template_file, data, output_file):
    output = StringIO()
    interpreter = em.Interpreter(
        output=output,
        globals=data,
    )
    with open(template_file, 'r') as h:
        try:
            interpreter.file(h)
            content = output.getvalue()
        except Exception as e:
            if os.path.exists(output_file):
                os.remove(output_file)
            print(f'Exception when expanding {template_file} into {output_file}: {e}', file=sys.stderr)
            # raise
        finally:
            interpreter.shutdown()

    if os.path.exists(output_file):
        with open(output_file, 'r') as h:
            if h.read() == content:
                return
    else:
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

    with open(output_file, 'w') as h:
        h.write(content)

def _run_npm_install(args):
    npm = shutil.which('npm')
    if not npm:
        npm = shutil.which('yarn')
    if not npm:
        print('  Unable to locate npm or yarn. Did not perform automatic '
              'installation of Node.js package dependencies.\n\n'
              '  Please run: \n'
              '     npm install()\n'
              '  or\n'
              '     yarn install()')
        return ERROR_RETURN
    
    print(f'  Running \'{os.path.basename(npm)} install\' command.')
    rclnodejs_pkg = f"rclnodejs{'@' + args.rclnodejs_version if args.rclnodejs_version else ''}"
    return subprocess.run([npm, 'install', rclnodejs_pkg])




