# `create-package` command
The `rclnodejs-cli create-package` command creates a hybrid ROS2-Nodejs package that can coexist and participate with other ROS2 packages in a ROS2 workspace and be run using the ROS2 `launch` facility. A ROS2-Nodejs package consist of a ROS2 package, specifically an `ament-cmake` ROS2 package, overlaid with a Nodejs package. This command can also be run from the `ros2` cli as described in the [Using the command from the `ros2` commandline](#Using-the-command-from-the-ros2-commandline) section. 

## Key Features
* Creates a ROS2 ament_cmake packages and overlays it with a custom Nodejs package.
* `--typescript` commandline option to configure the package for use with TypeScript.
* Includes the ROS2 JavaScript client library, 
[rclnodejs](https://github.com/RobotWebTools/rclnodejs) as a runtime dependency.
* Creates and example JavaScript/TypeScript ROS2 publisher node and ROS2 launch-description.
* Customized `CMakeList.txt` `install()` rules to install key runtime files to the package share/ folder

# Usage #
Create a new ROS2-Nodejs package basic usage:
```
  rclnodejs-cli create-package <package_name>
  rclnodejs-cli create-package <package_name> --typescript
```
Alternatively using npx:
```
  npx rclnodejs-cli create-package <package_name>
  npx rclnodejs-cli create-package <package_name> --typescript
```
Be aware that when using `npx` to run `rclnodejs-cli`, this package includes a dependency on the rclnodejs package which has a lengthy install postinstall step. Thus if you frequently use `rclnodejs-cli` you may benefit from more responsiveness by installing this package globally.

View the `create-package` options:
```
  rclnodejs-cli create-package -h
or 
  npx rclnodejs-cli create-package -h
```
```
           _                 _       _
  _ __ ___| |_ __   ___   __| | ___ (_)___
 | '__/ __| | '_ \ / _ \ / _` |/ _ \| / __|
 | | | (__| | | | | (_) | (_| |  __/| \__ \
 |_|  \___|_|_| |_|\___/ \__,_|\___|/ |___/
                                  |__/
Usage: rclnodejs create-package <package_name> [options...]

Create a ROS2 package for Nodejs development.

Options:
  --description <description>               The description given in the package.xml
  --destination-directory <directory_path>  Directory where to create the package directory
  --license <license>                       The license attached to this package
  --maintainer-email <email>                Email address of the maintainer of this package
  --maintainer-name <name>                  Name of the maintainer of this package
  --no-init                                 Do not run "npm init"
  --rclnodejs-version <x.y.z>               The version of rclnodejs to use
  --typescript                              Configure as a TypeScript Node.js project
  --dependencies <ros_packages...>          list of ROS dependencies
  -h, --help                                display help for command
```

Note: Package naming should conform to the [ROS2 Patterns and Conventions](http://wiki.ros.org/ROS/Patterns/Conventions), e.g., use `_` underscores as separators in package names instead of '-' dashes.

# Working with a ROS2-Nodejs Package #
The new package directory content should be similar to this listing.
```
CMakeLists.txt
__init__.py
jsconfig.json
launch/
  example.launch.py
node_modules/
package.json
package.xml
src/
  index.js
```

**Note:
CMakeLists.txt includes `install` rules that you will need to customize for your project layout and runtime needs.**

### 1. Use the [colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html) build utility to install the key JavaScript resources into the `./install/share/` folder.
```
  colcon build
```
Your package folder will now include the standard ROS2 package directories: `build/`, `install/` and `log/`. The `install/` directory includes configuration scripts and if you look deep into the `install/ros2_nodejs/share/ros2_nodejs` folder you will see the `install()` rules in the CMakeLists.txt have installed the key JavaScript resources from the `src/` and `launch/` directories.

### 2. Add your new ROS2-Nodejs package to your ROS environment. 

From your `<package-directory>` run:  

Linux
```
  source install/setup.bash 
```
Windows
```
  install\setup.bat #windows
```

### 3. Verify that the ROS2-Nodejs package is part of your ROS2 environment using the `ros2` CLI command. 
```
  ros2 pkg list
```
This command will output a long list of the packages in your ROS2 environment. Scroll through the list and verify it contains the ros2_nodejs package.

### 4. Launch example.launch.py
We can now use the `ros2 launch` command to run the `example.launch.py` launch-description. This launch file defines how to startup the example app in our ROS2-Nodejs package. The example app creates a ROS2 node and publisher that sends a message every second to the topic named `foo`. See `src/index.js` for the JavaScript implementation details.

Launch `example.launch.py` as shown below:
```
  ros2 launch ros2_nodejs example.launch.py
```
To view the messages being published to the `foo` topic, open a separate shell configured with your ROS2 environment and enter:
```
  ros2 topic echo foo
```
A message should appear every second.

# Working With Typescript #
If you would like to work with TypeScript instead of JavaScript use the `--typescript` commandline option as shown.
```
  rclnodejs create-package <mypkg> --typescript
or 
  rclnodejs-cli create-package <mypkg> --typescript
```
The ROS2-Nodejs package will include a `tsconfig.json` file and a TypeScript example at `src/index.ts`.


# Using the command from the `ros2` commandline #
An alternative way to use the `create-package` command is from the `ros2` commandline. 

Before using, you must first extend your ROS2 environment by running the `install/setup.[bash|bat|sh|ps1]` script from the root folder of the `rclnode-cli package`. For background on configuring your ROS2 environement see this [tutorial](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/).

Verify the command is installed properly:
```
  ros2 pkg -h
```
You should see `create_nodejs` in the Commands list similar to the output shown below.
```
usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...

Various package related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  create         Create a new ROS2 package
  create_nodejs  Create a ROS2 package for Nodejs development.
  executables    Output a list of package specific executables
  list           Output a list of available packages
  prefix         Output the prefix path of a package
  xml            Output the XML of the package manifest or a specific tag

  Call `ros2 pkg <command> -h` for more detailed usage.
```

Next create a new ROS2-Nodejs package as shown below:
```
  ros2 pkg create_nodejs <pkg_name>
or
  ros2 pkg create_nodejs <pkg_name> --typescript
```


