# Getting Started With Your ROS2-Nodejs Package

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
