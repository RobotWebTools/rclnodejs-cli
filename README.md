# rclnodejs-cli
Standalone commandline tools and `ros2 cli` extensions for use with the ROS2 [rclnodejs]() client library.

## Prerequisites
* ROS2 Foxy or greater installed
* Node.js version between 10.23.1 - 12.x
* npm or yarn package manager
* **A shell environment that includes ROS2.** See [Configuring a ROS2 Environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/) for more info.

## Installation
Install the `rclnodejs-cli` package globally or alternatively use [npx](https://medium.com/@maybekatz/introducing-npx-an-npm-package-runner-55f7d4bd282b) to run rclnodejs-cli directly from a shell.

Note: Your shell environment should include ROS2. 

To install rclnodejs-cli globally run the following command from a shell:
```
> npm install -g rclnodejs-cli
or
> yarn global add rclnodejs-cli
```

### Optional
You can extend the `ros2` cli with additional commands and options from this package by sourcing the `install/setup.[bat,bash,ps1,sh] file.

On Linux run:
```
> source <rclnodejs-cli-dir>/install/setup.bash
```
On Windows run:
```
> <rclnodejs-cli-dir>\install\setup.bat
or
> <rclnodejs-cli-dir>\install\setup.ps1
```

For more information on using an rclnodejs-cli tool from the `ros2` cli see the references to tool/command user-guides below. 


## Usage
List available commands.

If rclnodejs-cli is installed globally run:
```
> rclnodejs-cli -h
```
Alternatively use `npx` as shown:
```
> npx rclnodejs-cli -h
```
The default commandline output follows:

```
           _                 _       _
  _ __ ___| |_ __   ___   __| | ___ (_)___
 | '__/ __| | '_ \ / _ \ / _` |/ _ \| / __|
 | | | (__| | | | | (_) | (_| |  __/| \__ \
 |_|  \___|_|_| |_|\___/ \__,_|\___|/ |___/
                                  |__/
Usage: rclnodejs [command] [options]
    
Options:
  -h, --help                               display help for command

Commands:
  create-package [options] <package-name>  Create a ROS2 package for Nodejs development.
  generate-ros-messages                    Generate JavaScript code from ROS2 IDL interfaces
  help [command]                           display help for command

```

List a subcommand details.
```
> rclnodejs-cli <subcommand> -h
or
> npx rclnodejs-cli <subcommand> -h
```

Example
```
> rclnodejs-cli create-package -h
or
> npx rclnodejs-cli create-package -h
```

## create-package command
The `rclnodejs-cli create-package` command creates a hybrid ROS2-Nodejs package that can coexist and participate with other ROS2 packages in a ROS2 workspace and can be run using the ROS2 `launch` facility. A ROS2-Nodejs package consist of a ROS2 package, specifically an `ament-cmake` ROS2 package, overlaid with a Nodejs package.

[Learn more about the create-package tool](package-creation-tool/README.md).

## generate-ros-messages command
Generate JavaScript messages corresponding to the interfaces (.IDL)
in your ROS2 environment. Run this command in the root folder of a Nodejs package that includes rlcnodejs as a dependency. The JavaScript message files are created in the `node_modules/rclnodejs/generated/` folder of the current Nodejs package.

[Learn more about the generate-ros-messages tool](message-generator-tool/README.md).

## Contributors
[Wayne Parrott](https://github.com/wayneparrott)
