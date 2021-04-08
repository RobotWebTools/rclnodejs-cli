# generate-ros-messages command
The `generate-ros-messages` command creates JavaScript messages corresponding to the interfaces (.IDL) in your ROS2 environment. The JavaScript message files are created in the `node_modules/rclnodejs/generated/` folder of the current Nodejs package.

# Usage #
Run the `generate-ros-messages` command in the root directory of a ROS2-Nodejs package, i.e., the directory containing the package.json file. The Nodejs package must include `rclnodejs` as a dependent package.

```
  cd <your-nodejs-package>

  rclnodejs-cli generate-ros-messages
or
  npx rclnodejs-cli generate-ros-messages

  _ __ ___| |_ __   ___   __| | ___ (_)___ 
 | '__/ __| | '_ \ / _ \ / _` |/ _ \| / __|
 | | | (__| | | | | (_) | (_| |  __/| \__ \
 |_|  \___|_|_| |_|\___/ \__,_|\___|/ |___/
                                  |__/     
Start JavaScript message generation...

Generation complete.
```
