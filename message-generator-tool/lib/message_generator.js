/* eslint-disable no-console */
/* eslint-disable no-useless-concat */
const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');
const process = require('process');

class MsgGenerator {
  configureCli(cli) {
    const createPkgCmd = cli.command('generate-ros-messages');
    createPkgCmd.description('Generate JavaScript code from ROS2 IDL interfaces').action(() => {
      this.generateMessages();
    });
  }

  // eslint-disable-next-line class-methods-use-this
  generateMessages() {
    // confirm running from node package root folder, i.e., folder contains package.json
    const nodeModules = path.join(process.cwd(), 'node_modules');
    if (!fs.existsSync(nodeModules)) {
      console.error('Error: unable to locate node_modules directory.\n'
        + 'The current directory does not appear to be an initialized Nodejs package.');
      process.exit(-1);
    }
    // confirm rclnodejs has been installed in the node package
    const rclnodejs = path.join(process.cwd(), 'node_modules', 'rclnodejs');
    if (!fs.existsSync(rclnodejs)) {
      console.error('Error: the rclnodejs package does not appear to be installed in your node package.\n' + 'Please install rclnodejs using "npm i rclnodejs".');
      process.exit(-1);
    }

    const binFolder = path.join(nodeModules, '.bin');
    let script = 'generate-ros-messages';

    // check for older version of generate-ros-messages, e.g., rclnodejs v0.18.1
    if (fs.existsSync(path.join(binFolder, 'generate-messages'))) {
      script = 'generate-messages';
    }

    // We can safely assume the generate-ros-message or generate-messages binary script from
    // the rclnodejs pkg is available as the rclnodejs package is dependency.
    const cmd = `npx ${script}`;

    const child = exec(cmd);
    child.stdout.on('data', (data) => {
      console.log(data);
    });
    child.stderr.on('err', (data) => {
      console.warn(data);
    });
  }
}

module.exports = MsgGenerator;
