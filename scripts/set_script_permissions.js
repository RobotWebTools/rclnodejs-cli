#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

function updateScriptPermissions() {
  if (process.platform == 'win32') return;
  
  let folder = path.join(__dirname, '..', 'package-creation-tool', 'scripts');
  let fd = fs.openSync(folder, 'r');
  fs.fchmodSync(fd, 0o775);

  let script = path.join(folder, 'create_ros_nodejs_pkg.sh');
  fd = fs.openSync(script, 'r');
  fs.fchmodSync(fd, 0o775);
}

updateScriptPermissions();
