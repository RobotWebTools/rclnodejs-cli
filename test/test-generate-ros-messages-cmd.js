const childProcess = require('child_process');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const rimraf = require('rimraf');

describe('rclnodejs-cli generate-ros-messages command', function () {
  this.timeout(0);

  it('test generate-messages operation', (done) => {
    const cwd = process.cwd();

    const msgsFolder = path.join(cwd, 'node_modules', 'rclnodejs', 'generated');
    if (fs.existsSync(msgsFolder)) {
      rimraf.sync(msgsFolder);
    }

    const cli = path.join(path.dirname(cwd), 'rclnodejs-cli');
    const cmd = `npx ${cli} generate-ros-messages`;
    childProcess.execSync(cmd);

    assert.ok(
      fs.existsSync(msgsFolder),
      'No generated message folder found',
    );
    assert.ok(
      fs.existsSync(path.join(msgsFolder, 'std_msgs')),
      'std_msgs folder found',
    );
    done();
  });

  it('test generate-messages in non-package folder', (done) => {
    const cwd = process.cwd();
    const cli = path.join(cwd, '..', 'rclnodejs-cli');
    const tmpDir = fs.mkdtempSync('deleteme');
    childProcess.execSync(`npx ${cli} generate-ros-messages`, [], {
      cwd: tmpDir,
    });

    const msgsFolder = path.join(tmpDir, 'node_modules', 'rclnodejs', 'generated');
    assert.ok(
      !fs.existsSync(msgsFolder),
      'Generated message folder should not have been found',
    );

    if (fs.existsSync(tmpDir)) {
      fs.rmdirSync(tmpDir, { recursive: true });
    }

    done();
  });
});
