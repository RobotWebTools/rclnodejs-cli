const childProcess = require('child_process');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const rimraf = require('rimraf');

// HACK: on some windows systems the test_pkg folder was not
// being fully deleted without multiple tries. So try up to n attempts.
function rmdir(dir) {
  // eslint-disable-next-line no-plusplus
  for (let i = 0; i < 5; i++) {
    if (fs.existsSync(dir)) {
      rimraf.sync(dir);
    } else {
      break;
    }
  }
}

function verifyPackage(pkgPath, typescript = false, noInit = false, rclnodejsVersion = undefined) {
  assert.ok(fs.existsSync(pkgPath));
  assert.ok(fs.existsSync(path.join(pkgPath, 'package.xml')));
  assert.ok(fs.existsSync(path.join(pkgPath, 'package.json')));
  assert.ok(fs.existsSync(path.join(pkgPath, 'src')));

  if (typescript) {
    assert.ok(fs.existsSync(path.join(pkgPath, 'src', 'index.ts')));
    assert.ok(fs.existsSync(path.join(pkgPath, 'tsconfig.json')));
  } else {
    assert.ok(fs.existsSync(path.join(pkgPath, 'src', 'index.js')));
  }

  const nodeModulesDirExists = fs.existsSync(path.join(pkgPath, 'node_modules'));
  assert.ok(noInit ? !nodeModulesDirExists : nodeModulesDirExists);

  if (!noInit && rclnodejsVersion) {
    // load package.json and find the version number of the rclnodejs dependency
    // eslint-disable-next-line global-require, import/no-dynamic-require
    const pkg = require(path.join(pkgPath, 'package.json'));
    assert.ok(
      pkg
      && pkg.dependencies
      && pkg.dependencies.rclnodejs
      && pkg.dependencies.rclnodejs.endsWith(rclnodejsVersion)
    );
  }
}

function scriptExt() {
  return process.platform === 'win32' ? '.bat' : '.sh';
}

describe('rclnodejs-cli package-creation-tool', function () {
  this.timeout(0);

  const pkgName = 'test_pkg';

  beforeEach(function () {
    const cwd = process.cwd();
    const tmpDir = path.join(cwd, pkgName);
    rmdir(tmpDir);
  });

  afterEach(function () {
    const cwd = process.cwd();
    const tmpDir = path.join(cwd, pkgName);
    rmdir(tmpDir);
  });

  it('ros2cli-extension installation', (done) => {
    const cwd = process.cwd();

    let script = `run_ros2cli_pkg_help${scriptExt()}`;
    script = path.join(cwd, 'test', script);
    const stdoutBuf = childProcess.execSync(script, [], {
      cwd,
    });

    const lines = stdoutBuf.toString().match(/[^\r\n]+/g);
    let found = false;
    // eslint-disable-next-line no-restricted-syntax
    for (const line of lines) {
      if (line.trim().startsWith('create_nodejs')) {
        found = true;
        break;
      }
    }

    assert.ok(found);
    done();
  });

  it('ros2cli-extension create javascript package', (done) => {
    const cwd = process.cwd();

    let script = `run_ros2cli_pkg_create_nodejs${scriptExt()}`;
    script = path.join(cwd, 'test', script);
    childProcess.execSync(`${script} ${pkgName}`);

    const pkgPath = path.join(cwd, pkgName);
    verifyPackage(pkgPath, false);

    done();
  });

  it('ros2cli-extension create typescript package', (done) => {
    const cwd = process.cwd();

    let script = `run_ros2cli_pkg_create_nodejs${scriptExt()}`;
    script = path.join(cwd, 'test', script);
    childProcess.execSync(`${script} ${pkgName} --typescript`);

    const pkgPath = path.join(cwd, pkgName);
    verifyPackage(pkgPath, true);

    done();
  });

  it('ros2cli-extension create javascript package --rclnodejs-version', (done) => {
    const cwd = process.cwd();

    let script = `run_ros2cli_pkg_create_nodejs${scriptExt()}`;
    script = path.join(cwd, 'test', script);
    const version = '0.18.1';
    childProcess.execSync(`${script} ${pkgName} --rclnodejs-version ${version}`);

    const pkgPath = path.join(cwd, pkgName);
    verifyPackage(pkgPath, false, false, version);

    done();
  });

  it('ros2cli-extension create javascript package --no-init', (done) => {
    const cwd = process.cwd();

    let script = `run_ros2cli_pkg_create_nodejs${scriptExt()}`;
    script = path.join(cwd, 'test', script);
    childProcess.execSync(`${script} ${pkgName} --no-init`);

    const pkgPath = path.join(cwd, pkgName);
    verifyPackage(pkgPath, false, true);

    done();
  });

  it('rclnodejs-cli create-package javascript', (done) => {
    const cli = path.join(__dirname, '..', '..', 'rclnodejs-cli');
    childProcess.execSync(`npx ${cli} create-package ${pkgName}`).toString();

    const pkgPath = path.join(cli, pkgName);
    verifyPackage(pkgPath, false);

    done();
  });

  it('rclnodejs-cli create-package typescript', (done) => {
    const cli = path.join(__dirname, '..', '..', 'rclnodejs-cli');
    childProcess.execSync(`npx ${cli} create-package ${pkgName} --typescript`).toString();

    const pkgPath = path.join(cli, pkgName);
    verifyPackage(pkgPath, true);

    done();
  });

  it('rclnodejs-cli create-package javascript --no-init', (done) => {
    const cli = path.join(__dirname, '..', '..', 'rclnodejs-cli');
    childProcess.execSync(`npx ${cli} create-package ${pkgName} --no-init`).toString();

    const pkgPath = path.join(cli, pkgName);
    verifyPackage(pkgPath, false, false);

    done();
  });

  it('rclnodejs-cli create-package javascript --rclnodejs-version', (done) => {
    const version = '0.18.1';
    const cli = path.join(__dirname, '..', '..', 'rclnodejs-cli');
    childProcess.execSync(`npx ${cli} create-package ${pkgName} --rclnodejs-version ${version}`).toString();

    const pkgPath = path.join(cli, pkgName);
    verifyPackage(pkgPath, false, false, version);

    done();
  });
});
