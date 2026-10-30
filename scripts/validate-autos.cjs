const { readdirSync } = require('node:fs');
const { join, resolve } = require('node:path');
const { spawnSync } = require('node:child_process');

const root = resolve(__dirname, '..');
const autos = readdirSync(join(root, 'autos'))
  .filter(name => name.endsWith('.auto.json')).sort().map(name => join('autos', name));
autos.push('TeamCode/src/main/assets/autos/pollen-navigation-test.auto.json');
const cli = join(root, 'node_modules/@horizon36596/zenith-cli/dist/index.js');
const result = spawnSync(process.execPath, [cli, 'validate', ...autos, '--java', 'TeamCode'],
  { cwd: root, stdio: 'inherit' });
if (result.error) throw result.error;
process.exit(result.status === null ? 1 : result.status);
