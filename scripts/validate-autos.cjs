const { join, resolve } = require('node:path');
const { spawnSync } = require('node:child_process');
const { validateTeam } = require('./zenith-project.cjs');

const root = resolve(__dirname, '..');
let autos;
try { autos = validateTeam(root).files.map(name => join('autos', name)); }
catch (error) { console.error(error.message); process.exit(1); }
const cli = join(root, 'node_modules/@horizon36596/zenith-cli/dist/index.js');
// Team factories are indirect; exact registry/resource parity is checked above and by runtime JUnit tests.
const result = spawnSync(process.execPath, [cli, 'validate', ...autos, '--json'],
  { cwd: root, encoding: 'utf8', maxBuffer: 8 * 1024 * 1024 });
if (result.error) throw result.error;
if (result.stderr) process.stderr.write(result.stderr);
if (result.stdout) {
  const report = JSON.parse(result.stdout);
  for (const file of report.files) {
    for (const warning of file.seasonWarnings || []) console.log('warning: '+file.path+': '+warning);
    const groups = new Map();
    for (const finding of file.findings) {
      const key = finding.severity+' '+finding.code+': '+finding.message;
      const previous=groups.get(key);
      if (previous) previous.count++;
      else groups.set(key,{count:1,step:finding.stepId});
    }
    for (const [message,group] of groups)
      console.log(file.path+' ['+group.step+'] '+message+(group.count > 1 ? ' (repeated '+group.count+' times by pinned scanner)' : ''));
  }
  console.log(report.errors+' Zenith error(s); team registry/profile/argument contracts agree.');
}
process.exit(result.status === null ? 1 : result.status);
