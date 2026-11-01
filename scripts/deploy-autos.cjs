const path=require('node:path');
const {planTeamDeploy,applyTeamDeploy}=require('./zenith-project.cjs');
async function main() {
  const args=process.argv.slice(2);
  if (args.some(a=>!['--dry-run','--check'].includes(a))) throw Error('Usage: npm run zenith:deploy -- [--dry-run|--check]');
  const plan=await planTeamDeploy(path.resolve(__dirname,'..'));
  for (const [file] of plan.writes) console.log('write: '+file);
  for (const file of plan.removals) console.log('remove managed: '+file);
  if (args.includes('--check') && (plan.writes.length || plan.removals.length))
    throw Error('Zenith outputs are stale/duplicated; run npm run zenith:deploy');
  if (!args.includes('--dry-run') && !args.includes('--check')) applyTeamDeploy(plan);
  console.log(plan.writes.length+' write(s), '+plan.removals.length+' removal(s); reserved pollen Test wrapper retained.');
}
main().catch(error=>{ console.error(error.message); process.exitCode=1; });
