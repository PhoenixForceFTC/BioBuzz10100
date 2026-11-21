#!/usr/bin/env bash
# Decide how a change reaches the robot: full APK install, Sloth hot reload, or nothing.
#
# usage: classify-change.sh [base-ref]
#   base-ref  what is already on the robot: a commit, tag or branch (default: the upstream of the current
#             branch, else origin/V1). Uncommitted and untracked files always count as changes.
#
# Rules come from tools/deploy.ps1: a full install is needed after changing libraries, build.gradle,
# FtcRobotController, assets or @Pinned classes, and for the first deploy on a new robot. A new OpMode class is also
# treated as a full install (unverified whether hot reload registers it). Auto routine JSON
# files are APK assets, so an auto change is a full install. Java-only changes can use Sloth hot reload.
# Read-only: it changes nothing. Works with the bash 3.2 that ships with macOS.
set -u
cd "$(git rev-parse --show-toplevel)" || exit 2

base="${1:-}"
if [ -z "$base" ]; then
  base="$(git rev-parse --abbrev-ref --symbolic-full-name '@{upstream}' 2>/dev/null || echo origin/V1)"
fi
git rev-parse --verify --quiet "$base^{commit}" >/dev/null || { echo "unknown base ref: $base" >&2; exit 2; }
mb="$(git merge-base HEAD "$base")"

files="$( { git diff --name-only "$mb"; git ls-files --others --exclude-standard; } | sort -u )"
# Files that did not exist at the base: a brand-new OpMode class is treated as a full install, because it is not
# known whether Sloth hot reload registers a new @Autonomous/@TeleOp class with the Driver Station.
added="$( { git diff --name-only --diff-filter=A "$mb"; git ls-files --others --exclude-standard; } | sort -u )"
if [ -z "$files" ]; then
  echo "base: $base ($(git rev-parse --short "$mb"))"
  echo "VERDICT: NOTHING TO DEPLOY (no changes since the base)"
  exit 0
fi

full=""; java=""; other=""
while IFS= read -r f; do
  [ -z "$f" ] && continue
  case "$f" in
    TeamCode/src/main/assets/*)                     full="$full$f (APK asset: autos, field, robot, manifest)\n" ;;
    autos/*)                                        full="$full$f (auto source; deploy writes it into the APK assets)\n" ;;
    */build.gradle|build.gradle|*.gradle|gradle.properties|gradle/*|settings.gradle)
                                                    full="$full$f (build configuration)\n" ;;
    FtcRobotController/*|libs/*)                    full="$full$f (SDK or libraries)\n" ;;
    */AndroidManifest.xml|*/src/main/res/*)         full="$full$f (manifest or resources)\n" ;;
    TeamCode/src/main/java/*.java)
      if [ -f "$f" ] && grep -q "@Pinned" "$f"; then
        full="$full$f (@Pinned class)\n"
      elif [ -f "$f" ] && printf '%s\n' "$added" | grep -qxF "$f" && grep -qE "@(Autonomous|TeleOp)" "$f"; then
        full="$full$f (new OpMode class)\n"
      else
        java="$java$f\n"
      fi ;;
    *)                                              other="$other$f\n" ;;
  esac
done <<EOF
$files
EOF

echo "base: $base ($(git rev-parse --short "$mb"))"
[ -n "$full" ]  && { echo; echo "Needs a FULL INSTALL because of:"; printf "$full" | sed 's/^/  /'; }
[ -n "$java" ]  && { echo; echo "Java only (hot reload is enough on its own):"; printf "$java" | sed 's/^/  /'; }
[ -n "$other" ] && { echo; echo "No effect on the robot (docs, tests, tools, CI):"; printf "$other" | sed 's/^/  /'; }
echo
if [ -n "$full" ]; then
  echo "VERDICT: FULL INSTALL"
elif [ -n "$java" ]; then
  echo "VERDICT: SLOTH HOT RELOAD (a full install also works)"
else
  echo "VERDICT: NOTHING TO DEPLOY (only files that do not reach the robot changed)"
fi
