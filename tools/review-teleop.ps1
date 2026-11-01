[CmdletBinding()]
param(
    [ValidateSet('RedMecanumTeleOp','BlueMecanumTeleOp','DrivetrainTestOpMode','GateTestOpMode',
        'IntakeTestOpMode','TransferTestOpMode','DepollinatorTestOpMode','TurretTestOpMode',
        'IntakeShooterTestOpMode','ShotCalibrationOpMode','PinpointTestOpMode','AutoAimTestOpMode','LimelightTagTestOpMode',
        'GoalStateTestOpMode','PedroDriveTestOpMode')]
    [string]$OpMode = 'RedMecanumTeleOp',
    [ValidateRange(1,300)][int]$Seconds = 120,
    [ValidateSet('no-tags','raised-audience','raised-audience-normalized')][string]$Profile = 'no-tags',
    [string]$Replay,
    [switch]$Smoke
)
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $arguments = @(':TeamCode:simTeleOpReview', '--console=plain', "-Psim.opMode=$OpMode",
        "-Psim.seconds=$Seconds", "-Psim.profile=$Profile")
    if ($Replay) {
        if (-not (Test-Path -LiteralPath $Replay -PathType Leaf)) { throw "Track not found: $Replay" }
        $arguments += "-Psim.replay=$((Resolve-Path -LiteralPath $Replay).Path)"
    }
    if ($Smoke) { $arguments += '-Psim.smoke=true' }
    & .\gradlew.bat @arguments
    exit $LASTEXITCODE
} finally { Pop-Location }
