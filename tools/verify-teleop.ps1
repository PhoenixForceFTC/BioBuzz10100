[CmdletBinding()]
param(
    [switch]$Sim,
    [switch]$FullSim,
    [ValidateSet('Drivetrain', 'Gate', 'Mechanisms', 'Sensors', 'Competition')]
    [string[]]$Scenario = @('Drivetrain', 'Gate', 'Mechanisms', 'Sensors', 'Competition')
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent

function Assert-TestReport {
    param([string]$Task, [string[]]$Expected, [bool]$Simulation, [datetime]$Started)
    $resultDirectory = Join-Path $repoRoot "TeamCode/build/test-results/$Task"
    $reports = @(Get-ChildItem -LiteralPath $resultDirectory -Filter 'TEST-*.xml' -ErrorAction Stop)
    if ($reports.Count -eq 0) { throw "No JUnit XML reports from $Task" }
    $actual = @()
    foreach ($report in $reports) {
        if ($report.LastWriteTime -lt $Started.AddSeconds(-2)) { throw "Stale report: $($report.FullName)" }
        [xml]$xml = Get-Content -LiteralPath $report.FullName -Raw
        if ([int]$xml.testsuite.failures -ne 0 -or [int]$xml.testsuite.errors -ne 0 -or [int]$xml.testsuite.skipped -ne 0) {
            throw "Failed, errored, or skipped tests in $($report.Name)"
        }
        foreach ($test in $xml.testsuite.testcase) {
            $isSim = $test.classname -like 'org.firstinspires.ftc.teamcode.sim.*'
            if ($Simulation -ne $isSim) { throw "Unexpected test in ${Task}: $($test.classname)" }
            $actual += "$($test.classname)#$($test.name)"
        }
    }
    if ($actual.Count -eq 0) { throw "No tests executed by $Task" }
    foreach ($name in $Expected) {
        if ($name -notin $actual) { throw "Expected test was not executed by ${Task}: $name" }
    }
    Write-Host "$Task verified: $($actual.Count) tests, zero failures/errors/skips."
}

Push-Location $repoRoot
try {
    if ($Sim -and $FullSim) { throw 'Choose -Sim or -FullSim.' }
    if ($PSBoundParameters.ContainsKey('Scenario') -and -not $Sim) { throw '-Scenario requires -Sim.' }
    $baseline = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'teleop-test-baseline.json') -Raw | ConvertFrom-Json
    if ($FullSim) {
        $inventory = @(Get-Content -LiteralPath 'simloop/opmode-coverage.json' -Raw | ConvertFrom-Json)
        $sourceNames = @(Get-ChildItem -LiteralPath 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode' -Recurse -Filter '*.java' |
            Where-Object { (Get-Content -LiteralPath $_.FullName -Raw) -match '@TeleOp\s*\(' } |
            ForEach-Object { $_.BaseName })
        foreach ($name in $sourceNames) {
            if (@($inventory | Where-Object { $_.opMode -eq $name }).Count -ne 1) {
                throw "Classify new/duplicate OpMode ${name} in simloop/opmode-coverage.json (coverage or explicit unsupported reason)."
            }
        }
        foreach ($entry in $inventory) {
            if ($entry.opMode -notin $sourceNames) { throw "Stale OpMode inventory entry: $($entry.opMode)" }
            if ($entry.coverage -notin @('behavior','lifecycle','unsupported')) { throw 'Invalid coverage classification.' }
            if ($entry.coverage -eq 'unsupported' -and -not $entry.reason) { throw 'Unsupported coverage needs a reason.' }
        }
        Write-Host "OpMode inventory checked: $($sourceNames.Count) entries (including explicit unsupported boundaries)."
    }
    $started = Get-Date
    # Rerun just the test task, so discovery checks use fresh reports; build/lint retain incremental work.
    & .\gradlew.bat :TeamCode:testDebugUnitTest --rerun :TeamCode:assembleDebug :TeamCode:lintDebug --console=plain
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Assert-TestReport 'testDebugUnitTest' $baseline.unitTests $false $started

    if ($Sim -or $FullSim) {
        $simArguments = @(':TeamCode:simTeleOpTest', '--console=plain')
        $expected = @($baseline.simulationTests)
        if ($Sim) {
            $selectedClasses = @('SimRuntimeTest') + @($Scenario | ForEach-Object { "${_}SimTest" })
            foreach ($class in $selectedClasses) { $simArguments += @('--tests', "*.$class") }
            $expected = @($expected | Where-Object {
                $class = ($_.Split('#')[0] -split '\.')[-1]
                $class -in $selectedClasses
            })
        }
        if ($expected.Count -eq 0) { throw 'No expected simulations selected; check the baseline manifest.' }
        $started = Get-Date
        & .\gradlew.bat @simArguments
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        Assert-TestReport 'simTeleOpTest' $expected $true $started
        Write-Host 'Actual-OpMode simulations complete. Review model/convention limits; physical calibration is not certified.'
    } else {
        Write-Host 'Simulation not requested. Use -Sim -Scenario Drivetrain,Gate,Mechanisms,Sensors,Competition or -FullSim.'
    }
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
} finally {
    Pop-Location
}
exit 0
