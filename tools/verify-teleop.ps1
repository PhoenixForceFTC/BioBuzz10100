[CmdletBinding()]
param(
    [switch]$Sim,
    [switch]$FullSim,
    [ValidateSet('Drivetrain', 'Gate')]
    [string[]]$Scenario = @('Drivetrain', 'Gate')
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
        Write-Host 'Full-robot, red/blue competition, and physical checks remain outside this milestone.'
    } else {
        Write-Host 'Simulation not requested. Use -Sim [-Scenario Gate|Drivetrain] or -FullSim for all available simulations.'
    }
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
} finally {
    Pop-Location
}
exit 0
