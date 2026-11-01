[CmdletBinding()]
param([string]$Apk = 'TeamCode/build/outputs/apk/debug/TeamCode-debug.apk')
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
Add-Type -AssemblyName System.IO.Compression.FileSystem
$apkPath = if ([IO.Path]::IsPathRooted($Apk)) { $Apk } else { Join-Path $repoRoot $Apk }
$manifestPath = Join-Path $repoRoot 'TeamCode/src/main/assets/autos/deployment-manifest.json'
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
$archive = [IO.Compression.ZipFile]::OpenRead($apkPath)
$sha = [Security.Cryptography.SHA256]::Create()
$verified = 0
try {
    $expected = @{}
    foreach ($property in $manifest.outputs.PSObject.Properties) {
        if (-not $property.Name.StartsWith('TeamCode/src/main/assets/autos/')) { continue }
        $entryName = 'assets/autos/' + $property.Name.Substring('TeamCode/src/main/assets/autos/'.Length)
        $expected[$entryName] = $property.Value
    }
    $expected['assets/autos/deployment-manifest.json'] = (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
    foreach ($entry in $archive.Entries) {
        if ($entry.FullName.StartsWith('assets/autos/') -and $entry.FullName.EndsWith('.json') -and -not $expected.ContainsKey($entry.FullName)) {
            throw "Unexpected packaged autonomous asset: $($entry.FullName)"
        }
    }
    foreach ($entryName in $expected.Keys) {
        $entry = $archive.GetEntry($entryName)
        if ($null -eq $entry) { throw "APK missing $entryName" }
        $stream = $entry.Open()
        try { $hash = [BitConverter]::ToString($sha.ComputeHash($stream)).Replace('-', '').ToLowerInvariant() }
        finally { $stream.Dispose() }
        if ($hash -ne $expected[$entryName]) { throw "APK has stale content: $entryName" }
        $verified++
    }
    Write-Host "$verified packaged Zenith assets match the source deployment manifest."
} finally {
    $archive.Dispose()
    $sha.Dispose()
}
