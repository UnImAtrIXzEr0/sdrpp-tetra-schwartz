# Fetch a build input (Windows counterpart of fetch_cached.sh).
# Usage: fetch_cached.ps1 <file name> <upstream url> <output path>
# 1. Take the copy stored in this repository's "build-cache" release, if present.
# 2. Otherwise download from upstream, retrying failed attempts.
# 3. On push builds, store a freshly downloaded file in the build-cache release,
#    so a later outage of the upstream host no longer breaks the build.
param([string]$Name, [string]$Url, [string]$OutFile)

$repo = $env:GITHUB_REPOSITORY
$tmp = Join-Path $env:RUNNER_TEMP "fetch_cached"
New-Item -ItemType Directory $tmp -Force | Out-Null
$cached = Join-Path $tmp $Name

$null = gh release download build-cache --repo $repo --pattern $Name --dir $tmp --clobber 2>&1
if ($LASTEXITCODE -eq 0 -and (Test-Path $cached) -and (Get-Item $cached).Length -gt 0) {
    Copy-Item $cached $OutFile -Force
    Write-Host "${Name}: taken from build-cache"
    exit 0
}

$ok = $false
for ($i = 1; $i -le 6 -and -not $ok; $i++) {
    try {
        Invoke-WebRequest -Uri $Url -OutFile $OutFile -TimeoutSec 300
        $ok = $true
    } catch {
        Write-Host "${Name}: attempt $i failed: $($_.Exception.Message)"
        if ($i -lt 6) { Start-Sleep -Seconds (30 * $i) }
    }
}
if (-not $ok) {
    Write-Host "::error::download of $Name failed: $Url"
    exit 1
}
Write-Host "${Name}: downloaded from upstream"

if ($env:GITHUB_EVENT_NAME -eq "push") {
    Copy-Item $OutFile $cached -Force
    $null = gh release upload build-cache --repo $repo --clobber $cached 2>&1
    if ($LASTEXITCODE -eq 0) { Write-Host "${Name}: stored in build-cache" }
    else { Write-Host "::warning::$Name could not be stored in build-cache" }
}
exit 0
