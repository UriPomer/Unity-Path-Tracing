$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$fixtureRoot = Join-Path $scriptDir 'Fixtures/ReSTIRGI_StrictPass'
$verifier = Join-Path $scriptDir 'Verify-LatestReSTIRGILogs.ps1'

& $verifier -OutputRoot $fixtureRoot -RequireFreshReSTIRGI -RequireReuseFormulaCoverage
