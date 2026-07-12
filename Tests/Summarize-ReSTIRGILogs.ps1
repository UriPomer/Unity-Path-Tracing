param(
    [string]$OutputRoot = "E:\WorkDoing\Unity\RayTracing\Unity-Path-Tracing\Tools\Output",
    [string]$OutputDir = "",
    [int]$Top = 8,
    [double]$MaxFinalContributionLum = 0.0,
    [double]$MaxFrameGIDeltaLum = 0.0
)

$ErrorActionPreference = 'Stop'

function Fail([string]$message) {
    Write-Output "FAIL: $message"
    exit 1
}

function Parse-JsonLines([string]$path) {
    if (-not (Test-Path $path)) {
        Fail "Missing log file: $path"
    }

    $lines = Get-Content $path | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
    if ($lines.Count -eq 0) {
        Fail "Empty log file: $path"
    }

    return $lines | ForEach-Object { $_ | ConvertFrom-Json }
}

function Parse-OptionalJsonLines([string]$path) {
    if (-not (Test-Path $path)) {
        return @()
    }

    $lines = Get-Content $path | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
    if ($lines.Count -eq 0) {
        return @()
    }

    return @($lines | ForEach-Object { $_ | ConvertFrom-Json })
}

function Resolve-ReSTIRGILogDir {
    if (-not [string]::IsNullOrWhiteSpace($OutputDir)) {
        if (-not (Test-Path $OutputDir)) {
            Fail "OutputDir not found: $OutputDir"
        }
        return (Resolve-Path $OutputDir).Path
    }

    if (-not (Test-Path $OutputRoot)) {
        Fail "OutputRoot not found: $OutputRoot"
    }

    $latestDir = Get-ChildItem -Path $OutputRoot -Directory |
        Sort-Object Name -Descending |
        Where-Object { Test-CompleteReSTIRGILogDir $_.FullName } |
        Select-Object -First 1

    if ($null -eq $latestDir) {
        Fail "No complete Tools/Output/<timestamp>/ ReSTIR log directory was found under $OutputRoot"
    }

    return $latestDir.FullName
}

function Test-CompleteReSTIRGILogDir([string]$dir) {
    return (Test-Path (Join-Path $dir 'restir_gi_probe.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_di_stats.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_gi_temporal_stats.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_gi_spatial_stats.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_gi_final_stats.jsonl'))
}

$logDir = Resolve-ReSTIRGILogDir
$sessionPath = Join-Path $logDir 'restir_session.jsonl'
if (Test-Path $sessionPath) {
    $sessionRows = @(Parse-JsonLines $sessionPath)
    $start = $sessionRows | Where-Object { $_.event -eq 'session_start' } | Select-Object -First 1
    $end = $sessionRows | Where-Object { $_.event -eq 'session_end' } | Select-Object -Last 1
    $stats = @(Parse-JsonLines (Join-Path $logDir 'restir_telemetry_stats.jsonl'))
    $events = @(Parse-OptionalJsonLines (Join-Path $logDir 'restir_events.jsonl'))
    $performance = @(Parse-JsonLines (Join-Path $logDir 'restir_performance.jsonl'))
    $diRows = @(Parse-OptionalJsonLines (Join-Path $logDir 'restir_di_stats.jsonl'))
    $probeRows = @(Parse-OptionalJsonLines (Join-Path $logDir 'restir_gi_probe.jsonl'))
    $temporalRows = @(Parse-OptionalJsonLines (Join-Path $logDir 'restir_gi_temporal_stats.jsonl'))
    $spatialRows = @(Parse-OptionalJsonLines (Join-Path $logDir 'restir_gi_spatial_stats.jsonl'))
    $finalRows = @(Parse-OptionalJsonLines (Join-Path $logDir 'restir_gi_final_stats.jsonl'))
    $firstIssue = $events | Where-Object { $_.severity -eq 'warning' -or $_.severity -eq 'error' } | Select-Object -First 1
    $averageCallback = ($performance | Measure-Object -Property callbackMilliseconds -Average).Average
    $maximumCallback = ($performance | Measure-Object -Property callbackMilliseconds -Maximum).Maximum
    $criticalNonFinite = ($stats | Measure-Object -Property criticalNonFinite -Sum).Sum
    $criticalOutOfRange = ($stats | Measure-Object -Property criticalOutOfRange -Sum).Sum
    $criticalBufferContract = ($stats | Measure-Object -Property criticalBufferContract -Sum).Sum

    Write-Output "Logs: $logDir"
    Write-Output "Scene: $($start.sceneName)"
    Write-Output "Session: $($start.sessionId) schema=$($start.schemaVersion)"
    Write-Output "Modes: DI=$($start.useReSTIRDI) GI=$($start.useReSTIRGI) Denoise=$($start.denoise)"
    Write-Output "Accepted/dropped/readback errors: $($end.acceptedCaptures)/$($end.droppedCaptures)/$($end.readbackErrors)"
    Write-Output "Telemetry rows: $($stats.Count)"
    Write-Output "DI initial/temporal/shade rows: $(@($diRows | Where-Object { $_.stage -eq 'di_initial' }).Count)/$(@($diRows | Where-Object { $_.stage -eq 'di_temporal' }).Count)/$(@($diRows | Where-Object { $_.stage -eq 'di_shade' }).Count)"
    Write-Output "GI initial/temporal/spatial/final rows: $($probeRows.Count)/$($temporalRows.Count)/$($spatialRows.Count)/$($finalRows.Count)"
    Write-Output "Critical nonfinite/out-of-range/buffer-contract: $criticalNonFinite/$criticalOutOfRange/$criticalBufferContract"
    Write-Output ("Callback milliseconds avg/max: {0:N3}/{1:N3}" -f $averageCallback, $maximumCallback)
    if ($null -ne $firstIssue) {
        Write-Output "First warning/error: $($firstIssue.severity) $($firstIssue.event) $($firstIssue.message)"
    }
    exit 0
}

$probeRows = Parse-JsonLines (Join-Path $logDir 'restir_gi_probe.jsonl')
$temporalRows = Parse-JsonLines (Join-Path $logDir 'restir_gi_temporal_stats.jsonl')
$spatialRows = Parse-JsonLines (Join-Path $logDir 'restir_gi_spatial_stats.jsonl')
$finalRows = Parse-JsonLines (Join-Path $logDir 'restir_gi_final_stats.jsonl')
$frameRows = Parse-OptionalJsonLines (Join-Path $logDir 'restir_gi_frame_stats.jsonl')

$primaryHitRows = @($probeRows | Where-Object { $_.primaryHit -eq $true })
$reusableRows = @($primaryHitRows | Where-Object { $_.probeClass -eq 'reservoir_reusable' })
$activeRows = @($reusableRows | Where-Object { $_.activeValid -eq $true })
$positiveFinalRows = @($finalRows | Where-Object { $_.finalContributionPositive -eq $true -and $_.finalContributionFinite -eq $true })
$maxFinal = $finalRows | Sort-Object finalContributionLum -Descending | Select-Object -First 1
$topFinal = $finalRows | Sort-Object finalContributionLum -Descending | Select-Object -First $Top

Write-Output "Logs: $logDir"
$scenes = @($probeRows | Select-Object -ExpandProperty sceneName -Unique)
Write-Output "Scenes: $($scenes -join ', ')"
Write-Output "Primary-hit probes: $($primaryHitRows.Count)"
Write-Output "Reusable probes: $($reusableRows.Count)"
Write-Output "Reusable active probes: $($activeRows.Count)"
Write-Output "Temporal rows: $($temporalRows.Count)"
Write-Output "Spatial rows: $($spatialRows.Count)"
Write-Output "Final rows: $($finalRows.Count)"
Write-Output "Positive final rows: $($positiveFinalRows.Count)"
Write-Output ("Max finalContributionLum: frame={0} probe={1} lum={2} finalTargetLum={3} finalWeightSum={4} finalSampleCountM={5}" -f `
    $maxFinal.frameIndex,
    $maxFinal.selectedProbeId,
    $maxFinal.finalContributionLum,
    $maxFinal.finalTargetLum,
    $maxFinal.finalWeightSum,
    $maxFinal.finalSampleCountM)

if ($frameRows.Count -gt 0) {
    $maxFrameDelta = $frameRows | Sort-Object maxGIDeltaLum -Descending | Select-Object -First 1
    $maxFrameAfter = $frameRows | Sort-Object maxAfterGILum -Descending | Select-Object -First 1
    Write-Output ("Max frame GI delta: frame={0} lum={1} pixel=({2},{3}) positivePixels={4} ratio={5} avgDelta={6}" -f `
        $maxFrameDelta.frameIndex,
        $maxFrameDelta.maxGIDeltaLum,
        $maxFrameDelta.maxGIDeltaPixelX,
        $maxFrameDelta.maxGIDeltaPixelY,
        $maxFrameDelta.positiveGIDeltaPixels,
        $maxFrameDelta.positiveGIDeltaPixelRatio,
        $maxFrameDelta.avgGIDeltaLum)
    if ($null -ne $maxFrameDelta.PSObject.Properties['maxGIDeltaFinalWeightSum']) {
        Write-Output ("  max-pixel final reservoir: valid={0} target={1} weight={2} M={3} proposalPdf={4}" -f `
            $maxFrameDelta.maxGIDeltaFinalValid,
            $maxFrameDelta.maxGIDeltaFinalTargetLum,
            $maxFrameDelta.maxGIDeltaFinalWeightSum,
            $maxFrameDelta.maxGIDeltaFinalSampleCountM,
            $maxFrameDelta.maxGIDeltaFinalProposalPdf)
        Write-Output ("  max-pixel initial reservoir: valid={0} target={1} weight={2} M={3} proposalPdf={4}" -f `
            $maxFrameDelta.maxGIDeltaInitialValid,
            $maxFrameDelta.maxGIDeltaInitialTargetLum,
            $maxFrameDelta.maxGIDeltaInitialWeightSum,
            $maxFrameDelta.maxGIDeltaInitialSampleCountM,
            $maxFrameDelta.maxGIDeltaInitialProposalPdf)
        Write-Output ("  max-pixel primary: hit={0} mode={1} distance={2} albedo=[{3}]" -f `
            $maxFrameDelta.maxGIDeltaPrimaryHit,
            $maxFrameDelta.maxGIDeltaPrimaryMode,
            $maxFrameDelta.maxGIDeltaPrimaryDistance,
            ($maxFrameDelta.maxGIDeltaPrimaryAlbedo -join ','))
    }
    Write-Output ("Max after-GI frame luminance: frame={0} lum={1} pixel=({2},{3}) avgAfter={4}" -f `
        $maxFrameAfter.frameIndex,
        $maxFrameAfter.maxAfterGILum,
        $maxFrameAfter.maxAfterGIPixelX,
        $maxFrameAfter.maxAfterGIPixelY,
        $maxFrameAfter.avgAfterGILum)
}
else {
    Write-Output "Frame-wide GI delta rows: not present in this log directory"
}

Write-Output ""
Write-Output "Top final GI rows:"
foreach ($row in $topFinal) {
    Write-Output ("  frame={0} sample={1} probe={2} lum={3} target={4} weight={5} M={6} finalValid={7} initialValid={8} contribution=[{9}]" -f `
        $row.frameIndex,
        $row.sampleCount,
        $row.selectedProbeId,
        $row.finalContributionLum,
        $row.finalTargetLum,
        $row.finalWeightSum,
        $row.finalSampleCountM,
        $row.finalValid,
        $row.initialValid,
        ($row.finalContribution -join ','))
}

if ($MaxFinalContributionLum -gt 0.0 -and [double]$maxFinal.finalContributionLum -gt $MaxFinalContributionLum) {
    Fail ("Max finalContributionLum exceeded threshold: lum={0:R} threshold={1:R}" -f [double]$maxFinal.finalContributionLum, $MaxFinalContributionLum)
}

if ($frameRows.Count -gt 0 -and $MaxFrameGIDeltaLum -gt 0.0) {
    $maxFrameDelta = $frameRows | Sort-Object maxGIDeltaLum -Descending | Select-Object -First 1
    if ([double]$maxFrameDelta.maxGIDeltaLum -gt $MaxFrameGIDeltaLum) {
        Fail ("Max frame GI delta exceeded threshold: frame={0} lum={1:R} pixel=({2},{3}) threshold={4:R}" -f `
            $maxFrameDelta.frameIndex,
            [double]$maxFrameDelta.maxGIDeltaLum,
            $maxFrameDelta.maxGIDeltaPixelX,
            $maxFrameDelta.maxGIDeltaPixelY,
            $MaxFrameGIDeltaLum)
    }
}

Write-Output ""
Write-Output "PASS"
