param(
    [string]$OutputRoot = "E:\WorkDoing\Unity\RayTracing\Unity-Path-Tracing\Tools\Output",
    [switch]$RequireFreshReSTIRGI,
    [switch]$RequireReuseFormulaCoverage,
    [double]$MaxFinalContributionLum = 0.0
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
    if (-not (Test-Path $path)) { return @() }
    $lines = @(Get-Content $path | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
    return @($lines | ForEach-Object { $_ | ConvertFrom-Json })
}

function Assert-TelemetryStageRows($rows, [string]$sessionId, [string]$stage, [string]$path) {
    if ($rows.Count -eq 0) { Fail "Missing telemetry stage rows: $path" }
    foreach ($row in $rows) {
        if ($row.sessionId -ne $sessionId -or $row.stage -ne $stage -or
            [int]$row.frameIndex -lt 0 -or [int]$row.sampleCount -lt 0 -or
            [int]$row.generation -le 0 -or [int]$row.pixelIndex -lt 0) {
            Fail "Invalid telemetry correlation in $path"
        }
        if ($null -eq $row.payload -or $row.payload.Count -ne 24) {
            Fail "Telemetry payload must contain 24 floats: $path"
        }
        if ($row.severity -eq 'error') {
            Fail "Telemetry stage error: stage=$stage frame=$($row.frameIndex) reason=$($row.reason)"
        }
    }
}

function Verify-TelemetrySession([System.IO.DirectoryInfo]$directory) {
    $sessionRows = @(Parse-JsonLines (Join-Path $directory.FullName 'restir_session.jsonl'))
    $start = @($sessionRows | Where-Object { $_.event -eq 'session_start' } | Select-Object -First 1)
    $end = @($sessionRows | Where-Object { $_.event -eq 'session_end' } | Select-Object -Last 1)
    if ($start.Count -eq 0 -or $end.Count -eq 0 -or
        [string]::IsNullOrWhiteSpace($start[0].sessionId) -or $start[0].sessionId -ne $end[0].sessionId) {
        Fail "Telemetry session is missing correlated session_start/session_end rows"
    }

    $sessionId = [string]$start[0].sessionId
    if ($start[0].useReSTIRDI -ne $true -or $start[0].useReSTIRGI -ne $true -or $start[0].denoise -ne $false) {
        Fail "Runtime proof requires useReSTIRDI=true, useReSTIRGI=true, denoise=false"
    }
    if ([int]$end[0].acceptedCaptures -le 0 -or [int]$end[0].readbackErrors -ne 0) {
        Fail "Telemetry session ended with acceptedCaptures=$($end[0].acceptedCaptures), readbackErrors=$($end[0].readbackErrors)"
    }

    $stats = @(Parse-JsonLines (Join-Path $directory.FullName 'restir_telemetry_stats.jsonl'))
    foreach ($row in $stats) {
        if ($row.sessionId -ne $sessionId -or [int]$row.generation -le 0 -or
            [int]$row.renderWidth -le 0 -or [int]$row.renderHeight -le 0) {
            Fail "Invalid telemetry stats correlation"
        }
        if (([int]$row.modeFlags -band 3) -ne 3 -or ([int]$row.modeFlags -band 16) -ne 0) {
            Fail "Telemetry packet does not match DI+GI enabled, denoise disabled baseline at frame=$($row.frameIndex)"
        }
        if ([int]$row.criticalNonFinite -ne 0 -or [int]$row.criticalOutOfRange -ne 0 -or
            [int]$row.criticalBufferContract -ne 0) {
            Fail "Critical telemetry counter failed at frame=$($row.frameIndex)"
        }
    }

    $events = @(Parse-OptionalJsonLines (Join-Path $directory.FullName 'restir_events.jsonl'))
    $errorEvent = $events | Where-Object { $_.severity -eq 'error' } | Select-Object -First 1
    if ($null -ne $errorEvent) {
        Fail "Telemetry error event: event=$($errorEvent.event) frame=$($errorEvent.frameIndex) message=$($errorEvent.message)"
    }
    if (@($events | Where-Object { $_.sessionId -ne $sessionId }).Count -gt 0) {
        Fail "Telemetry event session ID mismatch"
    }

    $performance = @(Parse-JsonLines (Join-Path $directory.FullName 'restir_performance.jsonl'))
    if ($performance.Count -lt [int]$end[0].acceptedCaptures) {
        Fail "Telemetry performance rows do not cover accepted captures"
    }

    $probePath = Join-Path $directory.FullName 'restir_gi_probe.jsonl'
    $finalPath = Join-Path $directory.FullName 'restir_gi_final_stats.jsonl'
    $diPath = Join-Path $directory.FullName 'restir_di_stats.jsonl'
    $di = @(Parse-JsonLines $diPath)
    $diInitial = @($di | Where-Object { $_.stage -eq 'di_initial' })
    $diTemporal = @($di | Where-Object { $_.stage -eq 'di_temporal' })
    $diShade = @($di | Where-Object { $_.stage -eq 'di_shade' })
    Assert-TelemetryStageRows $diInitial $sessionId 'di_initial' $diPath
    Assert-TelemetryStageRows $diTemporal $sessionId 'di_temporal' $diPath
    Assert-TelemetryStageRows $diShade $sessionId 'di_shade' $diPath
    $probe = @(Parse-JsonLines $probePath)
    $final = @(Parse-JsonLines $finalPath)
    Assert-TelemetryStageRows $probe $sessionId 'gi_initial' $probePath
    Assert-TelemetryStageRows $final $sessionId 'gi_final' $finalPath

    $temporal = @(Parse-OptionalJsonLines (Join-Path $directory.FullName 'restir_gi_temporal_stats.jsonl'))
    $spatial = @(Parse-OptionalJsonLines (Join-Path $directory.FullName 'restir_gi_spatial_stats.jsonl'))
    if ($RequireReuseFormulaCoverage) {
        Assert-TelemetryStageRows $temporal $sessionId 'gi_temporal' 'restir_gi_temporal_stats.jsonl'
        Assert-TelemetryStageRows $spatial $sessionId 'gi_spatial' 'restir_gi_spatial_stats.jsonl'
    }

    $averageCallback = ($performance | Measure-Object -Property callbackMilliseconds -Average).Average
    $maximumCallback = ($performance | Measure-Object -Property callbackMilliseconds -Maximum).Maximum
    Write-Output 'PASS'
    Write-Output ''
    Write-Output "Logs verified: $($directory.FullName)"
    Write-Output "Telemetry schema: $($start[0].schemaVersion)"
    Write-Output "Accepted/dropped/readback errors: $($end[0].acceptedCaptures)/$($end[0].droppedCaptures)/$($end[0].readbackErrors)"
    Write-Output "DI initial/temporal/shade rows: $($diInitial.Count)/$($diTemporal.Count)/$($diShade.Count)"
    Write-Output "GI initial/final rows: $($probe.Count)/$($final.Count)"
    Write-Output "GI temporal/spatial rows: $($temporal.Count)/$($spatial.Count)"
    Write-Output ("Callback milliseconds avg/max: {0:N3}/{1:N3}" -f $averageCallback, $maximumCallback)
}

function Get-VectorComponent($row, [string]$name, [int]$index) {
    $vector = $row.$name
    if ($null -eq $vector -or $vector.Count -le $index) {
        Fail "Missing vector component: $name[$index]"
    }

    return [double]$vector[$index]
}

function Get-VectorMaxComponent($row, [string]$name) {
    $x = Get-VectorComponent $row $name 0
    $y = Get-VectorComponent $row $name 1
    $z = Get-VectorComponent $row $name 2
    return [Math]::Max($x, [Math]::Max($y, $z))
}

function Get-Scalar($row, [string]$name) {
    $property = $row.PSObject.Properties[$name]
    if ($null -eq $property) {
        Fail "Missing scalar field: $name"
    }

    return [double]$property.Value
}

function Test-TrueField($row, [string]$name) {
    $property = $row.PSObject.Properties[$name]
    return $null -ne $property -and $property.Value -eq $true
}

function Test-FinalRawShaderSchema($row) {
    $schemaProperty = $row.PSObject.Properties['finalDiagnosticSchemaVersion']
    if ($null -eq $schemaProperty) {
        return $false
    }

    return [double]$schemaProperty.Value -ge 2
}

function Assert-FinalRawShaderSchema($row) {
    if (-not (Test-FinalRawShaderSchema $row)) {
        return
    }

    $sourceProperty = $row.PSObject.Properties['finalReflectedRadianceSource']
    if ($null -eq $sourceProperty -or [string]$sourceProperty.Value -ne 'shaderRaw') {
        Fail "finalDiagnosticSchemaVersion>=2 requires finalReflectedRadianceSource=shaderRaw frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    }
}

function Assert-Near([double]$actual, [double]$expected, [string]$label) {
    $tolerance = [Math]::Max(1e-4, [Math]::Abs($expected) * 1e-4)
    if ([Math]::Abs($actual - $expected) -gt $tolerance) {
        Fail ("Formula mismatch for {0}: actual={1:R} expected={2:R} tolerance={3:R}" -f $label, $actual, $expected, $tolerance)
    }
}

function Assert-IntegerInRange([double]$actual, [double]$min, [double]$max, [string]$label) {
    $rounded = [Math]::Round($actual)
    if ([Math]::Abs($actual - $rounded) -gt 1e-4 -or $actual -lt $min -or $actual -gt $max) {
        Fail ("Invalid count field for {0}: actual={1:R} expected integer in [{2:R}, {3:R}]" -f $label, $actual, $min, $max)
    }
}

function Assert-SpatialDebugCountsAreClean($row) {
    $frameProbe = "frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderCompatibleNeighbors') 0 8 "spatialShaderCompatibleNeighbors $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateNeighbors') 0 8 "spatialShaderReevaluateNeighbors $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderJacobianNeighbors') 0 8 "spatialShaderJacobianNeighbors $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderCombinedNeighbors') 0 8 "spatialShaderCombinedNeighbors $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateFailInvalidSample') 0 8 "spatialShaderReevaluateFailInvalidSample $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateFailInvalidSurface') 0 8 "spatialShaderReevaluateFailInvalidSurface $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateFailDistance') 0 8 "spatialShaderReevaluateFailDistance $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateFailBrdf') 0 8 "spatialShaderReevaluateFailBrdf $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateFailBackfacing') 0 8 "spatialShaderReevaluateFailBackfacing $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialShaderReevaluateFailZeroTarget') 0 8 "spatialShaderReevaluateFailZeroTarget $frameProbe"
    Assert-IntegerInRange (Get-Scalar $row 'spatialSelectedNeighborIndex') -1 7 "spatialSelectedNeighborIndex $frameProbe"
}

function Assert-TemporalDebugStateIsClean($row) {
    $frameProbe = "frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    $selectedPrevious = $row.selectedPrevious -eq $true
    $combinedPrevious = Test-TrueField $row 'combinedPrevious'
    Assert-IntegerInRange (Get-Scalar $row 'selectedTemporalOffsetIndex') -1 4 "selectedTemporalOffsetIndex $frameProbe"

    if ($null -eq $row.PSObject.Properties['combinedPrevious']) {
        Fail "Fresh temporal row is missing combinedPrevious $frameProbe"
    }

    if ($selectedPrevious) {
        Assert-IntegerInRange (Get-Scalar $row 'selectedTemporalOffsetIndex') 0 4 "selected previous temporal offset $frameProbe"

        if ((Get-Scalar $row 'selectedTargetPdf') -le 0.0) {
            Fail "Selected temporal history row has non-positive selectedTargetPdf $frameProbe"
        }

        if ((Get-Scalar $row 'selectedPrevOriginalProposalPdf') -le 0.0) {
            Fail "Selected temporal history row has non-positive selectedPrevOriginalProposalPdf $frameProbe"
        }

        if ((Get-Scalar $row 'selectedPrevReuseProposalPdf') -le 0.0) {
            Fail "Selected temporal history row has non-positive selectedPrevReuseProposalPdf $frameProbe"
        }

        if ((Get-Scalar $row 'selectedPrevJacobian') -le 0.0) {
            Fail "Selected temporal history row has non-positive selectedPrevJacobian $frameProbe"
        }
    }

    if ($combinedPrevious) {
        if ((Get-Scalar $row 'combinedPrevTargetPdf') -le 0.0) {
            Fail "Combined temporal history row has non-positive combinedPrevTargetPdf $frameProbe"
        }

        if ((Get-Scalar $row 'combinedPrevOriginalProposalPdf') -le 0.0) {
            Fail "Combined temporal history row has non-positive combinedPrevOriginalProposalPdf $frameProbe"
        }

        if ((Get-Scalar $row 'combinedPrevReuseProposalPdf') -le 0.0) {
            Fail "Combined temporal history row has non-positive combinedPrevReuseProposalPdf $frameProbe"
        }

        if ((Get-Scalar $row 'combinedPrevJacobian') -le 0.0) {
            Fail "Combined temporal history row has non-positive combinedPrevJacobian $frameProbe"
        }

        if ((Get-Scalar $row 'combinedPrevSampleCountM') -le 0.0) {
            Fail "Combined temporal history row has non-positive combinedPrevSampleCountM $frameProbe"
        }
    }
}

$RestirGIFireflyLumLimit = 128.0
function Assert-WeightedReflectedRadiance(
    $row,
    [string]$rawName,
    [string]$weightedName,
    [string]$weightSumName,
    [string]$sampleCountName,
    [string]$label) {
    $weightSum = Get-Scalar $row $weightSumName
    $expectedX = (Get-VectorComponent $row $rawName 0) * $weightSum
    $expectedY = (Get-VectorComponent $row $rawName 1) * $weightSum
    $expectedZ = (Get-VectorComponent $row $rawName 2) * $weightSum

    $lum = [Math]::Max($expectedX, [Math]::Max($expectedY, $expectedZ))
    if ($lum -gt $RestirGIFireflyLumLimit -and $lum -gt 0.0) {
        $scale = $RestirGIFireflyLumLimit / $lum
        $expectedX *= $scale
        $expectedY *= $scale
        $expectedZ *= $scale
    }

    $expected = @($expectedX, $expectedY, $expectedZ)
    for ($component = 0; $component -lt 3; $component++) {
        $actual = Get-VectorComponent $row $weightedName $component
        Assert-Near $actual $expected[$component] "$label weightedReflectedRadiance component=$component frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    }
}

function Test-CompleteReSTIRGILogDir([string]$dir) {
    return (Test-Path (Join-Path $dir 'restir_gi_probe.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_di_stats.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_gi_temporal_stats.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_gi_spatial_stats.jsonl')) -and
        (Test-Path (Join-Path $dir 'restir_gi_final_stats.jsonl'))
}

if (-not (Test-Path $OutputRoot)) {
    Fail "Output root not found: $OutputRoot"
}

$latestDir = Get-ChildItem -Path $OutputRoot -Directory |
    Sort-Object Name -Descending |
    Where-Object { Test-CompleteReSTIRGILogDir $_.FullName } |
    Select-Object -First 1

if ($null -eq $latestDir) {
    Fail "No complete Tools/Output/<timestamp>/ ReSTIR log directory was found under $OutputRoot"
}

if (Test-Path (Join-Path $latestDir.FullName 'restir_session.jsonl')) {
    Verify-TelemetrySession $latestDir
    exit 0
}

$probeRows = Parse-JsonLines (Join-Path $latestDir.FullName 'restir_gi_probe.jsonl')
$diRows = Parse-JsonLines (Join-Path $latestDir.FullName 'restir_di_stats.jsonl')
$temporalRows = Parse-JsonLines (Join-Path $latestDir.FullName 'restir_gi_temporal_stats.jsonl')
$spatialRows = Parse-JsonLines (Join-Path $latestDir.FullName 'restir_gi_spatial_stats.jsonl')
$finalRows = Parse-JsonLines (Join-Path $latestDir.FullName 'restir_gi_final_stats.jsonl')

$primaryHitRows = @($probeRows | Where-Object { $_.primaryHit -eq $true })
if ($primaryHitRows.Count -eq 0) {
    Fail "No primary-hit probe rows found in $($latestDir.FullName)"
}

$RestirGIMinProposalPdf = 1e-3
$freshInitialRows = @($primaryHitRows | Where-Object {
    $_.initialValid -eq $true -and
    [double]$_.initialProposalPdf -gt 0 -and
    [double]$_.initialWeightSum -gt 0 -and
    [double]$_.initialSampleCountM -eq 1
})
if ($freshInitialRows.Count -eq 0) {
    Fail "No fresh initial GI reservoir rows found for proposalPdf inverse-weight checks in $($latestDir.FullName)"
}

foreach ($row in $freshInitialRows) {
    $expectedInitialWeight = 1.0 / [Math]::Max([double]$row.initialProposalPdf, $RestirGIMinProposalPdf)
    Assert-Near ([double]$row.initialWeightSum) $expectedInitialWeight "initialWeightSum=1/proposalPdf frame=$($row.frameIndex) probe=$($row.probeId)"
    Assert-Near ([double]$row.initialSelectedWeight) ([double]$row.initialWeightSum) "initialSelectedWeight mirrors weightSum frame=$($row.frameIndex) probe=$($row.probeId)"
    Assert-Near ([double]$row.initialProposalPdf) ([double]$row.secondaryProposalPdf) "initialProposalPdf propagates secondary proposalPdf frame=$($row.frameIndex) probe=$($row.probeId)"
}

$reusableActiveRows = @($primaryHitRows | Where-Object { $_.probeClass -eq 'reservoir_reusable' -and $_.activeValid -eq $true })
if ($reusableActiveRows.Count -eq 0) {
    Fail "No reusable active GI probe rows found in $($latestDir.FullName)"
}

$validTemporal = @($temporalRows | Where-Object {
    $_.proposalPdf -gt 0 -and
    $_.targetLum -gt 0 -and
    $_.weightSum -gt 0 -and
    $_.selectedWeight -gt 0 -and
    $_.sampleCountM -gt 0
})
if ($validTemporal.Count -eq 0) {
    Fail "No valid temporal GI summary rows found in $($latestDir.FullName)"
}

$temporalCurrentOnlyRows = @($validTemporal | Where-Object {
    $_.selectedPrevious -eq $false -and
    -not (Test-TrueField $_ 'combinedPrevious') -and
    [double]$_.sampleCountM -gt 0
})
if ($temporalCurrentOnlyRows.Count -eq 0) {
    Fail "No temporal current-only rows found for normalization formula checks in $($latestDir.FullName)"
}

foreach ($row in $temporalCurrentOnlyRows) {
    $targetLum = [double]$row.targetLum
    $expectedWeight = 1.0 / [Math]::Max([double]$row.proposalPdf, $RestirGIMinProposalPdf)
    $expectedPiSum = $targetLum * [Math]::Max([double]$row.sampleCountM, 1.0)
    $expectedDenominator = [double]$row.selectedTargetPdf * [double]$row.temporalPiSum

    Assert-Near ([double]$row.currentTargetPdf) $targetLum "temporal currentTargetPdf frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.selectedTargetPdf) $targetLum "temporal selectedTargetPdf frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalPi) $targetLum "temporal pi frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalPiSum) $expectedPiSum "temporal piSum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalNormalizationDenominator) $expectedDenominator "temporal normalizationDenominator frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.weightSum) $expectedWeight "temporal current-only weightSum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.selectedWeight) ([double]$row.weightSum) "temporal current-only selectedWeight mirrors weightSum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
}

$temporalSelectedHistoryRows = @($validTemporal | Where-Object {
    $_.selectedPrevious -eq $true -and
    [double]$_.selectedTargetPdf -gt 0 -and
    [double]$_.selectedPrevOriginalProposalPdf -gt 0 -and
    [double]$_.selectedPrevReuseProposalPdf -gt 0 -and
    [double]$_.selectedPrevJacobian -gt 0
})
foreach ($row in $temporalSelectedHistoryRows) {
    $expectedReuseProposalPdf = [Math]::Max(
        [double]$row.selectedPrevOriginalProposalPdf / [double]$row.selectedPrevJacobian,
        $RestirGIMinProposalPdf)
    $currentM = [Math]::Max([double]$row.sampleCountM - [double]$row.combinedPrevSampleCountM, 0.0)
    $expectedPiSum = [double]$row.currentTargetPdf * $currentM + [double]$row.combinedPrevPi * [double]$row.combinedPrevSampleCountM
    $expectedDenominator = [double]$row.selectedTargetPdf * [double]$row.temporalPiSum

    Assert-Near ([double]$row.selectedPrevReuseProposalPdf) $expectedReuseProposalPdf "temporal selected-history proposalPdf/jacobian frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalPi) ([double]$row.combinedPrevPi) "temporal selected-history pi frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalPiSum) $expectedPiSum "temporal selected-history piSum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalNormalizationDenominator) $expectedDenominator "temporal selected-history normalizationDenominator frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
}

$temporalCurrentSelectedHistoryRows = @($validTemporal | Where-Object {
    $_.selectedPrevious -eq $false -and
    (Test-TrueField $_ 'combinedPrevious') -and
    [double]$_.selectedTargetPdf -gt 0 -and
    [double]$_.combinedPrevOriginalProposalPdf -gt 0 -and
    [double]$_.combinedPrevReuseProposalPdf -gt 0 -and
    [double]$_.combinedPrevJacobian -gt 0 -and
    [double]$_.combinedPrevSampleCountM -gt 0
})
foreach ($row in $temporalCurrentSelectedHistoryRows) {
    $expectedReuseProposalPdf = [Math]::Max(
        [double]$row.combinedPrevOriginalProposalPdf / [double]$row.combinedPrevJacobian,
        $RestirGIMinProposalPdf)
    $currentM = [Math]::Max([double]$row.sampleCountM - [double]$row.combinedPrevSampleCountM, 0.0)
    $expectedPiSum = [double]$row.currentTargetPdf * $currentM + [double]$row.combinedPrevPi * [double]$row.combinedPrevSampleCountM
    $expectedDenominator = [double]$row.selectedTargetPdf * [double]$row.temporalPiSum

    Assert-Near ([double]$row.combinedPrevReuseProposalPdf) $expectedReuseProposalPdf "temporal current-selected history proposalPdf/jacobian frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalPi) ([double]$row.selectedTargetPdf) "temporal current-selected pi frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalPiSum) $expectedPiSum "temporal current-selected piSum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.temporalNormalizationDenominator) $expectedDenominator "temporal current-selected normalizationDenominator frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
}

if ($RequireReuseFormulaCoverage -and $temporalSelectedHistoryRows.Count -eq 0) {
    Fail "RequireReuseFormulaCoverage expected at least one temporal selected-history Jacobian row in $($latestDir.FullName)"
}

if ($RequireFreshReSTIRGI) {
    foreach ($row in $temporalRows) {
        Assert-TemporalDebugStateIsClean $row
    }
}

$combinedSpatial = @($spatialRows | Where-Object { $_.spatialShaderCombinedNeighbors -gt 0 })
if ($RequireReuseFormulaCoverage -and $combinedSpatial.Count -eq 0) {
    Fail "No spatial rows with combined neighbors found in $($latestDir.FullName)"
}

$spatialSelectedNeighborRows = @($spatialRows | Where-Object {
    [double]$_.spatialSelectedTargetPdf -gt 0 -and
    [double]$_.spatialSelectedNeighborOriginalProposalPdf -gt 0 -and
    [double]$_.spatialSelectedNeighborReuseProposalPdf -gt 0 -and
    [double]$_.spatialSelectedNeighborJacobian -gt 0 -and
    [double]$_.spatialSelectedNeighborTargetPdf -gt 0
})
foreach ($row in $spatialSelectedNeighborRows) {
    $expectedReuseProposalPdf = [Math]::Max(
        [double]$row.spatialSelectedNeighborOriginalProposalPdf / [double]$row.spatialSelectedNeighborJacobian,
        $RestirGIMinProposalPdf)
    $expectedDenominator = [double]$row.spatialSelectedTargetPdf * [double]$row.spatialPiSum

    Assert-Near ([double]$row.spatialSelectedNeighborReuseProposalPdf) $expectedReuseProposalPdf "spatial selected-neighbor proposalPdf/jacobian frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.spatialSelectedTargetPdf) ([double]$row.spatialSelectedNeighborTargetPdf) "spatial selected target mirrors selected-neighbor target frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
    Assert-Near ([double]$row.spatialNormalizationDenominator) $expectedDenominator "spatial selected-neighbor normalizationDenominator frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
}

if ($RequireReuseFormulaCoverage -and $spatialSelectedNeighborRows.Count -eq 0) {
    Fail "RequireReuseFormulaCoverage expected at least one spatial selected-neighbor Jacobian row in $($latestDir.FullName)"
}

if ($RequireFreshReSTIRGI) {
    foreach ($row in $spatialRows) {
        Assert-SpatialDebugCountsAreClean $row
    }
}

$validFinal = @($finalRows | Where-Object {
    $_.finalContributionFinite -eq $true -and
    $_.finalContributionPositive -eq $true -and
    $_.globalLightDeltaPositive -eq $true
})
$finalRawShaderSchemaRows = @($finalRows | Where-Object { Test-FinalRawShaderSchema $_ })
$targetPdfCheckedRows = 0
$targetPdfRawShaderCheckedRows = 0
$weightedReflectedCheckedRows = 0
if ($validFinal.Count -eq 0) {
    Fail "No valid final GI contribution rows found in $($latestDir.FullName)"
}

foreach ($row in $validFinal) {
    Assert-FinalRawShaderSchema $row
    $hasRawShaderSchema = Test-FinalRawShaderSchema $row

    for ($component = 0; $component -lt 3; $component++) {
        $finalWeighted = Get-VectorComponent $row 'finalWeightedReflectedRadiance' $component
        $initialWeighted = Get-VectorComponent $row 'initialWeightedReflectedRadiance' $component
        $expectedContribution =
            $finalWeighted * [double]$row.misFinalWeight +
            $initialWeighted * [double]$row.misInitialWeight
        $actualContribution = Get-VectorComponent $row 'finalContribution' $component
        $actualDelta = Get-VectorComponent $row 'globalLightDelta' $component
        $label = "finalContribution frame=$($row.frameIndex) probe=$($row.selectedProbeId) component=$component"
        Assert-Near $actualContribution $expectedContribution $label
        Assert-Near $actualDelta $actualContribution "globalLightDelta frame=$($row.frameIndex) probe=$($row.selectedProbeId) component=$component"
    }

    if ($row.finalValid -eq $true -and $row.hasFinalSample -eq $true -and [double]$row.finalWeightSum -gt 0) {
        $finalTargetFromRadiance = Get-VectorMaxComponent $row 'finalReflectedRadiance'
        Assert-Near ([double]$row.finalTargetLum) $finalTargetFromRadiance "finalTargetLum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
        Assert-WeightedReflectedRadiance $row 'finalReflectedRadiance' 'finalWeightedReflectedRadiance' 'finalWeightSum' 'finalSampleCountM' 'final'
        $targetPdfCheckedRows++
        if ($hasRawShaderSchema) { $targetPdfRawShaderCheckedRows++ }
        $weightedReflectedCheckedRows++
    }

    if ($row.initialValid -eq $true -and $row.hasInitialSample -eq $true -and [double]$row.initialWeightSum -gt 0) {
        $initialTargetFromRadiance = Get-VectorMaxComponent $row 'initialReflectedRadiance'
        Assert-Near ([double]$row.initialTargetLum) $initialTargetFromRadiance "initialTargetLum frame=$($row.frameIndex) probe=$($row.selectedProbeId)"
        Assert-WeightedReflectedRadiance $row 'initialReflectedRadiance' 'initialWeightedReflectedRadiance' 'initialWeightSum' 'initialSampleCountM' 'initial'
        $targetPdfCheckedRows++
        if ($hasRawShaderSchema) { $targetPdfRawShaderCheckedRows++ }
        $weightedReflectedCheckedRows++
    }
}

if ($targetPdfCheckedRows -eq 0) {
    Fail "No final GI rows were eligible for targetPdf/reflectedRadiance formula checks"
}

if ($RequireFreshReSTIRGI) {
    if ($finalRawShaderSchemaRows.Count -eq 0) {
        Fail "RequireFreshReSTIRGI expected finalDiagnosticSchemaVersion>=2 shader-raw final rows in $($latestDir.FullName)"
    }

    if ($targetPdfRawShaderCheckedRows -eq 0) {
        Fail "RequireFreshReSTIRGI expected at least one shader-raw targetPdf branch in $($latestDir.FullName)"
    }
}

$scenes = @($probeRows | Select-Object -ExpandProperty sceneName -Unique)
$sceneSummary = if ($scenes.Count -gt 0) { $scenes -join ', ' } else { '(unknown)' }
$maxFinal = $finalRows | Sort-Object finalContributionLum -Descending | Select-Object -First 1
if ($MaxFinalContributionLum -gt 0.0 -and [double]$maxFinal.finalContributionLum -gt $MaxFinalContributionLum) {
    Fail ("Max finalContributionLum exceeded threshold in {0}: frame={1} probe={2} lum={3:R} threshold={4:R}" -f `
        $latestDir.FullName, $maxFinal.frameIndex, $maxFinal.selectedProbeId, [double]$maxFinal.finalContributionLum, $MaxFinalContributionLum)
}
$uniqueDiSamples = @($diRows | ForEach-Object { '{0}:{1}' -f $_.lightType, $_.lightIndex } | Select-Object -Unique)
$mostReusedDiSample = $diRows |
    Group-Object { '{0}:{1}' -f $_.lightType, $_.lightIndex } |
    Sort-Object Count -Descending |
    Select-Object -First 1

Write-Output "PASS"
Write-Output ""
Write-Output "Logs verified: $($latestDir.FullName)"
Write-Output "Scenes: $sceneSummary"
Write-Output "Primary-hit rows: $($primaryHitRows.Count)"
Write-Output "ProposalPdf-checked fresh reservoirs: $($freshInitialRows.Count)"
Write-Output "Reusable active rows: $($reusableActiveRows.Count)"
Write-Output "Temporal rows: $($validTemporal.Count)"
Write-Output "Temporal current-only formula rows: $($temporalCurrentOnlyRows.Count)"
Write-Output "Temporal selected-history Jacobian rows: $($temporalSelectedHistoryRows.Count)"
Write-Output "Temporal current-selected history rows: $($temporalCurrentSelectedHistoryRows.Count)"
Write-Output "Spatial rows: $($combinedSpatial.Count)"
Write-Output "Spatial selected-neighbor Jacobian rows: $($spatialSelectedNeighborRows.Count)"
Write-Output "Final rows: $($validFinal.Count)"
Write-Output "Final raw-reflected shader-schema rows: $($finalRawShaderSchemaRows.Count)"
Write-Output "Formula-checked final rows: $($validFinal.Count)"
Write-Output "TargetPdf-checked final branches: $targetPdfCheckedRows"
Write-Output "Shader-raw targetPdf branches: $targetPdfRawShaderCheckedRows"
Write-Output "Weighted-reflected formula branches: $weightedReflectedCheckedRows"
Write-Output ("Max finalContributionLum: frame={0} probe={1} lum={2} finalTargetLum={3} finalWeightSum={4} finalSelectedWeight={5}" -f `
    $maxFinal.frameIndex, $maxFinal.selectedProbeId, $maxFinal.finalContributionLum, $maxFinal.finalTargetLum, $maxFinal.finalWeightSum, $maxFinal.finalSelectedWeight)
Write-Output "Unique DI light samples: $($uniqueDiSamples.Count)"
Write-Output ("Most reused DI sample: {0} ({1} frames)" -f $mostReusedDiSample.Name, $mostReusedDiSample.Count)
