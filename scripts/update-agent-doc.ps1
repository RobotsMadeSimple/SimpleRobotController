<#
.SYNOPSIS
  Regenerate the catalog-generated parts of docs/AGENT-PROGRAMMING.md (step index, per-step field
  reference, error codes) and docs/program-text.md (argument reference) from the step catalog.

.DESCRIPTION
  The generator lives in the tests (AgentDocTests / DocTableTests), so the docs and the drift
  checks share one implementation. With SRS_UPDATE_DOCS=1 those tests rewrite the generated blocks
  instead of failing; run this after changing RobotControl/Programs/StepCatalog.cs or ErrorCodes.cs,
  then review the diff. Without -Update it only checks (fails when a doc has drifted).

.EXAMPLE
  pwsh scripts/update-agent-doc.ps1
  pwsh scripts/update-agent-doc.ps1 -Check -ArtifactsPath $env:TEMP\art
#>
param(
    [switch]$Check,
    [string]$ArtifactsPath = ""
)

$repo = Split-Path -Parent $PSScriptRoot
$filter = "FullyQualifiedName~AgentDocTests|FullyQualifiedName~DocTableTests"
$testArgs = @("test", (Join-Path $repo "RobotControl.Tests"), "--filter", $filter)
if ($ArtifactsPath) { $testArgs += @("--artifacts-path", $ArtifactsPath) }

if ($Check) { Remove-Item Env:SRS_UPDATE_DOCS -ErrorAction SilentlyContinue }
else { $env:SRS_UPDATE_DOCS = "1" }
try {
    & dotnet @testArgs
    $code = $LASTEXITCODE
} finally {
    Remove-Item Env:SRS_UPDATE_DOCS -ErrorAction SilentlyContinue
}
if (-not $Check -and $code -eq 0) {
    # Second pass without the update flag: the rewritten docs must now pass as-is.
    & dotnet @testArgs
    $code = $LASTEXITCODE
}
exit $code
