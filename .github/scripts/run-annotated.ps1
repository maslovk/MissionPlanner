# Runs a command. If it fails, the end of its output is repeated as a workflow error annotation,
# which (unlike the job log) can be read without signing in to GitHub, e.g. through
# GET /repos/{owner}/{repo}/check-runs/{job_id}/annotations.
#   pwsh .github/scripts/run-annotated.ps1 cmake --preset windows-x64 -S ExtLibs/mpcore
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Command)

$exe = $Command[0]
$arguments = @($Command | Select-Object -Skip 1)
$output = & $exe @arguments 2>&1 | ForEach-Object { "$_" }
$code = $LASTEXITCODE
$output | ForEach-Object { Write-Host $_ }
if ($code -ne 0) {
    $tail = ($output | Select-Object -Last 40) -join "`n"
    $tail = $tail.Replace('%', '%25').Replace("`r", '%0D').Replace("`n", '%0A')
    Write-Host "::error title=$exe exited with $code::$tail"
    exit $code
}
