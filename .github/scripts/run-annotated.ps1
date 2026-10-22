# Defines Invoke-Annotated, which runs a command block and, if the command fails, repeats the end
# of its output as a workflow error annotation. Unlike the job log, annotations can be read without
# signing in to GitHub (GET /repos/{owner}/{repo}/check-runs/{job_id}/annotations).
#   . .github/scripts/run-annotated.ps1
#   Invoke-Annotated { cmake --preset windows-x64 -S ExtLibs/mpcore "-DSOME_PATH=$path" }
# It returns the command's output lines (they are also printed).
# The command is a script block, not a list of arguments, so PowerShell passes its arguments to
# the program as written (as a parameter list it would split "-DX=C:\..." at the colon).

function Invoke-Annotated([scriptblock]$Command) {
    # a native command writing to stderr must not stop the step before its output is reported
    $ErrorActionPreference = 'Continue'
    $output = & $Command 2>&1 | ForEach-Object { "$_" }
    $code = $LASTEXITCODE
    $output | ForEach-Object { Write-Host $_ }
    if ($code -ne 0) {
        $tail = ($output | Select-Object -Last 40) -join "`n"
        Write-Host "::error title=Command exited with $code::$(ConvertTo-AnnotationText $tail)"
        exit $code
    }
    return $output
}

function ConvertTo-AnnotationText([string]$Text) {
    return $Text.Replace('%', '%25').Replace("`r", '%0D').Replace("`n", '%0A')
}
