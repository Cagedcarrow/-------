param(
    [Parameter(Mandatory=$true)]
    [string]$Command
)

$originalPath = $env:PATH
try {
    $env:PATH = ($env:PATH -split ';' | Where-Object {$_ -notlike "*matlab2026*"}) -join ';'
    & "E:\matlab2025b\bin\matlab.exe" -batch $Command
} finally {
    $env:PATH = $originalPath
}
