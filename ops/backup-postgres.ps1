param(
    [Parameter(Mandatory = $true)]
    [string]$DatabaseUrl,

    [string]$OutputDirectory = ".\backups",

    [int]$RetentionDays = 30
)

$ErrorActionPreference = "Stop"
if ($RetentionDays -lt 1) {
    throw "RetentionDays must be at least 1 day."
}
if (-not (Get-Command pg_dump -ErrorAction SilentlyContinue)) {
    throw "pg_dump is required on PATH."
}
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Force -Path $resolvedOutput | Out-Null
$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$backupPath = Join-Path $resolvedOutput "easytowing-$stamp.dump"

& pg_dump --dbname=$DatabaseUrl --format=custom --file=$backupPath --no-owner --no-privileges
if ($LASTEXITCODE -ne 0) {
    throw "pg_dump failed with exit code $LASTEXITCODE"
}
if (-not (Test-Path -LiteralPath $backupPath -PathType Leaf)) {
    throw "pg_dump did not create the expected backup: $backupPath"
}
$backupSize = (Get-Item -LiteralPath $backupPath).Length
if ($backupSize -le 0) {
    throw "pg_dump created an empty backup: $backupPath"
}

Get-ChildItem -LiteralPath $resolvedOutput -Filter "easytowing-*.dump" -File |
    Where-Object { $_.LastWriteTime -lt (Get-Date).AddDays(-$RetentionDays) } |
    Remove-Item -Force

Write-Output "Created PostgreSQL backup: $backupPath ($backupSize bytes)"
