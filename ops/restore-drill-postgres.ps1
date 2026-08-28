param(
    [Parameter(Mandatory = $true)]
    [string]$BackupPath,

    [Parameter(Mandatory = $true)]
    [string]$RestoreDatabaseUrl
)

$ErrorActionPreference = "Stop"
$resolvedBackup = [System.IO.Path]::GetFullPath($BackupPath)
if (-not (Test-Path -LiteralPath $resolvedBackup -PathType Leaf)) {
    throw "Backup file does not exist: $resolvedBackup"
}
if ([string]::IsNullOrWhiteSpace($RestoreDatabaseUrl)) {
    throw "RestoreDatabaseUrl is required and must point to an isolated database."
}

# Do not use --clean: a restore drill must never delete objects in the target.
& pg_restore --dbname=$RestoreDatabaseUrl --no-owner --no-privileges --exit-on-error --single-transaction $resolvedBackup
if ($LASTEXITCODE -ne 0) {
    throw "pg_restore failed with exit code $LASTEXITCODE"
}

$check = @'
SELECT 'organizations' AS table_name, count(*) AS row_count FROM organizations
UNION ALL SELECT 'users', count(*) FROM users
UNION ALL SELECT 'projects', count(*) FROM projects
UNION ALL SELECT 'project_revisions', count(*) FROM project_revisions
UNION ALL SELECT 'engineering_jobs', count(*) FROM engineering_jobs
UNION ALL SELECT 'audit_events', count(*) FROM audit_events
ORDER BY table_name;
'@
$counts = & psql --dbname=$RestoreDatabaseUrl --no-psqlrc --tuples-only --command=$check
if ($LASTEXITCODE -ne 0) {
    throw "Post-restore verification failed with exit code $LASTEXITCODE"
}

Write-Output "Restore drill completed against the isolated target."
Write-Output ($counts -join [Environment]::NewLine)
