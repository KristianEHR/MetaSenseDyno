$ErrorActionPreference = 'Stop'
$ts = Get-Date -Format 'yyyyMMdd_HHmmss'
$dir = "backups/restore_$ts"

New-Item -ItemType Directory -Path $dir -Force | Out-Null

git rev-parse HEAD | Out-File -FilePath "$dir/rollback_base.txt" -Encoding ascii
git status --porcelain=v1 | Out-File -FilePath "$dir/rollback_status.txt" -Encoding ascii
git ls-files --others --exclude-standard | Out-File -FilePath "$dir/rollback_untracked.txt" -Encoding ascii
git diff | Out-File -FilePath "$dir/rollback_before.patch" -Encoding ascii
git diff --staged | Out-File -FilePath "$dir/rollback_staged.patch" -Encoding ascii
git bundle create "$dir/repo_backup_$ts.bundle" --all

Write-Output "RESTORE_POINT=$dir"
