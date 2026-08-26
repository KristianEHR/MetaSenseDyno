# Direct PowerShell Git Consolidation
$ErrorActionPreference = "Stop"

cd "C:\PlatformIO\MetaSense-DYNO"

Write-Host "=" * 70
Write-Host "GIT CONSOLIDATION - MetaSense-DYNO (PowerShell)" 
Write-Host "=" * 70

# Step 1: Current branch
Write-Host "`n[1] Current branch:"
$branch = & git rev-parse --abbrev-ref HEAD 2>&1
Write-Host "  $branch"

# Step 2: Checkout main
Write-Host "`n[2] Checkout main:"
try {
    & git checkout main 2>&1 | ForEach-Object { Write-Host "  $_" }
    Write-Host "  ✓ Success"
} catch {
    Write-Host "  Error: $_"
}

# Step 3: Verify
Write-Host "`n[3] Verify branch:"
$branch = & git rev-parse --abbrev-ref HEAD 2>&1
Write-Host "  Current: $branch"

# Step 4: Merge
Write-Host "`n[4] Merge feature branch:"
try {
    $output = & git merge feature/cleanup-and-bugfixes --no-edit -m "Consolidation: Merge feature to main" 2>&1
    Write-Host "  $output"
} catch {
    Write-Host "  Note: $_"
}

# Step 5: Show log
Write-Host "`n[5] Recent commits:"
& git log --oneline -3 2>&1 | ForEach-Object { Write-Host "  $_" }

# Step 6: Delete feature branch
Write-Host "`n[6] Delete feature branch:"
try {
    & git branch -d feature/cleanup-and-bugfixes 2>&1 | ForEach-Object { Write-Host "  $_" }
} catch {
    Write-Host "  Note: $_"
}

# Step 7: List branches
Write-Host "`n[7] Branches:"
& git branch -a 2>&1 | ForEach-Object { Write-Host "  $_" }

# Step 8: Status
Write-Host "`n[8] Status:"
& git status 2>&1 | Select-Object -First 3 | ForEach-Object { Write-Host "  $_" }

Write-Host "`n" + ("=" * 70)
Write-Host "✓ Consolidation script complete"
Write-Host "=" * 70 + "`n"

# Write result file
@"
CONSOLIDATION COMPLETED
Time: $(Get-Date)
Script: PowerShell consolidation
Branch: main
"@ | Out-File -FilePath "consolidation_ps_result.txt"

Write-Host "Result saved to consolidation_ps_result.txt"
