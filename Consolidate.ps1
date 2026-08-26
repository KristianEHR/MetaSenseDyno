#!/usr/bin/env pwsh
<#
.SYNOPSIS
Git Consolidation Script - Merge feature branch to main
#>

$ErrorActionPreference = "Continue"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "GIT CONSOLIDATION - MetaSense-DYNO" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

Push-Location "C:\PlatformIO\MetaSense-DYNO"

try {
    Write-Host "`n[STEP 1] Current state..." -ForegroundColor Yellow
    & git rev-parse --abbrev-ref HEAD
    
    Write-Host "`n[STEP 2] Checkout main..." -ForegroundColor Yellow
    & git checkout main 2>&1
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  ERROR: Could not checkout main" -ForegroundColor Red
        exit 1
    }
    Write-Host "  ✓ Switched to main" -ForegroundColor Green
    
    Write-Host "`n[STEP 3] Merge feature branch..." -ForegroundColor Yellow
    & git merge feature/cleanup-and-bugfixes --no-edit -m "Consolidation: Merge feature branch to main" 2>&1
    Write-Host "  ✓ Merge complete" -ForegroundColor Green
    
    Write-Host "`n[STEP 4] Delete feature branch..." -ForegroundColor Yellow
    & git branch -d feature/cleanup-and-bugfixes 2>&1
    Write-Host "  ✓ Feature branch deleted" -ForegroundColor Green
    
    Write-Host "`n[STEP 5] Show log..." -ForegroundColor Yellow
    & git log --oneline -5 2>&1
    
    Write-Host "`n[STEP 6] Final status..." -ForegroundColor Yellow
    & git status 2>&1
    
    Write-Host "`n========================================" -ForegroundColor Cyan
    Write-Host "✓ CONSOLIDATION COMPLETE" -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    
}
catch {
    Write-Host "ERROR: $_" -ForegroundColor Red
    exit 1
}
finally {
    Pop-Location
}
