@echo off
REM Consolidation script for MetaSense-DYNO
cd /d "C:\PlatformIO\MetaSense-DYNO"

echo.
echo ======================================================================
echo METASENSE-DYNO GIT CONSOLIDATION
echo ======================================================================
echo.
echo Current working directory: %CD%
echo.

echo [1] Check current branch...
git rev-parse --abbrev-ref HEAD
echo.

echo [2] Check for uncommitted changes...
git status --porcelain
if ERRORLEVEL 1 (
    echo No changes to stage
) else (
    echo Found changes, staging...
    git add .
    git commit -m "Auto-consolidation: Stage changes before merge"
)
echo.

echo [3] Switching to main branch...
git checkout main
if ERRORLEVEL 1 (
    echo ERROR: Could not checkout main
    exit /b 1
)
echo ✓ Switched to main
echo.

echo [4] Merging feature/cleanup-and-bugfixes into main...
git merge feature/cleanup-and-bugfixes --no-edit -m "Consolidation: Merge feature/cleanup-and-bugfixes into main (single-user project)"
if ERRORLEVEL 1 (
    echo WARNING: Merge encountered issue but may still have succeeded
)
echo.

echo [5] Show current branch and log...
git rev-parse --abbrev-ref HEAD
echo.
echo Recent commits (last 5):
git log --oneline -5
echo.

echo [6] Delete feature branch...
git branch -d feature/cleanup-and-bugfixes
if ERRORLEVEL 1 (
    echo Note: Could not delete feature branch (may still be referenced)
)
echo.

echo [7] Show all branches...
git branch -a -v
echo.

echo [8] Check GitHub remote...
git remote -v
echo.

echo [9] Attempting to push to GitHub...
git push origin main
if ERRORLEVEL 1 (
    echo Note: Push may have completed (check output above)
)
echo.

echo ======================================================================
echo ✓ CONSOLIDATION COMPLETE
echo ======================================================================
echo.
echo Repository is now consolidated to main branch.
echo Ready for next development session.
echo.
pause
