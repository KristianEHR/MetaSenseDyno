# Git Repository Backup Status & Strategy

**Created**: 2026-08-24  
**Project**: MetaSense-DYNO (Nissan Leaf ZE1 Dyno Controller)  
**Phase**: 14C Complete  

---

## ✅ Existing Git Bundles (Backup Verification)

Your project already has comprehensive git bundles for disaster recovery:

### Available Backups

| Location | Date | Type | Status |
|----------|------|------|--------|
| `backups/repo_backup_20260802_100851.bundle` | Aug 2, 2026 10:08 | Git Bundle | ✅ Available |
| `backups/restore_20260810_173908/repo_backup_20260810_173908.bundle` | Aug 10, 2026 17:39 | Git Bundle | ✅ Available |
| `backups/restore_20260815_134058/repo_backup_20260815_134058.bundle` | Aug 15, 2026 13:40 | Git Bundle | ✅ Available |
| `backups/restore_point_20260824_153000/` | Aug 24, 2026 15:30 | Full Directory | ✅ Available |

### Oldest Backup Age

**Latest bundle**: 22 days old (from Aug 2nd)  
**Recommendation**: Create new bundle now to capture Phase 14C work

---

## Create New Backup (Phase 14C State)

The Phase 14C consolidation work completed successfully. Create a fresh backup to capture this state:

### Quick Create Commands

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# Option 1: Simple git bundle (fastest, smallest)
git bundle create backups\MetaSense-DYNO-Phase-14C.bundle --all

# Option 2: Timestamped bundle
git bundle create "backups\MetaSense-DYNO-Phase-14C-$(Get-Date -Format yyyyMMdd_HHmmss).bundle" --all

# Verify bundle
git bundle verify backups\MetaSense-DYNO-Phase-14C.bundle
```

### Full Directory Backup (Optional)

```powershell
# Clean build artifacts first (optional)
cd C:\PlatformIO\MetaSense-DYNO
git clean -fdn  # Dry run - see what would be removed
# git clean -fdx  # Remove untracked files (CAREFUL!)

# Create backup
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
Compress-Archive -Path C:\PlatformIO\MetaSense-DYNO `
  -DestinationPath "C:\MetaSense-DYNO-Phase-14C-$timestamp.zip" `
  -CompressionLevel Optimal

# Or use Python tool
cd C:\PlatformIO\MetaSense-DYNO
python backup_tool.py --full
```

---

## Phase 14C Backup Checklist

### What's Being Backed Up

✅ **Source Code**:
- `src/Input.cpp` - LeafCanConfig.h integrated
- `src/LeafCan.cpp` - CAN frame decoder with config
- `include/LeafCanConfig.h` - Centralized CAN parameters (NEW)
- All other source files and headers

✅ **Configuration**:
- `platformio.ini` - Build configuration
- `include/SettingsConfig.h` - Runtime settings (created, not integrated)
- Device customizations

✅ **Data**:
- `data/` - LittleFS filesystem files
- All user configurations and calibration data

✅ **History**:
- Complete git commit history (14A, 14B, 14C phases)
- All branches and tags
- Author information and timestamps

✅ **Build Artifacts** (in full backup only):
- `.pio/` - PlatformIO build cache
- `.vscode/` - VS Code settings
- Recent build logs

### What's NOT in Git Bundle (Only in Full Zip)

⚠️ **Large/Temporary Items**:
- PlatformIO cache (`.pio/build/`) - rebuilt on next build
- Dependency downloads - redownloaded on build
- Temporary log files
- IDE state files

✓ **These items are small and don't affect bundle size**

---

## Restore Procedures

### Restore from Latest Bundle (If Disaster Occurs)

```powershell
# Clone from bundle to new location
git clone "C:\PlatformIO\MetaSense-DYNO\backups\repo_backup_20260802_100851.bundle" `
  "C:\PlatformIO\MetaSense-DYNO-RESTORED"

# Verify history
cd C:\PlatformIO\MetaSense-DYNO-RESTORED
git log --oneline -5
```

### Restore to Same Location (Advanced)

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# Fetch from bundle into current repo
git fetch "backups\repo_backup_20260802_100851.bundle" main:restored-main

# Switch to restored branch
git checkout restored-main

# Or merge back to main
git merge restored-main
```

### Restore from Full Zip Backup

```powershell
# Extract anywhere
Expand-Archive -Path "C:\MetaSense-DYNO-Phase-14C-*.zip" `
  -DestinationPath "C:\MetaSense-DYNO-FROM-ZIP"

cd C:\MetaSense-DYNO-FROM-ZIP
git log --oneline -3  # Verify git history intact
```

---

## Backup Strategy Recommendation

### Weekly Automation Script

Create `backup_weekly.ps1`:

```powershell
# Run via Task Scheduler weekly
$projectPath = "C:\PlatformIO\MetaSense-DYNO"
$backupPath = "$projectPath\backups"
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"

cd $projectPath

# Create timestamped bundle
git bundle create "$backupPath\MetaSense-Phase-14C-$timestamp.bundle" --all

# Clean old backups (keep last 5)
Get-ChildItem "$backupPath\MetaSense-Phase-14C-*.bundle" | `
  Sort-Object LastWriteTime -Descending | `
  Select-Object -Skip 5 | `
  Remove-Item

Write-Host "Backup created: $timestamp"
```

### Monthly Full Backup

```powershell
# Keep full backups longer (monthly)
$timestamp = Get-Date -Format "yyyyMMdd"
Compress-Archive -Path "C:\PlatformIO\MetaSense-DYNO" `
  -DestinationPath "E:\Backups\MetaSense-DYNO-$timestamp.zip"  # External drive
```

---

## Testing Your Backups

### Verify Bundle Integrity

```powershell
git bundle verify "C:\PlatformIO\MetaSense-DYNO\backups\repo_backup_20260802_100851.bundle"
```

**Expected output**:
```
The bundle contains these refs:
<commit-hash> refs/heads/main
The bundle records a complete history.
```

### Test Restore (Dry Run)

```powershell
cd C:\Temp

# Clone without modifying production repo
git clone "C:\PlatformIO\MetaSense-DYNO\backups\repo_backup_20260802_100851.bundle" test-restore

# Verify it works
cd test-restore
git log --oneline -10
```

---

## Repository Current State

### Git Status

```
Location: C:\PlatformIO\MetaSense-DYNO
Remote: GitHub (configured as 'origin')
Status: Clean (no uncommitted changes)
Latest: Phase 14C consolidation + WebSocket optimization
```

### View Current History

```powershell
cd C:\PlatformIO\MetaSense-DYNO
git log --oneline --graph -10
```

**Should show**:
- Phase 14C commit with integration work
- Phase 14B with SettingsConfig.h
- Phase 14A with LeafCanConfig discovery
- Previous commits for earlier phases

### Branches

```powershell
git branch -a          # List all branches
git tag -l             # List all tags
```

---

## Critical Data Locations

**If you ONLY have time for ONE backup**, make sure it includes:

```
C:\PlatformIO\MetaSense-DYNO\
├── src/Input.cpp              ← Master input task (critical)
├── src/LeafCan.cpp            ← CAN frame decoder (critical)
├── include/LeafCanConfig.h    ← CAN parameters (critical)
├── include/SettingsConfig.h   ← Settings template (important)
├── platformio.ini             ← Build configuration (critical)
├── data/                       ← LittleFS files (important)
├── .git/                       ← Entire git repository (includes all history)
└── backups/                    ← Previous backups for reference
```

**Size**: Full backup ~300-500 MB (with all artifacts)  
**Git bundle only**: ~30-50 MB (source + history)

---

## External Backup Locations

### Recommended Multi-Location Strategy

1. **Local Git Bundle**: `C:\PlatformIO\MetaSense-DYNO\backups\` (daily/weekly)
2. **External USB Drive**: Copy monthly bundle to external drive
3. **GitHub**: Push to remote (automatic sync with git push)
4. **Cloud Storage**: OneDrive/Google Drive with zip backup
5. **NAS/Network Drive**: If available, weekly backup

### Quick Export to USB

```powershell
# Copy latest bundle to USB drive (E:)
Copy-Item "C:\PlatformIO\MetaSense-DYNO\backups\*.bundle" -Destination "E:\MetaSense-Backup\"

# Or compress for cloud
Compress-Archive -Path "C:\PlatformIO\MetaSense-DYNO" `
  -DestinationPath "C:\MetaSense-Phase-14C-$(Get-Date -Format yyyyMMdd).zip"
# Then upload to OneDrive/Google Drive
```

---

## When to Backup

| Event | Action |
|-------|--------|
| **After major phase** | Create timestamped bundle |
| **Before risky changes** | Create "pre-change" backup |
| **Weekly routine** | Automated via Task Scheduler |
| **Before firmware upload to device** | Quick bundle create |
| **After successful device testing** | Tag release + backup |
| **Before major refactor** | Full zip + bundle |

---

## Troubleshooting

### Bundle Creation Seems Stuck

```powershell
# Check disk space
Get-Volume

# Verify repo
git fsck

# Try creating in temp location
git bundle create C:\Temp\test.bundle --all
```

### Can't Restore from Bundle

```powershell
# Verify bundle first
git bundle verify "path\to\bundle.bundle"

# Try with verbose
git clone -v "path\to\bundle.bundle" destination 2>&1 | head -20

# Check permissions
Get-Acl "path\to\bundle.bundle"
```

### Repository Corruption

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# Diagnose
git fsck --full

# Repair attempt
git gc --aggressive

# If damaged: restore from bundle
cd C:\
git clone "C:\PlatformIO\MetaSense-DYNO\backups\repo_backup_20260802_100851.bundle" MetaSense-DYNO-FIXED
```

---

## Summary

✅ **Current Status**: Phase 14C work completed and ready to backup  
✅ **Previous Backups**: 3 git bundles available for reference  
⚠️ **Recommendation**: Create new Phase 14C bundle to capture latest state  
✅ **Recovery Ready**: Multiple backup options available (git bundle, full zip)  
✅ **Device**: Running latest firmware at 192.168.0.211  

**Next Step**: Run one of the backup commands above to secure Phase 14C work!

