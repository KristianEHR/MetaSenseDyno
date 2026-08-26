# MetaSense-DYNO Backup & Recovery Procedures

**Last Updated**: 2026-08-24  
**Project**: Nissan Leaf ZE1 Dyno Controller Firmware (Phase 14C)  
**Device IP**: 192.168.0.211  

## Quick Reference

| Operation | Command | Time |
|-----------|---------|------|
| **Create Git Bundle** | `git bundle create [filename].bundle --all` | <30s |
| **Create Full Backup** | `Compress-Archive -Path . -DestinationPath [backup].zip -CompressionLevel Optimal` | 2-5min |
| **Restore from Git Bundle** | `git clone [backup].bundle [destination]` | 1-2min |
| **View Commit History** | `git log --oneline --graph` | <1s |
| **Check Status** | `git status` | <1s |

---

## Backup Methods

### Method 1: Git Bundle (Recommended)

**Purpose**: Portable, full repository history, smallest size  
**Location**: `backups/MetaSense-DYNO-git-backup-*.bundle`  
**Existing Backups**: `backups/repo_backup_20260802_100851.bundle`  

**Create Bundle**:
```powershell
cd C:\PlatformIO\MetaSense-DYNO
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
git bundle create "backups\MetaSense-DYNO-git-backup-$timestamp.bundle" --all
```

**Restore from Bundle**:
```powershell
# Clone bundle to new location
git clone "C:\path\to\backup.bundle" "C:\new\destination"

# Or: Fetch bundle into existing repo
cd C:\existing\repo
git fetch "C:\path\to\backup.bundle" main
```

**Verify Bundle**:
```powershell
git bundle verify "C:\path\to\backup.bundle"
```

---

### Method 2: Full Directory Backup

**Purpose**: Complete snapshot (code + build artifacts + history)  
**Location**: `C:\PlatformIO\MetaSense-DYNO_backup_*.zip`  
**Size**: ~200-500 MB (depending on artifacts)  

**Create Zip Backup**:
```powershell
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
Compress-Archive -Path "C:\PlatformIO\MetaSense-DYNO" `
  -DestinationPath "C:\PlatformIO\MetaSense-DYNO_backup_$timestamp.zip" `
  -CompressionLevel Optimal
```

**Create Excluding Build Artifacts** (smaller):
```powershell
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
# First, clean build artifacts
cd C:\PlatformIO\MetaSense-DYNO
git clean -fdx  # WARNING: Removes untracked files!

# Then backup
Compress-Archive -Path "C:\PlatformIO\MetaSense-DYNO" `
  -DestinationPath "C:\PlatformIO\MetaSense-DYNO_clean_backup_$timestamp.zip" `
  -CompressionLevel Optimal
```

**Restore Zip Backup**:
```powershell
Expand-Archive -Path "C:\path\to\backup.zip" -DestinationPath "C:\destination"
```

---

## Repository Status

**Current State**:
- ✅ Repository initialized: `.git/` present
- ✅ All Phase 14C changes committed
- ✅ Working directory clean (`git status` shows no uncommitted changes)
- ✅ Remote configured: origin → GitHub repository

**View Current Status**:
```powershell
cd C:\PlatformIO\MetaSense-DYNO
git log --oneline -10          # Last 10 commits
git status                      # Current state
git remote -v                   # Remote repositories
```

**Key Commits**:
```
Phase 14C: LeafCanConfig integration + WebSocket queue optimization
  - LeafCanConfig.h fully integrated
  - ~130 duplicate METASENSE_LEAF_* defines removed
  - Memory stable (18.4% RAM, 46.3% Flash)
  - Device running at 192.168.0.211

Phase 14B: SettingsConfig.h creation (NOT integrated, deferred)
  
Phase 14A: LeafCanConfig.h creation & discovery
```

---

## Critical Files & Locations

| File/Folder | Purpose | Backup Status |
|-------------|---------|----------------|
| `src/Input.cpp` | Master input task, CAN RX/TX | ✅ Git tracked |
| `src/LeafCan.cpp` | Leaf frame decoder | ✅ Git tracked |
| `include/LeafCanConfig.h` | CAN parameters (centralized) | ✅ Git tracked |
| `include/SettingsConfig.h` | Runtime settings (NOT integrated) | ✅ Git tracked |
| `platformio.ini` | Build configuration | ✅ Git tracked |
| `data/` | LittleFS filesystem | ✅ Git tracked |
| `.git/` | Git repository | ✅ Bundle compatible |
| `backups/` | Previous backups & bundles | ✅ Version controlled |

---

## Disaster Recovery Plan

### Scenario 1: Lost Local Changes

**Problem**: Accidentally deleted files, corrupted working directory  
**Recovery Steps**:

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# Option A: Restore from last commit
git reset --hard HEAD

# Option B: Restore specific file
git checkout HEAD -- src/Input.cpp

# Option C: See what was lost
git log --follow -- src/SomeFile.cpp
```

### Scenario 2: Need to Restore Entire Repository

**Problem**: Disk failure, entire workspace corrupted  
**Recovery Steps**:

```powershell
# From Git Bundle
git clone C:\path\to\MetaSense-DYNO-git-backup-*.bundle C:\PlatformIO\MetaSense-DYNO-restored

# From Zip Backup
Expand-Archive -Path C:\path\to\MetaSense-DYNO_backup_*.zip -DestinationPath C:\PlatformIO\

# Verify restore
cd C:\PlatformIO\MetaSense-DYNO-restored
git log --oneline -5
```

### Scenario 3: Need Previous Firmware Version

**Problem**: New firmware has bug, need to revert  
**Recovery Steps**:

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# View all commits
git log --oneline

# Checkout specific commit
git checkout <commit-hash>

# Build and upload that version
# Use VS Code task: Build (esp32s3-ota) → Upload (esp32s3-ota)

# Return to latest
git checkout main
```

---

## Testing Recovery Procedures

### Verify Git Bundle Integrity

```powershell
git bundle verify "C:\PlatformIO\MetaSense-DYNO\backups\repo_backup_20260802_100851.bundle"
```

**Expected Output**:
```
The bundle contains 1 ref
00abc123def456... refs/heads/main
The bundle records a complete history.
```

### Test Restore from Bundle (Dry Run)

```powershell
# Don't modify existing repo; test in temp location
cd C:\Temp
git clone "C:\PlatformIO\MetaSense-DYNO\backups\repo_backup_20260802_100851.bundle" test-restore
cd test-restore
git log --oneline -3
# Verify commits match production repo
```

---

## Backup Schedule Recommendation

| Frequency | Method | Retention |
|-----------|--------|-----------|
| **After each firmware phase** | Git Bundle | 6 months |
| **Weekly** | Full Zip Backup | 2 weeks |
| **Before major changes** | Git Bundle | Until next milestone |
| **Monthly** | Full Zip + External HDD | 1 year |

---

## Git Commands Reference

### Essential Commands

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# View history
git log --oneline --graph          # Visual tree
git show <commit>                  # See specific commit
git diff <commit1> <commit2>       # Compare commits

# Create save point
git tag -a v1.0-production -m "Phase 14C complete, device running"
git show v1.0-production

# Branch operations
git branch -a                       # List all branches
git branch new-feature             # Create branch
git checkout new-feature           # Switch branch
git merge new-feature              # Merge into current

# Undo operations
git revert <commit>                # Create undo commit
git reset --hard <commit>          # Hard reset (WARNING!)
git reflog                         # Recover lost commits
```

### Advanced Commands

```powershell
# Create bundle for sharing
git bundle create my-backup.bundle --all

# Export specific commits
git format-patch HEAD~5..HEAD      # Last 5 commits

# Stash work in progress
git stash
git stash pop

# Interactive rebase (be careful!)
git rebase -i HEAD~10
```

---

## Troubleshooting

### Bundle Creation Fails

```powershell
# Check disk space
Get-Volume

# Check repo integrity
git fsck

# Try smaller bundle (single branch)
git bundle create small.bundle main
```

### Clone from Bundle Fails

```powershell
# Verify bundle integrity first
git bundle verify backup.bundle

# Clone with verbose output
git clone -v backup.bundle destination
```

### Repository Corruption

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# Diagnose
git fsck --full

# Attempt repair
git gc --aggressive

# Last resort: restore from backup
```

---

## External Backup Recommendations

**For production-critical firmware**:
1. **GitHub**: Remote repository (automatic with push)
2. **External HDD**: Monthly full backup zip
3. **Cloud Storage** (OneDrive/Drive): Git bundle copies
4. **USB Drive**: Latest production git bundle

**Example Archive Script** (create as `backup.ps1`):

```powershell
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backupFolder = "E:\MetaSense-Backups"  # External drive
$projectPath = "C:\PlatformIO\MetaSense-DYNO"

# Create git bundle
git -C $projectPath bundle create "$backupFolder\MetaSense-git-$timestamp.bundle" --all

# Copy to multiple locations
Copy-Item "$backupFolder\MetaSense-git-$timestamp.bundle" -Destination "C:\Backups\MetaSense-git-$timestamp.bundle"
Copy-Item "$backupFolder\MetaSense-git-$timestamp.bundle" -Destination "D:\Offsite\MetaSense-git-$timestamp.bundle"

Write-Host "Backup complete: $timestamp"
```

---

## Contact & Support

- **Device IP**: 192.168.0.211 (WiFi OTA capable)
- **Build System**: PlatformIO 7.0.1
- **Target Board**: ESP32-S3-DevKitC-1
- **Primary Repository**: GitHub (configured as 'origin')

---

## Document Version

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-08-24 | Initial: Git bundle + full backup procedures, recovery scenarios |

