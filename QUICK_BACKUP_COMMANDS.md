# QUICK BACKUP COMMANDS

```powershell
# Git Bundle (recommended - smallest, full history)
cd C:\PlatformIO\MetaSense-DYNO
git bundle create backups\MetaSense-backup-$(Get-Date -Format yyyyMMdd_HHmmss).bundle --all

# Verify bundle
git bundle verify backups\MetaSense-backup-*.bundle | head -5

# Restore from bundle
git clone backups\MetaSense-backup-*.bundle C:\MetaSense-RESTORE

# Full directory zip
Compress-Archive -Path C:\PlatformIO\MetaSense-DYNO -DestinationPath C:\MetaSense-$(Get-Date -Format yyyyMMdd).zip -CompressionLevel Optimal
```

## Status Check

```powershell
cd C:\PlatformIO\MetaSense-DYNO
git log --oneline -3          # Last 3 commits
git status                    # Current state
git remote -v                 # Remote repos
ls backups\*.bundle           # List backups
```

## Emergency Restore

```powershell
# If everything is lost, restore from backup
git clone C:\path\to\MetaSense-backup-*.bundle C:\PlatformIO\MetaSense-DYNO-NEW
```
