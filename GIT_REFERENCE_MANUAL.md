# GIT REFERENCE MANUAL - MetaSense-DYNO

**Quick Access for Common Tasks**

---

## BACKUP (PRIMARY FOCUS)

### Create Backup

```powershell
# Git Bundle (recommended - smallest, portable, full history)
cd C:\PlatformIO\MetaSense-DYNO
git bundle create backups\MetaSense-DYNO-Phase-14C-FINAL.bundle --all

# Timestamped Bundle
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
git bundle create "backups\MetaSense-Phase-14C-$ts.bundle" --all

# Full Directory Backup
$ts = Get-Date -Format "yyyyMMdd"
Compress-Archive -Path C:\PlatformIO\MetaSense-DYNO `
  -DestinationPath "C:\MetaSense-DYNO-$ts.zip" `
  -CompressionLevel Optimal
```

### Verify Backup

```powershell
# Check bundle integrity
git bundle verify backups\MetaSense-DYNO-Phase-14C-FINAL.bundle

# List bundle contents
git bundle list-heads backups\MetaSense-DYNO-Phase-14C-FINAL.bundle
```

### Restore from Backup

```powershell
# Clone from bundle (creates new copy)
git clone backups\MetaSense-DYNO-Phase-14C-FINAL.bundle C:\MetaSense-RESTORED

# Fetch from bundle (updates existing repo)
cd C:\PlatformIO\MetaSense-DYNO
git fetch backups\MetaSense-DYNO-Phase-14C-FINAL.bundle main:restored-main

# Restore zip backup
Expand-Archive -Path C:\MetaSense-DYNO-20260824.zip `
  -DestinationPath C:\restored-from-zip
```

---

## HISTORY & LOG

### View Commit History

```powershell
# Last N commits
git log --oneline -10              # Last 10 as single lines
git log -5                         # Last 5 with full details
git log --graph --oneline --all    # Visual tree of all branches
git log --pretty=format:"%h - %s" -10  # Custom format

# By author
git log --author="Name"            # Commits by author
git log --oneline | grep "keyword" # Search by message

# By date
git log --since="2 weeks ago"      # Since date
git log --until="1 week ago"       # Until date
```

### View Specific Commits

```powershell
# Show commit details
git show HEAD                      # Latest commit
git show <commit-hash>             # Specific commit
git show HEAD~3                    # 3 commits ago
git show <commit>:file.txt         # File at specific commit

# Compare commits
git diff <commit1> <commit2>       # Between two commits
git diff HEAD~5..HEAD              # Last 5 commits
```

### Search History

```powershell
# Find commits by message
git log --grep="LeafCanConfig"     # By commit message
git log -S"function_name"          # By content change
git log -p -- src/Input.cpp        # Changes to specific file

# Find who changed what
git blame src/Input.cpp            # Line-by-line author
git log -p src/Input.cpp           # Full history of file
```

---

## BRANCHES & TAGS

### Branch Operations

```powershell
# List branches
git branch                         # Local branches only
git branch -a                      # All branches (local + remote)
git branch -v                      # With last commit
git branch --merged                # Merged into current
git branch --no-merged             # Not merged yet

# Create & switch branches
git branch feature-name            # Create branch
git checkout feature-name          # Switch to branch
git checkout -b feature-name       # Create and switch
git switch feature-name            # Modern syntax (Git 2.23+)

# Delete branches
git branch -d feature-name         # Delete (safe)
git branch -D feature-name         # Force delete
git push origin --delete feature-name  # Delete remote
```

### Merge Branches

```powershell
# Merge into current branch
git merge feature-name             # Create merge commit
git merge --no-ff feature-name     # Always create merge commit
git merge --squash feature-name    # Combine commits before merge

# Rebase (linear history)
git rebase main                    # Rebase current onto main
git rebase -i HEAD~5               # Interactive rebase (last 5)
```

### Tags (Milestones)

```powershell
# Create tags
git tag v1.0-Phase-14C-Final       # Lightweight tag
git tag -a v1.0 -m "Release v1.0"  # Annotated tag (recommended)
git tag -l                         # List tags
git tag -l "v1*"                   # Filter tags

# Push tags
git push origin v1.0               # Push specific tag
git push origin --tags             # Push all tags

# Delete tags
git tag -d v1.0                    # Delete local
git push origin --delete v1.0      # Delete remote
```

---

## STATUS & CHANGES

### Check Status

```powershell
# Repository status
git status                         # Full status
git status -s                      # Short status
git status --porcelain             # Very short (for scripts)

# What changed
git diff                           # Unstaged changes
git diff --staged                  # Staged changes
git diff HEAD                      # All changes vs HEAD
git diff <branch1> <branch2>       # Between branches
```

### Stage & Commit

```powershell
# Stage changes
git add src/Input.cpp              # Stage specific file
git add .                          # Stage all changes
git add -p                         # Interactive (hunk by hunk)
git add -u                         # Update tracking

# Commit
git commit -m "Message"            # Simple commit
git commit -m "Title" -m "Body"    # With description
git commit --amend                 # Fix last commit
git commit --amend --no-edit       # Keep message, update files

# Unstage changes
git reset src/Input.cpp            # Unstage file
git reset                          # Unstage all
git reset --hard                   # Discard all changes
```

---

## UNDO & RECOVERY

### Undo Changes

```powershell
# Discard changes
git checkout -- src/Input.cpp      # Discard in working directory
git restore src/Input.cpp          # Modern syntax
git clean -fd                      # Remove untracked files (dry run: -fdn)

# Undo commits
git revert <commit>                # Create undo commit (safe)
git reset --soft HEAD~1            # Undo, keep changes staged
git reset --mixed HEAD~1           # Undo, keep changes unstaged
git reset --hard HEAD~1            # Undo, discard changes (DANGEROUS!)

# Back up before big changes
git stash                          # Stash current work
git stash list                     # List stashed changes
git stash pop                      # Restore most recent stash
git stash apply stash@{0}          # Apply specific stash
```

### Recover Lost Commits

```powershell
# Find lost commits
git reflog                         # Recent commits you've been on
git reflog expire --expire=now --all  # Force cleanup
git fsck --lost-found              # Find unreachable objects

# Restore lost commit
git reset --hard <commit-from-reflog>  # Go back to that commit
git cherry-pick <orphaned-commit>      # Pick specific commit
```

---

## REMOTES & SYNC

### Remote Configuration

```powershell
# List remotes
git remote -v                      # Verbose (shows URLs)
git remote show origin             # Details about remote

# Add/Remove remotes
git remote add backup C:\backup.bundle  # Add local backup
git remote add github https://github.com/...
git remote remove backup           # Remove remote
git remote rename origin github     # Rename remote

# Configure remote
git config remote.origin.url "https://..."  # Change URL
```

### Push & Pull

```powershell
# Push changes
git push                           # Push to default remote
git push origin main               # Push specific branch
git push origin --all              # Push all branches
git push --tags                    # Push all tags

# Pull changes
git pull                           # Fetch + merge
git pull --rebase                  # Fetch + rebase
git fetch                          # Fetch only (no merge)
git fetch --all                    # Fetch from all remotes
```

---

## ADVANCED

### Interactive Rebase

```powershell
# Rewrite last 5 commits
git rebase -i HEAD~5

# In editor:
# pick - use commit
# reword - use commit, edit message
# squash - use commit, combine with previous
# fixup - use commit, discard message
# drop - remove commit
```

### Cherry Pick

```powershell
# Apply specific commits
git cherry-pick <commit>           # Single commit
git cherry-pick <commit1> <commit2>  # Multiple commits
git cherry-pick <start>..<end>     # Range of commits
git cherry-pick --continue         # After resolving conflicts
```

### Format Patch & Apply

```powershell
# Export commits
git format-patch -5                # Last 5 as patches

# Share patches
git send-email *.patch             # Email patches
git apply < patch-file             # Apply patch
git am patch-file                  # Apply with message
```

### Bisect (Find Breaking Commit)

```powershell
# Start bisect
git bisect start
git bisect bad                     # Mark current as broken
git bisect good <commit>           # Mark old commit as good
# Git tests commits; mark each as good/bad
git bisect good                    # Current works
git bisect bad                     # Current broken
git bisect reset                   # Return to original
```

---

## CONFIGURATION

### Global Settings

```powershell
# View config
git config --list                  # All settings
git config --global --list         # Global settings
git config user.name               # Specific setting

# Set config
git config --global user.name "Your Name"
git config --global user.email "email@example.com"
git config --global core.editor "code"
git config --global alias.lg "log --graph --oneline --all"
```

### Ignore Files

```powershell
# .gitignore entries
.pio/build/                   # PlatformIO build
.vscode/settings.json         # VS Code settings
*.log                         # Log files
.DS_Store                     # macOS
```

---

## REPO MAINTENANCE

### Cleanup & Optimization

```powershell
# Clean up
git gc                             # Garbage collection
git gc --aggressive                # More thorough
git prune                          # Remove unreachable objects
git clean -fd                      # Remove untracked files

# Verify integrity
git fsck --full                    # Check for corruption
git reflog expire --expire=now --all  # Clean reflog
```

### Large Files Management

```powershell
# Find large files
git rev-list --all --objects | `
  Select-String -Pattern "size" | `
  Sort-Object -Property Length | `
  Select-Object -Last 10

# Remove large file
git filter-branch --tree-filter 'rm -f largefile' HEAD
git push --force origin main       # Force push (careful!)
```

---

## TROUBLESHOOTING

### Common Issues

```powershell
# Merge conflicts
git status                         # See conflicts
# Edit conflicted files, then:
git add .
git commit -m "Resolve conflicts"

# Wrong branch
git checkout main                  # Switch to correct branch

# Accidental commit
git reset --soft HEAD~1            # Keep changes, undo commit
git commit -m "Fixed message"      # Re-commit

# Lost commits
git reflog                         # Find it
git reset --hard <hash>            # Restore

# Corrupted repo
git fsck --full                    # Check
git gc --aggressive                # Repair
```

### Performance

```powershell
# Slow operations
git gc --aggressive                # Optimize repository
git clean -fd                      # Remove junk

# Large clone
git clone --depth 1 <url>          # Shallow clone
git fetch --unshallow              # Later make it full
```

---

## QUICK ALIASES (Add to Config)

```powershell
# Fast commands
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.lg "log --graph --oneline --all"
git config --global alias.unstage "reset HEAD --"
git config --global alias.undo "reset --soft HEAD~1"

# Then use:
git st                    # Instead of git status
git co main               # Instead of git checkout main
git lg                    # Pretty log
```

---

## EVERYDAY WORKFLOW

### Daily Work

```powershell
cd C:\PlatformIO\MetaSense-DYNO

# Start day: update from remote
git pull origin main

# During day: commit regularly
git status -s              # Quick view
git add src/Input.cpp      # Stage changes
git commit -m "Fixed CAN timeout"

# End of day: push to GitHub
git push origin main

# Create backup
git bundle create "backups\daily-$(Get-Date -Format yyyyMMdd).bundle" --all
```

### Before Major Changes

```powershell
# Create checkpoint
git tag -a pre-14D -m "Before Phase 14D refactor"

# Create branch for experimental work
git checkout -b experimental
# ... make changes ...
git commit -m "WIP: experimental feature"

# If successful, merge back
git checkout main
git merge experimental

# If failed, discard
git checkout main
git branch -D experimental

# Get back your tag
git reset --hard pre-14D
```

---

## REFERENCE

- **Status codes**: M=modified, A=added, D=deleted, ??=untracked
- **Commit hash**: First 7 chars usually enough: `abc1234`
- **HEAD**: Current commit, `HEAD~1` = parent, `HEAD~5` = 5 back
- **Branches**: `main` = default, usually tracked from remote `origin/main`
- **Tags**: Lightweight (pointer) vs Annotated (full commit info)

**For help on any command**: `git <command> --help` or `git help <command>`

