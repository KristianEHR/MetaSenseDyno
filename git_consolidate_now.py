#!/usr/bin/env python3
"""
CONSOLIDATION SCRIPT: Merge feature branch to main
Suitable for single-user, single-project development
"""
import subprocess
import os

os.chdir(r"C:\PlatformIO\MetaSense-DYNO")

def run_git(cmd, verbose=True):
    """Run git command and return output"""
    result = subprocess.run(cmd, capture_output=True, text=True, shell=True)
    output = result.stdout + (result.stderr if result.returncode != 0 else "")
    if verbose:
        print(f"$ {cmd}")
        print(output)
    return output, result.returncode

print("=" * 70)
print("METASENSE-DYNO GIT CONSOLIDATION")
print("=" * 70)
print("Consolidating feature/cleanup-and-bugfixes into main branch")
print("(Single user, single project - feature branch not needed)\n")

# Step 1: Verify clean working directory
print("STEP 1: Check working directory...")
output, code = run_git("git status --porcelain")
if output.strip():
    print(f"⚠️  WARNING: Uncommitted changes detected:\n{output}")
    print("Committing changes first...\n")
    run_git("git add .")
    run_git("git commit -m 'WIP: Changes before consolidation'")
else:
    print("✓ Working directory is clean\n")

# Step 2: Switch to main
print("STEP 2: Switch to main branch...")
run_git("git checkout main")

# Step 3: Merge feature branch
print("\nSTEP 3: Merge feature/cleanup-and-bugfixes into main...")
run_git("git merge feature/cleanup-and-bugfixes -m 'Consolidation: Merge feature/cleanup-and-bugfixes into main'")

# Step 4: Verify merge
print("\nSTEP 4: Verify consolidation...")
run_git("git log --oneline -5")

# Step 5: Check status
print("\nSTEP 5: Final status...")
run_git("git status")

# Step 6: Show all branches
print("\nSTEP 6: Branch status after merge...")
run_git("git branch -a -v")

# Step 7: (Optional) Delete feature branch
print("\nSTEP 7: Cleanup - Delete feature branch...")
response = input("Delete feature/cleanup-and-bugfixes branch? (y/n): ").strip().lower()
if response == 'y':
    run_git("git branch -d feature/cleanup-and-bugfixes")
    print("✓ Feature branch deleted\n")
else:
    print("✓ Feature branch kept\n")

# Step 8: Push to GitHub (if configured)
print("STEP 8: Push to GitHub...")
run_git("git remote -v")
response = input("Push main to GitHub? (y/n): ").strip().lower()
if response == 'y':
    run_git("git push origin main")
    print("✓ Pushed to GitHub\n")
else:
    print("✓ Push skipped\n")

print("=" * 70)
print("✓ CONSOLIDATION COMPLETE")
print("=" * 70)
print("\nYour repository is now consolidated:")
print("  • All changes on main branch")
print("  • Feature branch removed (if selected)")
print("  • Pushed to GitHub (if selected)")
print("  • Ready for next chat session\n")
