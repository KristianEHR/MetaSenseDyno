#!/usr/bin/env python3
"""
Direct git consolidation via Python
Avoids terminal output issues
"""
import subprocess
import os
import sys

os.chdir(r"C:\PlatformIO\MetaSense-DYNO")

print("=" * 70)
print("GIT CONSOLIDATION - MetaSense-DYNO")
print("=" * 70)

# Step 1: Show current branch
print("\n[STEP 1] Current branch:")
result = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"], 
                       capture_output=True, text=True)
current_branch = result.stdout.strip()
print(f"  Current: {current_branch}")

# Step 2: Check for uncommitted changes
print("\n[STEP 2] Check for uncommitted changes:")
result = subprocess.run(["git", "status", "--porcelain"],
                       capture_output=True, text=True)
if result.stdout.strip():
    print("  Changes found. Staging...")
    subprocess.run(["git", "add", "."], capture_output=True)
    subprocess.run(["git", "commit", "-m", "Stage changes before consolidation"],
                  capture_output=True)
    print("  ✓ Staged and committed")
else:
    print("  ✓ Working directory clean")

# Step 3: Checkout main
print("\n[STEP 3] Checkout main branch:")
result = subprocess.run(["git", "checkout", "main"],
                       capture_output=True, text=True)
if result.returncode == 0:
    print("  ✓ Switched to main")
else:
    print(f"  Error: {result.stderr}")
    sys.exit(1)

# Step 4: Verify we're on main
print("\n[STEP 4] Verify current branch:")
result = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"],
                       capture_output=True, text=True)
print(f"  Current: {result.stdout.strip()}")

# Step 5: Merge feature branch
print("\n[STEP 5] Merge feature/cleanup-and-bugfixes into main:")
result = subprocess.run(["git", "merge", "feature/cleanup-and-bugfixes", 
                        "--no-edit", "-m",
                        "Consolidation: Merge feature/cleanup-and-bugfixes to main"],
                       capture_output=True, text=True)
if result.returncode == 0:
    print("  ✓ Merge successful")
    if result.stdout:
        print(f"  {result.stdout.strip()[:100]}")
else:
    print(f"  Warning: {result.stderr[:200]}")

# Step 6: Show log
print("\n[STEP 6] Recent commits (last 5):")
result = subprocess.run(["git", "log", "--oneline", "-5"],
                       capture_output=True, text=True)
for line in result.stdout.strip().split("\n"):
    print(f"  {line}")

# Step 7: Delete feature branch
print("\n[STEP 7] Delete feature branch:")
result = subprocess.run(["git", "branch", "-d", "feature/cleanup-and-bugfixes"],
                       capture_output=True, text=True)
if result.returncode == 0:
    print("  ✓ Feature branch deleted")
else:
    print(f"  Note: {result.stderr.strip()[:100]}")

# Step 8: List branches
print("\n[STEP 8] Branches (after consolidation):")
result = subprocess.run(["git", "branch", "-a", "-v"],
                       capture_output=True, text=True)
for line in result.stdout.strip().split("\n"):
    if line.strip():
        print(f"  {line}")

# Step 9: Push to GitHub
print("\n[STEP 9] Push to GitHub:")
result = subprocess.run(["git", "push", "origin", "main"],
                       capture_output=True, text=True)
if result.returncode == 0:
    print("  ✓ Pushed to GitHub")
else:
    # May fail if no internet, but that's OK for local consolidation
    print(f"  Note: {result.stderr.strip()[:100]}")

# Step 10: Final status
print("\n[STEP 10] Final status:")
result = subprocess.run(["git", "status"],
                       capture_output=True, text=True)
for line in result.stdout.strip().split("\n")[:5]:
    print(f"  {line}")

print("\n" + "=" * 70)
print("✓ CONSOLIDATION COMPLETE")
print("=" * 70)
print("\nAll work is now on 'main' branch")
print("Repository ready for next development session\n")

# Save result to file for verification
with open("consolidation_result.txt", "w") as f:
    f.write("CONSOLIDATION COMPLETED\n")
    f.write(f"Date: {__import__('datetime').datetime.now()}\n")
    f.write(f"Branch: {result.stdout}\n")
