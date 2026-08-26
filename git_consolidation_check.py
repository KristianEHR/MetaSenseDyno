#!/usr/bin/env python3
"""Consolidate git repository to main branch"""
import subprocess
import sys
import os

os.chdir(r"C:\PlatformIO\MetaSense-DYNO")

def run_git(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True, shell=True)
    return result.stdout + (result.stderr if result.returncode != 0 else "")

# Get current state
output = []
output.append("=" * 60)
output.append("GIT CONSOLIDATION - BEFORE")
output.append("=" * 60)

output.append("\n1. Current Branch:")
current = run_git("git rev-parse --abbrev-ref HEAD")
output.append(current)

output.append("\n2. Main Branch (Last 5 commits):")
output.append(run_git("git log --oneline -5 main"))

output.append("\n3. Feature Branch (Last 5 commits):")
output.append(run_git("git log --oneline -5 feature/cleanup-and-bugfixes"))

output.append("\n4. Current Status:")
output.append(run_git("git status"))

output.append("\n5. All Branches:")
output.append(run_git("git branch -a -v"))

output.append("\n6. Commits on feature/cleanup-and-bugfixes NOT in main:")
output.append(run_git("git log --oneline main..feature/cleanup-and-bugfixes"))

# Write to file
with open("git_consolidation_state.txt", "w") as f:
    f.write("\n".join(output))

print("State written to git_consolidation_state.txt")
print("\n".join(output[:30]))  # Print first 30 lines
