#!/usr/bin/env python3
"""Extract git log information and write to file"""
import subprocess
import os

os.chdir(r"C:\PlatformIO\MetaSense-DYNO")

with open("git_status_dump.txt", "w") as f:
    f.write("=== GIT STATUS ===\n")
    result = subprocess.run(["git", "status"], capture_output=True, text=True)
    f.write(result.stdout)
    f.write(result.stderr)
    
    f.write("\n\n=== GIT LOG (Last 15 commits on current branch) ===\n")
    result = subprocess.run(["git", "log", "--oneline", "-15"], capture_output=True, text=True)
    f.write(result.stdout)
    f.write(result.stderr)
    
    f.write("\n\n=== ALL BRANCHES ===\n")
    result = subprocess.run(["git", "branch", "-a", "-v"], capture_output=True, text=True)
    f.write(result.stdout)
    f.write(result.stderr)
    
    f.write("\n\n=== MAIN BRANCH LOG ===\n")
    result = subprocess.run(["git", "log", "--oneline", "-10", "main"], capture_output=True, text=True)
    f.write(result.stdout)
    f.write(result.stderr)
    
    f.write("\n\n=== DIFFERENCE between current and main ===\n")
    result = subprocess.run(["git", "log", "--oneline", "feature/cleanup-and-bugfixes", "^main"], capture_output=True, text=True)
    f.write(result.stdout)
    if result.stderr:
        f.write("STDERR: " + result.stderr)
    
    f.write("\n\n=== Unstaged files ===\n")
    result = subprocess.run(["git", "diff", "--name-status"], capture_output=True, text=True)
    f.write(result.stdout)
    if result.stderr:
        f.write("STDERR: " + result.stderr)

print("Status written to git_status_dump.txt")
