#!/usr/bin/env python3
"""Check git repository status"""
import subprocess
import json
import os

os.chdir(r"C:\PlatformIO\MetaSense-DYNO")

print("=== GIT STATUS ===")
result = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True)
if result.returncode == 0:
    if result.stdout.strip():
        print(f"Unstaged changes:\n{result.stdout}")
    else:
        print("Working directory: CLEAN (no unstaged changes)")
else:
    print(f"Error: {result.stderr}")

print("\n=== GIT BRANCHES ===")
result = subprocess.run(["git", "branch", "-a"], capture_output=True, text=True)
print(result.stdout if result.returncode == 0 else f"Error: {result.stderr}")

print("\n=== GIT LOG (Last 10 commits) ===")
result = subprocess.run(["git", "log", "--oneline", "-10"], capture_output=True, text=True)
print(result.stdout if result.returncode == 0 else f"Error: {result.stderr}")

print("\n=== CURRENT BRANCH ===")
result = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"], capture_output=True, text=True)
print(result.stdout.strip() if result.returncode == 0 else f"Error: {result.stderr}")

print("\n=== REMOTE CONFIG ===")
result = subprocess.run(["git", "remote", "-v"], capture_output=True, text=True)
print(result.stdout if result.returncode == 0 else f"Error: {result.stderr}")
