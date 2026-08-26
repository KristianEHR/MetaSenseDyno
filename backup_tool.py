#!/usr/bin/env python3
"""
MetaSense-DYNO Backup Tool
Creates git bundles and directory backups with timestamps
"""

import os
import shutil
import subprocess
from datetime import datetime
import sys

def create_git_bundle():
    """Create git bundle backup"""
    os.chdir(r"C:\PlatformIO\MetaSense-DYNO")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    bundle_path = f"backups\\MetaSense-DYNO-git-backup-{timestamp}.bundle"
    
    try:
        result = subprocess.run(
            ["git", "bundle", "create", bundle_path, "--all"],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        if result.returncode == 0:
            file_size = os.path.getsize(bundle_path) / (1024 * 1024)  # MB
            print(f"✓ Git bundle created: {bundle_path}")
            print(f"  Size: {file_size:.2f} MB")
            return True
        else:
            print(f"✗ Bundle creation failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def create_directory_backup():
    """Create full directory zip backup"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    src = r"C:\PlatformIO\MetaSense-DYNO"
    dst = f"C:\\PlatformIO\\MetaSense-DYNO_backup_{timestamp}.zip"
    
    try:
        print(f"Creating full backup (this may take a few minutes)...")
        shutil.make_archive(dst.replace('.zip', ''), 'zip', src)
        
        file_size = os.path.getsize(dst) / (1024 * 1024)  # MB
        print(f"✓ Full backup created: {dst}")
        print(f"  Size: {file_size:.2f} MB")
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == "__main__":
    print("MetaSense-DYNO Backup Tool")
    print("=" * 50)
    
    if len(sys.argv) > 1 and sys.argv[1] == "--full":
        print("Creating git bundle + full directory backup...")
        b1 = create_git_bundle()
        b2 = create_directory_backup()
        if b1 and b2:
            print("\n✓ All backups completed successfully!")
        else:
            print("\n⚠ Some backups failed - check output above")
    else:
        print("Creating git bundle backup (fastest)...")
        if create_git_bundle():
            print("\n✓ Backup completed successfully!")
            print("\nFor full backup: python backup_tool.py --full")
        else:
            print("\n✗ Backup failed")
            sys.exit(1)
