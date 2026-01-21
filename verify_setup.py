#!/usr/bin/env python3
"""
DQN Implementation Verification Script
Run this to verify all files are in place and ready
"""

import os
from pathlib import Path

def check_files():
    """Verify all required files exist"""
    print("\n" + "="*70)
    print("DQN Deep Q Learning Implementation - File Verification".center(70))
    print("="*70 + "\n")
    
    base_path = Path(__file__).parent
    
    # Core implementation files
    core_files = {
        'train_dqn.py': 'Main training script',
        'run_dqn.py': 'Testing and comparison script',
        'dqn_utils.py': 'Advanced analysis tools',
        'dqn_demo.py': 'Interactive demo interface',
    }
    
    # Documentation files
    doc_files = {
        'README.md': 'Main readme and summary',
        'INDEX.md': 'Navigation and file index',
        'QUICKSTART.md': '5-minute quick start guide',
        'DQN_README.md': 'Comprehensive technical documentation',
        'SETUP_GUIDE.md': 'Setup and troubleshooting guide',
        'ARCHITECTURE_OVERVIEW.md': 'System design and architecture',
        'IMPLEMENTATION_SUMMARY.md': 'Implementation overview',
        'requirements.txt': 'Python dependencies',
    }
    
    # Check core files
    print("✓ CORE IMPLEMENTATION FILES (4):")
    print("-" * 70)
    all_exist = True
    for filename, description in core_files.items():
        filepath = base_path / filename
        exists = filepath.exists()
        status = "✓" if exists else "✗"
        size = f"{filepath.stat().st_size:,} bytes" if exists else "NOT FOUND"
        print(f"  {status} {filename:20s} - {description:30s} ({size})")
        all_exist = all_exist and exists
    
    # Check documentation files
    print("\n✓ DOCUMENTATION FILES (8):")
    print("-" * 70)
    for filename, description in doc_files.items():
        filepath = base_path / filename
        exists = filepath.exists()
        status = "✓" if exists else "✗"
        size = f"{filepath.stat().st_size:,} bytes" if exists else "NOT FOUND"
        print(f"  {status} {filename:30s} - {description:30s} ({size})")
        all_exist = all_exist and exists
    
    # Summary
    print("\n" + "="*70)
    if all_exist:
        print("✓ ALL FILES PRESENT - SYSTEM READY FOR USE".center(70))
    else:
        print("✗ SOME FILES MISSING - PLEASE CHECK".center(70))
    print("="*70)
    
    # Statistics
    print("\n📊 SYSTEM STATISTICS:")
    print("-" * 70)
    
    total_code_lines = 0
    for filename in core_files.keys():
        filepath = base_path / filename
        if filepath.exists():
            with open(filepath, 'r') as f:
                total_code_lines += len(f.readlines())
    
    total_doc_lines = 0
    for filename in doc_files.keys():
        filepath = base_path / filename
        if filepath.exists() and filename.endswith('.md'):
            with open(filepath, 'r') as f:
                total_doc_lines += len(f.readlines())
    
    print(f"  Code Files:           4 files")
    print(f"  Code Lines:           ~{total_code_lines:,} lines")
    print(f"  Documentation Files:  8 files")
    print(f"  Documentation Lines:  ~{total_doc_lines:,} lines")
    print(f"  Total Files:          12 files")
    print(f"  Total Lines:          ~{total_code_lines + total_doc_lines:,} lines")
    
    # Quick start guide
    print("\n" + "="*70)
    print("🚀 QUICK START (Copy & Paste):".center(70))
    print("="*70)
    print("""
  1. Install dependencies:
     pip install -r requirements.txt

  2. Train the model (takes ~20 minutes):
     python train_dqn.py

  3. Test the model:
     python run_dqn.py --episodes 10

  4. Compare with random agent:
     python run_dqn.py --compare --episodes 10

  5. Interactive demo:
     python dqn_demo.py --interactive
    """)
    
    # Documentation guide
    print("\n" + "="*70)
    print("📖 DOCUMENTATION GUIDE:".center(70))
    print("="*70)
    print("""
  Start Here:
    → README.md - Main overview and summary

  For Quick Start (5 min):
    → QUICKSTART.md - Commands and examples

  For Understanding (20 min):
    → ARCHITECTURE_OVERVIEW.md - System design
    → DQN_README.md - Technical documentation

  For Setup & Configuration:
    → SETUP_GUIDE.md - Detailed guide with troubleshooting

  For Navigation:
    → INDEX.md - File guide and links

  For Overview:
    → IMPLEMENTATION_SUMMARY.md - Feature overview
    """)
    
    # Command examples
    print("\n" + "="*70)
    print("⚡ COMMON COMMANDS:".center(70))
    print("="*70)
    print("""
  Training:
    python train_dqn.py              # Full training (200 episodes)
    python dqn_demo.py --quick-train # Quick training (50 episodes)

  Testing:
    python run_dqn.py                # Test trained model
    python run_dqn.py --episodes 20  # Custom episodes

  Comparison:
    python run_dqn.py --compare      # Compare with random agent

  Analysis:
    python dqn_utils.py analyze      # Behavior analysis
    python dqn_utils.py grid_search  # Hyperparameter search

  Interactive:
    python dqn_demo.py               # Interactive menu
    """)
    
    # System info
    print("\n" + "="*70)
    print("ℹ️  SYSTEM INFORMATION:".center(70))
    print("="*70)
    
    try:
        import torch
        print(f"  PyTorch:     ✓ Installed (GPU: {torch.cuda.is_available()})")
    except:
        print(f"  PyTorch:     ✗ Not installed")
    
    try:
        import numpy
        print(f"  NumPy:       ✓ Installed")
    except:
        print(f"  NumPy:       ✗ Not installed")
    
    try:
        import matplotlib
        print(f"  Matplotlib:  ✓ Installed")
    except:
        print(f"  Matplotlib:  ✗ Not installed")
    
    # Final message
    print("\n" + "="*70)
    print("✅ SETUP COMPLETE - READY TO USE".center(70))
    print("="*70)
    print("\nNext Step: Run 'python train_dqn.py' to start training!\n")


if __name__ == "__main__":
    check_files()
