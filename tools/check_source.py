#!/usr/bin/env python3
"""Check Python syntax, XML and numerical contracts without a ROS installation."""
import ast
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parents[1]
for path in sorted(root.rglob('*.py')):
    if not any(part in ('build','install','.git') for part in path.parts):
        ast.parse(path.read_text(encoding='utf-8'),filename=str(path))
ET.parse(root/'package.xml')
result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests',
                       '-p','test_core.py','-v'],cwd=root)
raise SystemExit(result.returncode)
