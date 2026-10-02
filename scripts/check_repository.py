"""Check this standalone repository without importing motor code or opening IO."""
import json
from pathlib import Path
import re
import sys
from urllib.parse import unquote
from ros2_contract import check_ros2_contract

ROOT = Path(__file__).resolve().parents[1]
IGNORE_PARTS = {'.git', '.vscode', '__pycache__', 'build', 'devel', 'install', 'logs', 'tmp'}
HISTORICAL = {'docs/迁移记录.json', 'docs/离线校验.json'}
SECRET_RULES = {
    'private_key': r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
    'github_token': r'(?:github_pat_[A-Za-z0-9_]{20,}|gh[pousr]_[A-Za-z0-9]{30,})',
    'aws_access_key': r'AKIA[0-9A-Z]{16}',
    'api_secret_assignment': r'(?i)(?:password|passwd|api_key|access_token|secret)\s*[:=]\s*[\x22\x27][^\x22\x27]{8,}',
}


def eligible(path):
    relative = path.relative_to(ROOT)
    return (relative.as_posix() not in HISTORICAL and
            not any(part in IGNORE_PARTS or part.startswith('build-') for part in relative.parts))


def main():
    errors = []
    files = sorted(p for p in ROOT.rglob('*') if p.is_file() and eligible(p))
    required = ['LICENSE', 'NOTICE', 'README.md', 'CMakeLists.txt', 'config/motors.yaml',
                'common/src/usb_can.cpp', 'scripts/build_ros2.sh', 'scripts/run_ros2.sh',
                'tests/cpp/protocol_test.cpp', 'tests/cpp/usb2can_io_test.cpp',
                'common/include/joint_motion.hpp', 'common/src/joint_motion.cpp', 'common/ruckig.cmake',
                'tests/cpp/trajectory_test.cpp', 'tests/cpp/ros2_motion_test.cpp',
                'third_party/ruckig/LICENSE', 'config/trajectory.example.yaml',
                'ros2_ws/src/usb2can_demo_lingzu/package.xml', 'launch/usb2can_joystick.launch.py']
    for name in required:
        if not (ROOT/name).is_file():
            errors.append('Missing required file: ' + name)
    links = 0
    for path in files:
        relative = path.relative_to(ROOT).as_posix()
        if path.stat().st_size >= 95 * 1024 * 1024:
            errors.append('File exceeds publication size limit: ' + relative)
        if path.suffix in ('.pdf', '.so'):
            continue
        try:
            content = path.read_text(encoding='utf-8-sig')
        except UnicodeDecodeError:
            errors.append('Unexpected binary file: ' + relative)
            continue
        for name, pattern in SECRET_RULES.items():
            if re.search(pattern, content):
                errors.append('Potential credential [' + name + ']: ' + relative)
        if path.suffix == '.py':
            compile(content, relative, 'exec')
        if path.suffix in ('.sh', '.yaml', '.yml', '.xml', '.launch') and '\r' in path.read_bytes().decode('utf-8-sig'):
            errors.append('Use LF newlines for: ' + relative)
        if path.suffix == '.md':
            for match in re.finditer(r'\]\((<[^>]+>|[^)\n]+)\)', content):
                target = match.group(1).strip('<>')
                if re.match(r'^(?:https?://|mailto:|#)', target):
                    continue
                if re.match(r'^(?:[A-Za-z]:|/)', target):
                    errors.append('Local absolute document link: ' + relative)
                    continue
                linked = path.parent / unquote(target.split('#')[0])
                if not linked.exists():
                    errors.append('Missing document link in ' + relative + ': ' + target)
                links += 1
    errors.extend(check_ros2_contract(ROOT))
    result = {'passed': not errors, 'files_checked': len(files), 'local_links_checked': links,
              'ros2_launch_and_package_checked': True, 'credential_scan_passed': not any('credential' in e for e in errors),
              'hardware_opened': False, 'errors': errors}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
