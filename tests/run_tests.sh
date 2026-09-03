#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)

python3 -m unittest "$repository_root/tests/test_runtime_config.py" -v
bash "$repository_root/tests/test_portable_launchers.sh"
bash "$repository_root/tests/test_repository_hygiene.sh"
