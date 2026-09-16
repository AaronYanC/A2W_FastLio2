#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)

python3 -m unittest "$repository_root/tests/test_runtime_config.py" -v
python3 -m unittest "$repository_root/tests/test_runtime_user_guide.py" -v
python3 -m unittest "$repository_root/tests/test_stage_contracts.py" -v
bash "$repository_root/tests/test_portable_launchers.sh"
bash "$repository_root/tests/test_repository_hygiene.sh"
bash "$repository_root/tests/test_full_validation_runner.sh"
