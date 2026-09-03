#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$repository_root"

fixed_home='/home/'"dndx"
legacy_name='G2_'"PICK"
pattern="$fixed_home|~/$legacy_name|$legacy_name"
violations=()

while IFS= read -r -d '' path; do
    [[ -f "$path" ]] || continue
    if LC_ALL=C grep -nE "$pattern" "$path" >/dev/null 2>&1; then
        violations+=("$path")
    fi
done < <(git ls-files -z)

if ((${#violations[@]} > 0)); then
    printf 'Fixed local paths remain in tracked files:\n' >&2
    printf '  %s\n' "${violations[@]}" >&2
    exit 1
fi

printf 'PASS: tracked files contain no fixed local paths\n'
