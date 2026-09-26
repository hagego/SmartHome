#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    printf 'Usage: %s <base-directory>\n' "$0" >&2
    exit 1
fi

BASE_DIR="$1"

if [[ ! -d "$BASE_DIR" ]]; then
    printf 'Directory not found: %s\n' "$BASE_DIR" >&2
    exit 1
fi

for device_dir in "$BASE_DIR"/*/; do
    [[ -d "$device_dir" ]] || continue
    device_name="$(basename "$device_dir")"

    printf '%s\n' "$device_name"

    name_file="${device_dir}name"
    if [[ -f "$name_file" ]]; then
        printf '  %s\n' "$(<"$name_file")"
    fi

    for property_file in "$device_dir"*; do
        [[ -f "$property_file" ]] || continue
        property_name="$(basename "$property_file")"
        [[ "$property_name" == "name" ]] && continue
        property_value="$(<"$property_file")"
        updated_at="$(date -r "$property_file" '+%Y-%m-%d %H:%M:%S')"
        printf '  %s: %s (%s)\n' "$property_name" "$property_value" "$updated_at"
    done
done

