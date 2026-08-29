#!/usr/bin/env bash
export BASE_DIR=/var/lib/nRF24

set -euo pipefail

: "${BASE_DIR:?Set BASE_DIR to the directory where MQTT messages will be stored.}"

# Optional comma-separated allowlist
MESSAGE_TYPES='s,S,t,T,w,W,a,A,r,R,b,B'

# Leave empty to subscribe to all clientMessage topics.
MESSAGE_TYPES="${MESSAGE_TYPES:-}"

readonly TOPIC_FILTER='nRF24Client/clientMessage/+/+'

command -v mosquitto_sub >/dev/null 2>&1 || {
    printf 'mosquitto_sub is required but was not found in PATH.\n' >&2
    exit 127
}

declare -a topic_filters=()

if [[ -n "$MESSAGE_TYPES" ]]; then
    IFS=',' read -r -a allowed_types <<<"$MESSAGE_TYPES"
    for message_type in "${allowed_types[@]}"; do
        message_type="${message_type//[[:space:]]/}"
        [[ -n "$message_type" ]] || continue
        topic_filters+=("-t" "nRF24Client/clientMessage/+/$message_type")
    done

    if [[ ${#topic_filters[@]} -eq 0 ]]; then
        printf 'MESSAGE_TYPES was provided but contained no valid message types.\n' >&2
        exit 1
    fi
else
    topic_filters=("-t" "$TOPIC_FILTER")
fi

nohup mosquitto_sub -h localhost "${topic_filters[@]}" -v | while IFS=' ' read -r topic payload; do
    IFS='/' read -r namespace message_group client_id message_type <<<"$topic"

    if [[ "$namespace" != 'nRF24Client' || "$message_group" != 'clientMessage' || -z "$client_id" || -z "$message_type" ]]; then
        printf 'Ignoring unexpected MQTT topic: %s\n' "$topic" >&2
        continue
    fi

    if [[ -n "$MESSAGE_TYPES" ]]; then
        allowed=0
        IFS=',' read -r -a allowed_types <<<"$MESSAGE_TYPES"
        for allowed_type in "${allowed_types[@]}"; do
            allowed_type="${allowed_type//[[:space:]]/}"
            if [[ "$message_type" == "$allowed_type" ]]; then
                allowed=1
                break
            fi
        done

        if (( allowed == 0 )); then
            printf 'Ignoring disallowed message type: %s\n' "$message_type" >&2
            continue
        fi
    fi

    destination="$BASE_DIR/$client_id/$message_type"
    mkdir -p "$(dirname "$destination")"
    printf '%s\n' "$payload" >"$destination"
done &
