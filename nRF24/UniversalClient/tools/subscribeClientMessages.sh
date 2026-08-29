#!/usr/bin/env bash
export BASE_DIR=/var/lib/nRF24

set -euo pipefail

: "${BASE_DIR:?Set BASE_DIR to the directory where MQTT messages will be stored.}"

readonly TOPIC_FILTER='nRF24Client/clientMessage/+/+'

command -v mosquitto_sub >/dev/null 2>&1 || {
    printf 'mosquitto_sub is required but was not found in PATH.\n' >&2
    exit 127
}

nohup mosquitto_sub -h localhost -t "$TOPIC_FILTER" -v | while IFS=' ' read -r topic payload; do
    IFS='/' read -r namespace message_group client_id message_type <<<"$topic"

    if [[ "$namespace" != 'nRF24Client' || "$message_group" != 'clientMessage' || -z "$client_id" || -z "$message_type" ]]; then
        printf 'Ignoring unexpected MQTT topic: %s\n' "$topic" >&2
        continue
    fi

    destination="$BASE_DIR/$client_id/$message_type"
    mkdir -p "$(dirname "$destination")"
    printf '%s\n' "$payload" >"$destination"
done &
