#!/usr/bin/env bash

IMG="glider_hybrid_whoi"
CONTAINER_ID=""

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -i)
        IMG="$2"
        shift 2
        ;;
    --name)
        CONTAINER_ID="$2"
        shift 2
        ;;
    *)    # unknown option
        echo "Unknown option" >&2
        exit 1
        ;;
esac
done

if [ -z "$CONTAINER_ID" ]; then
    CONTAINER_ID="$(docker ps -qf "ancestor=${IMG}")"
fi

num_containers="$(echo "$CONTAINER_ID" | wc -l)"

if [ "$num_containers" = "0" ]; then
    echo "There are no running containers to join" >&2
    exit 1
elif [ "$num_containers" -gt "1" ]; then
    echo "There are multiple containers to join. You must specify one" >&2
    exit 1
fi

docker exec --privileged -e DISPLAY -it "${CONTAINER_ID}" bash
