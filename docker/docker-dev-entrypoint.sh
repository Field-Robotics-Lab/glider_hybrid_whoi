#! /bin/bash
set -e

eval "$(fixuid)"

if [ -n "$XAUTHORITY_ENTRY" ]; then
    # Tell xauth to use the wild protocol. This ensures that it will try to use
    # this cookie when connecting, even though host names are
    # different. Alternatively, you could mangle $DISPLAY, explicitly create a
    # wild entry using `xauth generate`, etc.
    echo "$XAUTHORITY_ENTRY" | sed -e 's/^..../ffff/' | xauth nmerge -
fi

source "/home/$GLIDER_HYBRID_WHOI_USER/glider_hybrid_whoi/devel/setup.bash"

exec "$@"
