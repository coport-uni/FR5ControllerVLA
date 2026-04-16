#!/usr/bin/env bash
# Display an image on an X server from inside the dev container.
#
# Two display modes are supported:
#
#   1. ssh   -- target an SSH X11-forwarded display on the NUC host
#               (e.g. you ran `ssh -X` from a laptop). The image will
#               appear on the laptop screen.
#               Activated when CLAUDE_SSH_DISPLAY is set, or via
#               --mode=ssh.
#
#   2. host  -- target the NUC's local Xwayland :0 (GNOME desktop).
#               Default when no SSH display is available. The image
#               will appear on the NUC's monitor.
#
# In both modes we copy the relevant MIT-MAGIC-COOKIE-1 into the
# container's xauth and point $DISPLAY at the matching unix socket
# under /tmp/.X11-unix/.
#
# Usage:
#   claude_test/show_image.sh [--mode=ssh|host] <image-path>
#
# Environment variables (used in ssh mode):
#   CLAUDE_SSH_DISPLAY   -- e.g. ":10"  (from `echo $DISPLAY` on NUC)
#   CLAUDE_SSH_XAUTH     -- e.g. "/home/inno-controller/.Xauthority"
#                           (from `echo $XAUTHORITY` on NUC; if unset
#                           the script searches the active sshd child)

set -euo pipefail

mode=""
image_path=""

for arg in "$@"; do
    case "$arg" in
        --mode=ssh)  mode="ssh" ;;
        --mode=host) mode="host" ;;
        --mode=*)    echo "error: unknown mode: $arg" >&2; exit 2 ;;
        -h|--help)
            sed -n '2,25p' "$0"
            exit 0
            ;;
        *)
            if [[ -z "$image_path" ]]; then
                image_path="$arg"
            else
                echo "error: unexpected argument: $arg" >&2
                exit 2
            fi
            ;;
    esac
done

if [[ -z "$image_path" ]]; then
    echo "usage: $0 [--mode=ssh|host] <image-path>" >&2
    exit 2
fi

if [[ ! -f "$image_path" ]]; then
    echo "error: image not found: $image_path" >&2
    exit 1
fi

if [[ -z "$mode" ]]; then
    if [[ -n "${CLAUDE_SSH_DISPLAY:-}" ]]; then
        mode="ssh"
    else
        mode="host"
    fi
fi

# ---------------------------------------------------------------------------
# Resolve target_display + xauth_file based on mode.
# ---------------------------------------------------------------------------
target_display=""
xauth_file=""

if [[ "$mode" == "host" ]]; then
    target_display="${DISPLAY:-}"
    if [[ -z "$target_display" ]]; then
        echo "error: DISPLAY is not set (host mode)" >&2
        exit 1
    fi

    xwayland_pid=$(pgrep -x Xwayland | head -n1 || true)
    if [[ -z "$xwayland_pid" ]]; then
        echo "error: Xwayland process not found on host" >&2
        exit 1
    fi
    host_auth=$(tr '\0' '\n' < "/proc/${xwayland_pid}/cmdline" \
        | awk '/Xwaylandauth/ {print; exit}')
    xauth_file="/proc/${xwayland_pid}/root${host_auth}"

elif [[ "$mode" == "ssh" ]]; then
    target_display="${CLAUDE_SSH_DISPLAY:-}"
    if [[ -z "$target_display" ]]; then
        echo "error: CLAUDE_SSH_DISPLAY not set (ssh mode)" >&2
        echo "  hint: on the NUC SSH shell, run claude_test/setup_ssh_x11.sh" >&2
        exit 1
    fi

    # SSH-forwarded DISPLAY uses TCP form like "localhost:10.0".
    # Convert to ":10" so we hit the unix socket in /tmp/.X11-unix/.
    if [[ "$target_display" == localhost:* ]]; then
        target_display=":${target_display#localhost:}"
        target_display="${target_display%.*}"
    fi

    xauth_file="${CLAUDE_SSH_XAUTH:-}"
    if [[ -z "$xauth_file" ]]; then
        # Search the sshd child processes for the X11-forwarding session.
        for pid in $(pgrep -f "sshd: " || true); do
            cand=$(tr '\0' '\n' < "/proc/${pid}/environ" 2>/dev/null \
                | awk -F= '/^XAUTHORITY=/ {print $2; exit}')
            if [[ -n "$cand" && -r "/proc/${pid}/root${cand}" ]]; then
                xauth_file="/proc/${pid}/root${cand}"
                break
            fi
        done
    fi
    if [[ -z "$xauth_file" || ! -r "$xauth_file" ]]; then
        echo "error: cannot locate SSH XAUTHORITY file" >&2
        echo "  set CLAUDE_SSH_XAUTH to the value of \$XAUTHORITY" \
             "in the NUC SSH shell" >&2
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# Extract cookie and register it for the container.
# ---------------------------------------------------------------------------
display_num="${target_display#:}"
display_num="${display_num%%.*}"

socket_path="/tmp/.X11-unix/X${display_num}"
if [[ ! -S "$socket_path" ]]; then
    echo "error: X11 socket not found: $socket_path" >&2
    echo "  for ssh mode, ensure you ran ssh -X to the NUC first" >&2
    exit 1
fi

cookie=$(xauth -f "$xauth_file" list \
    | awk '/MIT-MAGIC-COOKIE-1/ {print $3; exit}')
if [[ -z "$cookie" ]]; then
    echo "error: no MIT-MAGIC-COOKIE-1 in $xauth_file" >&2
    exit 1
fi

xauth add ":${display_num}" MIT-MAGIC-COOKIE-1 "$cookie" >/dev/null
xauth add "${HOSTNAME}/unix:${display_num}" \
    MIT-MAGIC-COOKIE-1 "$cookie" >/dev/null

echo "mode=${mode} display=:${display_num} xauth=${xauth_file}"
DISPLAY=":${display_num}" exec feh --title "claude-x11" "$image_path"
