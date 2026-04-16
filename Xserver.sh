#!/usr/bin/env bash
# Bridge an SSH X11-forwarded display from the NUC host into this
# dev container so that GUI clients launched here appear on the
# laptop running `ssh -X`.
#
# Workflow:
#   1. On the laptop:    ssh -X inno-controller@<NUC>   (xclock works)
#   2. On the NUC shell: source ./Xserver.sh
#        -> exports CLAUDE_SSH_DISPLAY / CLAUDE_SSH_XAUTH on the NUC
#   3. In the container: source ./Xserver.sh
#        -> picks the active sshd X11 session, exports
#           CLAUDE_SSH_DISPLAY / CLAUDE_SSH_XAUTH for use by
#           claude_test/show_image.sh
#
# After sourcing inside the container, run for example:
#   claude_test/show_image.sh outputs/captured_images/realsense_333422300435.png
#
# Notes:
#   - X11Forwarding is already enabled in NUC sshd_config.
#   - X11UseLocalhost defaults to yes, but sshd still creates the
#     unix socket /tmp/.X11-unix/X<N> which the container can use.

set -u

_xserver_log() { printf '[Xserver] %s\n' "$*"; }

_xserver_in_container() {
    [[ -f /.dockerenv ]]
}

if ! _xserver_in_container; then
    # NUC shell: just publish the current SSH session's values.
    if [[ -n "${DISPLAY:-}" && "${DISPLAY}" == localhost:* ]]; then
        export CLAUDE_SSH_DISPLAY="${DISPLAY}"
        export CLAUDE_SSH_XAUTH="${XAUTHORITY:-${HOME}/.Xauthority}"
        _xserver_log "NUC session detected"
        _xserver_log "  CLAUDE_SSH_DISPLAY=${CLAUDE_SSH_DISPLAY}"
        _xserver_log "  CLAUDE_SSH_XAUTH=${CLAUDE_SSH_XAUTH}"
        _xserver_log "now run the same source command inside the container"
    else
        _xserver_log "DISPLAY is not an SSH-forwarded display (${DISPLAY:-unset})"
        _xserver_log "did you connect with 'ssh -X'?"
        return 1 2>/dev/null || exit 1
    fi
    return 0 2>/dev/null || exit 0
fi

# --- inside container ---
# Find any sshd child whose XAUTHORITY points to a readable cookie file.
chosen_pid=""
chosen_display=""
chosen_xauth=""

for pid in $(pgrep -f "sshd: " 2>/dev/null); do
    env_file="/proc/${pid}/environ"
    [[ -r "$env_file" ]] || continue
    disp=$(tr '\0' '\n' < "$env_file" \
        | awk -F= '/^DISPLAY=/ {print $2; exit}')
    xauth=$(tr '\0' '\n' < "$env_file" \
        | awk -F= '/^XAUTHORITY=/ {print $2; exit}')
    [[ -z "$disp" || -z "$xauth" ]] && continue
    abs_xauth="/proc/${pid}/root${xauth}"
    [[ -r "$abs_xauth" ]] || continue
    chosen_pid="$pid"
    chosen_display="$disp"
    chosen_xauth="$abs_xauth"
    break
done

if [[ -z "$chosen_pid" ]]; then
    _xserver_log "no active sshd X11 session found"
    _xserver_log "ensure you are connected to the NUC with 'ssh -X'"
    return 1 2>/dev/null || exit 1
fi

export CLAUDE_SSH_DISPLAY="$chosen_display"
export CLAUDE_SSH_XAUTH="$chosen_xauth"

_xserver_log "container: bridging SSH X11 from sshd pid=${chosen_pid}"
_xserver_log "  CLAUDE_SSH_DISPLAY=${CLAUDE_SSH_DISPLAY}"
_xserver_log "  CLAUDE_SSH_XAUTH=${CLAUDE_SSH_XAUTH}"
_xserver_log "now run e.g.:"
_xserver_log "  claude_test/show_image.sh outputs/captured_images/realsense_333422300435.png"
