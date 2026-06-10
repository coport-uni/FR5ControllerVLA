#!/usr/bin/env bash
#
# setup-claude-code.sh
# Automated script to install and log in to Claude Code on Ubuntu.
#
# Usage:
#   chmod +x setup-claude-code.sh
#   ./setup-claude-code.sh
#
# Requirements:
#   A Claude Pro, Max, Teams, Enterprise, or Console (API) account.
#   The free Claude.ai plan cannot use Claude Code.

set -euo pipefail

echo "=================================================="
echo " Install and log in to Claude Code"
echo "=================================================="

# 1. Check required tools
if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required. Installing it."
  sudo apt update
  sudo apt install -y curl ca-certificates
fi

# 2. Install Claude Code
# Use Anthropic's official native install script.
# This is a single-command install that runs without Node.js.
if command -v claude >/dev/null 2>&1; then
  echo "Claude Code is already installed. Checking the version."
  claude --version
else
  echo "Installing Claude Code."
  curl -fsSL https://claude.ai/install.sh | bash
fi

# 3. Add to PATH
# The native install usually places the claude binary in ~/.local/bin.
# The install script does not update the current shell's PATH, so add it directly.
# Opening a new shell with docker exec inside the container resets PATH, so we
# export it in the current shell and persist it to both ~/.bashrc (interactive
# shells) and ~/.profile (login shells) so claude is found in later sessions too.
CLAUDE_BIN_DIR="$HOME/.local/bin"
echo "Adding $CLAUDE_BIN_DIR to PATH."
export PATH="$CLAUDE_BIN_DIR:$PATH"
for rc in "$HOME/.bashrc" "$HOME/.profile"; do
  if ! grep -q '.local/bin' "$rc" 2>/dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$rc"
  fi
done

echo "Installed version:"
claude --version

# 4. Log in
# On first run you choose a login method and a one-time OAuth flow begins.
# When the browser window opens, sign in with your Anthropic account and click Authorize.
#
# On a headless server or SSH session where the browser does not open automatically,
# copy the URL shown in the terminal into your browser's address bar to get an auth code,
# then paste that code back into the terminal and press Enter to authenticate.
echo "=================================================="
echo " Starting login. Follow the prompts to authenticate."
echo "=================================================="
claude

echo "Setup is complete. From now on, run with the 'claude' command."
