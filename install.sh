#!/usr/bin/env bash
#
# One-clone bootstrap for the JeTank ROS2 workspace.
#
#   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
#   git clone git@github.com:kvgork/jetank_ros_main.git
#   cd jetank_ros_main
#   ./install.sh
#
# This script lives inside the seed package (jetank_ros_main). It stages the
# workspace-root template files, clones the seven sibling packages via
# vcstool, installs pixi if missing, and resolves the env. See the
# one-clone-install plan for the full design.
#
set -euo pipefail

#---------------------------------------------------------------------
# Flags
#---------------------------------------------------------------------
ASSUME_YES=false
DO_BUILD=false
SKIP_PIXI=false
CLONE_PROTOCOL=ssh

usage() {
  cat <<'USAGE'
Usage: ./install.sh [options]

Bootstraps the JeTank ROS2 workspace from the single jetank_ros_main clone.

Options:
  --yes         Non-interactive; assume "yes" to all prompts (CI / fresh box).
  --build       Run `pixi run build` after the env is provisioned.
  --skip-pixi   Skip pixi install/bootstrap (env provisioned externally).
  --ssh         Clone siblings over SSH (git@github.com:...).   [default]
  --https       Clone siblings over HTTPS (https://github.com/...).
  -h, --help    Show this help and exit.
USAGE
}

confirm() {
  # confirm "Question?" -> returns 0 on yes, 1 on no. Auto-yes with --yes.
  $ASSUME_YES && return 0
  local reply
  read -r -p "$1 [y/N] " reply
  [[ "$reply" =~ ^[Yy]([Ee][Ss])?$ ]]
}

for arg in "$@"; do
  case "$arg" in
    --yes)       ASSUME_YES=true ;;
    --build)     DO_BUILD=true ;;
    --skip-pixi) SKIP_PIXI=true ;;
    --ssh)       CLONE_PROTOCOL=ssh ;;
    --https)     CLONE_PROTOCOL=https ;;
    -h|--help)   usage; exit 0 ;;
    *) echo "ERROR: unknown flag '$arg'" >&2; usage; exit 1 ;;
  esac
done

#---------------------------------------------------------------------
# 1. Locate workspace root
#---------------------------------------------------------------------
SEED_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(basename "$(dirname "$SEED_DIR")")" != "src" ]]; then
  echo "ERROR: expected to live in <ws>/src/jetank_ros_main/ ; found $SEED_DIR" >&2
  echo "       Clone into <ws>/src/ and re-run from there." >&2
  exit 1
fi
WS_ROOT="$(cd "$SEED_DIR/../.." && pwd)"
echo "==> Workspace root: $WS_ROOT"

#---------------------------------------------------------------------
# 2. Stage workspace-root template files
#---------------------------------------------------------------------
echo "==> Staging workspace template files into $WS_ROOT ..."
# -n = never overwrite existing files; preserves any local edits the user made.
cp -an "$SEED_DIR/workspace_template/." "$WS_ROOT/"

#---------------------------------------------------------------------
# 3. Clone sibling sub-packages via vcstool
#---------------------------------------------------------------------
if ! command -v vcs >/dev/null 2>&1; then
  echo "==> vcstool not found; installing with pip (--user) ..."
  python3 -m pip install --user vcstool
  export PATH="$HOME/.local/bin:$PATH"
fi

REPOS_FILE="$SEED_DIR/jetank.repos"
CLEANUP_REPOS=false
if [[ "$CLONE_PROTOCOL" == "https" ]]; then
  REPOS_FILE="$(mktemp)"
  CLEANUP_REPOS=true
  sed 's|git@github.com:|https://github.com/|g' "$SEED_DIR/jetank.repos" > "$REPOS_FILE"
fi

echo "==> Cloning sibling packages into $WS_ROOT/src ($CLONE_PROTOCOL) ..."
# vcs import is idempotent: re-running skips repos that already exist.
vcs import --recursive "$WS_ROOT/src" < "$REPOS_FILE"

$CLEANUP_REPOS && rm -f "$REPOS_FILE"

#---------------------------------------------------------------------
# 4. Install pixi if missing
#---------------------------------------------------------------------
if ! $SKIP_PIXI; then
  if ! command -v pixi >/dev/null 2>&1; then
    # pixi may already be installed but not yet on PATH.
    if [[ -x "$HOME/.pixi/bin/pixi" ]]; then
      export PATH="$HOME/.pixi/bin:$PATH"
    elif confirm "pixi is not installed. Install it to ~/.pixi/bin?"; then
      PIXI_NO_PATH_UPDATE=1 curl -fsSL https://pixi.sh/install.sh | bash
      export PATH="$HOME/.pixi/bin:$PATH"
    else
      echo "ERROR: pixi not installed and consent declined; aborting." >&2
      exit 1
    fi
  fi

  echo "==> Running pixi install (this materialises the env) ..."
  (cd "$WS_ROOT" && pixi install)
fi

#---------------------------------------------------------------------
# 5. Optionally build
#---------------------------------------------------------------------
if $DO_BUILD; then
  echo "==> Running pixi run build ..."
  (cd "$WS_ROOT" && pixi run build)
fi

#---------------------------------------------------------------------
# Done
#---------------------------------------------------------------------
cat <<EOF

==> Done. Next steps:
      cd $WS_ROOT
      pixi run build        # build all packages
      pixi run gazebo       # boot the simulation
      pixi shell            # drop into the env

EOF
