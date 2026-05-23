#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Developer convenience: source this from your shell to get the same
# overlay environment that scripts/start.sh sets up, so you can run
# ros2 commands (`ros2 topic`, `ros2 service`, `ros2 launch`, etc.)
# against this package's vendored ROS install without rbnx boot.
#
# Usage (from a fresh shell):
#   source <pkg_root>/scripts/dev_source.sh
#
# Mirrors the injection logic in start.sh; see that file's header for
# the rationale (idempotent-marker workaround for nested colcon
# overlays). DO NOT exec this script — it must be sourced so the
# environment vars stick in your shell.
#
# Re-entry guard: if you accidentally `source` this from a script
# that itself was sourced from us, just no-op the second time.
if [[ "${_PIPER_MOVEIT_DEV_SOURCING_IN_PROGRESS:-0}" == "1" ]]; then
    return 0 2>/dev/null || exit 0
fi
export _PIPER_MOVEIT_DEV_SOURCING_IN_PROGRESS=1

# Resolve the package root. When sourced, $0 is the host shell, not
# this file — fall back to ${BASH_SOURCE[0]}.
_PIPER_MOVEIT_DEV_THIS="${BASH_SOURCE[0]:-$0}"
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$_PIPER_MOVEIT_DEV_THIS")/.." && pwd)}"
unset _PIPER_MOVEIT_DEV_THIS

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

OVERLAY_INSTALL="$PKG/rbnx-build/ws/install"
if [[ -f "$OVERLAY_INSTALL/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$OVERLAY_INSTALL/setup.bash"
else
    echo "[piper_moveit/dev_source] WARN: overlay missing — run scripts/build.sh first" >&2
fi

_prepend_unique() {
    local var="$1" val="$2"
    local cur="${!var:-}"
    case ":${cur}:" in
        *":${val}:"*) ;;
        *) export "$var"="${val}${cur:+:${cur}}" ;;
    esac
}

if [[ -d "$OVERLAY_INSTALL" ]]; then
    for _prefix in "$OVERLAY_INSTALL"/*/; do
        _prefix="${_prefix%/}"
        [[ -d "$_prefix/share" ]] || continue
        _prepend_unique AMENT_PREFIX_PATH "$_prefix"
        _prepend_unique CMAKE_PREFIX_PATH "$_prefix"
        for _site in \
            "$_prefix"/local/lib/python*/dist-packages \
            "$_prefix"/lib/python*/site-packages \
            "$_prefix"/lib/python*/dist-packages
        do
            [[ -d "$_site" ]] && _prepend_unique PYTHONPATH "$_site"
        done
        for _libdir in \
            "$_prefix"/lib \
            "$_prefix"/local/lib
        do
            [[ -d "$_libdir" ]] && _prepend_unique LD_LIBRARY_PATH "$_libdir"
        done
    done
    unset _prefix _site _libdir
fi

CODEGEN_PROTO="$PKG/rbnx-build/codegen/proto_gen"
CODEGEN_MCP="$PKG/rbnx-build/codegen/robonix_mcp_types"
[[ -d "$CODEGEN_PROTO" ]] && _prepend_unique PYTHONPATH "$CODEGEN_PROTO"
[[ -d "$CODEGEN_MCP"   ]] && _prepend_unique PYTHONPATH "$CODEGEN_MCP"
_prepend_unique PYTHONPATH "$PKG"

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    _prepend_unique PYTHONPATH "$ROBONIX_API"
fi

# External rbnx-package dependency: piper_description.
# Same resolution chain as scripts/start.sh's _find_piper_description_prefix.
_pdesc=""
if [[ -n "${PIPER_DESCRIPTION_RBNX_PREFIX:-}" ]]; then
    _pdesc="$PIPER_DESCRIPTION_RBNX_PREFIX"
elif [[ -d "$PKG/../piper_description/rbnx-build/ws/install/piper_description/share" ]]; then
    _pdesc="$( cd "$PKG/../piper_description/rbnx-build/ws/install/piper_description" && pwd )"
elif [[ -d "$PKG/../piper_description_rbnx/rbnx-build/ws/install/piper_description/share" ]]; then
    _pdesc="$( cd "$PKG/../piper_description_rbnx/rbnx-build/ws/install/piper_description" && pwd )"
fi
if [[ -n "$_pdesc" && -d "$_pdesc/share" ]]; then
    _prepend_unique AMENT_PREFIX_PATH "$_pdesc"
    _prepend_unique CMAKE_PREFIX_PATH "$_pdesc"
    for _site in \
        "$_pdesc"/local/lib/python*/dist-packages \
        "$_pdesc"/lib/python*/site-packages \
        "$_pdesc"/lib/python*/dist-packages
    do
        [[ -d "$_site" ]] && _prepend_unique PYTHONPATH "$_site"
    done
    for _libdir in "$_pdesc"/lib "$_pdesc"/local/lib; do
        [[ -d "$_libdir" ]] && _prepend_unique LD_LIBRARY_PATH "$_libdir"
    done
    unset _site _libdir
fi
unset _pdesc

unset OVERLAY_INSTALL PKG _PIPER_MOVEIT_DEV_SOURCING_IN_PROGRESS
