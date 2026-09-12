#!/usr/bin/env bash
# tools/export_lib.sh - 単一libだけを公開用repoレイアウトで書出す
# submodule/subtree不使用。git archiveで対象libのみ抽出するため他libは混入しない。
#
# usage:
#   tools/export_lib.sh <LibName> <dest-dir> [--version X.Y.Z] [--push <remote-url> [--branch NAME]]
#
# example:
#   tools/export_lib.sh Encoder /tmp/pub/Encoder --version 0.1.0
#   tools/export_lib.sh Encoder /tmp/pub/Encoder --push https://github.com/<org>/Encoder.git
set -euo pipefail

LIB="${1:?usage: export_lib.sh <LibName> <dest-dir> [--version V] [--push URL [--branch B]]}"
DEST="${2:?usage: export_lib.sh <LibName> <dest-dir> [--version V] [--push URL [--branch B]]}"
shift 2

VERSION="0.1.0"
PUSH_URL=""
BRANCH="main"
while [ $# -gt 0 ]; do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --push) PUSH_URL="$2"; shift 2 ;;
        --branch) BRANCH="$2"; shift 2 ;;
        *) echo "unknown option: $1" >&2; exit 1 ;;
    esac
done

REPO_ROOT="$(git rev-parse --show-toplevel)"
SRC="lib/${LIB}"
if [ ! -d "${REPO_ROOT}/${SRC}" ]; then
    echo "error: no such lib: ${SRC}" >&2
    exit 1
fi

LOWER="$(echo "$LIB" | tr '[:upper:]' '[:lower:]')"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

# 対象libのみ抽出（他libは混入しない）
git -C "$REPO_ROOT" archive HEAD "$SRC" | tar -x -C "$STAGE"

mkdir -p "${DEST}/src"
cp "$STAGE/$SRC"/* "${DEST}/src"/

# 公開用マニフェスト生成
cat > "${DEST}/library.json" <<EOF
{
  "name": "${LOWER}",
  "version": "${VERSION}",
  "description": "mirs ESP32 common library: ${LIB} (TODO: describe)",
  "keywords": "mirs, esp32",
  "authors": { "name": "mirs" },
  "frameworks": "arduino",
  "platforms": "espressif32"
}
EOF

cat > "${DEST}/library.properties" <<EOF
name=${LIB}
version=${VERSION}
author=mirs
sentence=TODO: describe
frameworks=arduino
architectures=esp32
EOF

# 兄弟libへの依存を検出して通知（公開repo側のlib_deps設定用）
DEPS=""
for inc in $(grep -ho '#include "[^"]*"' "${DEST}/src"/* 2>/dev/null | sed 's/#include "//;s/"//' | sort -u); do
    base="$(basename "$inc")"
    for d in "${REPO_ROOT}"/lib/*/; do
        dep="$(basename "$d")"
        if [ "$dep" != "$LIB" ] && [ -f "${d}${base}" ]; then
            DEPS="${DEPS} ${dep}"
        fi
    done
done
if [ -n "$DEPS" ]; then
    echo "NOTE: ${LIB} requires sibling lib(s):${DEPS}"
    echo "      add them to the published library.json dependencies or export together."
fi

if [ -n "$PUSH_URL" ]; then
    if [ ! -d "${DEST}/.git" ]; then
        git -C "$DEST" init -b "$BRANCH" -q
    fi
    git -C "$DEST" add -A
    git -C "$DEST" -c user.name="mirs" -c user.email="mirs@localhost" \
        commit -qm "Export ${LIB} v${VERSION} from mirs_esp_pio"
    git -C "$DEST" push "$PUSH_URL" "HEAD:refs/heads/${BRANCH}"
    echo "pushed ${LIB} v${VERSION} to ${PUSH_URL} (${BRANCH})"
else
    echo "exported ${LIB} v${VERSION} to ${DEST} (src/ + manifests)"
fi
