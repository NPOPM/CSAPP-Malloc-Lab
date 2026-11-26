#!/bin/bash
# 复现并归档报告 4.5.3 里的各个实验变体。
#
# 用法：bash docs/experiments/run.sh [变体名...]     不带参数跑全部
#
# 产物：
#   docs/experiments/variants/<变体>.c    该变体的完整 mm.c
#   docs/experiments/results/<变体>.txt   mdriver -V 的原始输出
#
# 所有变体都从 base-mm.c（V3 提交时的快照）派生，因此在 V3 之后的代码上
# 也能随时重跑，不受当前 mm.c 改动的影响。
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="$HERE/base-mm.c"
VDIR="$HERE/variants"
OUT="$HERE/results"
mkdir -p "$VDIR" "$OUT"

[ -f "$BASE" ] || { echo "缺少 base-mm.c"; exit 1; }

run_one () {
  name=$1
  WORK=/tmp/exp-$name
  rm -rf "$WORK"; mkdir -p "$WORK"

  # 用仓库里除 mm.c 之外的文件搭出编译环境，mm.c 由 base + patch 生成
  cp -r "$HERE/../.."/. "$WORK"/
  rm -f "$WORK/mm.c"
  rm -rf "$WORK/.git"
  cp "$BASE" "$WORK/mm.c"
  cd "$WORK" || return 1

  python3 "$HERE/patch.py" "$name" mm.c || { echo "$name: patch 失败"; cd - >/dev/null; return 1; }

  # 归档变体代码（无论编译是否成功都留下，方便对照）
  cp mm.c "$VDIR/$name.c"

  make clean >/dev/null 2>&1
  if make mdriver 2>&1 | grep -qE "\berror\b"; then
    echo "$name: 编译失败"
    cd - >/dev/null || return 1
    return 1
  fi

  {
    echo "===== 变体: $name"
    ./mdriver -V 2>&1 | tail -15
  } | tee "$OUT/$name.txt"

  cd - >/dev/null || true
}

if [ $# -gt 0 ]; then
  for v in "$@"; do run_one "$v"; done
else
  for v in bestfit fine split64 round_pow2 round_sel round_sel2; do run_one "$v"; done
fi
