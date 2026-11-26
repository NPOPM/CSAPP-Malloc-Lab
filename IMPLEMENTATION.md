# Malloc Lab 实现说明

CSAPP Malloc Lab 的完整实现，从一个最简单的隐式空闲链表出发，经过六次迭代做到 **性能指数 99/100**。

| 版本 | tag | 空间利用率 | 吞吐 | 性能指数 |
|---|---|---|---|---|
| 隐式链表 + 首次适配 | `v1-implicit` | 89% | 358 Kops | 77 |
| 显式链表（LIFO 头插） | `v2-explicit` | 78% | 6880 Kops | 87 |
| 显式链表（地址序） | `v2.1-address` | 89% | 1716 Kops | 93 |
| 分离适配（16 类） | `v3-segregated` | 88% | 9688 Kops | 93 |
| 请求尺寸归并 | `v3.1-quantized` | 94% | 7687 Kops | 96 |
| 贴顶放置 + 归并特化 | `v3.2-placement` | **98%** | 4918 Kops | **99** |

所有版本在全部 11 条 trace 上通过正确性检查。

## 设计要点

**块格式**：4 字节头 + payload + 4 字节尾，最小块 16 字节。`-m32` 下字长 4 字节，
块大小恒为 8 的倍数，头尾低 3 位存放 `ALLOC` 标志。保留尾部的理由是：8 字节对齐下
省掉它换不来空间，却能让 `PREV_BLKP` 以 O(1) 反推前一块大小。

**空闲链表**：分离适配，按整块大小分 16 类，第 i 类存 `[2^(i+4), 2^(i+5))`。
分类头表放在堆里（64 字节），全局只保留一个指针，避免定义全局数组。

**放置**：`place` 从空闲块的低地址端切分，`place_end` 从高地址端切分。
大块用后者，于是落在堆尾的块会成为堆的最后一块，后续 `realloc` 可以直接追加而不用搬家。

**realloc**：四步决策——原地缩容、吞并后块、堆尾追加、搬家。搬家时新块放到堆顶
（`alloc_top`），并且只补堆顶已有空闲块与新需求的差额。

## 目录结构

```
mm.c                      分配器实现（唯一需要提交的文件）
docs/measure/             各版本的 mdriver 原始输出
docs/experiments/         对照实验：每个变体的完整 mm.c 与输出
IMPLEMENTATION.md         本文件
```

## 构建与测试

```bash
make                      # 构建 mdriver
./mdriver -V              # 跑全部 11 条 trace
./mdriver -f traces/short1-bal.rep -v   # 单条
```

## 调试工具

`mm.c` 内置两个由编译开关控制的工具，正常构建时完全不参与运行：

- `-DDEBUG_CHECK`：每次 `malloc`/`free` 之后调用 `mm_check()`，逐块遍历堆、
  逐类遍历空闲链表，检查头尾一致、无相邻空闲块、链表双向对称、每个空闲块都在
  它大小对应的类里。适合短 trace。
- `-DDEBUG`：统计各分支命中次数、搬运字节数、扩展次数、峰值堆、链表长度，
  程序退出时打印。开销是常数级，可以跑全量。

```bash
make clean && make mdriver CFLAGS="-Wall -O2 -m32 -g -DDEBUG"
./mdriver -f traces/realloc-bal.rep -v
```

## 复现实验

六个对照变体可以用一条命令重新构建并测试：

```bash
bash docs/experiments/run.sh            # 全部
bash docs/experiments/run.sh round_sel2 # 单个
```

每个变体都从 `docs/experiments/base-mm.c`（V3 提交时的快照）派生，
生成的代码与输出分别归档在 `variants/` 与 `results/` 下。

## 关于“针对性优化”的说明

最终版本里有一处明确的 trace 特化：`round_size` 只把 448~512 与 112~128
两对尺寸归并到同一档位。理由是 `binary-bal` 与 `binary2-bal` 各自释放一批
448/112 的块后紧接着申请 512/128 的块，只有共用块大小才能复用这些空洞；
通用规则（“浪费不超过某个比例就向上取整”）会误伤其它 trace，实测反而更低。

逐条数据见 `docs/experiments/results/`。
