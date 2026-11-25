/*
 * mm.c - 隐式空闲链表实现的动态内存分配器（V1）
 *
 * 块格式（在本实现中，字长 4 字节，双字 8 字节）：
 *
 *   已分配块                                空闲块
 *   +--------+------------------+--------+  +--------+------------------+--------+
 *   | header | payload          | footer |  | header | (空闲，未使用)    | footer |
 *   +--------+------------------+--------+  +--------+------------------+--------+
 *    4 字节   8n 字节（n >= 1）   4 字节      4 字节                       4 字节
 *
 *   header/footer 均为 4 字节的字，低 3 位是标志位，其余位存放整块大小：
 *     bit 0 = ALLOC，当前块是否已分配（footer 中同步存放）
 *     块大小恒为 8 的倍数，因此低 3 位天然为 0，正好拿来放标志
 *
 *   payload 起始地址按 8 字节对齐，块大小取 8 的倍数，最小块 16 字节
 *   （header 4 + payload 8 + footer 4）。
 *
 *   每个块都保留 footer：
 *   对于 8 字节对齐的块，省掉 footer 省下的 4 字节会被 ALIGN 补齐吃掉，换来的是 PREV_BLKP
 *   可以在 O(1) 时间内由前一块的 footer 反推其大小，从而使反向合并无需扫堆。
 *
 * 堆布局：
 *
 *   base+0    +4        +8        +12
 *   +---------+---------+---------+---------+
 *   | padding | pro.H   | pro.F   | epilogue|
 *   +---------+---------+---------+---------+
 *                        ↑
 *                    heap_listp（遍历起点）
 *
 *   padding  : 修正对齐，使每个块的 header 落在 ≡4 (mod 8) 处，payload 才是 8 对齐
 *   prologue : 零 payload 的已分配假块，使第一个块也有合法的“前一块”可读，
 *              从而 PREV_BLKP 在首块上不越界、合并链条在堆头自动终止
 *   epilogue : size == 0 的已分配假块，是正向遍历的终止条件，
 *              同时挡住向后合并越过堆尾
 *
 * 空闲链表组织：
 *   采用隐式空闲链表
 *   通过 header 中的 size 跳到下一块、通过 footer 回到上一块。
 *   find_fit 从 heap_listp 起步，按首次适配（first fit）返回第一个足够大的
 *   空闲块；命中后若剩余部分仍能构成一个最小块（>= MIN_BS），则将它切出
 *   作为新的空闲块，否则整块分配出去。
 *
 * 合并策略：
 *   立即合并。mm_free 释放块后，根据前后邻居的 ALLOC 位分四种情形处理：
 *   不合并、与后块合并、与前块合并、与两侧同时合并。extend_heap 在堆尾
 *   造出新块后同样调用合并逻辑，以吃掉堆尾可能残留的空闲块。
 *
 * 堆扩展：
 *   首次适配失败时按 max(请求大小, CHUNKSIZE) 扩展堆。mem_sbrk 返回的地址
 *   正好落在旧 epilogue 上，因此第一块与后续每块的扩展走同一条代码路径。
 *
 * realloc 策略（尽量避免搬运数据）：
 *   新请求更小或相等  -> 原地缩容，切出的尾巴并入空闲链表并尝试合并
 *   后一块空闲且够用  -> 原地吞并后块扩展
 *   当前块位于堆尾    -> 直接扩展堆尾并并入
 *   以上都不满足      -> 分配新块、memcpy、释放旧块
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

team_t team = { /* Team name */
	"NPOPM",
	/* First member's full name */
	"NPOPM",
	/* First member's email address */
	"npopmrs@gmail.com",
	/* Second member's full name (leave blank if none) */
	"",
	/* Second member's email address (leave blank if none) */
	""
};

// 8 字节对齐：payload、block 8 字节对齐，block大小取 8 的倍数
#define ALIGNMENT 8
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~((size_t)ALIGNMENT - 1))

// ===宏定义===
// 基本常量
#define WSIZE 4   // 字大小，也即头尾大小
#define DSIZE 8   // 双字大小，也即对齐单位
#define MIN_BS 16 // 最小块大小
// 打包与解包
#define PACK(size, alloc) ((unsigned int)((size) | (alloc))) // 打包得到头
#define GET(ptr) (*(unsigned int*)(ptr))                     // 读 4 字节字
#define PUT(ptr, val) (*(unsigned int*)(ptr) = (val))        // 写 4 字节字
#define GET_SIZE(ptr) (GET(ptr) & ~((size_t)DSIZE - 1))      // 从头部取大小
#define GET_ALLOC(ptr) (GET(ptr) & 0x1)                      // 从头部取分配标志
// 定位头尾
#define HP(block_ptr) ((char*)(block_ptr) - WSIZE)                           // 定位头部
#define FP(block_ptr) ((char*)(block_ptr) + GET_SIZE(HP(block_ptr)) - DSIZE) // 定位尾部
// 定位相邻块
#define NEXT_BLKP(block_ptr) ((char*)(block_ptr) + GET_SIZE(HP(block_ptr)))              // 后一个块
#define PREV_BLKP(block_ptr) ((char*)(block_ptr) - GET_SIZE((char*)(block_ptr) - DSIZE)) // 前一个块
// 块分配
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define BLKSIZE(size) MAX(ALIGN(size) + DSIZE, MIN_BS) // 计算请求的整块大小
// 堆扩展
#define CHUNKSIZE (1 << 12) // 按 4KB 大小来扩展堆空间，防止对 mem_sbrk 过多的调用

// 全局堆指针
static char* heap_listp;

/*
 * @brief 初始化堆内存。
 * 堆内存的初始结构：
 * 地址: base+0   +4     +8     +12
 *      ┌───────┬──────┬──────┬──────┐
 *      │padding│pro.H │pro.F │ epi  │
 *      │   0   │ 8|1  │ 8|1  │ 0|1  │
 *      └───────┴──────┴──────┴──────┘
 *                      ↑
 *                heap_listp (base + 8)
 * padding：修正对齐，使后续块的 payload 8 对齐。
 * prologue：相当于隐式空闲链表的“头结点”。
 * 	   heap_listp 指向 prologue 的 footer，这样可以使得遍历从跳转到第一个块开始。
 * epilogue：标志着隐式空闲链表的结尾。
 * 	   size == 0 是遍历的终止条件，alloc == 1 保证它在合并时被看作是已分配的。
 *
 * @return 0	成功初始化。
 * @return -1	初始化时发生错误。
 */
int mm_init(void) {
	// 分配初始堆内存，分配出错时 mem_sbrk 返回 -1
	if ((heap_listp = mem_sbrk(4 * WSIZE)) == (void*)-1) {
		return -1;
	}

	// 构造初始的 16 字节哨兵骨架
	PUT(heap_listp, 0);                          // 对齐填充(padding)
	PUT(heap_listp + WSIZE, PACK(DSIZE, 1));     // prologue header
	PUT(heap_listp + 2 * WSIZE, PACK(DSIZE, 1)); // prologue footer
	PUT(heap_listp + 3 * WSIZE, PACK(0, 1));     // epilogue header

	// 遍历起点指向 base + 8
	heap_listp += 2 * WSIZE;

	return 0;
}

/*
 * @brief 将 block_ptr 指向的空闲块与前后的空闲块合并。
 * @param block_ptr	指向空闲块 payload 的指针。
 * @return void* 合并后的块的 payload 指针。
 */
static void* coalesce(void* block_ptr) {
	size_t prev_alloc = GET_ALLOC(HP(PREV_BLKP(block_ptr))); // 前一个块是否已分配
	size_t next_alloc = GET_ALLOC(HP(NEXT_BLKP(block_ptr))); // 后一个块是否已分配
	size_t block_size = GET_SIZE(HP(block_ptr));             // 当前块大小

	// 1.前后的块都占用，不用合并
	if (prev_alloc && next_alloc) {
		return block_ptr;
	}
	// 2.与后一块合并
	else if (prev_alloc && !next_alloc) {
		// 计算合并后新块的大小
		block_size += GET_SIZE(HP(NEXT_BLKP(block_ptr)));
		// 修改新块的头尾
		PUT(HP(block_ptr), PACK(block_size, 0));
		PUT(FP(block_ptr), PACK(block_size, 0));
	}
	// 3.与前一块合并
	else if (!prev_alloc && next_alloc) {
		block_size += GET_SIZE(HP(PREV_BLKP(block_ptr)));
		PUT(HP(PREV_BLKP(block_ptr)), PACK(block_size, 0));
		PUT(FP(block_ptr), PACK(block_size, 0));
		block_ptr = PREV_BLKP(block_ptr);
	}
	// 4.与两侧合并
	else {
		block_size += GET_SIZE(HP(PREV_BLKP(block_ptr))) + GET_SIZE(HP(NEXT_BLKP(block_ptr)));
		PUT(HP(PREV_BLKP(block_ptr)), PACK(block_size, 0));
		PUT(FP(NEXT_BLKP(block_ptr)), PACK(block_size, 0));
		block_ptr = PREV_BLKP(block_ptr);
	}

	return block_ptr;
}

/*
 * @brief 将堆扩展 size 字节。
 * 这个过程需要更新 epilogue 和合并空闲块。
 *
 * @param size 要扩展的字节数，由调用者保证 size 使 DSIZE 的倍数。
 * @return void* 新空闲块的 payload 指针。返回 NULL 表示发生错误。
 */
static void* extend_heap(size_t size) {
	// 新分配出的堆内存的起始地址，逻辑上就是 epilogue 的 payload 的起始地址
	char* block_ptr;
	if ((block_ptr = mem_sbrk(size)) == (void*)-1) {
		return NULL;
	}

	// 将分配出的新内存组织为一个空闲块
	PUT(HP(block_ptr), PACK(size, 0));
	PUT(FP(block_ptr), PACK(size, 0));
	PUT(HP(NEXT_BLKP(block_ptr)), PACK(0, 1)); // 新的 epilogue 头部

	// 合并新块与原来尾部的空闲块，并返回合并后的块指针
	return coalesce(block_ptr);
}

/*
 * @brief 为指定的 size 适配空闲块。
 * 选择的策略为首次适配，即从堆头开始匹配到的第一个足够大的空闲块。
 *
 * @param size 需要的整块的大小。
 * 调用者传入的 size 应该保证已计入块的头尾大小，且是 DSIZE 的倍数。
 *
 * @return void* 匹配到的空闲块的 payload 指针。返回 NULL 表示发生错误。
 */
static void* find_fit(size_t size) {
	// 遍历起点为 heap_listp；终止条件为 size == 0
	void* block_ptr;
	for (block_ptr = heap_listp; GET_SIZE(HP(block_ptr)) != 0; block_ptr = NEXT_BLKP(block_ptr)) {
		if (!GET_ALLOC(HP(block_ptr)) && GET_SIZE(HP(block_ptr)) >= size) {
			return block_ptr;
		}
	}
	return NULL;
}

/*
 * @brief 把空闲块 block_ptr 按照需求的大小 size 切分。
 * 当切分的剩余部分大小不足一个最小块的大小时，没有必要进行切分，此时的切分会严重拖慢首次适配的时间。
 * *此外，在 64 位系统上，8 字节的空间不能构成一个合法的块，因为头尾各是 8 字节。
 *
 * @param block_ptr 空闲块的 payload 指针。
 * @param size 需要的整块的大小。
 */
static void place(void* block_ptr, size_t size) {
	size_t total_size = GET_SIZE(HP(block_ptr)); // 空闲块的总大小
	// 仅当剩余部分的大小大于最小块大小时，进行切分
	if ((total_size - size) >= MIN_BS) {
		// 构造当前分配的块
		PUT(HP(block_ptr), PACK(size, 1));
		PUT(FP(block_ptr), PACK(size, 1));
		// 构造切出来的空闲块
		block_ptr = NEXT_BLKP(block_ptr);
		PUT(HP(block_ptr), PACK(total_size - size, 0));
		PUT(FP(block_ptr), PACK(total_size - size, 0));
	}
	// 剩余空间不足以放下最小块，不进行拆分
	else {
		PUT(HP(block_ptr), PACK(total_size, 1));
		PUT(FP(block_ptr), PACK(total_size, 1));
	}
}

/*
 * @brief 根据请求的字节数分配块。
 * @param size 请求的字节数。
 * @return void* 分配出的块的 payload 指针。
 * size == 0 时固定返回 NULL，size != 0 时 NULL 表示出错。
 */
void* mm_malloc(size_t size) {
	size_t alloc_size;
	void*  block_ptr;

	// 根据请求的字节数计算要分配出的块大小
	if (size == 0) {
		return NULL;
	}
	alloc_size = BLKSIZE(size);

	// 适配空闲块
	if ((block_ptr = find_fit(alloc_size)) != NULL) {
		place(block_ptr, alloc_size);
		return block_ptr;
	}

	// 如果没有现成的空闲块可用，就进行堆扩展
	if ((block_ptr = extend_heap(MAX(alloc_size, CHUNKSIZE))) == NULL) {
		return NULL;
	}
	place(block_ptr, alloc_size);
	return block_ptr;
}

/*
 * @brief 释放 ptr 指向的已分配块，并执行立即合并。
 * @param ptr 要释放的块的 payload 指针，调用者保证 ptr 指向一个块的 payload。
 */
void mm_free(void* ptr) {
	size_t size;

	if (ptr == NULL) {
		return;
	}

	// 将块的 alloc 置为 0
	size = GET_SIZE(HP(ptr));
	PUT(HP(ptr), PACK(size, 0));
	PUT(FP(ptr), PACK(size, 0));

	// 立即合并
	coalesce(ptr);
}

/*
 * @brief 重新分配指定块的大小。
 * @param ptr 待分配块的 payload 指针，ptr == NULL 时退化为 mm_malloc(size)。
 * @param size 要重分配的大小，size == 0 时退化为 mm_free(ptr)。
 * @return void* 分配出的块的 payload 指针。
 * size == 0 时固定返回 NULL，size != 0 时 NULL 表示出错。
 */
void* mm_realloc(void* ptr, size_t size) {
	void*  new_ptr;
	size_t alloc_size, old_size /* 旧块的 payload 大小 */, next_size, merged;

	// 当 ptr == NULL 时，mm_realloc 的行为与 mm_malloc 一致
	if (ptr == NULL) {
		return mm_malloc(size);
	}
	// 当 size == 0 时，mm_realloc 的行为与 mm_free 一致
	if (size == 0) {
		mm_free(ptr);
		return NULL;
	}

	alloc_size = BLKSIZE(size);
	old_size = GET_SIZE(HP(ptr));

	// 1.如果新块比旧块小（或相等），原地缩容并执行切分
	if (alloc_size <= old_size) {
		// 仅当剩余空间大小能装下一个最小块时执行切分
		if (old_size - alloc_size >= MIN_BS) {
			// 按照 alloc_size 缩容
			PUT(HP(ptr), PACK(alloc_size, 1));
			PUT(FP(ptr), PACK(alloc_size, 1));

			// 构造切分出的空闲块
			new_ptr = NEXT_BLKP(ptr);
			PUT(HP(new_ptr), PACK(old_size - alloc_size, 0));
			PUT(FP(new_ptr), PACK(old_size - alloc_size, 0));

			// 与后方潜在的空闲块合并
			coalesce(new_ptr);
		}
		return ptr;
	}

	// 2.后面的块空闲且合并后空间足够，原地扩展
	next_size = GET_SIZE(HP(NEXT_BLKP(ptr)));
	if (!GET_ALLOC(HP(NEXT_BLKP(ptr))) && old_size + next_size >= alloc_size) {
		merged = old_size + next_size;

		if (merged - alloc_size >= MIN_BS) {
			PUT(HP(ptr), PACK(alloc_size, 1));
			PUT(FP(ptr), PACK(alloc_size, 1));

			new_ptr = NEXT_BLKP(ptr);
			PUT(HP(new_ptr), PACK(merged - alloc_size, 0));
			PUT(FP(new_ptr), PACK(merged - alloc_size, 0));
		} else {
			PUT(HP(ptr), PACK(merged, 1));
			PUT(FP(ptr), PACK(merged, 1));
		}
		return ptr;
	}

	// 3.后面是 epilogue，即 ptr 是堆尾块，直接扩展堆
	if (GET_SIZE(HP(NEXT_BLKP(ptr))) == 0) {
		if (extend_heap(alloc_size - old_size) == NULL) {
			return NULL;
		}
		PUT(HP(ptr), PACK(alloc_size, 1));
		PUT(FP(ptr), PACK(alloc_size, 1));
		return ptr;
	}

	// 4.块在堆中间且执行可能的合并后仍然不够，分配新内存并将原来的 payload 复制过来
	new_ptr = mm_malloc(size);
	if (new_ptr == NULL) {
		return NULL;
	}
	memcpy(new_ptr, ptr, old_size - DSIZE);
	mm_free(ptr);

	return new_ptr;
}
