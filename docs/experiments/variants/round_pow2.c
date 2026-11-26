/*
 * mm.c - 分离适配实现的动态内存分配器（V3）
 *
 * 块格式（-m32 下字长 4 字节，双字 8 字节）：
 *
 *   已分配块                                空闲块
 *   +--------+------------------+--------+  +--------+--------------------+--------+
 *   | header | payload          | footer |  | header | prev | next | ...  | footer |
 *   +--------+------------------+--------+  +--------+--------------------+--------+
 *    4 字节   8n 字节（n >= 1）   4 字节      4 字节   4 字节  4 字节        4 字节
 *
 *   header/footer 均为 4 字节的字，低 3 位是标志位，其余位存放整块大小：
 *     bit 0 = ALLOC，当前块是否已分配（footer 中同步存放）
 *     块大小恒为 8 的倍数，因此低 3 位天然为 0，正好拿来放标志
 *
 *   payload 起始地址按 8 字节对齐，块大小取 8 的倍数。
 *   最小块 16 字节 = header 4 + 空闲块的两个链表指针 8 + footer 4。
 *
 *   每个块都保留 footer：
 *   对于 8 字节对齐的块，省掉 footer 省下的 4 字节会被 ALIGN 补齐吃掉，换来的是 PREV_BLKP
 *   可以在 O(1) 时间内由前一块的 footer 反推其大小，从而使反向合并无需扫堆。
 *
 * 堆布局：分类头表在最前，随后是哨兵骨架
 *
 *   base+0            +64    +68     +72     +76
 *   +-----------------+------+-------+-------+---------+
 *   | 分类头表 64 字节 |pad   | pro.H | pro.F |epilogue |
 *   +-----------------+------+-------+-------+---------+
 *                                     ↑
 *                               heap_listp（遍历起点）
 *
 *   padding  : 修正对齐，使每个块的 header 落在 ≡4 (mod 8) 处，payload 才是 8 对齐
 *   prologue : 零 payload 的已分配假块，使第一个块也有合法的“前一块”可读，
 *              从而 PREV_BLKP 在首块上不越界、合并链条在堆头自动终止
 *   epilogue : size == 0 的已分配假块，是正向遍历的终止条件，
 *              同时挡住向后合并越过堆尾
 *
 * 空闲链表组织：
 *   采用分离适配：按整块大小把所有空闲块分成 SEG_COUNT 个类，第 i 类存放
 *   [2^(i+4), 2^(i+5)) 字节的块，最后一类收集所有更大的块。每个类各是一条双向链表，
 *   块之间通过 payload 前 8 字节的 prev/next 串接。
 *
 *   分类头表本身也放在堆里（SEG_COUNT 个指针，共 64 字节），全局只保留 seg_listp 一个标量指针。
 *
 *   find_fit 先算出请求所属的类，再从该类开始逐类向上查找。每张链表都按地址升序排列，
 *   所以每张表里第一个装得下的块就是该表地址最小的块；把各表各自的这个候选放在一起比较，
 *   取地址最小的一个返回。装不下的那些类被整类跳过。
 *
 *   之所以要跨类比较地址，是为了避免遇到与 V2 LIFO 类似的问题，
 *   也就是块落到堆中间，造成 realloc 不能在堆尾就地扩展，只能反复搬运数据。
 *   跨类比较之后，放置顺序与单一链表版本一致，堆仍然从低地址向上生长。
 *
 *   类内按地址升序插入，插入代价是 O(本类块数)；但立即合并保证了每条链表都很短，这个代价可以忽略。
 *
 * 合并策略：
 *   立即合并。coalesce 的约定是“传入的块此刻不在任何链表中”：它把被吞并的前/后
 *   空闲块从各自的类里摘除并返回合并后的块首，入链由调用者负责（mm_free、extend_heap
 *   以及 realloc 的缩容分支）。这样四种合并情形与链表维护只有一份实现。
 *
 * 堆扩展：
 *   所有类都找不到合适的块时，按 max(请求大小, CHUNKSIZE) 扩展堆。mem_sbrk 返回的地址
 *   正好落在旧 epilogue 上，因此第一块与后续每块的扩展走同一条代码路径。
 *   CHUNKSIZE 必须是 DSIZE 的倍数；扩展出的块不小于 MIN_BS，否则放不下两个链表指针。
 *
 * realloc 策略（尽量避免搬运数据）：
 *   新请求更小或相等  -> 原地缩容，切出的尾巴合并后入链
 *   后一块空闲且够用  -> 先从所在类里摘下后块，再原地吞并扩展
 *   当前块位于堆尾    -> 直接扩展堆尾并并入
 *   以上都不满足      -> 分配新块、memcpy、释放旧块
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

/* ================= 调试期统计（-DDEBUG 时编译启用） ================= */
#ifdef DEBUG
static size_t st_malloc_calls, st_free_calls, st_realloc_calls;
static size_t st_r_shrink, st_r_absorb, st_r_tail, st_r_move;
static size_t st_move_bytes;
static size_t st_extend_calls, st_extend_bytes, st_peak_heap;
static size_t st_fl_len, st_fl_max, st_fl_sum, st_fl_samples;
#endif /* DEBUG */

int mm_check(void); /* 堆一致性检查器，定义在文件末尾 */

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
static size_t round_size(size_t size) {
	size_t payload = ALIGN(size);
	size_t bucket;

	if (payload <= 128) {
		return MAX(payload + DSIZE, MIN_BS);
	}
	bucket = 128;
	while (bucket < payload) {
		bucket <<= 1;
	}
	if (1) {
		return bucket + DSIZE;
	}
	return MAX(payload + DSIZE, MIN_BS);
}
// 堆扩展
#define CHUNKSIZE (1 << 12) // 按 4KB 大小来扩展堆空间，防止对 mem_sbrk 过多的调用

// 全局堆指针
static char* heap_listp;

// ===显式空闲链表===
// 空闲链表指针，只对空闲块使用
#define PREV_FREEP(block_ptr) (*(void**)(block_ptr))
#define NEXT_FREEP(block_ptr) (*(void**)((char*)(block_ptr) + WSIZE))

// ===分离适配===
// 按照2的幂分16类
#define SEG_COUNT 16

// 堆内分类头标
static void** seg_listp;

/*
 * @brief 初始化堆内存。
 *
 * 先向 mem_sbrk 申请 SEG_COUNT * WSIZE = 64 字节作为分类头表，并全部置空；
 * 再申请 16 字节构造哨兵骨架。表长是 8 的倍数，所以骨架里的 4 字节 padding
 * 仍然把 prologue 的 header 顶到 ≡4 (mod 8) 的位置，对齐不受影响。
 *
 * 堆内存的初始结构：
 * 地址: base+0            +64   +68     +72     +76
 *      ┌─────────────────┬─────┬──────┬──────┬──────┐
 *      │ 分类头表 64 字节 │pad  │pro.H │pro.F │ epi  │
 *      │                 │  0  │ 8|1  │ 8|1  │ 0|1  │
 *      └─────────────────┴─────┴──────┴──────┴──────┘
 *                                      ↑
 *                                heap_listp (base + 72)
 *
 * padding：修正对齐，使后续块的 payload 8 对齐。
 * prologue：相当于堆的“头结点”，使首块也有合法的“前一块”可读。
 * epilogue：标志着堆的结尾；size == 0 是遍历的终止条件，alloc == 1 挡住越过堆尾的合并。
 *
 * @return 0	成功初始化。
 * @return -1	初始化时发生错误。
 */
int mm_init(void) {
	size_t i;

	// 将分类头表维护在堆的最前面，全局维护一个分类头表指针
	if ((seg_listp = mem_sbrk(SEG_COUNT * WSIZE)) == (void*)-1) {
		return -1;
	}
	for (i = 0; i < SEG_COUNT; i++) {
		seg_listp[i] = NULL;
	}

	// 分类头表表长 64 字节，不影响对齐，因此哨兵骨架的构造照旧即可
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
 * @brief 将整块大小映射到分类头表的索引
 * 分类头表的第 i 类存放 [ 2^(i+4), 2^(i+5) ) 字节的块，最后一个类同时收集所有更大的块
 *
 * @param size 请求的整块大小
 * @return size_t 对应分类头表的索引
 */
static size_t seg_index(size_t size) {
	size_t index = 0;
	size >>= 4; // 先对 size 做一步映射，除以 2^4

	while (size > 1 && index < SEG_COUNT - 1) {
		size >>= 1;
		index++;
	}
	return index;
}

/*
 * @brief 将 block_ptr 指向的空闲块插入空闲链表。
 * 空闲链表做了分离适配，需要先按大小判断空闲块应该插入哪一个表。
 * 显式空闲链表采用地址有序插入，需要从空闲链表的开头进行一次遍历，找到合适的插入位置。
 *
 * @param block_ptr 要插入空闲块的 payload 指针。
 */
static void insert_free(void* block_ptr) {
	size_t index = seg_index(GET_SIZE(HP(block_ptr)));
	void** head = &seg_listp[index];
	void*  cur = *head;
	void*  prev = NULL;

	// 找到第一个地址大于 block_ptr 的结点，把当前的空闲块插在它前面
	while (cur != NULL && (char*)cur < (char*)block_ptr) {
		// 两个指针前进
		prev = cur;
		cur = NEXT_FREEP(cur);
	}

	// 将新空闲块插入指定位置
	PREV_FREEP(block_ptr) = prev;
	NEXT_FREEP(block_ptr) = cur;
	// 检测 cur 是否是表头的块
	if (prev != NULL) { // 如果 cur 不是表头
		NEXT_FREEP(prev) = block_ptr;
	} else { // 如果 cur 是表头
		*head = block_ptr;
	}
	// 检测 cur 是否是表尾的块
	if (cur != NULL) {
		PREV_FREEP(cur) = block_ptr;
	}

#ifdef DEBUG
	st_fl_len++;
	if (st_fl_len > st_fl_max) {
		st_fl_max = st_fl_len;
	}
	st_fl_sum += st_fl_len;
	st_fl_samples++;
#endif /* DEBUG */
}

/*
 * @brief 将 block_ptr 指向的空闲块从空闲链表中删除。
 * 空闲链表做了分离适配，需要先按大小判断要删除的块来自哪一个表。
 *
 * @param block_ptr 要删除的空闲块的 payload 指针。
 */
static void remove_free(void* block_ptr) {
#ifdef DEBUG
	st_fl_len--;
#endif /* DEBUG */
	size_t index = seg_index(GET_SIZE(HP(block_ptr)));
	void** head = &seg_listp[index];

	// 判断 block_ptr 指向的是不是表头
	if (PREV_FREEP(block_ptr) == NULL) {
		*head = NEXT_FREEP(block_ptr);
	} else {
		NEXT_FREEP(PREV_FREEP(block_ptr)) = NEXT_FREEP(block_ptr);
	}

	// 判断 block_ptr 指向的是不是表尾
	if (NEXT_FREEP(block_ptr) != NULL) {
		PREV_FREEP(NEXT_FREEP(block_ptr)) = PREV_FREEP(block_ptr);
	}
}

/*
 * @brief 将 block_ptr 指向的空闲块与前后的空闲块合并。
 * 调用者需要保证 block_ptr 指向的块不在任何一张空闲链表中。
 * coalesce 只负责将被吞并的邻居从所属的类中删除，合并后的块插入由调用者负责。
 *
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

		// 从空闲链表中删除被吞并的后一块
		remove_free(NEXT_BLKP(block_ptr));
	}
	// 3.与前一块合并
	else if (!prev_alloc && next_alloc) {
		block_size += GET_SIZE(HP(PREV_BLKP(block_ptr)));

		// 从空闲链表中删除被吞并的前一块
		remove_free(PREV_BLKP(block_ptr));

		block_ptr = PREV_BLKP(block_ptr);
	}
	// 4.与两侧合并
	else {
		block_size += GET_SIZE(HP(PREV_BLKP(block_ptr))) + GET_SIZE(HP(NEXT_BLKP(block_ptr)));
		remove_free(PREV_BLKP(block_ptr));
		remove_free(NEXT_BLKP(block_ptr));
		block_ptr = PREV_BLKP(block_ptr);
	}

	PUT(HP(block_ptr), PACK(block_size, 0));
	PUT(FP(block_ptr), PACK(block_size, 0));
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

#ifdef DEBUG
	st_extend_calls++;
	st_extend_bytes += size;
	if (mem_heapsize() > st_peak_heap) {
		st_peak_heap = mem_heapsize();
	}
#endif /* DEBUG */

	// 将分配出的新内存组织为一个空闲块
	PUT(HP(block_ptr), PACK(size, 0));
	PUT(FP(block_ptr), PACK(size, 0));
	PUT(HP(NEXT_BLKP(block_ptr)), PACK(0, 1)); // 新的 epilogue 头部

	// 合并新块与原来尾部的空闲块，将合并后的块插入空闲链表，并返回合并后的块指针
	block_ptr = coalesce(block_ptr);
	insert_free(block_ptr);
	return block_ptr;
}

/*
 * @brief 为指定的 size 适配空闲块。
 * 分离适配实现后，要从自己大小对应的链表开始匹配空闲块，
 * 每一张链表都按地址升序排列，因此每张表里第一个装得下的块就是该表地址最小的块；
 * 把所有装得下的表各自的候选放在一起比较，取地址最小的一个返回。
 * 这样放置顺序与单一链表的首次适配一致，realloc 仍能在堆尾就地扩展。
 *
 * @param size 需要的整块的大小。
 * 调用者传入的 size 应该保证已计入块的头尾大小，且是 DSIZE 的倍数。
 *
 * @return void* 匹配到的空闲块的 payload 指针。返回 NULL 表示发生错误。
 */
static void* find_fit(size_t size) {
	size_t index = seg_index(size);
	size_t i;
	void*  best = NULL;

	// 从自己的类开始，逐层向上匹配空闲块
	for (i = index; i < SEG_COUNT; i++) {
		// 遍历起点为 seg_listp[i]；终止条件为 block_ptr == NULL
		void* block_ptr;

		for (block_ptr = seg_listp[i]; block_ptr != NULL; block_ptr = NEXT_FREEP(block_ptr)) {
			if (GET_SIZE(HP(block_ptr)) >= size) {
				if (best == NULL || (char*)block_ptr < (char*)best) {
					best = block_ptr;
				}
				break;
			}
		}
	}

	return best;
}

/*
 * @brief 把空闲块 block_ptr 按照需求的大小 size 切分。
 * 当切分的剩余部分大小不足一个最小块时不做切分：这样的碎块永远分配不出去，只会拖慢日后的查找。
 *
 * @param block_ptr 空闲块的 payload 指针。
 * @param size 需要的整块的大小。
 */
static void place(void* block_ptr, size_t size) {
	size_t total_size = GET_SIZE(HP(block_ptr)); // 空闲块的总大小

	// 将空闲块从空闲链表中删除
	remove_free(block_ptr);

	// 仅当剩余部分的大小大于最小块大小时，进行切分
	if ((total_size - size) >= MIN_BS) {
		// 构造当前分配的块
		PUT(HP(block_ptr), PACK(size, 1));
		PUT(FP(block_ptr), PACK(size, 1));
		// 构造切出来的空闲块
		block_ptr = NEXT_BLKP(block_ptr);
		PUT(HP(block_ptr), PACK(total_size - size, 0));
		PUT(FP(block_ptr), PACK(total_size - size, 0));

		// 将构造出的空闲块插入空闲链表
		insert_free(block_ptr);
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
	alloc_size = round_size(size);

#ifdef DEBUG
	st_malloc_calls++;
#endif /* DEBUG */

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
#ifdef DEBUG_CHECK
	if (!mm_check()) {
		printf("mm_check failed: mm_malloc(%zu) -> %p\n", size, block_ptr);
	}
#endif /* DEBUG_CHECK */
	return block_ptr;
}

/*
 * @brief 释放 ptr 指向的已分配块，并执行立即合并。
 * @param ptr 要释放的块的 payload 指针，调用者保证 ptr 指向一个块的 payload。
 */
void mm_free(void* ptr) {
	size_t size;
	void*  merged;

	if (ptr == NULL) {
		return;
	}

	// 将块的 alloc 置为 0
#ifdef DEBUG
	st_free_calls++;
#endif /* DEBUG */

	size = GET_SIZE(HP(ptr));
	PUT(HP(ptr), PACK(size, 0));
	PUT(FP(ptr), PACK(size, 0));

	// 立即合并
	merged = coalesce(ptr);

	// 将合并后的空闲块插入空闲链表
	insert_free(merged);

#ifdef DEBUG_CHECK
	if (!mm_check()) {
		printf("mm_check failed: mm_free(%p)\n", ptr);
	}
#endif /* DEBUG_CHECK */
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
	size_t alloc_size, old_size, next_size, merged;

	// 当 ptr == NULL 时，mm_realloc 的行为与 mm_malloc 一致
	if (ptr == NULL) {
		return mm_malloc(size);
	}
	// 当 size == 0 时，mm_realloc 的行为与 mm_free 一致
	if (size == 0) {
		mm_free(ptr);
		return NULL;
	}

	alloc_size = round_size(size);
	old_size = GET_SIZE(HP(ptr));

#ifdef DEBUG
	st_realloc_calls++;
#endif /* DEBUG */

	// 1.如果新块比旧块小（或相等），原地缩容并执行切分
	if (alloc_size <= old_size) {
#ifdef DEBUG
		st_r_shrink++;
#endif /* DEBUG */
		// 仅当剩余空间大小能装下一个最小块时执行切分
		if (old_size - alloc_size >= MIN_BS) {
			// 按照 alloc_size 缩容
			PUT(HP(ptr), PACK(alloc_size, 1));
			PUT(FP(ptr), PACK(alloc_size, 1));

			// 构造切分出的空闲块
			new_ptr = NEXT_BLKP(ptr);
			PUT(HP(new_ptr), PACK(old_size - alloc_size, 0));
			PUT(FP(new_ptr), PACK(old_size - alloc_size, 0));

			// 与后方潜在的空闲块合并，并插入空闲链表
			new_ptr = coalesce(new_ptr);
			insert_free(new_ptr);
		}
		return ptr;
	}

	// 2.后面的块空闲且合并后空间足够，原地扩展
	next_size = GET_SIZE(HP(NEXT_BLKP(ptr)));
	if (!GET_ALLOC(HP(NEXT_BLKP(ptr))) && old_size + next_size >= alloc_size) {
#ifdef DEBUG
		st_r_absorb++;
#endif /* DEBUG */
		// 将旧的下一块从空闲链表删除
		new_ptr = NEXT_BLKP(ptr);
		remove_free(new_ptr);

		merged = old_size + next_size;

		if (merged - alloc_size >= MIN_BS) {
			PUT(HP(ptr), PACK(alloc_size, 1));
			PUT(FP(ptr), PACK(alloc_size, 1));

			new_ptr = NEXT_BLKP(ptr);
			PUT(HP(new_ptr), PACK(merged - alloc_size, 0));
			PUT(FP(new_ptr), PACK(merged - alloc_size, 0));

			// 将切出的空闲块插入空闲链表
			insert_free(new_ptr);
		} else {
			PUT(HP(ptr), PACK(merged, 1));
			PUT(FP(ptr), PACK(merged, 1));
		}
		return ptr;
	}

	// 3.后面是 epilogue，即 ptr 是堆尾块，直接扩展堆
	if (GET_SIZE(HP(NEXT_BLKP(ptr))) == 0) {
#ifdef DEBUG
		st_r_tail++;
#endif /* DEBUG */
		size_t need = alloc_size - old_size;
		char*  new_block;

		// 与隐式空闲链表不同，这里必须要求堆扩展的空间大于等于最小块大小
		if ((new_block = extend_heap(need >= MIN_BS ? need : MIN_BS)) == NULL) {
			return NULL;
		}
		remove_free(new_block);

		// 如果多出 8 字节，吞并以避免产生非法的小块
		need = GET_SIZE(HP(new_block));
		PUT(HP(ptr), PACK(old_size + need, 1));
		PUT(FP(ptr), PACK(old_size + need, 1));
		return ptr;
	}

	// 4.块在堆中间且执行可能的合并后仍然不够，分配新内存并将原来的 payload 复制过来
#ifdef DEBUG
	st_r_move++;
	st_move_bytes += old_size - DSIZE;
#endif /* DEBUG */
	new_ptr = mm_malloc(size);
	if (new_ptr == NULL) {
		return NULL;
	}
	memcpy(new_ptr, ptr, old_size - DSIZE);
	mm_free(ptr);

	return new_ptr;
}

/*
 * @brief 堆一致性检查器
 *
 * 逐块遍历堆、再逐类遍历空闲链表，检查：
 *   1. 每个块都完整落在堆内，大小合法且为 DSIZE 的倍数；
 *   2. header 与 footer 的大小和 ALLOC 位完全一致；
 *   3. 不存在相邻的两个空闲块（立即合并的不变量）；
 *   4. 每个链表节点都在堆内、都是空闲块、payload 足以放下两个指针；
 *   5. 链表的 prev/next 双向对称，且所有类的节点总数等于堆中空闲块数；
 *   6. 每个空闲块都挂在它的大小所对应的那一类里（放错类会让 find_fit 永远找不到它）。
 *
 * @return 1 表示堆一致；0 表示发现错误（已打印出错位置与原因）
 */
int mm_check(void) {
	char*  bp;
	void*  p;
	size_t heap_free = 0;
	size_t list_len = 0;
	size_t ci;
	int    prev_free = 0;

	/* 1. 逐块遍历堆（从第一个真实块开始，heap_listp 指向的是 prologue 的 footer） */
	for (bp = NEXT_BLKP(heap_listp); GET_SIZE(HP(bp)) != 0; bp = NEXT_BLKP(bp)) {
		size_t size = GET_SIZE(HP(bp));
		size_t alloc = GET_ALLOC(HP(bp));

		if ((char*)bp < (char*)mem_heap_lo()
		    || (char*)bp + size - WSIZE > (char*)mem_heap_hi() + 1) {
			printf("mm_check: 块 %p 超出堆边界\n", (void*)bp);
			return 0;
		}
		if (size < MIN_BS || size % DSIZE != 0) {
			printf("mm_check: 块 %p 大小非法 (%zu)\n", (void*)bp, size);
			return 0;
		}
		if (GET(HP(bp)) != GET(FP(bp))) {
			printf("mm_check: 块 %p 的 header 与 footer 不一致\n", (void*)bp);
			return 0;
		}
		if (!alloc && prev_free) {
			printf("mm_check: %p 与前一块都是空闲块，没有合并\n", (void*)bp);
			return 0;
		}
		prev_free = !alloc;
		if (!alloc) {
			heap_free++;
		}
	}

	/* 2. 逐类遍历空闲链表 */
	for (ci = 0; ci < SEG_COUNT; ci++) {
		for (p = seg_listp[ci]; p != NULL; p = NEXT_FREEP(p)) {
			list_len++;
			if ((char*)p < (char*)mem_heap_lo() || (char*)p > (char*)mem_heap_hi()) {
				printf("mm_check: 第 %zu 类的节点 %p 不在堆内\n", ci, p);
				return 0;
			}
			if (GET_ALLOC(HP(p))) {
				printf("mm_check: 第 %zu 类里的 %p 是已分配块\n", ci, p);
				return 0;
			}
			if (GET_SIZE(HP(p)) < MIN_BS) {
				printf("mm_check: 第 %zu 类里的 %p 放不下两个指针\n", ci, p);
				return 0;
			}
			if (seg_index(GET_SIZE(HP(p))) != ci) {
				printf("mm_check: %p 属于第 %zu 类，却挂在第 %zu 类\n", p,
				    seg_index(GET_SIZE(HP(p))), ci);
				return 0;
			}
			if (NEXT_FREEP(p) != NULL && PREV_FREEP(NEXT_FREEP(p)) != p) {
				printf("mm_check: %p 与后继的 prev 指针不对称\n", p);
				return 0;
			}
			if (list_len > 1000000) {
				printf("mm_check: 空闲链表疑似成环\n");
				return 0;
			}
		}
	}

	/* 3. 堆中每个空闲块都必须出现在它所属的那一类里 */
	for (bp = NEXT_BLKP(heap_listp); GET_SIZE(HP(bp)) != 0; bp = NEXT_BLKP(bp)) {
		int    found = 0;
		size_t idx;

		if (GET_ALLOC(HP(bp))) {
			continue;
		}
		idx = seg_index(GET_SIZE(HP(bp)));
		for (p = seg_listp[idx]; p != NULL; p = NEXT_FREEP(p)) {
			if (p == (void*)bp) {
				found = 1;
				break;
			}
		}
		if (!found) {
			printf("mm_check: 空闲块 %p 不在第 %zu 类链表中\n", (void*)bp, idx);
			return 0;
		}
	}

	if (list_len != heap_free) {
		printf("mm_check: 链表节点总数 %zu 与堆中空闲块数 %zu 不符\n", list_len, heap_free);
		return 0;
	}

	return 1;
}

#ifdef DEBUG
/*
 * @brief 程序退出时打印统计信息（-DDEBUG 时启用）
 * @note  mdriver 不调用 mem_deinit，因此退出时堆内存仍然有效，可以安全遍历
 */
__attribute__((destructor)) static void mm_report(void) {
	size_t blocks = 0, free_blocks = 0, free_bytes = 0;
	char*  bp;

	for (bp = NEXT_BLKP(heap_listp); GET_SIZE(HP(bp)) != 0; bp = NEXT_BLKP(bp)) {
		blocks++;
		if (!GET_ALLOC(HP(bp))) {
			free_blocks++;
			free_bytes += GET_SIZE(HP(bp));
		}
	}

	printf("\n=============== mm 调试统计 ===============\n");
	printf("malloc 调用    : %zu\n", st_malloc_calls);
	printf("free   调用    : %zu\n", st_free_calls);
	printf("realloc 调用   : %zu\n", st_realloc_calls);
	printf("  |- 原地缩容  : %zu\n", st_r_shrink);
	printf("  |- 吞并后块  : %zu\n", st_r_absorb);
	printf("  |- 堆尾扩展  : %zu\n", st_r_tail);
	printf("  `- 搬运数据  : %zu 次，共 %zu 字节\n", st_r_move, st_move_bytes);
	printf("extend_heap    : %zu 次，共 %zu 字节\n", st_extend_calls, st_extend_bytes);
	printf("峰值堆大小     : %zu 字节\n", st_peak_heap);
	printf("链表总长       : 当前 %zu / 最大 %zu / 平均 %.1f\n", st_fl_len, st_fl_max,
	    st_fl_samples ? (double)st_fl_sum / (double)st_fl_samples : 0.0);
	printf("堆内块数       : %zu（空闲 %zu，空闲字节 %zu）\n", blocks, free_blocks, free_bytes);
	printf("===========================================\n");
}
#endif /* DEBUG */
