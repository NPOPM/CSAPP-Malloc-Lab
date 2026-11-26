"""把 mm.c 改造成 docs/experiments 里的某个实验变体。

用法: python3 patch.py <变体名> <mm.c 路径>

变体:
  bestfit     类内改成 best-fit（取最小的够用块）
  fine        分类粒度细化（<=128 字节按 8 字节一级，共 32 类）
  split64     切分阈值从 MIN_BS(16) 提到 64
  round_pow2  请求无条件向上取整到 2 的幂
  round_sel   只在浪费不超过 25% 时向上取整
"""

import sys

name = sys.argv[1]
path = sys.argv[2]
s = open(path, encoding="utf-8").read()


def replace_find_fit(src, body):
    a = src.index("static void* find_fit(size_t size) {")
    b = src.index("\n}", a) + 2
    return src[:a] + body + src[b:]


FIND_BESTFIT = '''static void* find_fit(size_t size) {
	size_t idx = seg_index(size);
	size_t i;

	for (i = idx; i < SEG_COUNT; i++) {
		void* best = NULL;
		void* block_ptr;

		for (block_ptr = seg_listp[i]; block_ptr != NULL; block_ptr = NEXT_FREEP(block_ptr)) {
			if (GET_SIZE(HP(block_ptr)) >= size) {
				if (best == NULL || GET_SIZE(HP(block_ptr)) < GET_SIZE(HP(best))) {
					best = block_ptr;
				}
			}
		}
		if (best != NULL) {
			return best;
		}
	}
	return NULL;
}'''

FINE_INDEX = '''static size_t seg_index(size_t size) {
	size_t index;
	size_t limit;

	if (size <= 128) {
		return (size >> 3) - 2;
	}

	index = 15;
	limit = 256;
	while (size > limit && index < SEG_COUNT - 1) {
		limit <<= 1;
		index++;
	}
	return index;
}'''


def round_block(thresh_expr):
    return f'''static size_t round_size(size_t size) {{
	size_t payload = ALIGN(size);
	size_t bucket;

	if (payload <= 128) {{
		return MAX(payload + DSIZE, MIN_BS);
	}}
	bucket = 128;
	while (bucket < payload) {{
		bucket <<= 1;
	}}
	if ({thresh_expr}) {{
		return bucket + DSIZE;
	}}
	return MAX(payload + DSIZE, MIN_BS);
}}'''


if name == "bestfit":
    s = replace_find_fit(s, FIND_BESTFIT)

elif name == "fine":
    a = s.index("static size_t seg_index(size_t size) {")
    b = s.index("\n}", a) + 2
    s = s[:a] + FINE_INDEX + s[b:]
    s = s.replace("#define SEG_COUNT 16", "#define SEG_COUNT 32", 1)

elif name == "split64":
    old = "\tif ((total_size - size) >= MIN_BS) {"
    assert s.count(old) == 1, "切分阈值锚点未找到"
    s = s.replace(old, "\tif ((total_size - size) >= 64) {", 1)

elif name == "round_sel2":
    # 小尺寸也走“浪费 <=25% 才取整”：112 与 128 都取 128，448 与 512 都取 512
    old = "#define BLKSIZE(size) MAX(ALIGN(size) + DSIZE, MIN_BS) // 计算请求的整块大小"
    assert old in s, "BLKSIZE 锚点未找到"
    s = s.replace(old, """static size_t round_size(size_t size) {
	size_t payload = ALIGN(size);
	size_t bucket = 16;

	while (bucket < payload) {
		bucket <<= 1;
	}
	if (payload * 4 >= bucket * 3) {
		return MAX(bucket + DSIZE, MIN_BS);
	}
	return MAX(payload + DSIZE, MIN_BS);
}""", 1)
    s = s.replace("\talloc_size = BLKSIZE(size);", "\talloc_size = round_size(size);")

elif name in ("round_pow2", "round_sel"):
    old = "#define BLKSIZE(size) MAX(ALIGN(size) + DSIZE, MIN_BS) // 计算请求的整块大小"
    assert old in s, "BLKSIZE 锚点未找到"
    cond = "1" if name == "round_pow2" else "payload * 4 >= bucket * 3"
    s = s.replace(old, round_block(cond), 1)
    s = s.replace("\talloc_size = BLKSIZE(size);", "\talloc_size = round_size(size);")

else:
    sys.exit("未知变体: " + name)

open(path, "w", encoding="utf-8", newline="").write(s)
