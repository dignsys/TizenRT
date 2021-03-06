/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <debug.h>
#include <tinyara/mm/mm.h>

#if CONFIG_MM_NHEAPS > 0
extern struct mm_heap_s g_mmheap[CONFIG_MM_NHEAPS];
#endif
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: mm_get_heap
 *
 * Description:
 *   returns a heap which type is matched with ttype
 *
 ****************************************************************************/
struct mm_heap_s *mm_get_heap(void *address)
{
	int heap_idx;
#ifdef CONFIG_MM_KERNEL_HEAP
	struct tcb_s *tcb;

	tcb = sched_gettcb(getpid());
	if (tcb->flags & TCB_FLAG_TTYPE_MASK == TCB_FLAG_TTYPE_KERNEL) {
		return &g_kmmheap;
	} else
#endif
	{
#if CONFIG_MM_NHEAPS > 0
		heap_idx = mm_get_heapindex(address);
		if (heap_idx == INVALID_HEAP_IDX) {
			mdbg("address is not in heap region.\n");
			return NULL;
		}
		return &g_mmheap[heap_idx];
#else
		return BASE_HEAP;
#endif
	}

}
/****************************************************************************
 * Name: mm_get_heap_with_index
 ****************************************************************************/
#if CONFIG_MM_NHEAPS > 0
struct mm_heap_s *mm_get_heap_with_index(int index)
{
	if (index >= CONFIG_MM_NHEAPS) {
		mdbg("heap index is out of range.\n");
		return NULL;
	}
	return &g_mmheap[index];
}

/****************************************************************************
 * Name: mm_get_heapindex
 ****************************************************************************/
int mm_get_heapindex(void *mem)
{
	int heap_idx;
	if (mem == NULL) {
		return 0;
	}
	for (heap_idx = 0; heap_idx < CONFIG_MM_NHEAPS; heap_idx++) {
		if (mem < (void *)(g_mmheap[heap_idx].mm_heapstart + g_mmheap[heap_idx].mm_heapsize) && mem >= (void *)g_mmheap[heap_idx].mm_heapstart) {
			return heap_idx;
		}
	}
	return INVALID_HEAP_IDX;
}
#endif
