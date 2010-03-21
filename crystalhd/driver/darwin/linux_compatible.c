//  linux_compatible.c
//  BroadcomCrystalHD V1.0
//
//  Created by Scott Davilla on 08.03.09
//  Copyright 2009-2010 4pi Analysis, Inc. All rights reserved.
//
//  ** LICENSE *************************************************************************
//
//  Copyright 2009-2010 4pi Analysis, Inc. All rights reserved.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  * Redistributions of source code must retain the above copyright notice, this list
//    of conditions and the following disclaimer.
//  
//  * Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or other
//    materials provided with the distribution.
//  
//  * Neither the name of 4pi Analysis, Inc. nor the names of its contributors may be
//    used to endorse or promote products derived from this software without specific
//    prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
//  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
//  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
//  DAMAGE.
//
//  ************************************************************************************

#include "BroadcomCrystalHD.h"
#include "linux_compatible.h"

// the OSX kext way to do extern "C" {}
__BEGIN_DECLS
extern void *kern_os_malloc(size_t size);
extern void  kern_os_free(void * addr);
__END_DECLS

void udelay(unsigned int microseconds)
{
  // Spin delay for a number of microseconds
	IODelay(microseconds);
}
unsigned long msleep_interruptible(unsigned int msecs)
{
	// FIXME, make into interruptable sleep
	// Sleep the calling thread for a number of milliseconds
	IOSleep(msecs);
	return(0);
}

unsigned long readl(IOVirtualAddress addr)
{
	return OSReadLittleInt32((volatile void*)addr, 0);
}
void writel(unsigned long value, IOVirtualAddress addr)
{
	OSWriteLittleInt32((volatile void*)addr, 0, value);
}

int pci_read_config_byte(void *dev, uint8_t off, u8 *val)
{
	*val = BroadcomCrystalHD::getPciNub()->configRead8(off);
	return(0);
}
int pci_read_config_word(void *dev, uint8_t off, u16 *val)
{
	*val = BroadcomCrystalHD::getPciNub()->configRead16(off);
	return(0);
}
int pci_read_config_dword(void *dev, uint8_t off, u32 *val)
{
	*val = BroadcomCrystalHD::getPciNub()->configRead32(off);
	return(0);
}

int pci_write_config_byte(void *dev, uint8_t off, u8 val)
{
	BroadcomCrystalHD::getPciNub()->configWrite8(off, val);
	return(0);
}
int pci_write_config_word(void *dev, uint8_t off, u16 val)
{
	BroadcomCrystalHD::getPciNub()->configWrite16(off, val);
	return(0);
}
int pci_write_config_dword(void *dev, uint8_t off, u32 val)
{
	BroadcomCrystalHD::getPciNub()->configWrite32(off, val);
	return(0);
}

struct pci_pool* pci_pool_create(const char *name, void *pdev, size_t size, size_t align, int flags)
{
	// big fake out here, we alloc a page and hand out from that page.
	uint32_t pool_size = PAGE_SIZE;
	struct pci_pool *page_block;

	page_block = (pci_pool*)kern_os_malloc(sizeof(struct pci_pool));

	page_block->chunk_size = size;
	page_block->chunk_align = align * 2;
	page_block->memory = IOBufferMemoryDescriptor::withOptions(
		kIOMemoryPhysicallyContiguous, pool_size, PAGE_SIZE);

	if (page_block->memory == 0) {
		return false;
	}
	if (page_block->memory->prepare() != kIOReturnSuccess) {
		SAFE_RELEASE(page_block->memory);
		return false;
	}

	page_block->freeStart = page_block->memory->getBytesNoCopy();
	page_block->freeBytes = page_block->memory->getCapacity();
	bzero(page_block->freeStart, page_block->freeBytes);

	return page_block;
}
void* pci_pool_alloc(struct pci_pool *pool, int mem_flags, dma_addr_t *handle)
{
	uint32_t    size;
	void        *vaddr;
	IOByteCount segLength;

	if (pool->chunk_align == 0)
		return 0;
      
	size = pool->chunk_size;
	// Locate next alignment boundary.
	vaddr = (void *)(((uint32_t)pool->freeStart + (pool->chunk_align - 1)) & ~(pool->chunk_align - 1));

	// Add alignment padding to the allocation size.
	size += (uint32_t)vaddr - (uint32_t)pool->freeStart;
	if (size > pool->freeBytes)
		return 0;

	pool->freeStart  = (void *)((uint32_t)pool->freeStart + size);
	pool->freeBytes -= size;

	if (handle) {
		*handle = pool->memory->getPhysicalSegment(
		(uint32_t)vaddr - (uint32_t)pool->memory->getBytesNoCopy(), &segLength);
	}

	return vaddr;
}
void pci_pool_free(struct pci_pool *pool, void *vaddr, dma_addr_t dma)
{
	// FIXME :)
}
void pci_pool_destroy(struct pci_pool *pool)
{
	if (pool->memory) {
		pool->memory->complete();
		SAFE_RELEASE(pool->memory);
	}
	kern_os_free(pool);
}

void *pci_alloc_consistent(void *pdev, size_t size, dma_addr_t *dma_handle)
{
	IOBufferMemoryDescriptor    *memDesc;
	IOVirtualAddress            virt_address;
	IOPhysicalAddress           phys_address;

	// construct a memory descriptor for a buffer below the 4Gb line,
	// addressable by 32 bit DMA and page aligned.
	memDesc = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
		kIOMemoryPhysicallyContiguous, size, PAGE_SIZE);
	if (memDesc) {
		IOByteCount		offset = 0;
		IOByteCount		length;

		memDesc->prepare();
		virt_address = (IOVirtualAddress)memDesc->getBytesNoCopy();
		phys_address = memDesc->getPhysicalSegment(offset, &length);

		g_bcm_dma_info->setObject(memDesc);
	} else {
		virt_address = NULL;
		phys_address = NULL;
		IOLog("pci_alloc_consistent:IOBufferMemoryDescriptor::inTaskWithOptions failed\n");
	}

	//IOLog("pci_alloc_consistent paddr(0x%X), size(0x%X)\n", (unsigned int)phys_address, size);
	*dma_handle = phys_address;
	return (void*)virt_address;
}
void pci_free_consistent(void *pdev, size_t size, void *vaddr, dma_addr_t dma_handle)
{
	// free a hw dma scatter/gather list located in host memory
	int                             index;
	OSArray                         *dma_info_array;
	IOBufferMemoryDescriptor        *memDesc;
	IOVirtualAddress                virt_address;

	// search for the correct dma_info by checking against passed virtual address
	dma_info_array = g_bcm_dma_info;
	for(index = 0; index < (int)dma_info_array->getCount(); index++) {
		memDesc = (IOBufferMemoryDescriptor*)dma_info_array->getObject(index);
		virt_address = (IOVirtualAddress)memDesc->getBytesNoCopy();
		if ((IOVirtualAddress)vaddr == virt_address) {
			//IOLog("pci_free_consistent padd(%p), size(0x%X)\n", dma_handle, size);
			// found it, now complete. removeObject will release it
			memDesc->complete();
			dma_info_array->removeObject(index);
			// should be able to just call memDesc->release() after memDesc->complete()
			// but on atv, there's something holding a ref to memDesc and so we leak memory
			// everytime the crystalhs driver closes. Doing the release this way fixes the mem
			// leak on atv and is fine under real 10.4/10.5 boxes.
			SAFE_RELEASE(memDesc);
			break;
		}
	}
}

void* kzalloc(size_t size, gfp_t flags)
{
	void *mem = kern_os_malloc(size);
	if (mem)
		bzero(mem, size);
	return(mem);
}
void* kalloc(size_t size, gfp_t flags)
{
	return( kern_os_malloc(size) );
}
void kfree(void *addr)
{
	kern_os_free(addr);
}

void* vmalloc(size_t size)
{
	return( kern_os_malloc(size) );
}
void vfree(void *addr)
{
	kern_os_free(addr);
}





