/******************************************************************************
 * Flash.h
 * 
 * Persistent storage for data structures implemented using flash memory.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 11May2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _FLASH_
#define _FLASH_

// Includes
//-----------------------------------------------------------------------------
#include "hardware/flash.h"
#include <list>


// Defines
//-----------------------------------------------------------------------------
#define kK                  (1024)
#define kM                  (kK * kK)
#define kDefaultSectors     (2)


// Flash class
/// Manages stored persistent data - storage and recovery. The flash is
/// responsible for storing zero or one blocks with a given signature. The
/// latest version of a block is recoverable. As the write unit to flash
/// is a sector - all blocks are sector-aligned.
//-----------------------------------------------------------------------------
class Flash {
    private:
        uint32_t muBaseAddress;
        uint32_t muSectors;

        class _BlockType {
            public:
                uint32_t muSignature;
                uint32_t muAddress;
                uint8_t muSize;

                _BlockType(uint32_t uSignature, uint32_t uOffset, uint8_t uSize);
                _BlockType &operator=(const _BlockType &cOther);
        };
        std::list<_BlockType> mlKnownBlocks;
        std::list<uint32_t> mlFreePages;

    public:
        /// Initialize flash singleton. Sets up all internal pointers.
        Flash(uint32_t uBase = 1 * kM, uint32_t uSectors = kDefaultSectors);

        /// Destructor - included for form
        ~Flash();

        /// Erase allocated flash area and reinitialize internal structures.
        /// Note that core1 will be halted and restarted - as will interrupts.
        bool eraseAndReset(void (*pfCore1EntryPoint)(void) = nullptr);

        /// Read latest valid block with the given signature. On
        /// return uSize is set to the size of the block. On failure
        /// returns nullptr and uSize is undefined. The block is
        /// in flash and should be copied to working memory.
        const uint8_t *readBlock(uint32_t uSignature, uint8_t &uSize);

        /// Write a block. Note that this call may temporarily
        /// disable interrupts and will stop what is going on on
        /// core1. The core1 entrypoint if any is therefore
        /// specified in this call so that it can be restarted
        /// if required. Returns 'true' on success.
        bool writeBlock(uint32_t uSignature, const uint8_t *pBlock, uint8_t uSize,
                        void (*pfCore1EntryPoint)(void) = nullptr);
};

#endif // _FLASH_