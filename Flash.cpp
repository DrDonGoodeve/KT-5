/******************************************************************************
 * Flash.cpp
 * 
 * Persistent storage for data structures implemented using flash memory.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 11May2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

// Includes
//-----------------------------------------------------------------------------
#include <stdio.h>
#include "pico/stdlib.h"
#include "memory.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "Flash.h"


// Defines
//-----------------------------------------------------------------------------
#define kPageSize           (1 << 8)
#define kSectorSize         (1 << 12)
#define kFlashSize          (2 * kM)
#define kBlockMagic         (0xf1a548a2)


// Local types
//-----------------------------------------------------------------------------
typedef struct {
    uint32_t muBlockMagic;
    uint32_t muUserSignature;
    uint8_t muBlockBytes;
    uint32_t muGeneration;
}_BlockHeader;

// Flash class methods
/// Manages stored persistent data - storage and recovery
//-----------------------------------------------------------------------------
Flash::_BlockType::_BlockType(uint32_t uSignature, uint32_t uAddress, uint8_t uSize, uint32_t uGeneration) :
    muSignature(uSignature), muAddress(uAddress), muSize(uSize), muGeneration(uGeneration) {
}

Flash::_BlockType &Flash::_BlockType::operator=(const Flash::_BlockType &cOther) {
    muSignature = cOther.muSignature;
    muAddress = cOther.muAddress;
    muSize = cOther.muSize;
    muGeneration = cOther.muGeneration;
    return *this;
}

Flash::Flash(uint32_t uSectors) :
    muBaseAddress(XIP_BASE + kFlashSize - (uSectors*kSectorSize)),
    muSectors(uSectors) {

    // Populate the list of known blocks. Run up from muBaseAddress by pages looking for valid
    // signatures. Higher address blocks supercede those found at lower addresses.
    uint32_t uTop(muBaseAddress + (kSectorSize * muSectors));
    printf("Flash Base:0x%08x, Top:0x%08x\r\n", muBaseAddress, uTop);
    for(uint32_t uAddress = muBaseAddress; uAddress < uTop; uAddress += kPageSize) {
        //printf("Reading page at 0x%08x\r\n", uAddress);
        const uint8_t *pRd((const uint8_t *)uAddress);
        _BlockHeader cHeader = {0x0, 0x0, 0x0, 0x0};
        memcpy(&cHeader, pRd, sizeof(cHeader));
        if (cHeader.muBlockMagic == kBlockMagic) {
            // Found valid block - populate into list
            Flash::_BlockType cBlock(cHeader.muUserSignature, uAddress, cHeader.muBlockBytes, cHeader.muGeneration);
            bool bFound(false);
            for(std::list<Flash::_BlockType>::iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
                if ((*cIter).muSignature == cBlock.muSignature) {
                    // If same signature exists in list already - replace it if the generation number is the same or higher
                    // otherwise we simply ignore it.
                    bFound = true;
                    if (cBlock.muGeneration >= (*cIter).muGeneration) {
                        (*cIter) = cBlock;
                        printf("Replaced signature 0x%08x with generation %d\r\n", (*cIter).muSignature, (*cIter).muGeneration);
                    } else {
                        printf("Ignored older 0x%08x block at generation %d\r\n", cBlock.muSignature, cBlock.muGeneration);
                    }
                }
            }
            if (false == bFound) {
                // Not seen this signature before - insert into list
                printf("First time signature 0x%08x at generation %d\r\n", cBlock.muSignature, cBlock.muGeneration);
                mlKnownBlocks.push_back(cBlock);
                //printf("Known blocks now has %d entries\r\n", mlKnownBlocks.size());
            }
        } else {
            mlFreePages.push_back(uAddress);
            //printf("Now have %d free pages\r\n", mlFreePages.size());
        }
    }
    printf("flash: %d blocks, %d pages free\r\n", mlKnownBlocks.size(), mlFreePages.size());
}

Flash::~Flash() {
}

/// Erase allocated flash area and reinitialize internal structures
bool Flash::eraseAndReset(void (*pfCore1EntryPoint)(void)) {
    // Interrupt and core-safe write
    uint32_t uInterrupts(save_and_disable_interrupts());
    multicore_reset_core1();

    // Do write
    flash_range_erase(muBaseAddress-XIP_BASE, muSectors * kSectorSize);

    // Re-enable core1 and interrupts
    if (pfCore1EntryPoint != nullptr) {
        multicore_launch_core1(pfCore1EntryPoint);
    }
    restore_interrupts(uInterrupts);

    // Reset data structures
    mlKnownBlocks.clear();
    uint32_t uTop(muBaseAddress + (kSectorSize * muSectors));
    mlFreePages.clear();
    for(uint32_t uAddress=muBaseAddress; uAddress < uTop; uAddress += kPageSize) {
        mlFreePages.push_back(uAddress);
    }
    return true;
}

/// Read latest valid block with the given signature. On
/// return uSize is set to the size of the block. On failure
/// returns nullptr and uSize is undefined.
const uint8_t *Flash::readBlock(uint32_t uSignature, uint8_t &uSize) {
    for(std::list<Flash::_BlockType>::iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
        if ((*cIter).muSignature == uSignature) {
            uSize = (*cIter).muSize;
            return (const uint8_t*)((*cIter).muAddress + sizeof(_BlockHeader));
        }
    }
    return nullptr;
}

/// Write a block. Note that this call may temporarily
/// disable interrupts and will stop what is going on on
/// core1. The core1 entrypoint if any is therefore
/// specified in this call so that it can be restarted
/// if required. Returns 'true' on success.
class _ScopedProtectFlash {
    private:
        void (*mpfCore1EntryPoint)(void);
        uint32_t muInterruptEnables;

    public:
        _ScopedProtectFlash(void (*pfCore1EntryPoint)(void) = nullptr) :
            mpfCore1EntryPoint(pfCore1EntryPoint) {
            muInterruptEnables = save_and_disable_interrupts();
            multicore_reset_core1();
        }

        ~_ScopedProtectFlash() {
            restore_interrupts(muInterruptEnables);
            if (mpfCore1EntryPoint != nullptr) {
                multicore_launch_core1(mpfCore1EntryPoint);
            }
        }
};

bool Flash::writeBlock(
    uint32_t uSignature, const uint8_t *pBlock, uint8_t uSize,
    void (*pfCore1EntryPoint)(void)) {

    // Check block size
    if ((uSize > (kPageSize - sizeof(_BlockHeader))) || (0 == uSize)) {
        return false;
    }

    // Is there a free page available?
    if (true == mlFreePages.empty()) {
        // Garbage collect! - erase all sectors not containing any 'good' blocks
        // and add to the free list.
        printf("GC: No free pages\r\n");
        
        // Make array of erasable sectors
        bool *pbErasableSectors(new bool[muSectors]);
        for(uint i=0; i<muSectors; i++) {
            pbErasableSectors[i] = true;
        }
        // Cannot erase any sector with live content
        for(std::list<Flash::_BlockType>::const_iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
            uint32_t uSectorBase((*cIter).muAddress & (~(kSectorSize-1)));  // Knock out all but sector base address
            uint uSector((uSectorBase - muBaseAddress) / kSectorSize);
            pbErasableSectors[uSector] = false;
        }

        // Erase all the erasable sectors and add the resulting free sectors to the free list
        {   _ScopedProtectFlash cSafe(pfCore1EntryPoint);
            for(uint i=0; i<muSectors; i++) {
                if (true == pbErasableSectors[i]) {
                    uint32_t uSectorBase(muBaseAddress + (i * kSectorSize));
                    printf("Erasing garbage sector at 0x%08x\r\n", uSectorBase);
                    flash_range_erase(uSectorBase-XIP_BASE, kSectorSize);
                    for(uint j=0; j<(kSectorSize/kPageSize); j++) {
                        uint32_t uPageBase(uSectorBase + (j * kPageSize));
                        mlFreePages.push_back(uPageBase);
                    }
                }
            }
        }
        delete []pbErasableSectors; pbErasableSectors = nullptr;

        // If free list remains empty - we are stuck without a page-moving
        // garbage collection. With more sectors than block types this will
        // never happen.
        printf("flash: now have %d free pages\r\n", mlFreePages.size());
        if (true == mlFreePages.empty()) {
            return false;
        }
    }

    // Determine generation of block
    uint32_t uGeneration(0);
    std::list<_BlockType>::iterator cBlockIterator(mlKnownBlocks.begin());
    for(; cBlockIterator != mlKnownBlocks.end(); ++cBlockIterator) {
        if ((*cBlockIterator).muSignature == uSignature) {
            uGeneration = ((*cBlockIterator).muGeneration + 1);
            break;
        }
    }

    //printf("Preparing to write %d byte block generation %d\r\n", uSize, uGeneration);

    // Perform write into free space.
    uint32_t uTargetAddress(mlFreePages.front());
    mlFreePages.pop_front();

    //printf("Have write target address 0x%08x\r\n", uTargetAddress);

    // Allocate space as 32-bit and recast to 32-bit align
    uint uWords(kPageSize / sizeof(uint32_t)); 
    uint32_t *pData32(new uint32_t[uWords]);
    uint8_t *pData8((uint8_t *)pData32);

    // Create RAM data to write
    _BlockHeader cHeader;
    cHeader.muBlockMagic = kBlockMagic;
    cHeader.muUserSignature = uSignature;
    cHeader.muBlockBytes = uSize;
    cHeader.muGeneration = uGeneration;
    memset(pData8, 0x00, kPageSize);
    memcpy(pData8, &cHeader, sizeof(_BlockHeader));
    memcpy(&pData8[sizeof(_BlockHeader)], pBlock, uSize);

    printf("Writing block 0x%08x at generation %d\r\n", cHeader.muUserSignature, cHeader.muGeneration);
    // Interrupt and core-safe write
    {   _ScopedProtectFlash cSafe(pfCore1EntryPoint);
        flash_range_program(uTargetAddress-XIP_BASE, pData8, kPageSize);
        //printf("Well...?\r\n");
    }
    delete []pData32;

    //printf("Write completed\r\n");

    // Update block tracking
    bool bFound(false);
    for(std::list<Flash::_BlockType>::iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
        if ((*cIter).muSignature == uSignature) {
            bFound = true;
            (*cIter).muAddress = uTargetAddress;
            (*cIter).muGeneration = uGeneration;
            (*cIter).muSize = uSize;
            break;
        }
    }
    if (false == bFound) {
        Flash::_BlockType cNew(uSignature, uTargetAddress, uSize, uGeneration);
        mlKnownBlocks.push_back(cNew);
    }

    //printf("Complete - mlKnownBlocks size is now %d\r\n", mlKnownBlocks.size());
    
    return true;
}
