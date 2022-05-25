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
#include "pico/stdlib.h"
#include "memory.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "Flash.h"


// Defines
//-----------------------------------------------------------------------------
#define kPageSize           (1 << 8)
#define kSectorSize         (1 << 12)
#define kBlockMagic         (0xf1a548a2)


// Local types
//-----------------------------------------------------------------------------
typedef struct {
    uint32_t muBlockMagic;
    uint32_t muUserSignature;
    uint8_t muBlockBytes;
}_BlockHeader;


// Flash class methods
/// Manages stored persistent data - storage and recovery
//-----------------------------------------------------------------------------
Flash::_BlockType::_BlockType(uint32_t uSignature, uint32_t uAddress, uint8_t uSize) :
    muSignature(uSignature), muAddress(uAddress), muSize(uSize) {
}

Flash::_BlockType &Flash::_BlockType::operator=(const Flash::_BlockType &cOther) {
    muSignature = cOther.muSignature;
    muAddress = cOther.muAddress;
    muSize = cOther.muSize;
    return *this;
}

Flash::Flash(uint32_t uBase, uint32_t uSectors) :
    muBaseAddress(XIP_BASE + ((((uBase / kSectorSize) + (((uBase%kSectorSize) != 0)?1:0)) * kSectorSize))),
    muSectors(uSectors) {

    // Populate the list of known blocks. Run up from muBaseAddress by pages looking for valid
    // signatures. Higher address blocks supercede those found at lower addresses.
    uint32_t uTop(muBaseAddress + (kSectorSize * muSectors));
    for(uint32_t uAddress = muBaseAddress; uAddress < uTop; uAddress += kPageSize) {
        const uint8_t *pRd((const uint8_t *)uAddress);
        _BlockHeader cHeader = {0x0, 0x0, 0x0};
        memcpy(&cHeader, pRd, sizeof(cHeader));
        if (cHeader.muBlockMagic == kBlockMagic) {
            // Found valid block - populate into list
            Flash::_BlockType cBlock(cHeader.muUserSignature, uAddress, cHeader.muBlockBytes);
            bool bFound(false);
            for(std::list<Flash::_BlockType>::iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
                if ((*cIter).muSignature == cBlock.muSignature) {
                    // If same signature exists in list already - replace it
                    (*cIter) = cBlock;
                    bFound = true;
                    break;
                }
            }
            if (false == bFound) {
                // Not seen this signature before - insert into list
                mlKnownBlocks.push_back(cBlock);
            }
        } else {
            mlFreePages.push_back(uAddress);
        }
    }
}

Flash::~Flash() {
}

/// Erase allocated flash area and reinitialize internal structures
bool Flash::eraseAndReset(void (*pfCore1EntryPoint)(void)) {
    // Interrupt and core-safe write
    uint32_t uInterrupts(save_and_disable_interrupts());
    multicore_reset_core1();

    // Do write
    flash_range_erase(muBaseAddress, muSectors * kSectorSize);

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
            return (const uint8_t*)((*cIter).muAddress);
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
            if (mpfCore1EntryPoint != nullptr) {
                restore_interrupts(muInterruptEnables);
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

    // Is there free space?
    if (false == mlFreePages.empty()) {
        // Perform write into free space.
        uint32_t uTargetAddress(mlFreePages.front());
        mlFreePages.pop_front();

        // Allocate space as 32-bit and recast to 32-bit align
        uint uWords(kPageSize / sizeof(uint32_t)); 
        uint32_t *pData32(new uint32_t[uWords]);
        uint8_t *pData8((uint8_t *)pData32);

        // Create RAM data to write
        _BlockHeader cHeader;
        cHeader.muBlockMagic = kBlockMagic;
        cHeader.muUserSignature = uSignature;
        cHeader.muBlockBytes = uSize;
        memset(pData8, 0x00, kPageSize);
        memcpy(pData8, &cHeader, sizeof(_BlockHeader));
        memcpy(&pData8[sizeof(_BlockHeader)], pBlock, uSize);

        // Interrupt and core-safe write
        {   _ScopedProtectFlash cSafe(pfCore1EntryPoint);
            flash_range_program(uTargetAddress, pData8, kPageSize);
        }
        delete []pData32;

        // Update block tracking
        bool bFound(false);
        for(std::list<Flash::_BlockType>::iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
            if ((*cIter).muSignature == uSignature) {
                bFound = true;
                (*cIter).muAddress = uTargetAddress;
                (*cIter).muSize = uSize;
                break;
            }
        }
        if (false == bFound) {
            Flash::_BlockType cNew(uSignature, uTargetAddress, uSize);
            mlKnownBlocks.push_back(cNew);
        }
        
        return true;

    } else {
        // Garbage collect! - erase all sectors not containing any 'good' blocks
        // and add to the free list.
        
        // Make array of erasable sectors
        bool *pbErasableSectors(new bool[muSectors]);
        for(uint i=0; i<muSectors; i++) {
            pbErasableSectors[i] = false;
        }
        for(std::list<Flash::_BlockType>::const_iterator cIter = mlKnownBlocks.begin(); cIter != mlKnownBlocks.end(); ++cIter) {
            uint32_t uSectorBase((*cIter).muAddress & (~(kSectorSize-1)));  // Knock out all but sector base address
            uint uSector((uSectorBase - muBaseAddress) / kSectorSize);
            pbErasableSectors[uSector] = true;
        }

        // Erase all the erasable sectors and add the resulting free sectors to the free list
        {   _ScopedProtectFlash cSafe(pfCore1EntryPoint);
            for(uint i=0; i<muSectors; i++) {
                if (true == pbErasableSectors[i]) {
                    uint32_t uSectorBase(muBaseAddress + (i * kSectorSize));
                    flash_range_erase(uSectorBase, kSectorSize);
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
        if (true == mlFreePages.empty()) {
            return false;
        }

        // We now have free space - tail recurse
        return writeBlock(uSignature, pBlock, uSize, pfCore1EntryPoint);
    }
}
