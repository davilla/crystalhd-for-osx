//
//  BroadcomCrystalHD.h
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

#include <IOKit/pci/IOPCIDevice.h>
#if IOMEMORYDESCRIPTOR_SUPPORTS_DMACOMMAND
    #include <IOKit/IODMACommand.h>
#endif
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOCommandGate.h>
#include "linux_compatible.h"

class IOCommandGate;

#define SAFE_RELEASE(x) do { if (x) x->release(); x = 0; } while(0)

// Icky, globals (fix later)
extern OSArray* g_bcm_dma_info;


class BroadcomCrystalHD : public IOService
{
    // Declare the metaclass information that is used for runtime
    OSDeclareDefaultStructors( BroadcomCrystalHD );
private:
    static IOPCIDevice          *m_pciNub;
    IOMemoryMap                 *m_pci_bar0;
    IOMemoryMap                 *m_pci_bar2;
    IOWorkLoop                  *m_workloop;
    IOCommandGate               *m_commandgate;
    IOInterruptEventSource      *m_interrupt_source;

    // BSD ioctl interface
    static struct cdevsw        m_devsw;
    IOLock                      *m_bds_lock;
    unsigned int                m_Major;
    unsigned int                m_LastMinor;
    OSDictionary                *m_Names;
    BroadcomCrystalHD           **m_DEVs;
    struct bc_link_user         **m_USERs;
    struct pci_dev              m_pci_dev;      // fake Linux pci_dev struct
    
    // BSD ioctl support functions
    IOReturn                    bsd_create_device(void);
    IOReturn                    bsd_destroy_device(void);
    dev_t                       bsd_create_node(void);
    bool                        bsd_register_node(dev_t dev, BroadcomCrystalHD *dev);
    BroadcomCrystalHD*          bsd_get_device(dev_t dev);

    // Private member functions
    virtual void                interruptOccured(IOInterruptEventSource *sender, int count);

protected:
    dev_t                       m_base_dev;			// BSD ioctl interface
    void                        *m_cdev_node;		// (character device's devfs node)
    IOService                   *_pmPolicyMaker;
    UInt32                      _pmPowerState;
    thread_call_t               _powerOffThreadCall;
    thread_call_t               _powerOnThreadCall;
    
public:
    // IOService overrides
    virtual bool                start(IOService* provider);
    virtual void                stop( IOService* provider);
    virtual IOWorkLoop*         getWorkLoop(void);
    virtual IOCommandGate       *getCommandGate(void);
	
    virtual IOReturn registerWithPolicyMaker(IOService * policyMaker);
    virtual IOReturn setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker);
    virtual void setPowerStateOff(void);
    virtual void setPowerStateOn(void);

    // Public functions
    static IOPCIDevice*         getPciNub(void);
};
