//
//  BroadcomCrystalHDPower.cpp
//  BroadcomCrystalHD
//
//  Created by Bolke de Bruin on 29-04-10.
//  Copyright 2010 CanTouch. All rights reserved.
//
//  ** LICENSE *************************************************************************
//
//  Copyright 2010 CanTouch. All rights reserved.
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
//  * Neither the name of CanTouch nor the names of its contributors may be
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

#include "BroadcomCrystalHD.h"

#include "crystalhd_lnx.h"

extern struct crystalhd_adp* g_adp_info;
//**************************************************************************
// POWER-MANAGEMENT CODE
//***************************************************************************
// These definitions and functions allow the driver to handle power state
// changes to support sleep and wake.
//**************************************************************************/

// Two power states are supported by the driver, On and Off.

enum {
    kPowerStateOff = 0,
    kPowerStateOn,
    kPowerStateCount
};

// An IOPMPowerState structure is added to the array for each supported
// power state. This array is used by the power management policy-maker
// to determine our capabilities and requirements for each power state.

static IOPMPowerState _PowerStateArray[ kPowerStateCount ] =
{
    { 1,0,0,0,0,0,0,0,0,0,0,0 },
    { 1,IOPMDeviceUsable,IOPMPowerOn,IOPMPowerOn,0,0,0,0,0,0,0,0 }
};

enum {
    kFiveSeconds = 5000000
};

//---------------------------------------------------------------------------
// handleSetPowerStateOff()
//
// The policy-maker has told the driver to turn off the device, and the
// driver has started a new thread to do this. This C function is the start
// of that thread. This function then calls itself through the command gate
// to gain exclusive access to the device.
//---------------------------------------------------------------------------

void handleSetPowerStateOff(thread_call_param_t param0, thread_call_param_t param1)
{
    BroadcomCrystalHD *self = (BroadcomCrystalHD*)param0;

    assert(self);

    if (param1 == 0) {
        self->getCommandGate()->runAction( (IOCommandGate::Action)
            handleSetPowerStateOff,
            (void *) 1 /* param1 */ );
    } else {
        self->setPowerStateOff();
        self->release();  // offset the retain in setPowerState()
    }
}

//---------------------------------------------------------------------------
// handleSetPowerStateOn()
//
// The policy-maker has told the driver to turn on the device, and the
// driver has started a new thread to do this. This C function is the start
// of that thread. This function then calls itself through the command gate
// to gain exclusive access to the device.
//---------------------------------------------------------------------------

void handleSetPowerStateOn(thread_call_param_t param0, thread_call_param_t param1)
{
    BroadcomCrystalHD* self = (BroadcomCrystalHD*)param0;

    assert(self);

    if (param1 == 0) {
        self->getCommandGate()->runAction( (IOCommandGate::Action)
            handleSetPowerStateOn,
            (void *) 1 /* param1 */ );
    } else {
        self->setPowerStateOn();
        self->release();  // offset the retain in setPowerState()
    }
}

//---------------------------------------------------------------------------
// registerWithPolicyMaker()
//
// The superclass invokes this function when it is time for the driver to
// register with the power management policy-maker of the device.
//
// The driver registers by passing to the policy-maker an array which
// describes the power states supported by the hardware and the driver.
//
// Argument:
//
// policyMaker - A pointer to the power management policy-maker of the
//               device.
//---------------------------------------------------------------------------

IOReturn BroadcomCrystalHD::registerWithPolicyMaker(IOService *policyMaker)
{
    IOReturn ret;

    // Initialize power management support state.
    _pmPowerState  = kPowerStateOn;
    _pmPolicyMaker = policyMaker;
	
    // No cheating here. Decouple the driver's handling of power change
    // requests from the power management work loop thread. Not strictly
    // necessary for the work that this driver is doing currently, but
    // useful as an example of how to do things in general.
    //
    // Allocate thread callouts for asynchronous power on and power off.
    // Allocation failure is not detected here, but before each use in
    // the setPowerState() method.
    IOLog("BroadcomCrystalHD: Registering power handler\n");

    _powerOffThreadCall = thread_call_allocate( 
        (thread_call_func_t)  handleSetPowerStateOff,
        (thread_call_param_t) this );

    _powerOnThreadCall  = thread_call_allocate(
        (thread_call_func_t)  handleSetPowerStateOn,
        (thread_call_param_t) this );

    ret = _pmPolicyMaker->registerPowerDriver(this,
        _PowerStateArray,
        kPowerStateCount);

    return ret;
}

//---------------------------------------------------------------------------
// setPowerState()
//
// The power management policy-maker for this device invokes this function
// to switch the power state of the device.
//
// Arguments:
//
// powerStateOrdinal - an index into the power state array for the state
//                     to switch to.
//
// policyMaker       - a pointer to the policy-maker.
//---------------------------------------------------------------------------

IOReturn BroadcomCrystalHD::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
    // The default return value is for an implied acknowledgement.
    // If the appropriate thread wasn't allocated earlier in
    // registerWithPolicyMaker(), then this is what will be returned.

    IOReturn result = IOPMAckImplied;
    // There's nothing to do if our current power state is the one
    // we're being asked to change to.
    if (_pmPowerState == powerStateOrdinal) {
        return result;
    }

    switch (powerStateOrdinal) {
        case kPowerStateOff:
            // The driver is being told to turn off the device for some reason.
            // It saves whatever state and context it needs to and then shuts
            // off the device.
            //
            // It may take some time to turn off the HW, so the driver spawns
            // a thread to power down the device and returns immediately to
            // the policy-maker giving an upper bound on the time it will need
            // to complete the power state transition.
            IOLog("BroadcomCrystalHD: Turning off\n");
            if (_powerOffThreadCall) {
                // Prevent the object from being freed while a call is pending.
                // If thread_call_enter() returns TRUE, then a call is already
                // pending, and the extra retain is dropped.

                retain();
                if (thread_call_enter(_powerOffThreadCall) == TRUE) {
                    release();
                }
                result = kFiveSeconds;
            }
            break;

        case kPowerStateOn:
            // The driver is being told to turn on the device.  It does so
            // and then restores any state or context which it has previously
            // saved.
            //
            // It may take some time to turn on the HW, so the driver spawns
            // a thread to power up the device and returns immediately to
            // the policy-maker giving an upper bound on the time it will need
            // to complete the power state transition.
            IOLog("BroadcomCrystalHD: Turning on\n");

            if (_powerOnThreadCall) {
                // Prevent the object from being freed while a call is pending.
                // If thread_call_enter() returns TRUE, then a call is already
                // pending, and the extra retain is dropped.

                retain();
                if (thread_call_enter(_powerOnThreadCall) == TRUE) {
                    release();
                }
                result = kFiveSeconds;
            }
            break;

        default:
            break;
    }

    return result;
}

//---------------------------------------------------------------------------
// setPowerStateOff()
//
// The policy-maker has told the driver to turn off the device, and this
// function is called by a new kernel thread, while holding the gate in
// the driver's work loop. Exclusive hardware access is assured.
//---------------------------------------------------------------------------

void BroadcomCrystalHD::setPowerStateOff(void)
{
    crystalhd_ioctl_data *temp;

    IOLog("BroadcomCrystalHD::setPowerStateOff\n");
    // At this point, all clients have been notified of the driver's
    // imminent transition to a power state that renders it "unusable".
    // And beacuse of the response to this notification by all clients,
    // the controller driver is guaranteed to be disabled when this
    // function is called.
    temp = chd_dec_alloc_iodata(g_adp_info, false);
    if (!temp) {
        IOLog("BroadcomCrystalHD: could not get ioctl data\n");
    }

    crystalhd_suspend(&g_adp_info->cmds, temp);

    chd_dec_free_iodata(g_adp_info, temp, false);

    _pmPowerState = kPowerStateOff;

    // Since the driver returned a non-acknowledgement when called at
    // setPowerState(), it sends an ACK to the policy-maker here to
    // indicate that our power state transition is complete.

    _pmPolicyMaker->acknowledgeSetPowerState();
}

//---------------------------------------------------------------------------
// setPowerStateOn()
//
// The policy-maker has told the driver to turn on the device, and this
// function is called by a new kernel thread, while holding the gate in
// the driver's work loop. Exclusive hardware access is assured.
//---------------------------------------------------------------------------

void BroadcomCrystalHD::setPowerStateOn(void)
{
    IOLog("BroadcomCrystalHD::setPowerStateOn\n");
    // Since the driver returned a non-acknowledgement when called at
    // setPowerState(), it sends an ACK to the policy-maker here to
    // indicate that our power state transition is complete.
    crystalhd_resume(&g_adp_info->cmds);

    _pmPowerState = kPowerStateOn;

    _pmPolicyMaker->acknowledgeSetPowerState();

    // With power restored, all clients will be notified that the driver
    // has became "usable". If a client wishes to use the driver, then the
    // driver can expect a call to its enable() method to start things off.
}

