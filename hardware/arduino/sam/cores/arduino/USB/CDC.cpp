/* Copyright (c) 2011, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

/* turbocharged by stimmer 20130202
 *  removed buffering, making code simpler and faster
 *  added multi byte read: size_t Serial_::read(uint8_t *buffer, size_t size)
 *  added LED blink code
 * please test
*/

#include "Arduino.h"
#include "USBAPI.h"
#include "Reset.h"

#ifdef CDC_ENABLED

/* For information purpose only since RTS is not always handled by the terminal application */
#define CDC_LINESTATE_DTR		0x01 // Data Terminal Ready
#define CDC_LINESTATE_RTS		0x02 // Ready to Send

#define CDC_LINESTATE_READY		(CDC_LINESTATE_RTS | CDC_LINESTATE_DTR)

typedef struct
{
	uint32_t	dwDTERate;
	uint8_t		bCharFormat;
	uint8_t 	bParityType;
	uint8_t 	bDataBits;
	uint8_t		lineState;
} LineInfo;

static volatile LineInfo _usbLineInfo = { 
    57600, // dWDTERate
    0x00,  // bCharFormat
    0x00,  // bParityType
    0x08,  // bDataBits
    0x00   // lineState
};

_Pragma("pack(1)")
static const CDCDescriptor _cdcInterface =
{
	D_IAD(0,2,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,1),

	//	CDC communication interface
	D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
	D_CDCCS(CDC_HEADER,0x10,0x01),								// Header (1.10 bcd)
	D_CDCCS(CDC_CALL_MANAGEMENT,1,1),							// Device handles call management (not)
	D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),				// SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
	D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE),	// Communication interface is master, data interface is slave 0
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),USB_ENDPOINT_TYPE_INTERRUPT,0x10, 0x10),

	//	CDC data interface
	D_INTERFACE(CDC_DATA_INTERFACE,2,CDC_DATA_INTERFACE_CLASS,0,0),
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,512,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,512,0)
};
static const CDCDescriptor _cdcOtherInterface =
{
	D_IAD(0,2,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,1),

	//	CDC communication interface
	D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
	D_CDCCS(CDC_HEADER,0x10,0x01),								// Header (1.10 bcd)
	D_CDCCS(CDC_CALL_MANAGEMENT,1,1),							// Device handles call management (not)
	D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),				// SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
	D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE),	// Communication interface is master, data interface is slave 0
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),USB_ENDPOINT_TYPE_INTERRUPT,0x10, 0x10),

	//	CDC data interface
	D_INTERFACE(CDC_DATA_INTERFACE,2,CDC_DATA_INTERFACE_CLASS,0,0),
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,64,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,64,0)
};
_Pragma("pack()")

int WEAK CDC_GetInterface(uint8_t* interfaceNum)
{
	interfaceNum[0] += 2;	// uses 2
	return USBD_SendControl(0,&_cdcInterface,sizeof(_cdcInterface));
}

int WEAK CDC_GetOtherInterface(uint8_t* interfaceNum)
{
	interfaceNum[0] += 2;	// uses 2
	return USBD_SendControl(0,&_cdcOtherInterface,sizeof(_cdcOtherInterface));
}

bool WEAK CDC_Setup(Setup& setup)
{
	uint8_t r = setup.bRequest;
	uint8_t requestType = setup.bmRequestType;

	if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
	{
		if (CDC_GET_LINE_CODING == r)
		{
			USBD_SendControl(0,(void*)&_usbLineInfo,7);
			return true;
		}
	}

	if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
	{
		if (CDC_SET_LINE_CODING == r)
		{
			USBD_RecvControl((void*)&_usbLineInfo,7);
			return true;
		}

		if (CDC_SET_CONTROL_LINE_STATE == r)
		{
			_usbLineInfo.lineState = setup.wValueL;
			// auto-reset into the bootloader is triggered when the port, already
			// open at 1200 bps, is closed.
			if (1200 == _usbLineInfo.dwDTERate)
			{
				// We check DTR state to determine if host port is open (bit 0 of lineState).
				if ((_usbLineInfo.lineState & 0x01) == 0)
					initiateReset(250);
				else
					cancelReset();
			}
			return true;
		}
	}
	return false;
}

void Serial_::begin(uint32_t baud_count)
{
	peeked=0;
	pinMode(72,OUTPUT);
	pinMode(73,OUTPUT);  
}

void Serial_::end(void)
{
}

int Serial_::available(void)
{
	LockEP lock(CDC_RX);
	USB_LED_UPDATE;  
	int r=UDD_FifoByteCount(CDC_RX);
	return r+peeked;	
}

int Serial_::peek(void) // not well tested yet - I don't know of any code which uses peek
{
	LockEP lock(CDC_RX);
	USB_LED_UPDATE;
	if(peeked) return peeked_u8;
	if(!UDD_FifoByteCount(CDC_RX)){
		return -1;
	}
	
	peeked=1;
	peeked_u8=UDD_Recv8(CDC_RX);
	if (!UDD_FifoByteCount(CDC_RX)){
		UDD_ReleaseRX(CDC_RX);
	}
	return peeked_u8;
}

int Serial_::read(void)
{
	LockEP lock(CDC_RX);
  	USB_LED_UPDATE;
	if(peeked){
		peeked=0;
		return peeked_u8;	  
	}
	if (!UDD_FifoByteCount(CDC_RX))
		return -1;
	
	uint8_t c = UDD_Recv8(CDC_RX);
	if (!UDD_FifoByteCount(CDC_RX)){
		UDD_ReleaseRX(CDC_RX);
	}
	return c;
}

size_t Serial_::read(uint8_t *buffer, size_t size)
{
	LockEP lock(CDC_RX);
	USB_LED_UPDATE;

	if(!peeked){
		size=min(size,UDD_FifoByteCount(CDC_RX));
		if(size<=0)return 0;
		UDD_Recv(CDC_RX, buffer, size);
		if (!UDD_FifoByteCount(CDC_RX)){
			UDD_ReleaseRX(CDC_RX);    
		}
		return size;
	}
	
	peeked=0;
	buffer[0]=peeked_u8;
	size=min(size-1,UDD_FifoByteCount(CDC_RX));
	if(size<=0)return 1;
	UDD_Recv(CDC_RX, buffer+1, size);
	if (!UDD_FifoByteCount(CDC_RX)){
		UDD_ReleaseRX(CDC_RX); 
	}
	return size+1;  	
}

void Serial_::flush(void)
{
	USBD_Flush(CDC_TX);
}

size_t Serial_::write(const uint8_t *buffer, size_t size)
{
	/* only try to send bytes if the high-level CDC connection itself
	 is open (not just the pipe) - the OS should set lineState when the port
	 is opened and clear lineState when the port is closed.
	 bytes sent before the user opens the connection or after
	 the connection is closed are lost - just like with a UART. */

	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)
	if (_usbLineInfo.lineState > 0)
	{
		LockEP lock(CDC_TX);
	  
		USB_TX_LED_ON;
		USB_LED_UPDATE;
    
		uint32_t n;
		int r = size;
		int p = 0;
		int c = 0;
		int t = 0;

		while (r)
		{        
			c = min(EPX_SIZE,r);
			t=UDD_Send(CDC_TX, &buffer[p], c);
			r-=t;
			p+=t;			
		}
		return n;
	}
	setWriteError();
	return 0;
}

size_t Serial_::write(uint8_t c) {
	return write(&c, 1);
}

// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial_::operator bool()
{
	// this is here to avoid spurious opening after upload
	if (millis() < 500)
		return false;

	bool result = false;

	if (_usbLineInfo.lineState > 0)
	{
		result = true;
	}

	delay(10);
	return result;
}

Serial_ SerialUSB;

#endif
