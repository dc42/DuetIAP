/* In-Application Programming application for Duet3D platforms
 *
 * This application is first written by RepRapFirmware to the end of the second
 * Flash bank and then started by RepRapFirmware.
 * 
 * Once this program is loaded, it performs in-application programming by
 * reading the new firmware binary from the SD card and replaces the corresponding
 * Flash content sector by sector.
 *
 * This application was written by Christian Hammacher (2016-2019) and is
 * licensed under the terms of the GPL v2.
 */

#include "Core.h"

#if SAM4S
# define IFLASH_ADDR		(IFLASH0_ADDR)
# define IFLASH_PAGE_SIZE	(IFLASH0_PAGE_SIZE)
#elif SAM3XA
# define IFLASH_ADDR		(IFLASH0_ADDR)
# define IFLASH_PAGE_SIZE	(IFLASH1_PAGE_SIZE)
#endif

const size_t baudRate = 115200;						// For USB diagnostics

#if SAM3XA
# ifdef __RADDS__
#  define SERIAL_AUX_DEVICE Serial1
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = { 14, 14 };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[2] = { 87, 77 };
const char * const defaultFwFile = "0:/sys/RepRapFirmware-RADDS.bin";	// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";
# elif defined(__ALLIGATOR__)
#  define SERIAL_AUX_DEVICE Serial1
const size_t NumSdCards = 1;
const Pin SdCardDetectPins[NumSdCards] = { 87 };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin };
const Pin SdSpiCSPins[2] = { 77 };
const char * const defaultFwFile = "0:/sys/RepRapFirmware-Alligator.bin";	// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";
# else
#  define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {NoPin, NoPin};				// Don't use the Card Detect pin on the Duet 085
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {67};
const char * const defaultFwFile = "0:/sys/RepRapFirmware.bin";			// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";
# endif
#endif

#if SAM4E
# define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {53, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {56};
const Pin DiagLedPin = 34;
const char * const defaultFwFile = "0:/sys/DuetWiFiFirmware.bin";		// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet";
#endif

#if SAM4S
# define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {44, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {56};
const Pin DiagLedPin = 62;
const char * const defaultFwFile = "0:/sys/DuetMaestroFirmware.bin";	// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet";
#endif

#if SAME70
# define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;

# ifdef SAME70XPLD

const Pin SdCardDetectPins[NumSdCards] = { PortCPin(16), NoPin };
const Pin DiagLedPin = NoPin;
const char * const defaultFwFile = "0:/sys/SAME70XPLDFirmware.bin";		// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/SAME70XPLD";

# elif !defined(DUET3_V05)

const Pin SdCardDetectPins[NumSdCards] = { PortAPin(6), NoPin };
const Pin DiagLedPin = PortCPin(20);
const char * const defaultFwFile = "0:/sys/Duet3Firmware.bin";			// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet3";

#else

const Pin DiagLedPin = PortCPin(20);
const uint32_t LINUX_XDMAC_TX_CH_NUM = 3;
const uint32_t LINUX_XDMAC_RX_CH_NUM = 4;
const uint8_t DmacChanLinuxTx = 5;				// These two should be
const uint8_t DmacChanLinuxRx = 6;				// kept in sync with RRF!
const uint32_t NvicPrioritySpi = 1;
const Pin LinuxTfrReadyPin = PortEPin(2);

const uint32_t TransferCompleteDelay = 400;								// DCS waits 500ms when the firmware image has been transferred
const uint32_t TransferTimeout = 2000;									// How long to wait before timing out

struct FlashVerifyRequest
{
	uint32_t firmwareLength;
	uint16_t crc16;
	uint16_t dummy;
};

# endif

const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[1] = { NoPin };

const uint32_t iapFirmwareSize = 0x20000;			// 128 KiB max (SAME70 has 128kb flash sectors so we can't erase a smaller amount)
const uint32_t firmwareFlashEnd = 0x004E0000;		// iape70 is designed to work with >= 1Mbyte flash

#else	// not a SAME70 variant

const uint32_t iapFirmwareSize = 0x10000;			// 64 KiB max
const uint32_t firmwareFlashEnd = IFLASH_ADDR + IFLASH_SIZE - iapFirmwareSize;

#endif


// Read and write only 2 KiB of data at once (must be multiple of IFLASH_PAGE_SIZE).
// Unfortunately we cannot increase this value further, because f_read() would mess up data
const size_t blockReadSize = 2048;

const size_t maxRetries = 5;						// Allow 5 retries max if anything goes wrong


enum ProcessState
{
	Initializing,
	UnlockingFlash,
#if SAM4E || SAM4S || SAME70
	ErasingFlash,
#endif
	WritingUpgrade,
	FillingSpace,
	LockingFlash
#ifdef DUET3_V05
	, VerifyingChecksum,
	SendingChecksumOK,
	SendingChecksumError
#endif
};


#ifndef DUET3_V05
void initFilesystem();
void getFirmwareFileName();
void openBinary();
#endif
void writeBinary();
#ifndef DUET3_V05
void closeAndDeleteBinary();
#endif
void Reset(bool success);

void sendUSB(uint32_t ep, const void* d, uint32_t len);
