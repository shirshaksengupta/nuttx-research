#include <stdint.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include "tiva_eeprom.h"


//*****************************************************************************
//
// This macro extracts the array index out of the peripheral number.
//
//*****************************************************************************
#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 28) & 0xf)

//*****************************************************************************
//
// This macro constructs the peripheral bit mask from the peripheral number.
//
//*****************************************************************************
#define SYSCTL_PERIPH_MASK(a)   (((a) & 0xffff) << (((a) & 0x001f0000) >> 16))

#define SYSCTL_RCGCBASE         0x400fe600

#define SYSCTL_SRBASE           0x400fe500

#define SYSCTL_RCGC0            0x400FE100  // Run Mode Clock Gating Control

#define SYSCTL_RCGC1            0x400FE104  // Run Mode Clock Gating Control

#define SYSCTL_RCGC2            0x400FE108  // Run Mode Clock Gating Control

#define SYSCTL_PERIPH_EEPROM0   0xf0005800

#define EEPROM_INIT_ERROR (-1)

#define EEPROM_INIT_OK (0)

//*****************************************************************************
//
// Macros for hardware access, both direct and via the bit-band region.
//
//*****************************************************************************
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x)))
#define HWREGH(x)                                                             \
        (*((volatile unsigned short *)(x)))
#define HWREGB(x)                                                             \
        (*((volatile unsigned char *)(x)))
#define HWREGBITW(x, b)                                                       \
        HWREG(((unsigned long)(x) & 0xF0000000) | 0x02000000 |                \
              (((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITH(x, b)                                                       \
        HWREGH(((unsigned long)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITB(x, b)                                                       \
        HWREGB(((unsigned long)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned long)(x) & 0x000FFFFF) << 5) | ((b) << 2))

//*****************************************************************************
//
// An array that maps the "peripheral set" number (which is stored in the upper
// nibble of the SYSCTL_PERIPH_* defines) to the SYSCTL_RCGC? register that
// controls the run-mode enable for that peripheral.
//
//*****************************************************************************
static const unsigned long g_pulRCGCRegs[] =
{
    SYSCTL_RCGC0,
    SYSCTL_RCGC1,
    SYSCTL_RCGC2
};


//*****************************************************************************
//
//! \addtogroup eeprom_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Useful macros to extract the number of EEPROM blocks available on the target
// device and the total EEPROM storage in bytes from the EESIZE register.
//
//*****************************************************************************
#define BLOCKS_FROM_EESIZE(x) (((x) & EEPROM_EESIZE_BLKCNT_M) >>              \
                               EEPROM_EESIZE_BLKCNT_S)
#define SIZE_FROM_EESIZE(x)   ((((x) & EEPROM_EESIZE_WORDCNT_M) >>            \
                                EEPROM_EESIZE_WORDCNT_S) * 4)

//*****************************************************************************
//
// Useful macro to extract the offset from a linear address.
//
//*****************************************************************************
#define OFFSET_FROM_ADDR(x) (((x) >> 2) & 0x0F)

//*****************************************************************************
//
// The key value required to initiate a mass erase.
//
//*****************************************************************************
#define EEPROM_MASS_ERASE_KEY ((uint32_t)0xE37B << EEPROM_EEDBGME_KEY_S)

#define EEPROMBlockFromAddr(ui32Addr) ((ui32Addr) >> 6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct tiva_dev_s.
 */

struct tiva_dev_s
{
  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int tiva_eeprom_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks);
static ssize_t tiva_eeprom_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t tiva_eeprom_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t tiva_eeprom_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_eeprom_write(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR const uint8_t *buf);
#endif
static int tiva_eeprom_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This structure holds the state of the MTD driver */

static struct tiva_dev_s g_lmdev =
{
  {
    tiva_eeprom_erase,
    tiva_eeprom_bread,
    tiva_eeprom_bwrite,
    tiva_eeprom_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    tiva_eeprom_write,
#endif
    tiva_eeprom_ioctl
  },
  /* Initialization of any other implementation specific data goes here */
};

//*****************************************************************************
//
//! Enables a peripheral.
//!
//! \param ulPeripheral is the peripheral to enable.
//!
//! This function enables peripherals.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! \note It takes five clock cycles after the write to enable a peripheral
//! before the the peripheral is actually enabled.  During this time, attempts
//! to access the peripheral result in a bus fault.  Care should be taken
//! to ensure that the peripheral is not accessed during this brief time
//! period.
//!
//! \return None.
//
//*****************************************************************************

void EEPROMEnable(unsigned long ulPeripheral)
{
 

    //
    // See if the peripheral index is 15, indicating a peripheral that is
    // accessed via the SYSCTL_RCGCperiph registers.
    //
    if((ulPeripheral & 0xf0000000) == 0xf0000000)
    {
        //
        // Enable this peripheral.
        //
        HWREGBITW(SYSCTL_RCGCBASE + ((ulPeripheral & 0xff00) >> 8),
                  ulPeripheral & 0xff) = 1;
    }

    else
    {
        //
        // Enable this peripheral.
        //
        HWREG(g_pulRCGCRegs[SYSCTL_PERIPH_INDEX(ulPeripheral)]) |=
            SYSCTL_PERIPH_MASK(ulPeripheral);
    }
}

//*****************************************************************************
//
//! Performs a software reset of a peripheral.
//!
//! \param ui32Peripheral is the peripheral to reset.
//!
//! This function performs a software reset of the specified peripheral.  An
//! individual peripheral reset signal is asserted for a brief period and then
//! de-asserted, returning the internal state of the peripheral to its reset
//! condition.
//!
//!
//! \return None.
//
//*****************************************************************************

void tiva_EEPROMReset(uint32_t ui32Peripheral)
{
    volatile uint_fast8_t ui8Delay;

    // //
    // // Check the arguments.
    // //
    // ASSERT(_SysCtlPeripheralValid(ui32Peripheral));

    //
    // Put the peripheral into the reset state.
    //
    HWREGBITW(SYSCTL_SRBASE + ((ui32Peripheral & 0xff00) >> 8),
              ui32Peripheral & 0xff) = 1;

    //
    // Delay for a little bit.
    //
    for(ui8Delay = 0; ui8Delay < 16; ui8Delay++)
    {
    }

    //
    // Take the peripheral out of the reset state.
    //
    HWREGBITW(SYSCTL_SRBASE + ((ui32Peripheral & 0xff00) >> 8),
              ui32Peripheral & 0xff) = 0;
}

//*****************************************************************************
//
// Block until the EEPROM peripheral is not busy.
//
//*****************************************************************************
static void tiva_EEPROMWaitForDone(void)
{
    //
    // Is the EEPROM still busy?
    //
    while(HWREG(EEPROM_EEDONE) & EEPROM_EEDONE_WORKING)
    {
        //
        // Spin while EEPROM is busy.
        //
    }
}

//*****************************************************************************
//
//! Performs any necessary recovery in case of power failures during write.
//!
//! This function \b must be called after EEPROMEnable() and before
//! the EEPROM is accessed.  It is used to check for errors in the EEPROM state
//! such as from power failure during a previous write operation.  The function
//! detects these errors and performs as much recovery as possible.
//!
//! If \b EEPROM_INIT_ERROR is returned, the EEPROM was unable to recover its
//! state.  If power is stable when this occurs, this indicates a fatal
//! error and is likely an indication that the EEPROM memory has exceeded its
//! specified lifetime write/erase specification.  If the supply voltage is
//! unstable when this return code is observed, retrying the operation once the
//! voltage is stabilized may clear the error.
//!
//! Failure to call this function after a reset may lead to incorrect operation
//! or permanent data loss if the EEPROM is later written.
//!
//! \return Returns \b EEPROM_INIT_OK if no errors were detected or \b
//! EEPROM_INIT_ERROR if the EEPROM peripheral cannot currently recover from
//! an interrupted write or erase operation.
//
//*****************************************************************************
uint32_t tiva_EEPROMInit(void)
{
    EEPROMEnable(SYSCTL_PERIPH_EEPROM0);

    uint32_t ui32Status;

    //
    // Insert a small delay (6 cycles + call overhead) to guard against the
    // possibility that this function is called immediately after the EEPROM
    // peripheral is enabled.  Without this delay, there is a slight chance
    // that the first EEPROM register read will fault if you are using a
    // compiler with a ridiculously good optimizer!
    //
    tiva_delay(2);

    //
    // Make sure the EEPROM has finished any ongoing processing.
    //3
    tiva_EEPROMWaitForDone();

    //
    // Read the EESUPP register to see if any errors have been reported.
    //
    ui32Status = HWREG(EEPROM_EESUPP);

    //
    // Did an error of some sort occur during initialization?
    //
    if(ui32Status & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY))
    {
        return(EEPROM_INIT_ERROR);
    }

    //
    // Perform a second EEPROM reset.
    //
    tiva_EEPROMReset(SYSCTL_PERIPH_EEPROM0);

    //
    // Wait for the EEPROM to complete its reset processing once again.
    //
    tiva_delay(2);
    tiva_EEPROMWaitForDone();

    //
    // Read EESUPP once again to determine if any error occurred.
    //
    ui32Status = HWREG(EEPROM_EESUPP);

    //
    // Was an error reported following the second reset?
    //
    if(ui32Status & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY))
    {
        return(EEPROM_INIT_ERROR);
    }

    //
    // The EEPROM does not indicate that any error occurred.
    //
    return(EEPROM_INIT_OK);
}

//*****************************************************************************
//
//! Determines the size of the EEPROM.
//!
//! This function returns the size of the EEPROM in bytes.
//!
//! \return Returns the total number of bytes in the EEPROM.
//
//*****************************************************************************
uint32_t tiva_EEPROMSizeGet(void)
{
    //
    // Return the size of the EEPROM in bytes.
    //
    return(SIZE_FROM_EESIZE(HWREG(EEPROM_EESIZE)));
}

//*****************************************************************************
//
//! Determines the number of blocks in the EEPROM.
//!
//! This function may be called to determine the number of blocks in the
//! EEPROM.  Each block is the same size and the number of bytes of storage
//! contained in a block may be determined by dividing the size of the device,
//! obtained via a call to the EEPROMSizeGet() function, by the number of
//! blocks returned by this function.
//!
//! \return Returns the total number of blocks in the device EEPROM.
//
//*****************************************************************************
uint32_t tiva_EEPROMBlockCountGet(void)
{
    //
    // Extract the number of blocks and return it to the caller.
    //
#ifdef EEPROM_SIZE_LIMIT
    //
    // If a size limit has been specified, fake the number of blocks to match.
    //
    return(EEPROM_SIZE_LIMIT / 48);
#else
    //
    // Return the actual number of blocks supported by the hardware.
    //
    return(BLOCKS_FROM_EESIZE(HWREG(EEPROM_EESIZE)));
#endif
}

//*****************************************************************************
//
// This function implements a workaround for a bug in Blizzard rev A silicon.
// It ensures that only the 1KB flash sector containing a given EEPROM address
// is erased if an erase/copy operation is required as a result of a following
// EEPROM write.
//
//*****************************************************************************
static void tiva_EEPROMSectorMaskSet(uint32_t ui32Address)
{
    uint32_t ui32Mask;

    //
    // Determine which page contains the passed EEPROM address.  The 2KB EEPROM
    // is implemented in 16KB of flash with each 1KB sector of flash holding
    // values for 32 consecutive EEPROM words (or 128 bytes).
    //
    ui32Mask = ~(1 << (ui32Address >> 7));

    tiva_delay(10);
    HWREG(0x400FD0FC) = 3;
    tiva_delay(10);
    HWREG(0x400AE2C0) = ui32Mask;
    tiva_delay(10);
    HWREG(0x400FD0FC) = 0;
    tiva_delay(10);
}

//*****************************************************************************
//
// Clear the FSM sector erase mask to ensure that any following main array
// flash erase operations operate as expected.
//
//*****************************************************************************
static void tiva_EEPROMSectorMaskClear(void)
{
    tiva_delay(10);
    HWREG(0x400FD0FC) = 3;
    tiva_delay(10);
    HWREG(0x400AE2C0) = 0;
    tiva_delay(10);
    HWREG(0x400FD0FC) = 0;
    tiva_delay(10);
}

//*****************************************************************************
//
//! Writes data to the EEPROM.
//!
//! \param pui32Data points to the first word of data to write to the EEPROM.
//! \param ui32Address defines the byte address within the EEPROM that the data
//! is to be written to.  This value must be a multiple of 4.
//! \param ui32Count defines the number of bytes of data that is to be written.
//! This value must be a multiple of 4.
//!
//! This function may be called to write data into the EEPROM at a given
//! word-aligned address.  The call is synchronous and returns only after
//! all data has been written or an error occurs.
//!
//! \return Returns 0 on success or non-zero values on failure.  Failure codes
//! are logical OR combinations of \b EEPROM_RC_WRBUSY, \b EEPROM_RC_NOPERM,
//! \b EEPROM_RC_WKCOPY, \b EEPROM_RC_WKERASE, and \b EEPROM_RC_WORKING.
//
//*****************************************************************************

static ssize_t tiva_eeprom_write(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR const uint8_t *buf)
{
    uint32_t ui32Status;

   	FAR uint32_t *pui32Data = (uint32_t *)buf;

    do
    {
        //
        // Read the status.
        //
        ui32Status = HWREG(EEPROM_EEDONE);
    }
    while(ui32Status & EEPROM_EEDONE_WORKING);

    //
    // Set the block and offset appropriately to program the first word.
    //
    HWREG(EEPROM_EEBLOCK) = EEPROMBlockFromAddr(offset);
    HWREG(EEPROM_EEOFFSET) = OFFSET_FROM_ADDR(offset);

    //
    // Convert the byte count to a word count.
    //
    nbytes /= 4;

    //
    // Write each word in turn.
    //
    while(nbytes)
    {
       

        //
        // Write the next word through the autoincrementing register.
        //
        HWREG(EEPROM_EERDWRINC) = *pui32Data;

        //
        // Wait a few cycles.  In some cases, the WRBUSY bit is not set
        // immediately and this prevents us from dropping through the polling
        // loop before the bit is set.
        //
        tiva_delay(10);

        //
        // Wait for the write to complete.
        //
        do
        {
            //
            // Read the status.
            //
            ui32Status = HWREG(EEPROM_EEDONE);
        }
        while(ui32Status & EEPROM_EEDONE_WORKING);

        //
        // Make sure we completed the write without errors.  Note that we
        // must check this per-word because write permission can be set per
        // block resulting in only a section of the write not being performed.
        //
        if(ui32Status & EEPROM_EEDONE_NOPERM)
        {
           
            return(ui32Status);
        }

        //
        // Move on to the next word.
        //
        pui32Data++;
        nbytes--;

        //
        // Do we need to move to the next block?  This is the case if the
        // offset register has just wrapped back to 0.  Note that we only
        // write the block register if we have more data to read.  If this
        // register is written, the hardware expects a read or write operation
        // next.  If a mass erase is requested instead, the mass erase will
        // fail.
        //
        if(nbytes && (HWREG(EEPROM_EEOFFSET) == 0))
        {
            HWREG(EEPROM_EEBLOCK) += 1;
        }
    }

    //
    // Clear the sector protection bits to prevent possible problems when
    // programming the main flash array later.
    //
    // if(CLASS_IS_TM4C123 && REVISION_IS_A0)
    // {
    //     tiva_EEPROMSectorMaskClear();
    // }

    //
    // Return the current status to the caller.
    //
    return(HWREG(EEPROM_EEDONE));
}


//*****************************************************************************
//
//! Reads data from the EEPROM.
//!
//! \param pui32Data is a pointer to storage for the data read from the EEPROM.
//! This pointer must point to at least \e ui32Count bytes of available memory.
//! \param ui32Address is the byte address within the EEPROM from which data is
//! to be read.  This value must be a multiple of 4.
//! \param ui32Count is the number of bytes of data to read from the EEPROM.
//! This value must be a multiple of 4.
//!
//! This function may be called to read a number of words of data from a
//! word-aligned address within the EEPROM.  Data read is copied into the
//! buffer pointed to by the \e pui32Data parameter.
//!
//! \return None.
//
//*****************************************************************************

static ssize_t tiva_eeprom_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buf)
{

	FAR uint32_t *pui32Data = (uint32_t *)buf;

    //
    // Set the block and offset appropriately to read the first word.
    //
    HWREG(EEPROM_EEBLOCK) = EEPROMBlockFromAddr(offset);
    HWREG(EEPROM_EEOFFSET) = OFFSET_FROM_ADDR(offset);

    //
    // Convert the byte count to a word count.
    //
    nbytes /= 4;

    //
    // Read each word in turn.
    //
    while(nbytes)
    {
        //
        // Read the next word through the autoincrementing register.
        //
        *pui32Data = HWREG(EEPROM_EERDWRINC);

        //
        // Move on to the next word.
        //
        pui32Data++;
        nbytes--;

        //
        // Do we need to move to the next block?  This is the case if the
        // offset register has just wrapped back to 0.  Note that we only
        // write the block register if we have more data to read.  If this
        // register is written, the hardware expects a read or write operation
        // next.  If a mass erase is requested instead, the mass erase will
        // fail.
        //
        if(nbytes && (HWREG(EEPROM_EEOFFSET) == 0))
        {
            HWREG(EEPROM_EEBLOCK) += 1;
        }
    }
}

static int tiva_eeprom_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
	return (-1);
}

static ssize_t tiva_eeprom_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buf)
{
	return (-1);
}

ssize_t tiva_eeprom_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buf)
{
	return (-1);
}

static int tiva_eeprom_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	return (-1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *up_eeprominitialize(void)
{
  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)&g_lmdev;
}
