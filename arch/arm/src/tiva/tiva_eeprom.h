#ifndef __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H
#define __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H

//*****************************************************************************
//
// The following are defines for the EEPROM register offsets.
//
//*****************************************************************************
#define EEPROM_EESIZE           0x400AF000  // EEPROM Size Information
#define EEPROM_EEBLOCK          0x400AF004  // EEPROM Current Block
#define EEPROM_EEOFFSET         0x400AF008  // EEPROM Current Offset
#define EEPROM_EERDWR           0x400AF010  // EEPROM Read-Write
#define EEPROM_EERDWRINC        0x400AF014  // EEPROM Read-Write with Increment
#define EEPROM_EEDONE           0x400AF018  // EEPROM Done Status
#define EEPROM_EESUPP           0x400AF01C  // EEPROM Support Control and
                                            // Status
#define EEPROM_EEUNLOCK         0x400AF020  // EEPROM Unlock
#define EEPROM_EEPROT           0x400AF030  // EEPROM Protection
#define EEPROM_EEPASS0          0x400AF034  // EEPROM Password
#define EEPROM_EEPASS1          0x400AF038  // EEPROM Password
#define EEPROM_EEPASS2          0x400AF03C  // EEPROM Password
#define EEPROM_EEINT            0x400AF040  // EEPROM Interrupt
#define EEPROM_EEHIDE0          0x400AF050  // EEPROM Block Hide 0
#define EEPROM_EEHIDE           0x400AF050  // EEPROM Block Hide
#define EEPROM_EEHIDE1          0x400AF054  // EEPROM Block Hide 1
#define EEPROM_EEHIDE2          0x400AF058  // EEPROM Block Hide 2
#define EEPROM_EEDBGME          0x400AF080  // EEPROM Debug Mass Erase
#define EEPROM_PP               0x400AFFC0  // EEPROM Peripheral Properties

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESIZE register.
//
//*****************************************************************************
#define EEPROM_EESIZE_WORDCNT_M 0x0000FFFF  // Number of 32-Bit Words
#define EEPROM_EESIZE_BLKCNT_M  0x07FF0000  // Number of 16-Word Blocks
#define EEPROM_EESIZE_WORDCNT_S 0
#define EEPROM_EESIZE_BLKCNT_S  16

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEBLOCK register.
//
//*****************************************************************************
#define EEPROM_EEBLOCK_BLOCK_M  0x0000FFFF  // Current Block
#define EEPROM_EEBLOCK_BLOCK_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEOFFSET
// register.
//
//*****************************************************************************
#define EEPROM_EEOFFSET_OFFSET_M                                              \
                                0x0000000F  // Current Address Offset
#define EEPROM_EEOFFSET_OFFSET_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWR register.
//
//*****************************************************************************
#define EEPROM_EERDWR_VALUE_M   0xFFFFFFFF  // EEPROM Read or Write Data
#define EEPROM_EERDWR_VALUE_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWRINC
// register.
//
//*****************************************************************************
#define EEPROM_EERDWRINC_VALUE_M                                              \
                                0xFFFFFFFF  // EEPROM Read or Write Data with
                                            // Increment
#define EEPROM_EERDWRINC_VALUE_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDONE register.
//
//*****************************************************************************
#define EEPROM_EEDONE_WORKING   0x00000001  // EEPROM Working
#define EEPROM_EEDONE_WKERASE   0x00000004  // Working on an Erase
#define EEPROM_EEDONE_WKCOPY    0x00000008  // Working on a Copy
#define EEPROM_EEDONE_NOPERM    0x00000010  // Write Without Permission
#define EEPROM_EEDONE_WRBUSY    0x00000020  // Write Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESUPP register.
//
//*****************************************************************************
#define EEPROM_EESUPP_ERETRY    0x00000004  // Erase Must Be Retried
#define EEPROM_EESUPP_PRETRY    0x00000008  // Programming Must Be Retried

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEUNLOCK
// register.
//
//*****************************************************************************
#define EEPROM_EEUNLOCK_UNLOCK_M                                              \
                                0xFFFFFFFF  // EEPROM Unlock

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPROT register.
//
//*****************************************************************************
#define EEPROM_EEPROT_PROT_M    0x00000007  // Protection Control
#define EEPROM_EEPROT_PROT_RWNPW                                              \
                                0x00000000  // This setting is the default. If
                                            // there is no password, the block
                                            // is not protected and is readable
                                            // and writable
#define EEPROM_EEPROT_PROT_RWPW 0x00000001  // If there is a password, the
                                            // block is readable or writable
                                            // only when unlocked
#define EEPROM_EEPROT_PROT_RONPW                                              \
                                0x00000002  // If there is no password, the
                                            // block is readable, not writable
#define EEPROM_EEPROT_ACC       0x00000008  // Access Control

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS0 register.
//
//*****************************************************************************
#define EEPROM_EEPASS0_PASS_M   0xFFFFFFFF  // Password
#define EEPROM_EEPASS0_PASS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS1 register.
//
//*****************************************************************************
#define EEPROM_EEPASS1_PASS_M   0xFFFFFFFF  // Password
#define EEPROM_EEPASS1_PASS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS2 register.
//
//*****************************************************************************
#define EEPROM_EEPASS2_PASS_M   0xFFFFFFFF  // Password
#define EEPROM_EEPASS2_PASS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEINT register.
//
//*****************************************************************************
#define EEPROM_EEINT_INT        0x00000001  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE0 register.
//
//*****************************************************************************
#define EEPROM_EEHIDE0_HN_M     0xFFFFFFFE  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE register.
//
//*****************************************************************************
#define EEPROM_EEHIDE_HN_M      0xFFFFFFFE  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE1 register.
//
//*****************************************************************************
#define EEPROM_EEHIDE1_HN_M     0xFFFFFFFF  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE2 register.
//
//*****************************************************************************
#define EEPROM_EEHIDE2_HN_M     0xFFFFFFFF  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDBGME register.
//
//*****************************************************************************
#define EEPROM_EEDBGME_ME       0x00000001  // Mass Erase
#define EEPROM_EEDBGME_KEY_M    0xFFFF0000  // Erase Key
#define EEPROM_EEDBGME_KEY_S    16

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_PP register.
//
//*****************************************************************************
#define EEPROM_PP_SIZE_M        0x0000FFFF  // EEPROM Size
#define EEPROM_PP_SIZE_64       0x00000000  // 64 bytes of EEPROM
#define EEPROM_PP_SIZE_128      0x00000001  // 128 bytes of EEPROM
#define EEPROM_PP_SIZE_256      0x00000003  // 256 bytes of EEPROM
#define EEPROM_PP_SIZE_512      0x00000007  // 512 bytes of EEPROM
#define EEPROM_PP_SIZE_1K       0x0000000F  // 1 KB of EEPROM
#define EEPROM_PP_SIZE_2K       0x0000001F  // 2 KB of EEPROM
#define EEPROM_PP_SIZE_3K       0x0000003F  // 3 KB of EEPROM
#define EEPROM_PP_SIZE_4K       0x0000007F  // 4 KB of EEPROM
#define EEPROM_PP_SIZE_5K       0x000000FF  // 5 KB of EEPROM
#define EEPROM_PP_SIZE_6K       0x000001FF  // 6 KB of EEPROM
#define EEPROM_PP_SIZE_S        0

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
uint32_t tiva_EEPROMInit(void);

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_EEPROM_H */