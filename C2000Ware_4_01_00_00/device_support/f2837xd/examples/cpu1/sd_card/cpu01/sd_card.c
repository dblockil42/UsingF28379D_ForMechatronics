//###########################################################################
//
// FILE:   sd_card.c
//
// TITLE:  Example program for reading files from an SD card.
//
//! \addtogroup cpu01_example_list
//! <h1>SD card using FAT file system (sd_card)</h1>
//!
//! This example application demonstrates reading a file system from
//! an SD card.  It makes use of FatFs, a FAT file system driver.
//!
//! For additional details about FatFs, see the following site:
//! http://elm-chan.org/fsw/ff/00index_e.html
//!
//! The application may be operated via a serial terminal attached to
//! UART0. The RS232 communication parameters should be set to 115,200 bits
//! per second, and 8-n-1 mode.  When the program is started a message will be
//! printed to the terminal.  Type ``help'' for command help.
//!
//
//###########################################################################
//
// $Release Date: $
// $Copyright:
// Copyright (C) 2013-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "third_party/fatfs/src/ff.h"
#include "third_party/fatfs/src/diskio.h"

//
// Defines
//
#define PATH_BUF_SIZE   80     // Defines the size of the buffers that hold the
                               // path, or temporary data from the SD card.
                               // There are two buffers allocated of this size.
                               // The buffer size must be large enough to hold
                               // the longest expected full path name,
                               // including the file name, and a trailing null
                               // character.
#define CMD_BUF_SIZE    64     // Defines the size of the buffer that holds
                               // the command line.
#define FRESULT_ENTRY(f)        { (f), (# f) } // A macro to make it easy to
                                               // add result codes to the table
#define NAME_TOO_LONG_ERROR     1              // Error reasons returned
#define OPENDIR_ERROR           2              // by ChangeDirectory().
#define NUM_FRESULT_CODES (sizeof(g_sFresultStrings) / sizeof(tFresultString))
#define TICKS_PER_SECOND        100
#define NUM_LIST_STRINGS        48
#define MAX_FILENAME_STRING_LEN (4 + 8 + 1 + 3 + 1)
#define NUM_STATUS_STRINGS      6
#define MAX_STATUS_STRING_LEN   (36 + 1)

//
// Globals
//
static char g_cCwdBuf[PATH_BUF_SIZE] = "/";  // This buffer holds the full path
                                             // to the current working
                                             // directory. Initially it is
                                             // root ("/").
static char g_cTmpBuf[PATH_BUF_SIZE];        // A temporary data buffer used
                                             // when manipulating file paths,or
                                             // reading data from the SD card.
static char g_cCmdBuf[CMD_BUF_SIZE];         // The buffer that holds the
                                             // command line.
static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;

//
// A structure that holds a mapping between an FRESULT numerical code,
// and a string representation.  FRESULT codes are returned from the FatFs
// FAT file system driver.
//
typedef struct
{
    FRESULT fresult;
    char *pcResultStr;
}
tFresultString;

//
// A table that holds a mapping between the numerical FRESULT code and
// it's name as a string.  This is used for looking up error codes for
// printing to the console.
//
tFresultString g_sFresultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_RW_ERROR),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_MKFS_ABORTED)
};

const char *g_ppcDirListStrings[NUM_LIST_STRINGS]; // Storage for the filename
                                                   // listbox widget string
                                                   // table.

//
// Storage for the names of the files in the current directory. Filenames
// are stored in format "(D) filename.ext" for directories or
// "(F) filename.ext" for files.
//
char g_pcFilenames[NUM_LIST_STRINGS][MAX_FILENAME_STRING_LEN];

//
// Storage for the strings which appear in the status box at the bottom of the
// display.
//
char g_pcStatus[NUM_STATUS_STRINGS][MAX_STATUS_STRING_LEN];

//
// Storage for the status listbox widget string table.
//
const char *g_ppcStatusStrings[NUM_STATUS_STRINGS] =
{
    g_pcStatus[0],
    g_pcStatus[1],
    g_pcStatus[2],
    g_pcStatus[3],
    g_pcStatus[4],
    g_pcStatus[5]
};
unsigned long g_ulStatusStringIndex = 0;

//
// Forward declarations for functions called by the widgets used in the user
// interface.
//
static FRESULT ChangeToDirectory(char *pcDirectory, unsigned long *pulReason);
static const char *StringFromFresult(FRESULT fresult);

//
// Function Prototypes
//
extern void UARTStdioIntHandler(void);

//
// StringFromFresult - This function returns a string representation of an
//                     error code that was returned from a function call to
//                     FatFs.  It can be used for printing human readable
//                     error messages.
//
static const char *
StringFromFresult(FRESULT fresult)
{
    unsigned int uIdx;

    //
    // Enter a loop to search the error code table for a matching
    // error code.
    //
    for(uIdx = 0; uIdx < NUM_FRESULT_CODES; uIdx++)
    {
        //
        // If a match is found, then return the string name of the
        // error code.
        //
        if(g_sFresultStrings[uIdx].fresult == fresult)
        {
            return(g_sFresultStrings[uIdx].pcResultStr);
        }
    }

    //
    // At this point no matching code was found, so return a
    // string indicating unknown error.
    //
    return("UNKNOWN ERROR CODE");
}

//
// SysTickHandler - This is the handler for this SysTick interrupt.  FatFs
//                  requires a timer tick every 10 ms for internal timing
//                  purposes.
//
__interrupt void
SysTickHandler(void)
{
    //
    // Call the FatFs tick timer.
    //
    disk_timerproc();
    PieCtrlRegs.PIEACK.all |= 1;
}

//
// Cmd_ls - This function implements the "ls" command.  It opens the current
//          directory and enumerates through the contents, and prints a line
//          for each item it finds.  It shows details such as file attributes,
//          time and date, and the file size, along with the name.  It shows a
//          summary of file sizes at the end along with free space.
//
int
Cmd_ls(int argc, char *argv[])
{
    unsigned long ulTotalSize, ulItemCount, ulFileCount, ulDirCount;
    FRESULT fresult;
    FATFS *pFatFs;

    //
    // Open the current directory for access.
    //
    fresult = f_opendir(&g_sDirObject, g_cCwdBuf);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    ulTotalSize = 0;
    ulFileCount = 0;
    ulDirCount = 0;
    ulItemCount = 0;

    //
    // Give an extra blank line before the listing.
    //
    UARTprintf("\n");

    //
    // Enter loop to enumerate through all directory entries.
    //
    for(;;)
    {
        //
        // Read an entry from the directory.
        //
        fresult = f_readdir(&g_sDirObject, &g_sFileInfo);

        //
        // Check for error and return if there is a problem.
        //
        if(fresult != FR_OK)
        {
            return(fresult);
        }

        //
        // If the file name is blank, then this is the end of the
        // listing.
        //
        if(!g_sFileInfo.fname[0])
        {
            break;
        }

        //
        // Print the entry information on a single line with formatting
        // to show the attributes, date, time, size, and name.
        //
        UARTprintf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9u  %s\n",
                   (g_sFileInfo.fattrib & AM_DIR) ? (uint32_t)'D' : (uint32_t)'-',
                   (g_sFileInfo.fattrib & AM_RDO) ? (uint32_t)'R' : (uint32_t)'-',
                   (g_sFileInfo.fattrib & AM_HID) ? (uint32_t)'H' : (uint32_t)'-',
                   (g_sFileInfo.fattrib & AM_SYS) ? (uint32_t)'S' : (uint32_t)'-',
                   (g_sFileInfo.fattrib & AM_ARC) ? (uint32_t)'A' : (uint32_t)'-',
                   (uint32_t)((g_sFileInfo.fdate >> 9) + 1980),
                   (uint32_t)((g_sFileInfo.fdate >> 5) & 15),
                   (uint32_t)(g_sFileInfo.fdate & 31),
                   (uint32_t)((g_sFileInfo.ftime >> 11)),
                   (uint32_t)((g_sFileInfo.ftime >> 5) & 63),
                   (uint32_t)(g_sFileInfo.fsize),
                   g_sFileInfo.fname);

        //
        // Add the information as a line in the listbox widget.
        //
        if(ulItemCount < NUM_LIST_STRINGS)
        {
            usprintf(g_pcFilenames[ulItemCount], "(%c) %12s",
                     (g_sFileInfo.fattrib & AM_DIR) ? 'D' : 'F',
                     g_sFileInfo.fname);
        }

        //
        // If the attribute is directory, then increment the directory count.
        //
        if(g_sFileInfo.fattrib & AM_DIR)
        {
            ulDirCount++;
        }
        //
        // Otherwise, it is a file.  Increment the file count, and
        // add in the file size to the total.
        //
        else
        {
            ulFileCount++;
            ulTotalSize += g_sFileInfo.fsize;
        }

        //
        // Move to the next entry in the item array we use to populate the
        // list box.
        //
        ulItemCount++;

        //
        // Wait for the UART transmit buffer to empty.
        //
//        UARTFlushTx(false);
    }

    //
    // Print summary lines showing the file, dir, and size totals.
    //
    UARTprintf("\n%4u File(s),%10u bytes total\n%4u Dir(s)",
               ulFileCount, ulTotalSize, ulDirCount);

    //
    // Get the free space.
    //
    fresult = f_getfree("/", &ulTotalSize, &pFatFs);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Display the amount of free space that was calculated.
    //
    UARTprintf(", %10uK bytes free\n", ulTotalSize * pFatFs->sects_clust / 2);

    //
    // Wait for the UART transmit buffer to empty.
    //
//    UARTFlushTx(false);

    //
    // Made it to here, return with no errors.
    //
    return(0);
}

//
// ChangeToDirectory - This function implements the "cd" command.  It takes an
//                     argument that specifies the directory to make the
//                     current working directory. Path separators must use a
//                     forward slash "/".  The argument to cd can be one of the
//                     following:
//                     * root ("/")
//                     * a fully specified path ("/my/path/to/mydir")
//                     * a single directory name that is in the current
//                       directory ("mydir")
//                     * parent directory ("..")
//                     It does not understand relative paths, so don't try
//                     something like this: ("../my/new/path")
//                     Once the new directory is specified, it attempts to open
//                     the directory to make sure it exists.  If the new path
//                     is opened successfully, then the current working
//                     directory (cwd) is changed to the new path. In cases of
//                     error, the pulReason parameter will be written with one
//                     of the following values:
//
static FRESULT
ChangeToDirectory(char *pcDirectory, unsigned long *pulReason)
{
    unsigned int uIdx;
    FRESULT fresult;

    //
    // Copy the current working path into a temporary buffer so
    // it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If the first character is /, then this is a fully specified
    // path, and it should just be used as-is.
    //
    if(pcDirectory[0] == '/')
    {
        //
        // Make sure the new path is not bigger than the cwd buffer.
        //
        if(strlen(pcDirectory) + 1 > sizeof(g_cCwdBuf))
        {
            *pulReason = NAME_TOO_LONG_ERROR;
            return(FR_OK);
        }
        //
        // If the new path name (in argv[1])  is not too long, then
        // copy it into the temporary buffer so it can be checked.
        //
        else
        {
            strncpy(g_cTmpBuf, pcDirectory, sizeof(g_cTmpBuf));
        }
    }
    //
    // If the argument is .. then attempt to remove the lowest level
    // on the CWD.
    //
    else if(!strcmp(pcDirectory, ".."))
    {
        //
        // Get the index to the last character in the current path.
        //
        uIdx = strlen(g_cTmpBuf) - 1;

        //
        // Back up from the end of the path name until a separator (/)
        // is found, or until we bump up to the start of the path.
        //
        while((g_cTmpBuf[uIdx] != '/') && (uIdx > 1))
        {
            //
            // Back up one character.
            //
            uIdx--;
        }

        //
        // Now we are either at the lowest level separator in the
        // current path, or at the beginning of the string (root).
        // So set the new end of string here, effectively removing
        // that last part of the path.
        //
        g_cTmpBuf[uIdx] = 0;
    }
    //
    // Otherwise this is just a normal path name from the current
    // directory, and it needs to be appended to the current path.
    //
    else
    {
        //
        // Test to make sure that when the new additional path is
        // added on to the current path, there is room in the buffer
        // for the full new path.  It needs to include a new separator,
        // and a trailing null character.
        //
        if(strlen(g_cTmpBuf) + strlen(pcDirectory) + 1 + 1 > sizeof(g_cCwdBuf))
        {
            *pulReason = NAME_TOO_LONG_ERROR;
            return(FR_INVALID_OBJECT);
        }
        //
        // The new path is okay, so add the separator and then append
        // the new directory to the path.
        //
        else
        {
            //
            // If not already at the root level, then append a /
            //
            if(strcmp(g_cTmpBuf, "/"))
            {
                strcat(g_cTmpBuf, "/");
            }

            //
            // Append the new directory to the path.
            //
            strcat(g_cTmpBuf, pcDirectory);
        }
    }

    //
    // At this point, a candidate new directory path is in chTmpBuf.
    // Try to open it to make sure it is valid.
    //
    fresult = f_opendir(&g_sDirObject, g_cTmpBuf);

    //
    // If it cant be opened, then it is a bad path.  Inform
    // user and return.
    //
    if(fresult != FR_OK)
    {
        *pulReason = OPENDIR_ERROR;
        return(fresult);
    }
    //
    // Otherwise, it is a valid new path, so copy it into the CWD and update
    // the screen.
    //
    else
    {
        strncpy(g_cCwdBuf, g_cTmpBuf, sizeof(g_cCwdBuf));
    }

    //
    // Return success.
    //
    return(FR_OK);
}

//
// Cmd_cd - This function implements the "cd" command.  It takes an argument
//          that specifies the directory to make the current working directory.
//          Path separators must use a forward slash "/".  The argument to cd
//          can be one of the following:
//          * root ("/")
//          * a fully specified path ("/my/path/to/mydir")
//          * a single directory name that is in the current directory("mydir")
//          * parent directory ("..")
//          It does not understand relative paths, so dont try something like
//          this: ("../my/new/path")
//          Once the new directory is specified, it attempts to open the
//          directory to make sure it exists.  If the new path is opened
//          successfully, then the current working directory (cwd) is changed
//          to the new path.
//
int
Cmd_cd(int argc, char *argv[])
{
    unsigned long ulReason;
    FRESULT fresult;

    //
    // Try to change to the directory provided on the command line.
    //
    fresult = ChangeToDirectory(argv[1], &ulReason);

    //
    // If an error was reported, try to offer some helpful information.
    //
    if(fresult != FR_OK)
    {
        switch(ulReason)
        {
            case OPENDIR_ERROR:
                UARTprintf("Error opening new directory.\n");
                break;

            case NAME_TOO_LONG_ERROR:
                UARTprintf("Resulting path name is too long.\n");
                break;

            default:
                UARTprintf("An unrecognized error was reported.\n");
                break;
        }
    }

    //
    // Return the appropriate error code.
    //
    return(fresult);
}

//
// Cmd_pwd - This function implements the "pwd" command.  It simply prints the
//           current working directory.
//
int
Cmd_pwd(int argc, char *argv[])
{
    //
    // Print the CWD to the console.
    //
    UARTprintf("%s\n", g_cCwdBuf);

    //
    // Wait for the UART transmit buffer to empty.
    //
//    UARTFlushTx(false);

    //
    // Return success.
    //
    return(0);
}

//
// Cmd_cat - This function implements the "cat" command.  It reads the contents
//           of a file and prints it to the console.  This should only be used
//           on text files.  If it is used on a binary file, then a bunch of
//           garbage is likely to printed on the console.
//
int
Cmd_cat(int argc, char *argv[])
{
    FRESULT fresult;
    unsigned short usBytesRead;

    //
    // First, check to make sure that the current path (CWD), plus
    // the file name, plus a separator and trailing null, will all
    // fit in the temporary buffer that will be used to hold the
    // file name.  The file name must be fully specified, with path,
    // to FatFs.
    //
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        UARTprintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    //
    // Now finally, append the file name to result in a fully specified file.
    //
    strcat(g_cTmpBuf, argv[1]);

    //
    // Open the file for reading.
    //
    fresult = f_open(&g_sFileObject, g_cTmpBuf, FA_READ);

    //
    // If there was some problem opening the file, then return
    // an error.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Enter a loop to repeatedly read data from the file and display it,
    // until the end of the file is reached.
    //
    do
    {
        //
        // Read a block of data from the file.  Read as much as can fit
        // in the temporary buffer, including a space for the trailing null.
        //
        fresult = f_read(&g_sFileObject, g_cTmpBuf, sizeof(g_cTmpBuf) - 1,
                         &usBytesRead);

        //
        // If there was an error reading, then print a newline and
        // return the error to the user.
        //
        if(fresult != FR_OK)
        {
            UARTprintf("\n");
            return(fresult);
        }

        //
        // Null terminate the last block that was read to make it a
        // null terminated string that can be used with printf.
        //
        g_cTmpBuf[usBytesRead] = 0;

        //
        // Print the last chunk of the file that was received.
        //
        UARTprintf("%s", g_cTmpBuf);

        //
        // Continue reading until less than the full number of bytes are
        // read.  That means the end of the buffer was reached.
        //
    }
    while(usBytesRead == sizeof(g_cTmpBuf) - 1);

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "write" command.  It writes the contents of the
// second argument into the file specified by the first argument. If the file
// does not exist, it creates a new file with the given file name.
//
//*****************************************************************************
int
Cmd_write(int argc, char *argv[])
{
    FRESULT fresult;
    unsigned short usBytesWritten;
    unsigned int i = 2;
    char writeBuff[CMD_BUF_SIZE] = {0};

    //
    // First, check to make sure that the current path (CWD), plus the file
    // name, plus a separator and trailing null, will all fit in the temporary
    // buffer that will be used to hold the file name.  The file name must be
    // fully specified, with path, to FatFs.
    //
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        UARTprintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    //
    // Append the file name to make a complete file path.
    //
    strcat(g_cTmpBuf, argv[1]);

    //
    // Open the file for writing.
    //
    fresult = f_open(&g_sFileObject, g_cTmpBuf, FA_WRITE | FA_CREATE_ALWAYS);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Creates the string to be written to a file.
    //
	while(i < argc)
	{
		strcat(writeBuff, argv[i]);

		//
		// Adds a space if necessary.
		//
		if(i < argc-1)
		{
			strcat(writeBuff, " ");
		}
		i++;
	}

    //
    // Writes the string into the given file in arg[1]. if file does not exist,
    // it creates a new file.
    //
    fresult = f_write(&g_sFileObject, writeBuff, CMD_BUF_SIZE-1, &usBytesWritten);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }
	
	//
	// Close the file after done with it.
	//
	f_close(&g_sFileObject);

    return(0);
}

//*****************************************************************************
//
// This function implements the "mkdir" command.  It creates a new directory
// with the name given in the argument.
//
//*****************************************************************************
int
Cmd_mkdir(int argc, char *argv[])
{
    FRESULT fresult;
    //
    // First, check to make sure that the current path (CWD), plus the file
    // name, plus a separator and trailing null, will all fit in the temporary
    // buffer that will be used to hold the file name.  The file name must be
    // fully specified, with path, to FatFs.
    //
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        UARTprintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    //
    // Append the directory name to make a complete file path to the new
    // directory.
    //
    strcat(g_cTmpBuf, argv[1]);

    //
    // Make a directory with the new file path.
    //
    fresult = f_mkdir(g_cTmpBuf);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    return(0);
}

//*****************************************************************************
//
// This function implements the "rm" command.  It removes the file or
// subdirectory specified by the argument.
//
//*****************************************************************************
int
Cmd_rm(int argc, char *argv[])
{
    FRESULT fresult;

    //
    // First, check to make sure that the current path (CWD), plus the file
    // name, plus a separator and trailing null, will all fit in the temporary
    // buffer that will be used to hold the file name.  The file name must be
    // fully specified, with path, to FatFs.
    //
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        UARTprintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    //
    // Append the file/sub-directory name to result in a fully specified path.
    //
    strcat(g_cTmpBuf, argv[1]);

    //
    // Unlink the file or sub-directory to be removed.
    //
    fresult = f_unlink(g_cTmpBuf);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Return success.
    //
    return(0);
}

//
// Cmd_help - This function implements the "help" command.  It prints a simple
//            list of the available commands with a brief description.
//
int
Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("\nAvailable commands\n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The
    // end of the table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

//
// This is the table that holds the command names, implementing functions,
// and brief description.
//
tCmdLineEntry g_psCmdTable[] =
{
    { "help",   Cmd_help,      " : Display list of commands" },
    { "h",      Cmd_help,   "    : alias for help" },
    { "?",      Cmd_help,   "    : alias for help" },
    { "ls",     Cmd_ls,      "   : Display list of files" },
    { "chdir",  Cmd_cd,         ": Change directory" },
    { "cd",     Cmd_cd,      "   : alias for chdir" },
    { "pwd",    Cmd_pwd,      "  : Show current working directory" },
    { "cat",    Cmd_cat,      "  : Show contents of a text file" },
    { "write",  Cmd_write,      ": writes to a file. Creates one if file doesn't exist"},
    { "mkdir",  Cmd_mkdir,      ": makes a new file directory"},
    { "rm",     Cmd_rm,      "   : removes a file or directory"},
    { 0, 0, 0 }
};

//
// __error__ - The error routine that is called if the driver library
//             encounters an error.
//
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//
// ConfigureUART - Configure the UART and its pins.  This must be called
//                 before UARTprintf().
//
void
ConfigureUART(void)
{
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SCI1);

    //
    // Configure GPIO Pins for UART mode.
    //
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
    EDIS;

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, SysCtlLowSpeedClockGet(SYSTEM_CLOCK_SPEED));
}

//
// Main - It performs initialization, then runs a command processing loop to
//        read commands from the console.
//
int
main(void)
{
    int nStatus;
    FRESULT fresult;

    //
    // Initialize System Control
    //
    InitSysCtrl();

#ifdef _FLASH
//
// Copy time critical code and Flash setup code to RAM
// This includes the following functions:  InitFlash();
// The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
// symbols are created by the linker. Refer to the device .cmd file.
//
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

//
// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
//
    InitFlash();
#endif

    //
    // Initialize interrupt controller and vector table
    //
    InitPieCtrl();
    InitPieVectTable();

    //
    // Set the system tick to fire 100 times per second.
    //
    SysTickInit();
    SysTickPeriodSet(SysCtlClockGet(SYSTEM_CLOCK_SPEED) / 100);
    SysTickIntRegister(SysTickHandler);
    SysTickIntEnable();
    SysTickEnable();

    //
    // Enable Interrupts
    //
    IntMasterEnable();

    //
    // Configure UART0 for debug output.
    //
    ConfigureUART();

    //
    // Print hello message to user.
    //
    UARTprintf("\n\nSD Card Example Program\n");
    UARTprintf("Type \'help\' for help.\n");

    //
    // Mount the file system, using logical disk 0.
    //
    fresult = f_mount(0, &g_sFatFs);
    if(fresult != FR_OK)
    {
        UARTprintf("f_mount error: %s\n", StringFromFresult(fresult));
        return(1);
    }

    //
    // Enter an (almost) infinite loop for reading and processing commands from
    // the user.
    //
    while(1)
    {
        //
        // Print a prompt to the console.  Show the CWD.
        //
        UARTprintf("\n%s> ", g_cCwdBuf);

        //
        // Get a line of text from the user.
        //
        UARTgets(g_cCmdBuf, sizeof(g_cCmdBuf));

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        nStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(nStatus == CMDLINE_BAD_CMD)
        {
            UARTprintf("Bad command!\n");
        }
        //
        // Handle the case of too many arguments.
        //
        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
        {
            UARTprintf("Too many arguments for command processor!\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(nStatus != 0)
        {
            UARTprintf("Command returned error code %s\n",
                       StringFromFresult((FRESULT)nStatus));
        }
    }
}

//
// End of file
//
