/*******************************************************************************
* MyMDDFS - Memory Disk Drive File System                                      *
********************************************************************************
* Description:                                                                 *
* Demo from Microhip to illustrate the use of MDDFS                            *
********************************************************************************
* Version : 1.00 - Sept 2011                                                   *
*******************************************************************************/

#define  MyMDDFS


/*******************************************************************************
//NOTE : DISABLE MACRO "SUPPORT_LFN" IN "FSconfig.h" FILE TO WORK WITH THIS DEMO
         EFFECTIVELY. DISABLING "SUPPORT_LFN" WILL SAVE LOT OF MEMORY FOR THIS
         DEMO.
********************************************************************************/

#include "MyApp.h"
#include "MDD File System/FSIO.h"

/******************************************************************************/

int MDDFS_IntStatus;
int MDDFS_SPICON_save;
int MDDFS_SPIBRG_save;

void MyMDDFS_SaveSPI(void)
{
    /* Disable all Interrupts */
    MDDFS_IntStatus = INTDisableInterrupts();

    /* Save SPI configuration */
    MDDFS_SPICON_save = SPICON1;
    MDDFS_SPIBRG_save = SPIBRG;

}


void MyMDDFS_RestoreSPI(void)
{
    /* set Slave Select high ((disable SPI chip select on MRF24WB0M)   */
    SD_CS = 1;

    /* Restore SPI Configuration */
    SPIENABLE = 0;
    SPIBRG = MDDFS_SPIBRG_save;
    SPICON1 = MDDFS_SPICON_save;
    SPIENABLE = 1;

    /* Restore the Interrupts */
    INTRestoreInterrupts(MDDFS_IntStatus);
}

/******************************************************************************/

char sendBuffer[] = "Hodor";
char send2[] = "2";
char receiveBuffer[50];


void MyMDDFS_Test (void)
{
   FSFILE * pointer;
   char path[30];
   char count = 30;
   char * pointer2;
   SearchRec rec;
   unsigned char attributes;
   unsigned char size = 0, i;

   MyMDDFS_SaveSPI();

   if (!MDD_MediaDetect()) {
       MyMDDFS_RestoreSPI();
       MyConsole_SendMsg("MyMDDFS - Error MDD_MediaDetect\n>");
       return;
   }

   // Initialize the library
   if (!FSInit()) {
       MyMDDFS_RestoreSPI();
       MyConsole_SendMsg("MyMDDFS - Error FSInit\n>");
       return;
   }

#ifdef ALLOW_WRITES
   // Create a file
   pointer = FSfopen ("FILE3.TXT", "w");
   if (pointer == NULL)
      MyConsole_SendMsg("MyMDDFS - Error FSfopen\n>");

   // Write 21 1-byte objects from sendBuffer into the file
   if (FSfwrite (sendBuffer, 1, 21, pointer) != 21)
      MyConsole_SendMsg("MyMDDFS - Error FSfwrite\n>");

   // FSftell returns the file's current position
   if (FSftell (pointer) != 21)
      MyConsole_SendMsg("MyMDDFS - Error FSftell\n>");

   // FSfseek sets the position one byte before the end
   // It can also set the position of a file forward from the
   // beginning or forward from the current position
   if (FSfseek(pointer, 1, SEEK_END))
      MyConsole_SendMsg("MyMDDFS - Error FSfseek\n>");

   // Write a 2 at the end of the string
   if (FSfwrite (send2, 1, 1, pointer) != 1)
      MyConsole_SendMsg("MyMDDFS - Error FSfwrite\n>");

   // Close the file
   if (FSfclose (pointer))
      MyConsole_SendMsg("MyMDDFS - Error FSfclose\n>");

   // Create a second file
   pointer = FSfopen ("FILE1.TXT", "w");
   if (pointer == NULL)
      MyConsole_SendMsg("MyMDDFS - Error FSfopen\n>");

   // Write the string to it again
   if (FSfwrite ((void *)sendBuffer, 1, 21, pointer) != 21)
      MyConsole_SendMsg("MyMDDFS - Error FSfwrite\n>");

   // Close the file
   if (FSfclose (pointer))
      MyConsole_SendMsg("MyMDDFS - Error FSfclose\n>");
#endif

   // Open file 1 in read mode
   pointer = FSfopen ("FILE3.TXT", "r");
   if (pointer == NULL)
      MyConsole_SendMsg("MyMDDFS - Error FSfopen\n>");

   if (FSrename ("FILE2.TXT", pointer))
      MyConsole_SendMsg("MyMDDFS - Error FSfrename\n>");

   // Read one four-byte object
   if (FSfread (receiveBuffer, 4, 1, pointer) != 1)
      MyConsole_SendMsg("MyMDDFS - Error FSfread\n>");

   // Check if this is the end of the file- it shouldn't be
   if (FSfeof (pointer))
      MyConsole_SendMsg("MyMDDFS - Error FSfeof\n>");

   // Close the file
   if (FSfclose (pointer))
      MyConsole_SendMsg("MyMDDFS - Error FSfclose\n>");

   // Make sure we read correctly
   if ((receiveBuffer[0] != 'T') ||
         (receiveBuffer[1] != 'h')  ||
         (receiveBuffer[2] != 'i')  ||
         (receiveBuffer[3] != 's'))
   {
      MyConsole_SendMsg("MyMDDFS - Error receiveBuffer\n>");
   }

#ifdef ALLOW_DIRS
   // Create a small directory tree
   // Beginning the path string with a '.' will create the tree in
   // the current directory.  Beginning with a '..' would create the
   // tree in the previous directory.  Beginning with just a '\' would
   // create the tree in the root directory.  Beginning with a dir name
   // would also create the tree in the current directory
   if (FSmkdir (".\\ONE\\TWO\\THREE"))
      MyConsole_SendMsg("MyMDDFS - Error FSmkdir\n>");

   // Change to directory THREE in our new tree
   if (FSchdir ("ONE\\TWO\\THREE"))
      MyConsole_SendMsg("MyMDDFS - Error FSchdir\n>");

   // Create another tree in directory THREE
   if (FSmkdir ("FOUR\\FIVE\\SIX"))
      MyConsole_SendMsg("MyMDDFS - Error FSmkdir\n>");

   // Create a third file in directory THREE
   pointer = FSfopen ("FILE3.TXT", "w");
   if (pointer == NULL)
      MyConsole_SendMsg("MyMDDFS - Error FSfopen\n>");

   // Get the name of the current working directory
   /* it should be "\ONE\TWO\THREE"       */
   pointer2 = FSgetcwd (path, count);
   if (pointer2 != path)
      MyConsole_SendMsg("MyMDDFS - Error FSgetcwd\n>");

   // Simple string length calculation
   i = 0;
   while(*(path + i) != 0x00)
   {
      size++;
      i++;
   }
   // Write the name to FILE3.TXT
   if (FSfwrite (path, size, 1, pointer) != 1)
      MyConsole_SendMsg("MyMDDFS - Error FSfwrite\n>");

   // Close the file
   if (FSfclose (pointer))
      MyConsole_SendMsg("MyMDDFS - Error FSfclose\n>");

   // Create some more directories
   if (FSmkdir ("FOUR\\FIVE\\SEVEN\\..\\EIGHT\\..\\..\\NINE\\TEN\\..\\ELEVEN\\..\\TWELVE"))
      MyConsole_SendMsg("MyMDDFS - Error FSmkdir\n>");

   /*******************************************************************
      Now our tree looks like this

      \ -> ONE -> TWO -> THREE -> FOUR -> FIVE -> SIX
                                                 -> SEVEN
                                                 -> EIGHT
                                            NINE -> TEN
                                                 -> ELEVEN
                                                 -> TWELVE
   ********************************************************************/

   // This will delete only directory eight
   // If we tried to delete directory FIVE with this call, the FSrmdir
   // function would return -1, since FIVE is non-empty
   if (FSrmdir ("\\ONE\\TWO\\THREE\\FOUR\\FIVE\\EIGHT", FALSE))
      MyConsole_SendMsg("MyMDDFS - Error FSrmdir\n>");

   // This will delete directory NINE and all three of its sub-directories
   if (FSrmdir ("FOUR\\NINE", TRUE))
      MyConsole_SendMsg("MyMDDFS - Error FSrmdir\n>");

   // Change directory to the root dir
   if (FSchdir ("\\"))
      MyConsole_SendMsg("MyMDDFS - Error FSchdir\n>");
#endif

#ifdef ALLOW_FILESEARCH
   // Set attributes
   attributes = ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN;

   // Functions "FindFirst" & "FindNext" can be used to find files
   // and directories with required attributes in the current working directory.

   // Find the first TXT file with any (or none) of those attributes that
   // has a name beginning with the letters "FILE"
   // These functions are more useful for finding out which files are
   // in your current working directory
   if (FindFirst ("FILE*.TXT", attributes, &rec))
      MyConsole_SendMsg("MyMDDFS - Error FindFirst\n>");

   // Keep finding files until we get FILE2.TXT
   while(rec.filename[4] != '2')
   {
      if (FindNext (&rec))
         MyConsole_SendMsg("MyMDDFS - Error FindNext\n>");
   }

   // Delete file 2
   // NOTE : "FSremove" function deletes specific file not directory.
   //        To delete directories use "FSrmdir" function
   if (FSremove (rec.filename))
      MyConsole_SendMsg("MyMDDFS - Error FSremove\n>");
#endif

/*********************************************************************
   The final contents of our card should look like this:
   \ -> FILE1.TXT
      -> ONE       -> TWO -> THREE -> FILE3.TXT
                                   -> FOUR      -> FIVE -> SIX
                                                        -> SEVEN

*********************************************************************/
   MyConsole_SendMsg("MyMDDFS - Test ok\n>");
   MyMDDFS_RestoreSPI();
}








