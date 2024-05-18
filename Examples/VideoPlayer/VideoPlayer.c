#include "OrionPublicPacket.h"
#include "fielddecode.h"
#include "OrionComm.h"
#include "StreamDecoder.h"
#include "FFmpeg.h"
#include "KlvParser.h"
#include "earthposition.h"
#include "linearalgebra.h"
#include "mathutilities.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <jpeglib.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, OrionNetworkVideo_t *pSettings, char *pVideoUrl, char *pRecordPath);
static int ProcessKeyboard(void);
static void SaveJpeg(uint8_t *pData, const double Lla[NLLA], uint64_t TimeStamp, int Width, int Height, const char *pPath, int Quality);
static void WriteExifData(struct jpeg_compress_struct *pInfo, const double Lla[NLLA], uint64_t TimeStamp);

int main(int argc, char **argv)
{
    uint8_t VideoFrame[1280 * 720 * 3] = {0}, MetaData[1024] = {0};
    OrionNetworkVideo_t Settings;
    char FilePath[256] = "", RecordPath[256] = "";
    int FrameCount = 0;

    // Process the command line arguments
    ProcessArgs(argc, argv, &Settings, FilePath, RecordPath);

    // Attempt to open the video file
    if (StreamOpen(FilePath, RecordPath) == 0)
    {
        printf("Failed to open video file %s\n", FilePath);
        KillProcess("", 1);
    }
    else
    {
        printf("Press S to capture a snapshot or Q to quit\n");
    }

    // Main loop for processing frames
    while (1)
    {
        if (StreamProcess())
        {
            printf("Captured %5d frames\r", ++FrameCount);
        }

        switch (ProcessKeyboard())
        {
            case 's':
            case 'S':
                // CaptureSnapshotAndSave(&FrameCount);  // Assume this function encapsulates the snapshot logic
                break;
            case 'q':
            case 'Q':
                KillProcess("Exiting...", 0);
                break;
            default:
                break;
        };

        fflush(stdout);
        usleep(5000);
    }

    return 0; // This line will never actually be reached
}// main

static void SaveJpeg(uint8_t *pData, const double Lla[NLLA], uint64_t TimeStamp, int Width, int Height, const char *pPath, int Quality)
{
    FILE *pFile;

    // If we manage to open the file we're trying to write to
    if ((pFile = fopen(pPath, "wb")) != NULL)
    {
        struct jpeg_compress_struct Info;
        struct jpeg_error_mgr Error;
        unsigned int i;

        // Not sure why this has to happen first...
        Info.err = jpeg_std_error(&Error);

        // Initialize the compressor subsystem
        jpeg_create_compress(&Info);
        jpeg_stdio_dest(&Info, pFile);

        // Populate some information regarding the image format
        Info.image_width      = Width;
        Info.image_height     = Height;
        Info.input_components = 3;
        Info.in_color_space   = JCS_RGB;

        // Now initialize all the internal stuff to defaults
        jpeg_set_defaults(&Info);

        // It's definitely called fastest for a reason... May want to disable this, though, for quality's sake
        Info.dct_method = JDCT_FASTEST;

        // Set quality and get to compressin'
        jpeg_set_quality(&Info, Quality, 1);
        jpeg_start_compress(&Info, 1);

        // Write the EXIF data (if any)
        WriteExifData(&Info, Lla, TimeStamp);

        // Allocate a scanline array
        JSAMPARRAY pScanLines = (JSAMPARRAY)malloc(Height * sizeof(JSAMPROW));

        // For each scanline in the image
        for (i = 0; i < Height; i++)
        {
            // Point this scanline row to the appropriate row in the input data
            pScanLines[i] = &pData[i * Info.image_width * Info.input_components];
        }

        // Write the JPEG data to disk
        jpeg_write_scanlines(&Info, pScanLines, Height);

        // Free the scanline array
        free(pScanLines);

        // Finally, finish and close the file
        jpeg_finish_compress(&Info);
        jpeg_destroy_compress(&Info);
        fclose(pFile);
    }

}// SaveJpeg

const char *LatLonToString(char *pBuffer, double Radians, char SuffixPos, char SuffixNeg)
{
    // Convert from lat/lon to unsigned degrees
    double Degrees = fabs(degrees(Radians));

    // Split into integer and fractional parts
    double Integer = (int)Degrees, Fraction = Degrees - Integer;

    // Finally, format the data as per the XMP spec
    sprintf(pBuffer, "%.0lf,%.6lf%c", Integer, Fraction * 60.0, (Radians < 0) ? SuffixNeg : SuffixPos);

    // Now return a pointer to the buffer that the user passed in
    return pBuffer;

}// LatLonToString

static void WriteExifData(struct jpeg_compress_struct *pInfo, const double Lla[NLLA], uint64_t TimeStamp)
{
    // Convert from UNIX microseconds to GPS milliseconds
    uint64_t GpsTime = TimeStamp / 1000 + (LEAP_SECONDS * 1000) - 315964800000ULL;
    uint32_t Week = GpsTime / 604800000ULL, Itow = GpsTime - Week * 604800000ULL;
    uint8_t Month, Day, Hour, Minute, Second;
    uint16_t Year;

    // Now get date and time from the reconstructed GPS time
    computeDateAndTimeFromWeekAndItow(Week, Itow, LEAP_SECONDS, &Year, &Month, &Day, &Hour, &Minute, &Second);

    // Check for valid GPS time
    if (Year > 2012)
    {
        char Exif[4096], Buffer[64];
        int i = 0;

        // XML header garbage
        i += sprintf(&Exif[i], "http://ns.adobe.com/xap/1.0/");
        Exif[i++] = 0;
        i += sprintf(&Exif[i], "<?xpacket begin='\xef\xbb\xbf' id='W5M0MpCehiHzreSzNTczkc9d'?>\n");
        i += sprintf(&Exif[i], "<x:xmpmeta xmlns:x='adobe:ns:meta/' x:xmptk='XMP Core 5.4.0'>\n");
        i += sprintf(&Exif[i], "<rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>\n\n");
        i += sprintf(&Exif[i], " <rdf:Description rdf:about='' xmlns:exif='http://ns.adobe.com/exif/1.0/'>\n");

        // GPS LLA camera position
        i += sprintf(&Exif[i], "  <exif:GPSLatitude>%s</exif:GPSLatitude>\n", LatLonToString(Buffer, Lla[LAT], 'N', 'S'));
        i += sprintf(&Exif[i], "  <exif:GPSLongitude>%s</exif:GPSLongitude>\n", LatLonToString(Buffer, Lla[LON], 'E', 'W'));
        i += sprintf(&Exif[i], "  <exif:GPSAltitude>%.1lf</exif:GPSAltitude>\n", Lla[ALT]);

        // GPS date/time
        i += sprintf(&Exif[i],"  <exif:GPSTimeStamp>%u:%02u:%02u %02u:%02u:%02u</exif:GPSTimeStamp>\n", Year, Month, Day, Hour, Minute, Second);

        // XML footer garbage
        i += sprintf(&Exif[i], " </rdf:Description>\n");
        i += sprintf(&Exif[i], "</rdf:RDF>\n");
        i += sprintf(&Exif[i], "</x:xmpmeta>\n");

        // Now write the data to the JPEG file
        jpeg_write_marker(pInfo, 0xe1, (const uint8_t *)Exif, i);        
    }

}// WriteExifData

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Kill the video stream parser/recorder
    StreamClose();

    // Close down the active file descriptors
    OrionCommClose();

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static void ProcessArgs(int argc, char **argv, OrionNetworkVideo_t *pSettings, char *pFilePath, char *pRecordPath)
{
    // Adjusted to handle only file paths
    switch (argc)
    {
    case 3: // Optional: Path for recording the output
        strncpy(pRecordPath, argv[2], 256);
        // Fall through intended
    case 2: // File path for input video
        strncpy(pFilePath, argv[1], 256);
        break;
    default:
        printf("USAGE: %s video_file.mts [record_file.ts]\n", argv[0]);
        KillProcess("Incorrect arguments", 1);
        break;
    };
}// ProcessArgs

#ifdef _WIN32
# include <conio.h>
#else
# include <termios.h>
#endif // _WIN32

// Look for a keypress from the user
static int ProcessKeyboard(void)
{
#ifdef _WIN32
    return (_kbhit() == 0) ? 0 : _getch();
#else
    struct termios Old, New;
    char c = 0;

    // If we can get the current attributes for stdin
    if (tcgetattr(fileno(stdin), &Old) >= 0)
    {
        // Copy the current attributes into a new structure
        New = Old;

        // Turn off the echo and canonical output and disable blocking
        New.c_lflag &= ~(ICANON | ECHO);
        New.c_cc[VMIN] = New.c_cc[VTIME] = 0;

        // If we can successfully overwrite the current settings
        if (tcsetattr(fileno(stdin), TCSANOW, &New) >= 0)
        {
            // Try reading from stdin
            if (read(fileno(stdin), &c, 1) != 1)
            {
                // If there's some sort of error, clear whatever came out of read()
                c = 0;
            }
        }

        // Finally, revert the stdin settings to what they were before we were called
        tcsetattr(fileno(stdin), TCSANOW, &Old);
    }

    // And last but not least, return the character we read from stdin (or NULL for nothing)
    return c;
#endif // _WIN32

}// ProcessKeyboard
