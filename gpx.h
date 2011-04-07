#ifndef _GPX_H
#define _GPX_H

#include "fat16.h"
#include "gps.h"


// possible state of a gpx-document
typedef enum
{
	GPX_DOC_CLOSED,
	GPX_DOC_OPENED,
	GPX_DOC_TRACK_OPENED,
	GPX_DOC_TRACKSEGMENT_OPENED,
	GPX_DOC_END
}GPX_DocState_t;


// structure of an gpx-document
typedef struct gpx_doc
{
	GPX_DocState_t state;									// state of the gpx-document
	File_t *file;											// filepointer to the file where the data should be saved.
} GPX_Document_t;

u8 GPX_LoggGPSCoordinates(GPX_Document_t *); 				// intializes the gpx-document with standard filename and adds points to the file
u8 GPX_DocumentInit(GPX_Document_t *);	 					// Init the new gpx-document
u8 GPX_DocumentOpen(s8 *, GPX_Document_t *);				// opens a new gpx-document. a new file is created on the sd-memorycard
u8 GPX_DocumentClose(GPX_Document_t *doc);					// closes the specified document saving remaining data to the file.
u8 GPX_TrackBegin(GPX_Document_t *doc);						// opens a new track within the open gpx-document
u8 GPX_TrackEnd(GPX_Document_t *doc);						// ends the actual track
u8 GPX_TrackSegmentBegin(GPX_Document_t *doc);				// begins a new tracksegment within the actual track
u8 GPX_TrackSegmentEnd(GPX_Document_t *doc);				// ends the actual track segment within the actual track
u8 GPX_TrackSegmentAddPoint(GPX_Document_t *);				// adds a point to the tracksegment

#endif //_GPX_H
