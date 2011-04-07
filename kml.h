#ifndef _KML_H
#define _KML_H

#include "fat16.h"
#include "gps.h"


// possible state of an kml-document
typedef enum
{
	KML_DOC_CLOSED,
	KML_DOC_OPENED,
	KML_DOC_PLACEMARK_OPENED,
	KML_DOC_LINESTRING_OPENED,
	KML_DOC_END
}KML_DocState_t;


// structure of an kml-document
typedef struct kml_doc
{
	KML_DocState_t state;									// state of the kml-document
	File_t *file;											// filepointer to the file where the data should be saved.
} KML_Document_t;


u8 KML_LoggGPSCoordinates(KML_Document_t *); 				// intializes the kml-document with standard filename and adds points to the file
u8 KML_DocumentInit(KML_Document_t *);	 					// Init the new kml-document
u8 KML_DocumentOpen(s8 *, KML_Document_t *);				// opens a new kml-document. a new file is created on the sd-memorycard
u8 KML_DocumentClose(KML_Document_t *doc);					// closes the specified document saving remaining data to the file.
u8 KML_PlaceMarkOpen(KML_Document_t *doc);					// opens a new placemark within the open kml-document
u8 KML_PlaceMarkClose(KML_Document_t *doc);					// closes the actual placemark
u8 KML_LineStringBegin(KML_Document_t *doc);				// begins a new linestring within the actual placemark
u8 KML_LineStringEnd(KML_Document_t *doc);					// close the actual linestring within the actual placemark
u8 KML_LineStringAddPoint(KML_Document_t *);				// adds a point from the gps (longitude, altitude, height) to the linestring

#endif //_KML_H
