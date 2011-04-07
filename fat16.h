#ifndef _FAT16_H
#define _FAT16_H


//________________________________________________________________________________________________________________________________________
// 
// Definitions
//				
//________________________________________________________________________________________________________________________________________

//#define		__USE_TIME_DATE_ATTRIBUTE
#define	FILE_MAX_OPEN	4				// The number of files that can accessed simultaneously. 
#define	SEEK_SET	0
#define	SEEK_CUR	1
#define	SEEK_END	2
#define	EOF	(-1) 
#define BYTES_PER_SECTOR	512
/*
________________________________________________________________________________________________________________________________________
 
	Structure of a filepointer
________________________________________________________________________________________________________________________________________
*/
typedef struct
{
	u32 FirstSectorOfFirstCluster;	// First sector of the first cluster of the file.
	u32 FirstSectorOfCurrCluster;	// First sector of the cluster which is edited at the moment.
	u8	SectorOfCurrCluster;		// The sector within the current cluster.
	u16 ByteOfCurrSector;			// The byte location within the current sector.
	u8	Mode;						// Mode of fileoperation (read,write)
	u32 Size;						// The size of the opend file in bytes.
	u32 Position;					// Pointer to a character within the file 0 < fileposition < filesize
	u32 DirectorySector;			// the sectorposition where the directoryentry has been made.
	u16	DirectoryIndex;				// The index to the directoryentry within the specified sector.
	u8 	Attribute;					// The attribute of the file opened.
	u8  Cache[BYTES_PER_SECTOR];	// Cache for read and write operation from or to the sd-card.
	u32 SectorInCache;				// The last sector read, which is still in the sector cache.
	u8	State;						// State of the filepointer (used/unused/...) 
} File_t;

//________________________________________________________________________________________________________________________________________
// 
// API to the FAT16 filesystem
//				
//________________________________________________________________________________________________________________________________________

u8		Fat16_Init(void);
u8		Fat16_Deinit(void);
u8 		Fat16_IsValid(void);
s8*		FAT16_GetVolumeLabel(void);
	
File_t *fopen_(s8 * const filename, const s8 mode);
s16 	fclose_(File_t *file);
u8		fexist_(s8 * const filename);
s16		fflush_(File_t * const file);
s16  	fseek_(File_t * const file, s32 offset, s16 origin);
s16		fgetc_(File_t * const file);
s16		fputc_(s8 c, File_t * const file);
u32 	fread_(void *buffer, u32 size, u32 count, File_t * const file); 
u32 	fwrite_(void *buffer, u32 size, u32 count, File_t * const file);
s16		fputs_(s8 * const string, File_t * const file);
s8 *  	fgets_(s8 * const string, s16 length, File_t * const file);
u8 		feof_(File_t * const file);



#endif //_FAT16_H




