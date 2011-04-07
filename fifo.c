#include "fifo.h"

u8 fifo_init (fifo_t* f, u8* buffer, const u16 size, u16 putvicsource, u16 getvicsource)
{
	if(f == NULL) return(0);
	f->buffer = buffer;
	f->size = size;
	f->putvicsource = putvicsource;
   	f->getvicsource = getvicsource;
	fifo_purge(f);
	return(1);
}

u8 fifo_put (fifo_t *f, const u8 data)
{
	if(f->buffer == 0) return(0);
	if (f->count >= f->size) return(0);	// return 0 in case of FIFO overflow.
	if(f->putvicsource != NO_ITLine) VIC_ITCmd(f->putvicsource, DISABLE);
	*(f->pwrite++) = data;	    // copy data byte to buffer
	if(f->pwrite >= f->buffer + f->size) f->pwrite = f->buffer; // start at the begining after reaching the end 
	f->count++;
	if(f->putvicsource != NO_ITLine) VIC_ITCmd(f->putvicsource, ENABLE);
	return(1);
}

u8 fifo_get (fifo_t *f, u8 *pdata)
{
	if(f->buffer == 0) return(0);
	if(!f->count) return(0);
 	if(f->getvicsource != NO_ITLine) VIC_ITCmd(f->getvicsource, DISABLE);
	*pdata = *(f->pread++);
	if(f->pread >= f->buffer + f->size) f->pread = f->buffer; // start at the begining after reaching the end 
	f->count--;
	if(f->getvicsource != NO_ITLine) VIC_ITCmd(f->getvicsource, ENABLE);
	return(1);
}

u8 fifo_get_wait (fifo_t *f, u8 *pdata)
{
	while (!f->count);

	return fifo_get(f, pdata);
}

void fifo_purge(fifo_t* f)
{
	if((f == NULL)) return;
	f->count = 0;
	f->pread = f->buffer;
	f->pwrite = f->buffer; 
	return;	
}
