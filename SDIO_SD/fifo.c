
#include "fifo.h"

//extern FIFO( MK_TO_COMP_FIFO_SIZE ) MK_to_Comp_fifo;
//extern FIFO( COMP_TO_MK_FIFO_SIZE ) Comp_to_MK_fifo;

FIFO_T MK_to_Comp_fifo_0;
FIFO_T Comp_to_MK_fifo_0;


//количество элементов в очереди
//#define FIFO_COUNT(fifo)     (fifo.head-fifo.tail)

//количество элементов в очереди
void FIFO_COUNT_(FIFO_T *fifo)     
{
//	fifo->overflow*
//	(fifo.head-fifo.tail);
}