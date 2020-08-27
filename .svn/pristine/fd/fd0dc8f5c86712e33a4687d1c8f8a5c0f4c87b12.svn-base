/*
 * safety_mode.c
 *
 *  Created on: 11/03/2016
 *      Author: lapa
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "timers.h"
//#include "state_machine.h"
#include "safety_mode.h"








bool insert_elem(BUF_ELEM *buf_elem, int8_t ble_item, uint16_t size_buf)
{
	if(buf_elem->cnt>=size_buf)
		return false;
	//memcpy(&buf_elem->array[buf_elem->wr_ptr],&ble_item,sizeof(int16_t));			//copia o item para o array
	buf_elem->array[buf_elem->wr_ptr]=ble_item;
	if((++buf_elem->wr_ptr)>=size_buf)
		buf_elem->wr_ptr=0;
	buf_elem->cnt++;
	return true;
}

bool read_elem(BUF_ELEM *buf_elem,int8_t *ble_item, uint16_t size_buf)
{
	if(!buf_elem->cnt)
		return false;
	//memcpy(ble_item, &buf_elem->array[buf_elem->rd_ptr],sizeof(int16_t));
	*ble_item=buf_elem->array[buf_elem->rd_ptr];
	if((++buf_elem->rd_ptr)>=size_buf)
		buf_elem->rd_ptr=0;
	buf_elem->cnt--;
	return true;
}

void safety_mode_clr_buff(BUF_ELEM *buf_elem, uint16_t size_buf)
{
	int8_t tmp;
	while(buf_elem->cnt)
		read_elem(buf_elem,&tmp, size_buf);

}



