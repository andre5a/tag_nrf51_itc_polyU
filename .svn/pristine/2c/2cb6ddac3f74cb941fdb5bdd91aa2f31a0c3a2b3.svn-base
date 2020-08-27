/*
 * safety_mode.h
 *
 *  Created on: 11/03/2016
 *      Author: lapa
 */

#ifndef SAFETY_MODE_H_
#define SAFETY_MODE_H_

#define RSSI_THRESHOLD_ALARM	-87//-85
#define RSSI_THRESHOLD_SAFETY	-77//-70
//#define RSSI_FAKE				RSSI_THRESHOLD_ALARM-1
#define SAFETY_MODE_TIME_ALARM_DISCONNECT	6000


#define SFT_ST_INIT		0
#define SFT_ST_NEAR		1
#define SFT_ST_FAR		2
#define SFT_ST_OUT		3
#define SFT_ST_RUN		4
#define SFT_ST_OFF		5
#define SFT_ST_ALARM	6
#define SFT_ST_REARM	7

#define NUM_RSSI_ELEM			8

//#define TIME_UNIT 2*MAX_CONN_INTERVAL_DEFAULT_MS

typedef struct
{
	int8_t		array[NUM_RSSI_ELEM];
	uint16_t 	wr_ptr;
	uint16_t 	rd_ptr;
	uint16_t 	cnt;
}BUF_ELEM;

bool insert_elem(BUF_ELEM *buf_elem, int8_t ble_item, uint16_t size_buf);

bool read_elem(BUF_ELEM *buf_elem,int8_t *ble_item, uint16_t size_buf);

void safety_mode_clr_buff(BUF_ELEM *buf_elem, uint16_t size_buf);
int8_t safety_diff_rssi2threshold(int8_t rssi);

void safety_state(uint8_t state);


#endif /* SAFETY_MODE_H_ */
