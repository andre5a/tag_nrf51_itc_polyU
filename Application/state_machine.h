/*
 * state_machine.h
 *
 * Created: 27-Apr-15 9:58:08 AM
 *  Author: Andre Quinta
 */ 


#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_


typedef struct
{
	unsigned char cstate;				//Estado corrente da maquina de estados do envio de mensagens
	unsigned char nxstate;				//Proximo estado da máquina de estados do envio de mensagens
	unsigned char pvstate;				//Estado anterior da máquina de estados do envio de mensagens
	unsigned char status;				//status do envio de mensagens
}STATE_MACHINE;


#endif /* STATE_MACHINE_H_ */


