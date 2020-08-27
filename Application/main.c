/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
  */
#include "mainfile.h"



int main(void)
{
	init();							//Init do sistema
	// Enter main loop.
	while(true)						//Loop principal
	{
    	power_manage_loop();		//Gestão energética do sistema
	}    
}





