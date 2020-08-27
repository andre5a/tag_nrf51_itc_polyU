#ifndef FLASH_H__
#define FLASH_H__

/**@brief Macro for getting the end of the flash available for application.
 *
 * @details    The result flash page number indicates the end boundary of the flash available
 *             to the application. If a bootloader is used, the end will be the start of the
 *             bootloader region. Otherwise, the end will be the size of the flash.
 */
/*
#define BLE_FLASH_PAGE_END \
    ((NRF_UICR->BOOTLOADERADDR != BLE_FLASH_EMPTY_MASK) \
        ? (NRF_UICR->BOOTLOADERADDR / BLE_FLASH_PAGE_SIZE) \
        : NRF_FICR->CODESIZE)

*/



typedef enum
{
	STATE_LAPA_UNREGISTERED=0,
	STATE_LAPA_REGISTERED=1,
	STATE_LAPA_POFF=2,
} lapa_state_t;

#define ADVERTISE_DISABLE	0
#define ADVERTISE_ENABLE	1



#define FLASH_STATE_LOAD_OK		0
#define FLASH_STATE_LOAD_ERR	1
#define FLASH_STATE_STORE_OK	2
#define FLASH_STATE_STORE_ERR	3
#define FLASH_STATE_UPDATE_OK	4
#define FLASH_STATE_UPDATE_ERR	5
#define FLASH_STATE_CLEAR_OK	6
#define FLASH_STATE_CLEAR_ERR	7





typedef struct
{
	uint8_t enable;
	uint8_t time_adv;
}ADVERTISE;


typedef struct
{
	uint8_t state;
	ADVERTISE adv2;
}DEVICE;


bool flash_write_test(DEVICE dev);
bool flash_read_test(DEVICE *dev);

bool flash_is_data_unsafe(void);
bool flash_write_setup_mem(DEVICE dev);
bool flash_write_setup(DEVICE dev);
bool flash_read_setup(DEVICE *dev);
void flash_set_defaults(void);

void flash_waiting_for_state(uint8_t state);
bool flash_ready(void);




bool flash_push(DEVICE dev);
bool flash_pop(DEVICE *dev);
uint8_t flash_num_elems_in_buffer(void);


#endif


