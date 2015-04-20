/*
===============================================================================
 Name        : seeeduino_arch_test.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

//#include <cr_section_macros.h>

#define TICKRATE1_HZ 1000

volatile bool ledOn = false;





#include <string.h>
#include "app_usbd_cfg.h"
//#include "board.h"
#include "cdc_vcom.h"

#include "app_usbd_cfg.h"

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include "spi.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#if 1
void
spi1_setup(void)
{
        Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 20, (IOCON_FUNC2 | IOCON_MODE_PULLDOWN));  /* SCK1 */
        Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 21, (IOCON_FUNC2 | IOCON_MODE_INACT));     /* MISO1 */
        Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 22, (IOCON_FUNC2 | IOCON_MODE_INACT));     /* MOSI1 */
	spi_init(SPI1);

        Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 29, (IOCON_FUNC1 | IOCON_MODE_PULLDOWN));  /* SCK0 */
        Chip_IOCON_PinMuxSet(LPC_IOCON, 0,  8, (IOCON_FUNC1 | IOCON_MODE_INACT));     /* MISO0 */
        Chip_IOCON_PinMuxSet(LPC_IOCON, 0,  9, (IOCON_FUNC1 | IOCON_MODE_INACT));     /* MOSI0 */
	spi_init(SPI0);

	//Chip_SSP_Init(LPC_SSP1);

	//Chip_SSP_SetClockRate(LPC_SSP1, 0, 2);

	//Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);

	//Chip_SSP_SetMaster(LPC_SSP1, 1);

	//Chip_SSP_Enable(LPC_SSP1);
}

#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/**
 * USB Standard Device Descriptor
 */
ALIGNED(4) const uint8_t USB_DeviceDescriptor[] = {
	USB_DEVICE_DESC_SIZE,				/* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,			/* bDescriptorType */
	WBVAL(0x0200),						/* bcdUSB */
	0xEF,								/* bDeviceClass */
	0x02,								/* bDeviceSubClass */
	0x01,								/* bDeviceProtocol */
	USB_MAX_PACKET0,					/* bMaxPacketSize0 */
	WBVAL(0x1FC9),						/* idVendor */
	WBVAL(0x0083),						/* idProduct */
	WBVAL(0x0100),						/* bcdDevice */
	0x01,								/* iManufacturer */
	0x02,								/* iProduct */
	0x03,								/* iSerialNumber */
	0x01								/* bNumConfigurations */
};

/**
 * USB FSConfiguration Descriptor
 * All Descriptors (Configuration, Interface, Endpoint, Class, Vendor)
 */
ALIGNED(4) uint8_t USB_FsConfigDescriptor[] = {
	/* Configuration 1 */
	USB_CONFIGURATION_DESC_SIZE,			/* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(									/* wTotalLength */
		USB_CONFIGURATION_DESC_SIZE     +
		USB_INTERFACE_ASSOC_DESC_SIZE   +	/* interface association descriptor */
		USB_INTERFACE_DESC_SIZE         +	/* communication control interface */
		0x0013                          +	/* CDC functions */
		1 * USB_ENDPOINT_DESC_SIZE      +	/* interrupt endpoint */
		USB_INTERFACE_DESC_SIZE         +	/* communication data interface */
		2 * USB_ENDPOINT_DESC_SIZE      +	/* bulk endpoints */
		0
		),
	0x02,									/* bNumInterfaces */
	0x01,									/* bConfigurationValue */
	0x00,									/* iConfiguration */
	USB_CONFIG_SELF_POWERED,				/* bmAttributes  */
	USB_CONFIG_POWER_MA(500),				/* bMaxPower */

	/* Interface association descriptor IAD*/
	USB_INTERFACE_ASSOC_DESC_SIZE,		/* bLength */
	USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,	/* bDescriptorType */
	USB_CDC_CIF_NUM,					/* bFirstInterface */
	0x02,								/* bInterfaceCount */
	CDC_COMMUNICATION_INTERFACE_CLASS,	/* bFunctionClass */
	CDC_ABSTRACT_CONTROL_MODEL,			/* bFunctionSubClass */
	0x00,								/* bFunctionProtocol */
	0x04,								/* iFunction */

	/* Interface 0, Alternate Setting 0, Communication class interface descriptor */
	USB_INTERFACE_DESC_SIZE,			/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_CIF_NUM,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: Alternate setting */
	0x01,								/* bNumEndpoints: One endpoint used */
	CDC_COMMUNICATION_INTERFACE_CLASS,	/* bInterfaceClass: Communication Interface Class */
	CDC_ABSTRACT_CONTROL_MODEL,			/* bInterfaceSubClass: Abstract Control Model */
	0x00,								/* bInterfaceProtocol: no protocol used */
	0x04,								/* iInterface: */
	/* Header Functional Descriptor*/
	0x05,								/* bLength: CDC header Descriptor size */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_HEADER,							/* bDescriptorSubtype: Header Func Desc */
	WBVAL(CDC_V1_10),					/* bcdCDC 1.10 */
	/* Call Management Functional Descriptor*/
	0x05,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_CALL_MANAGEMENT,				/* bDescriptorSubtype: Call Management Func Desc */
	0x01,								/* bmCapabilities: device handles call management */
	USB_CDC_DIF_NUM,					/* bDataInterface: CDC data IF ID */
	/* Abstract Control Management Functional Descriptor*/
	0x04,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_ABSTRACT_CONTROL_MANAGEMENT,	/* bDescriptorSubtype: Abstract Control Management desc */
	0x02,								/* bmCapabilities: SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported */
	/* Union Functional Descriptor*/
	0x05,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_UNION,							/* bDescriptorSubtype: Union func desc */
	USB_CDC_CIF_NUM,					/* bMasterInterface: Communication class interface is master */
	USB_CDC_DIF_NUM,					/* bSlaveInterface0: Data class interface is slave 0 */
	/* Endpoint 1 Descriptor*/
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_INT_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_INTERRUPT,		/* bmAttributes */
	WBVAL(0x0010),						/* wMaxPacketSize */
	0x02,			/* 2ms */           /* bInterval */

	/* Interface 1, Alternate Setting 0, Data class interface descriptor*/
	USB_INTERFACE_DESC_SIZE,			/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_DIF_NUM,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: no alternate setting */
	0x02,								/* bNumEndpoints: two endpoints used */
	CDC_DATA_INTERFACE_CLASS,			/* bInterfaceClass: Data Interface Class */
	0x00,								/* bInterfaceSubClass: no subclass available */
	0x00,								/* bInterfaceProtocol: no protocol used */
	0x04,								/* iInterface: */
	/* Endpoint, EP Bulk Out */
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_OUT_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_BULK,				/* bmAttributes */
	WBVAL(USB_FS_MAX_BULK_PACKET),		/* wMaxPacketSize */
	0x00,								/* bInterval: ignore for Bulk transfer */
	/* Endpoint, EP Bulk In */
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_IN_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_BULK,				/* bmAttributes */
	WBVAL(64),							/* wMaxPacketSize */
	0x00,								/* bInterval: ignore for Bulk transfer */
	/* Terminator */
	0									/* bLength */
};

/**
 * USB String Descriptor (optional)
 */
ALIGNED(4) const uint8_t USB_StringDescriptor[] = {
	/* Index 0x00: LANGID Codes */
	0x04,								/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	WBVAL(0x0409),	/* US English */    /* wLANGID */
	/* Index 0x01: Manufacturer */
	(3 * 2 + 2),						/* bLength (13 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'N', 0,
	'X', 0,
	'P', 0,
	/* Index 0x02: Product */
	(9 * 2 + 2),						/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'V', 0,
	'C', 0,
	'O', 0,
	'M', 0,
	' ', 0,
	'P', 0,
	'o', 0,
	'r', 0,
	't', 0,
	/* Index 0x03: Serial Number */
	(6 * 2 + 2),						/* bLength (8 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'N', 0,
	'X', 0,
	'P', 0,
	'-', 0,
	'7', 0,
	'7', 0,
	/* Index 0x04: Interface 1, Alternate Setting 0 */
	( 4 * 2 + 2),						/* bLength (4 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'V', 0,
	'C', 0,
	'O', 0,
	'M', 0,
};



/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/**
 * Global variable to hold Virtual COM port control data.
 */
VCOM_DATA_T g_vCOM;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* VCOM bulk EP_IN endpoint handler */
static ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
	VCOM_DATA_T *pVcom = (VCOM_DATA_T *) data;

	if (event == USB_EVT_IN) {
		pVcom->tx_flags &= ~VCOM_TX_BUSY;
	}
	return LPC_OK;
}

/* VCOM bulk EP_OUT endpoint handler */
static ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
	VCOM_DATA_T *pVcom = (VCOM_DATA_T *) data;

	switch (event) {
	case USB_EVT_OUT:
		pVcom->rx_count = USBD_API->hw->ReadEP(hUsb, USB_CDC_OUT_EP, pVcom->rx_buff);
		if (pVcom->rx_flags & VCOM_RX_BUF_QUEUED) {
			pVcom->rx_flags &= ~VCOM_RX_BUF_QUEUED;
			if (pVcom->rx_count != 0) {
				pVcom->rx_flags |= VCOM_RX_BUF_FULL;
			}

		}
		else if (pVcom->rx_flags & VCOM_RX_DB_QUEUED) {
			pVcom->rx_flags &= ~VCOM_RX_DB_QUEUED;
			pVcom->rx_flags |= VCOM_RX_DONE;
		}
		break;

	case USB_EVT_OUT_NAK:
		/* queue free buffer for RX */
		if ((pVcom->rx_flags & (VCOM_RX_BUF_FULL | VCOM_RX_BUF_QUEUED)) == 0) {
			USBD_API->hw->ReadReqEP(hUsb, USB_CDC_OUT_EP, pVcom->rx_buff, VCOM_RX_BUF_SZ);
			pVcom->rx_flags |= VCOM_RX_BUF_QUEUED;
		}
		break;

	default:
		break;
	}

	return LPC_OK;
}

/* Set line coding call back routine */
static ErrorCode_t VCOM_SetLineCode(USBD_HANDLE_T hCDC, CDC_LINE_CODING *line_coding)
{
	VCOM_DATA_T *pVcom = &g_vCOM;

	/* Called when baud rate is changed/set. Using it to know host connection state */
	pVcom->tx_flags = VCOM_TX_CONNECTED;	/* reset other flags */

	return LPC_OK;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Virtual com port init routine */
ErrorCode_t vcom_init(USBD_HANDLE_T hUsb, USB_CORE_DESCS_T *pDesc, USBD_API_INIT_PARAM_T *pUsbParam)
{
	USBD_CDC_INIT_PARAM_T cdc_param;
	ErrorCode_t ret = LPC_OK;
	uint32_t ep_indx;

	g_vCOM.hUsb = hUsb;
	memset((void *) &cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));
	cdc_param.mem_base = pUsbParam->mem_base;
	cdc_param.mem_size = pUsbParam->mem_size;
	cdc_param.cif_intf_desc = (uint8_t *) find_IntfDesc(pDesc->high_speed_desc, CDC_COMMUNICATION_INTERFACE_CLASS);
	cdc_param.dif_intf_desc = (uint8_t *) find_IntfDesc(pDesc->high_speed_desc, CDC_DATA_INTERFACE_CLASS);
	cdc_param.SetLineCode = VCOM_SetLineCode;

	ret = USBD_API->cdc->init(hUsb, &cdc_param, &g_vCOM.hCdc);

	if (ret == LPC_OK) {
		/* allocate transfer buffers */
		g_vCOM.rx_buff = (uint8_t *) cdc_param.mem_base;
		cdc_param.mem_base += VCOM_RX_BUF_SZ;
		cdc_param.mem_size -= VCOM_RX_BUF_SZ;

		/* register endpoint interrupt handler */
		ep_indx = (((USB_CDC_IN_EP & 0x0F) << 1) + 1);
		ret = USBD_API->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_in_hdlr, &g_vCOM);
		if (ret == LPC_OK) {
			/* register endpoint interrupt handler */
			ep_indx = ((USB_CDC_OUT_EP & 0x0F) << 1);
			ret = USBD_API->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_out_hdlr, &g_vCOM);

		}
		/* update mem_base and size variables for cascading calls. */
		pUsbParam->mem_base = cdc_param.mem_base;
		pUsbParam->mem_size = cdc_param.mem_size;
	}

	return ret;
}

/* Virtual com port buffered read routine */
uint32_t vcom_bread(uint8_t *pBuf, uint32_t buf_len)
{
	VCOM_DATA_T *pVcom = &g_vCOM;
	uint16_t cnt = 0;
	/* read from the default buffer if any data present */
	if (pVcom->rx_count) {
		cnt = (pVcom->rx_count < buf_len) ? pVcom->rx_count : buf_len;
		memcpy(pBuf, pVcom->rx_buff, cnt);
		pVcom->rx_rd_count += cnt;

		/* enter critical section */
		NVIC_DisableIRQ(USB0_IRQn);
		if (pVcom->rx_rd_count >= pVcom->rx_count) {
			pVcom->rx_flags &= ~VCOM_RX_BUF_FULL;
			pVcom->rx_rd_count = pVcom->rx_count = 0;
		}
		/* exit critical section */
		NVIC_EnableIRQ(USB0_IRQn);
	}
	return cnt;

}

/* Virtual com port read routine */
ErrorCode_t vcom_read_req(uint8_t *pBuf, uint32_t len)
{
	VCOM_DATA_T *pVcom = &g_vCOM;

	/* check if we queued Rx buffer */
	if (pVcom->rx_flags & (VCOM_RX_BUF_QUEUED | VCOM_RX_DB_QUEUED)) {
		return ERR_BUSY;
	}
	/* enter critical section */
	NVIC_DisableIRQ(USB0_IRQn);
	/* if not queue the request and return 0 bytes */
	USBD_API->hw->ReadReqEP(pVcom->hUsb, USB_CDC_OUT_EP, pBuf, len);
	/* exit critical section */
	NVIC_EnableIRQ(USB0_IRQn);
	pVcom->rx_flags |= VCOM_RX_DB_QUEUED;

	return LPC_OK;
}

/* Gets current read count. */
uint32_t vcom_read_cnt(void)
{
	VCOM_DATA_T *pVcom = &g_vCOM;
	uint32_t ret = 0;

	if (pVcom->rx_flags & VCOM_RX_DONE) {
		ret = pVcom->rx_count;
		pVcom->rx_count = 0;
	}

	return ret;
}

/* Virtual com port write routine*/
uint32_t vcom_write(uint8_t *pBuf, uint32_t len)
{
	VCOM_DATA_T *pVcom = &g_vCOM;
	uint32_t ret = 0;

	if ( (pVcom->tx_flags & VCOM_TX_CONNECTED) && ((pVcom->tx_flags & VCOM_TX_BUSY) == 0) ) {
		pVcom->tx_flags |= VCOM_TX_BUSY;

		/* enter critical section */
		NVIC_DisableIRQ(USB0_IRQn);
		ret = USBD_API->hw->WriteEP(pVcom->hUsb, USB_CDC_IN_EP, pBuf, len);
		/* exit critical section */
		NVIC_EnableIRQ(USB0_IRQn);
	}

	return ret;
}






static USBD_HANDLE_T g_hUsb;
static uint8_t g_rxBuff[256];
const  USBD_API_T *g_pUsbApi;


/* Initialize pin and clocks for USB0/USB1 port */
static void usb_pin_clk_init(void)
{
	/* enable USB main clock */
	Chip_Clock_SetUSBClockSource(SYSCTL_USBCLKSRC_PLLOUT, 1);
	/* Enable AHB clock to the USB block and USB RAM. */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USB);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);
	/* power UP USB Phy */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPAD_PD);
}


void USB_IRQHandler(void)
{
	uint32_t *addr = (uint32_t *) LPC_USB->EPLISTSTART;

	/*	WORKAROUND for artf32289 ROM driver BUG:
	    As part of USB specification the device should respond
	    with STALL condition for any unsupported setup packet. The host will send
	    new setup packet/request on seeing STALL condition for EP0 instead of sending
	    a clear STALL request. Current driver in ROM doesn't clear the STALL
	    condition on new setup packet which should be fixed.
	 */
	if ( LPC_USB->DEVCMDSTAT & _BIT(8) ) {	/* if setup packet is received */
		addr[0] &= ~(_BIT(29));	/* clear EP0_OUT stall */
		addr[2] &= ~(_BIT(29));	/* clear EP0_IN stall */
	}
	USBD_API->hw->ISR(g_hUsb);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
	USB_COMMON_DESCRIPTOR *pD;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
	uint32_t next_desc_adr;

	pD = (USB_COMMON_DESCRIPTOR *) pDesc;
	next_desc_adr = (uint32_t) pDesc;

	while (pD->bLength) {
		/* is it interface descriptor */
		if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

			pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
			/* did we find the right interface descriptor */
			if (pIntfDesc->bInterfaceClass == intfClass) {
				break;
			}
		}
		pIntfDesc = 0;
		next_desc_adr = (uint32_t) pD + pD->bLength;
		pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
	}

	return pIntfDesc;
}





void TIMER32_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER32_0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_0, 1);
		ledOn = !ledOn;
		Chip_GPIO_SetPinState(LPC_GPIO, 1, 11, ledOn);
		if ((vcom_connected() != 0)) {
			//vcom_write("Hello World!\r\n", 14);
			printf("Hi all!\r\n");
			//_write(1, "Hi everyone!\r\n", 14);
		}
	}
}

int
_write(int fd, char *buf, int nbytes)
{
	int ret;

	if (nbytes <= 0)
		return 0;

	if (fd != STDERR_FILENO && fd != STDOUT_FILENO) {
		errno = EBADF;
		return -1;
	}

	while ((ret = vcom_write(buf, nbytes)) == 0) {
		__WFI();
	}

	return ret;
}

int
_read(int fd, char *ptr, int len)
{
	int rd_cnt;
	int remaining = len;

	if (len == 0)
		return len;

	if (fd != STDIN_FILENO) {
		errno = EBADF;
		return -1;
	}

	while ((rd_cnt = vcom_bread(ptr, len)) == 0) {
		__WFI();
	}

	return rd_cnt;
}

void *_sbrk(int incr)
{
	static char *heap_end = NULL;
	extern char _pvHeapStart;
	char *ret;

	if (heap_end == NULL)
		heap_end = &_pvHeapStart;

	ret = heap_end;

	heap_end += incr;

	return ret;
}

int readline(char *buf, int buf_sz)
{
	char ch;
	int cnt = 0;

	--buf_sz;

	while (buf_sz > 0) {
		ch = getchar();
		if (ch == '\r' || ch == '\n') {
			*buf = '\0';
			return cnt;
		}

		*buf++ = ch;
		++cnt;
	}

	*buf = '\0';
	return buf;
}

void testILI9341(void);

void test_task(uint32_t arg)
{
	int led_state = 1;
	int last_curtick = 0;
	for (;;) {
		Chip_GPIO_SetPinState(LPC_GPIO, 1, 8, led_state);
		led_state = !led_state;
		last_curtick = curtick;
		sched_sleep("ttslp", 1500);
	}
}

void test2_task(uint32_t arg)
{
	int led_state = 1;
	int last_curtick = -1;
	for (;;) {
		if ((curtick != last_curtick) && (curtick - last_curtick) > 1000) {
			Chip_GPIO_SetPinState(LPC_GPIO, 1, 9, led_state);
			led_state = !led_state;
			last_curtick = curtick;
		}
	}
}

int main(void) {
	char buf[128];
	char txbuf[128];
	int  buf_sz;
	int  buf_rem_sz;

#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif
	SystemCoreClockUpdate();
	/* enable clocks and pinmux */
	usb_pin_clk_init();

	uint32_t timerFreq;

	Chip_TIMER_Init(LPC_TIMER32_0);
	timerFreq = Chip_Clock_GetSystemClockRate();

	Chip_TIMER_Reset(LPC_TIMER32_0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_0, 1);
	Chip_TIMER_SetMatch(LPC_TIMER32_0, 1, (timerFreq / TICKRATE1_HZ));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_0, 1);
	Chip_TIMER_Enable(LPC_TIMER32_0);
	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER_32_0_IRQn);
	NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);

	// TODO: insert code here
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 8);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 9);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 10);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 11);

	Chip_GPIO_SetPinState(LPC_GPIO, 1, 11, ledOn);


	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 17); // ILI9341 RESET
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 18); // ILI9341 CS
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 19); // ILI9341 CD

	Chip_GPIO_SetPinState(LPC_GPIO, 0, 18, 1); // ILI9341 CS

	spi1_setup();

	Chip_GPIO_SetPinState(LPC_GPIO, 1, 8, 1);

	//testILI9341();
	sched_init();

	task_create(test_task, 0, -100, 256, "test");
	task_create(test2_task, 0, -100, 256, "test2");


	/* Disable interrupts - sched_start() will re-enable them */
	crit_enter();
	SysTick_Config(SystemCoreClock / TICKRATE1_HZ);

	*((uint32_t *)0xE000ED20) = (0xff << 24) | (0xff << 16);

	//spi1_write8(0xAA);
	//spi1_write8(0xBB);
	//spi1_write8(0xCC);
	//spi1_write8(0xDD);

	sched_start();

	while (1) {
		__WFI();

		//continue;
		//spi1_write8(0xaa);
		//Chip_GPIO_SetPinState(LPC_GPIO, 1, 11, ledOn);
		//ledOn = !ledOn;
	}



	Chip_GPIO_SetPinState(LPC_GPIO, 1, 8, 0);


	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret = LPC_OK;
	uint32_t prompt = 0, rdCnt = 0;





	/* initialize USBD ROM API pointer. */
	g_pUsbApi = (const USBD_API_T *) LPC_ROM_API->usbdApiBase;

	/* initialize call back structures */
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB0_BASE;
	/*	WORKAROUND for artf44835 ROM driver BUG:
	    Code clearing STALL bits in endpoint reset routine corrupts memory area
	    next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
	    EP2_IN are used we need to specify 3 here. But as a workaround for this
	    issue specify 4. So that extra EPs control structure acts as padding buffer
	    to avoid data corruption. Corruption of padding memory doesn’t affect the
	    stack/program behaviour.
	 */
	usb_param.max_num_ep = 3 + 1;
	usb_param.mem_base = USB_STACK_MEM_BASE;
	usb_param.mem_size = USB_STACK_MEM_SIZE;

	/* Set the USB descriptors */
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	/* Note, to pass USBCV test full-speed only devices should have both
	   descriptor arrays point to same location and device_qualifier set to 0.
	 */
	desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.device_qualifier = 0;

	/* USB Initialization */
	ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
	if (ret == LPC_OK) {


		/*	WORKAROUND for artf32219 ROM driver BUG:
		    The mem_base parameter part of USB_param structure returned
		    by Init() routine is not accurate causing memory allocation issues for
		    further components.
		 */
		usb_param.mem_base = USB_STACK_MEM_BASE + (USB_STACK_MEM_SIZE - usb_param.mem_size);

		/* Init VCOM interface */
		ret = vcom_init(g_hUsb, &desc, &usb_param);
		if (ret == LPC_OK) {
			Chip_GPIO_SetPinState(LPC_GPIO, 1, 9, true);

			/*  enable USB interrupts */
			NVIC_EnableIRQ(USB0_IRQn);
			/* now connect */
			USBD_API->hw->Connect(g_hUsb, 1);
		}

	}

	bool blinky = false;
	Chip_GPIO_SetPinState(LPC_GPIO, 1, 8, 1);

	buf_rem_sz = sizeof(buf);
	buf_sz = 0;

	while (!prompt) {
		/* Check if host has connected and opened the VCOM port */
		if ((vcom_connected() != 0) && (prompt == 0)) {
			vcom_write("Hello World!!\r\n", 15);
			//printf("Hello World!!\r\n");
			prompt = 1;
			Chip_GPIO_SetPinState(LPC_GPIO, 1, 8, false);

		}
		__WFI();
	}

	while (1) {
		Chip_GPIO_SetPinState(LPC_GPIO, 1, 8, blinky);
		blinky = !blinky;
		if ((readline(buf, sizeof(buf))) != 0)
			printf("Echo: |%s|\r\n", buf);

		/* Sleep until next IRQ happens */
		__WFI();
	}



	// Force the counter to be placed into memory
	// Enter an infinite loop, just incrementing a counter
	while(1) {
		__WFI();
	}
	return 0 ;
}

#if 0

struct pid_state {
	/* Settings */
	float	setpoint;
	float	output_max;
	float	output_min;

	/* Working set */
	float	last_input;
	float	last_err;
	int8_t	last_sat;

	float	i_sum;
};

void pid_initialize(struct pid_state *state, float input)
{
	state->last_input = input;
	/*
	 * XXX: initialize i_sum to current output, for smooth transitioning
	 *      between auto/manual.
	 */
}

float pid_update(struct pid_state *state, float input)
{
	float err;
	float d_term;
	float i_term;
	float output;

	err = state->setpoint - input;

	i_term = state->i_sum + (state->ki * err);

#ifdef PID_USE_D_ERR
	d_term = (err - state->last_err);
#else
	d_term = (state->last_input - input);
#endif

	output = (state->kp * err) + (i_term) + (state->kd * d_term);

	/*
	 * Clamp output to a value between the min/max levels.
	 *
	 * Also deal with windup by readjusting the integral term so that
	 * it gives just enough to saturate to the min/max level.
	 */
	/* XXX: Alternatively, just don't update state->i_sum (and i_term)
	 *      if the saturation occurs in the same direction as the error.
	 *      That means, however, that we need to keep some information
	 *      about the saturation state, as the output also needs to be
	 *      calculated with the non-updated version of i_sum.
	 */
	if (output > state->output_max) {
		i_term -= (output - state->output_max);
		output = state->output_max;
		state->last_sat = +1;
	} else if (output < state->output_min) {
		i_term += (state->output_min - output);
		output = state->output_min;
		state->last_sat = -1;
	} else {
		state->last_sat = 0;
	}

	state->last_input = input;
	state->last_err   = err;
	state->i_sum      = i_term;

	return output;
}

#endif

