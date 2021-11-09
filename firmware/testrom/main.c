// This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
// License: BSD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <libliteeth/mdio.h>
#include <generated/csr.h>

/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void)
{
	char c[2];
	static char s[64];
	static int ptr = 0;

	if (readchar_nonblock())
	{
		c[0] = getchar();
		c[1] = 0;
		switch (c[0])
		{
		case 0x7f:
		case 0x08:
			if (ptr > 0)
			{
				ptr--;
				fputs("\x08 \x08", stdout);
			}
			break;
		case 0x07:
			break;
		case '\r':
		case '\n':
			s[ptr] = 0x00;
			fputs("\n", stdout);
			ptr = 0;
			return s;
		default:
			if (ptr >= (sizeof(s) - 1))
				break;
			fputs(c, stdout);
			s[ptr] = c[0];
			ptr++;
			break;
		}
	}

	return NULL;
}

static char *get_token(char **str)
{
	char *c, *d;

	c = (char *)strchr(*str, ' ');
	if (c == NULL)
	{
		d = *str;
		*str = *str + strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c + 1;
	return d;
}

static void prompt(void)
{
	printf("testrom-cmd> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
	puts("\nLiteX minimal demo app built "__DATE__
		 " "__TIME__
		 "\n");
	puts("Available commands:");
	puts("help               - Show this command");
	puts("reboot             - Reboot CPU");
#ifdef CSR_LEDS_BASE
	puts("led                - Led demo");
#endif
	puts("donut              - Spinning Donut demo");
	puts("helloc             - Hello C");
#ifdef WITH_CXX
	puts("hellocpp           - Hello C++");
#endif
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
	ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE
const uint16_t sine_falloff_fade[] = {0x3ff,0x3f8,0x3ea,0x3cf,0x3ae,0x387,0x354,0x31b,0x2df,0x29f,0x256,0x210,0x1c5,0x17e,0x138,0x0f4,0x0b8,0x07f,0x04d,0x028,0x00c,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000};
const uint16_t sine_pulse_fade[] = {0x000,0x000,0x000,0x017,0x04d,0x09a,0x0f4,0x15a,0x1c5,0x235,0x29f,0x2fd,0x354,0x39a,0x3cf,0x3f1,0x3ff,0x3f1,0x3cf,0x39a,0x354,0x2fd,0x29f,0x235,0x1c5,0x15a,0x0f4,0x09a,0x04d,0x017,0x000,0x000};

static void led_cmd(void)
{
		volatile uint32_t *p = (uint32_t*)CSR_LEDS_OUT0_ADDR;			
	for(int j = 0; j < 3; j++){
		for(int count = 0; count < 40; count++){
			for(int i = 0; i < 7; i++){
				p[i] = (uint32_t)sine_falloff_fade[((count<<1) + i*4) % 32] << (10 * j); /* GREEN*/
			}
		busy_wait(50);
		}
		
	}

	for(int i = 0; i < 7; i++){
				p[i] =0;
			}
}
#endif

extern void donut(void);

static void donut_cmd(void)
{
	printf("Donut demo...\n");
	donut();
}

extern void helloc(void);

static void helloc_cmd(void)
{
	printf("Hello C demo...\n");
	helloc();
}

static void vccio_cmd(char *c)
{
	int v = 0;
	sscanf(c, "%u", &v);

	vccio_enable_write(1);
	vccio_ch0_write(v);
	vccio_ch1_write(v);
	vccio_ch2_write(v);
	busy_wait(150);
}

static void eth_phy_check(void)
{

	printf("phy_mdio_read(%u)=%04x\n", 2, mdio_read(3, 2));
	printf("phy_mdio_read(%u)=%04x\n", 3, mdio_read(3, 3));
	busy_wait(100);
}

static void io_test(void)
{

	for (int i = 0; i < 32; i++)
	{
		gpioa_oe_write(1 << i);
		gpioa_out_write(0);
		busy_wait_us(100);
		if (gpioa_in_read() != ~(1 << i))
		{
			printf("PORTA: %08x != %08x\n I/O PORT A failed\n", gpioa_in_read(), ~(1 << i));
			return;
		}
	}

	printf("I/O PORT A passed\n");

	for (int i = 0; i < 32; i++)
	{
		gpiob_oe_write(1 << i);
		gpiob_out_write(0);
		busy_wait_us(100);
		if (gpiob_in_read() != ~(1 << i))
		{
			printf("PORTB: %08x != %08x\n I/O PORT B failed\n", gpiob_in_read(), ~(1 << i));
			return;
		}
	}

	printf("I/O PORT B passed\n");

	for (int i = 0; i < 14; i++)
	{
		gpioc_oe_write(1 << i);
		gpioc_out_write(0);
		busy_wait_us(100);
		if (gpioc_in_read() != (~(1 << i) & 0x3fff))
		{
			printf("PORTC: %08x != %08x\n I/O PORT C failed\n", gpioc_in_read(), ~(1 << i) & 0x3fff);
			return;
		}
	}

	printf("I/O PORT C passed\n");
}

#ifdef WITH_CXX
extern void hellocpp(void);

static void hellocpp_cmd(void)
{
	printf("Hello C++ demo...\n");
	hellocpp();
}
#endif

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
	char *str;
	char *token;

	str = readstr();
	if (str == NULL)
		return;
	token = get_token(&str);
	if (strcmp(token, "help") == 0)
		help();
	else if (strcmp(token, "reboot") == 0)
		reboot_cmd();
#ifdef CSR_LEDS_BASE
	else if (strcmp(token, "led") == 0)
		led_cmd();
#endif
	else if (strcmp(token, "donut") == 0)
		donut_cmd();
	else if (strcmp(token, "helloc") == 0)
		helloc_cmd();
#ifdef WITH_CXX
	else if (strcmp(token, "hellocpp") == 0)
		hellocpp_cmd();
#endif
	else if (strcmp(token, "vccio") == 0)
		vccio_cmd(get_token(&str));
	else if (strcmp(token, "eth_phy") == 0)
		eth_phy_check();
	else if (strcmp(token, "io_test") == 0)
		io_test();
	prompt();
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();

	help();
	prompt();

	while (1)
	{
		console_service();
	}

	return 0;
}
