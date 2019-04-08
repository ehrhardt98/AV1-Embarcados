/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

// Botão3
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX  19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

#define raio 0.325
#define pi 3.14159265359

struct ili9488_opt_t g_ili9488_display_opt;
volatile Bool f_rtt_alarme = false;
volatile float counter_vel = 0.0;
volatile float counter_dist = 0.0;
volatile float vel = 133.0;
volatile float dist = 0.0;
volatile char buffer_vel[32];
volatile char buffer_dist[32];

void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void configure_lcd(void);

void but3_callback(void)
{
	counter_vel++;
	counter_dist++;
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		vel = 3.6*raio*2*pi*counter_vel/4;
		dist = 2*pi*raio*counter_dist;
		sprintf(buffer_vel, "Velocidade: %f", vel);
		sprintf(buffer_dist, "Distancia: %f", dist);
		counter_vel = 0.0;
		f_rtt_alarme = true;                  // flag RTT alarme
		configure_lcd();
	}
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but3_callback);
	
	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);
	pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 20);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}


int main(void) {
	board_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	sysclk_init();	
	io_init();
	configure_lcd();
	
	f_rtt_alarme = true;
	
	while(1) {
		if(f_rtt_alarme) {
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 8;
			RTT_init(pllPreScale, irqRTTvalue);
			font_draw_text(&calibri_36, buffer_vel, 10, 100, 1);
			font_draw_text(&calibri_36, buffer_dist, 10, 200, 1);
			f_rtt_alarme = false;
		}
	}
}