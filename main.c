#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#define CORTEX_INTERRUPTS 16
#define NVIC_CHANNELS 43
#define PIN(bank, num) (uint32_t)(((bank - 'A') << 16) | num)
#define PINNO(pin) (uint32_t)(pin & 65535)
#define PINBANK(pin) (uint32_t)(pin >> 16)
#define REGISTERS(bank) (gpio *) (0x40010800 + (uintptr_t) (bank * 0x400))
#define RCC ((rcc *) (0x40021000))
#define BITS(x) (1ul << x)
#define SYSTICK ((struct systick *) 0xe000e010)

enum {INPUT_MODE, OUTPUT_MODE_10, OUTPUT_MODE_2, OUTPUT_MODE_50};
enum {ANALOG_MODE, FLOATING_INPUT, INPUT_PUPD, RESERVED};
enum {GPOPP, GPOOD, AFOPP, AFOOD};

typedef struct gpio {
   volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} gpio;

typedef struct rcc {
   volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR;
} rcc;

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

extern void _esram (void);
extern uint32_t _sdata, _edata, _sbss, _ebss, _etext;
static volatile uint32_t s_ticks; // volatile is important!!
static inline void enable_bank (uint32_t pin);
static inline void gpio_set_mode (uint32_t pin, uint32_t mode, uint32_t CNF);
static inline void gpio_set (uint32_t pin);
static inline void gpio_reset (uint32_t pin);
void SysTick_Handler(void);
static inline void systick_init(uint32_t ticks);
void delay(unsigned ms);
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now);
__attribute__((noreturn))void _reset (void);

__attribute__((section(".isr_vector"))) void (*const isr_vectors[CORTEX_INTERRUPTS + NVIC_CHANNELS]) (void) = {
_esram, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};

int main (void)
{
    uint32_t pin = PIN('C', 13);
    systick_init(16000000 / 1000);
    enable_bank(pin);
    gpio_set_mode(pin, OUTPUT_MODE_50, GPOPP);
    uint32_t timer, period = 500;
    for (;;) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on;       // This block is executed
            (on) ? gpio_reset(pin) : gpio_set(pin);
            on = !on;             // Toggle LED state
        }
        // Here we could perform other activities!
    };
    return 0;
}

__attribute__((noreturn))void _reset (void)
{
    //move .data to sram first
    for (uint32_t *loc = &_sdata, *data = &_etext; loc < &_edata;)
    {
        *loc++ = *data++;
    }
    for (uint32_t *loc = &_sbss; loc < &_ebss; loc++)
    {
        *loc = 0;
    }
    main();
    for (;;) (void) 0;
}

static inline void enable_bank (uint32_t pin){
    rcc *reg_address = RCC;
    uint32_t bank = PINBANK(pin);
    reg_address->APB2ENR |= BITS((bank + 2));
}

static inline void gpio_set_mode (uint32_t pin, uint32_t mode, uint32_t CNF){
    uint32_t pin_number = PINNO(pin);
    uint32_t bank = PINBANK(pin);
    uint32_t slot = pin_number & 7;
    gpio *reg_address = REGISTERS(bank);

    if (pin_number > 7) {
        reg_address->CRH &= ~((uint32_t)(15 << (slot * 4)));
        reg_address->CRH |= (uint32_t)((((CNF << 2) | mode) & 15) << (slot * 4));
    }

    else {
        reg_address->CRL &= ~((uint32_t)(15 << (slot * 4)));
        reg_address->CRL |= (uint32_t)((((CNF << 2) | mode) & 15) << (slot * 4));
    }
}

static inline void gpio_set (uint32_t pin){
    uint32_t pin_number = PINNO(pin);
    uint32_t bank = PINBANK(pin);
    uint32_t slot = pin_number & 15;
    gpio *reg_address = REGISTERS(bank);

    reg_address->BSRR = BITS(slot);
}

static inline void gpio_reset (uint32_t pin){
    uint32_t pin_number = PINNO(pin);
    uint32_t bank = PINBANK(pin);
    uint32_t slot = (pin_number & 15) + 16;
    gpio *reg_address = REGISTERS(bank);

    reg_address->BSRR = BITS(slot);
}

static inline void systick_init(uint32_t ticks) {
    if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = BITS(0) | BITS(1) | BITS(2);  // Enable systick
    RCC->APB2ENR |= (uint32_t)BITS(14);                   // Enable SYSCFG
}

void SysTick_Handler(void) {
    s_ticks++;
}

void delay(unsigned ms) {            // This function waits "ms" milliseconds
    uint32_t until = s_ticks + ms;      // Time in a future when we need to stop
    while (s_ticks < until) (void) 0;   // Loop until then
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
    if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
    if (*t == 0) *t = now + prd;                   // First poll? Set expiration
    if (*t > now) return false;                    // Not expired yet, return
    *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
    return true;                                   // Expired, return true
}
