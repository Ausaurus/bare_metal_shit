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
#define FLASH_ACR ((volatile uint32_t *) 0x40022000)
#define REFDEBOUNCE 30

enum {INPUT_MODE, OUTPUT_MODE_10, OUTPUT_MODE_2, OUTPUT_MODE_50};
enum {ANALOG_MODE, FLOATING_INPUT, INPUT_PUPD, RESERVED};
enum {GPOPP, GPOOD, AFOPP, AFOOD};
enum {NULLPD, PU};

typedef struct gpio {
   volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} gpio;

typedef struct rcc {
   volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR;
} rcc;

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

typedef struct {
    uint16_t c0, c1;
    volatile uint8_t stable;
} Debounce;

extern void _esram (void);
extern uint32_t _sdata, _edata, _sbss, _ebss, _etext;
static volatile uint32_t s_ticks; // volatile is important!!
static inline void enable_bank (uint32_t);
static inline void gpio_set_mode (uint32_t, uint32_t, uint32_t, uint32_t);
static inline void gpio_set (uint32_t);
static inline void gpio_reset (uint32_t);
static inline void gpio_toggle (uint32_t);
static inline uint8_t gpio_read (uint32_t);
static inline void debounce_check (volatile Debounce *, uint8_t);
// static inline bool gpio_read_hl (uint32_t, uint32_t *);
void SysTick_Handler(void);
static inline void systick_init(uint32_t);
void delay(uint32_t ms);
bool timer_expired(uint32_t *, uint32_t, uint32_t);
static inline void clock_setup();
__attribute__((noreturn))void _reset (void);

__attribute__((section(".isr_vector"))) void (*const isr_vectors[CORTEX_INTERRUPTS + NVIC_CHANNELS]) (void) = {
_esram, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};

uint32_t pinB13 = PIN('B', 13);
uint32_t pinB15 = PIN('B', 15);
uint32_t pinC13 = PIN('C', 13);
uint32_t pinA0 = PIN('A', 0);
uint32_t pinA6 = PIN('A', 6);

volatile Debounce dbA0 = {0, 0, 0};
volatile Debounce dbA6 = {0, 0, 0};

int main (void)
{
    clock_setup();

    systick_init(72000000 / 1000);
    enable_bank(pinB13);
    enable_bank(pinC13);
    enable_bank(pinA0);

    gpio_set_mode(pinB13, OUTPUT_MODE_50, GPOPP, NULLPD);
    gpio_set_mode(pinB15, OUTPUT_MODE_50, GPOPP, NULLPD);
    gpio_set_mode(pinC13, OUTPUT_MODE_50, GPOPP, NULLPD);
    gpio_set_mode(pinA0, INPUT_MODE, INPUT_PUPD, PU);
    gpio_set_mode(pinA6, INPUT_MODE, INPUT_PUPD, PU);
    gpio_set(pinC13);

    uint32_t timer, prd = 500;
    volatile uint32_t prevA0 = 0, prevA6 = 0;

    for (;;) {

        volatile uint8_t curA0 = dbA0.stable;
        volatile uint8_t curA6 = dbA6.stable;

        if (curA0 && !prevA0) {
            gpio_toggle(pinB13);
        }

        if (curA6 && !prevA6) {
            gpio_toggle(pinB15);
        }

        if (timer_expired(&timer, prd, s_ticks)) {
            gpio_toggle(pinC13);
        }
        // Here we could perform other activities!
        prevA0 = curA0;
        prevA6 = curA6;
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

static inline void gpio_set_mode (uint32_t pin, uint32_t mode, uint32_t CNF, uint32_t pupd){
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

    if (CNF == INPUT_PUPD) {
        if (pupd) {
            reg_address->ODR |= BITS(pin_number);
        }
        else {
            reg_address->ODR &= ~BITS(pin_number);
        }
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
    uint32_t slot = pin_number & 15;
    gpio *reg_address = REGISTERS(bank);

    reg_address->BRR = BITS(slot);
}

static inline void gpio_toggle (uint32_t pin){
    static uint32_t last_ran = 0;
    if (s_ticks - last_ran < 100) {
        return (void) 0;
    }
    uint32_t pin_number = PINNO(pin);
    uint32_t bank = PINBANK(pin);
    uint32_t slot = (pin_number & 15);
    gpio *reg_address = REGISTERS(bank);

    if ((reg_address->ODR >> slot) & 1) {
        reg_address->BRR = BITS(slot);
    }
    else {
        reg_address->BSRR = BITS(slot);
    }
    last_ran = s_ticks;
}

static inline uint8_t gpio_read(uint32_t pin) {
    uint32_t pin_number = PINNO(pin);
    uint32_t bank = PINBANK(pin);
    uint32_t slot = (pin_number & 15);
    gpio *reg_address = REGISTERS(bank);

    return ((reg_address->IDR >> slot) & 1u);
}

static inline void debounce_check (volatile Debounce *d, uint8_t lvl) {
    if (lvl) {
        d->c1++;
        d->c0 = 0;
        if (d->c1 >= REFDEBOUNCE) d->stable = 1;
    }

    else {
        d->c0++;
        d->c1 = 0;
        if (d->c0 >= REFDEBOUNCE) d->stable = 0;
    }
}

// static inline bool gpio_read_hl (uint32_t pin, uint32_t *prev) {
//     bool cur = statoln1;
//
//     bool falling = !cur && *prev;
//
//     *prev = cur;
//
//     return falling;
// }

static inline void systick_init(uint32_t ticks) {
    if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = BITS(0) | BITS(1) | BITS(2);  // Enable systick
}

void SysTick_Handler(void) {
    s_ticks++;
    debounce_check(&dbA0, gpio_read(pinA0));
    debounce_check(&dbA6, gpio_read(pinA6));
}

void delay(uint32_t ms) {            // This function waits "ms" milliseconds
    uint32_t until = s_ticks + ms;      // Time in a future when we need to stop
    uint32_t timer, period = ms;
    while (s_ticks < until) (void) 0;   // Loop until then
    while (true) {
        if (timer_expired(&timer, period, s_ticks)) break;
    }
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
    if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
    if (*t == 0) *t = now + prd;                   // First poll? Set expiration
    if (*t > now) return false;                    // Not expired yet, return
    *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
    return true;                                   // Expired, return true
}

static inline void clock_setup() {
    // turn on HSE
    RCC->CR |= (1u << 16);
    while ((RCC->CR & BITS(17)) == 0){}; // wait for HSE to be ready
    //
    *FLASH_ACR |= BITS(4) | 2u;

    // configure PLL
    RCC->CFGR |= BITS(18) | BITS(19) | BITS(20); // configure PLLMUL
    RCC->CFGR &= ~BITS(17); // configure PLLXTPRE
    RCC->CFGR |= BITS(16); // configure PLLSRC
    RCC->CR |= BITS(24); // turn on PLL
    while ((RCC->CR & BITS(25)) == 0){}; // waiting for PLL to be ready

    RCC->CFGR &= ~BITS(13); // BITS(10);configure APB2 prescaler
    RCC->CFGR |= (4u << 8);
    RCC->CFGR &= ~BITS(7); // configure AHB prescaler
    RCC->CFGR |= 2u; // select SYSCLK source
    while (((RCC->CFGR >> 2) & 3u) != 2u){}; // waiting PLL to be selected
}
