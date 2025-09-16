#include <stdint.h>
#include <sys/types.h>

#define CORTEX_INTERRUPTS 16
#define NVIC_CHANNELS 43
#define PIN(bank, num) (uint32_t)(((bank - 'A') << 16) | num)
#define PINNO(pin) (uint32_t)(pin & 65535)
#define PINBANK(pin) (uint32_t)(pin >> 16)
#define REGISTERS(bank) (gpio *) (0x40010800 + (uintptr_t) (bank * 0x400))
#define RCC (rcc *) (0x40021000)

enum {INPUT_MODE, OUTPUT_MODE_10, OUTPUT_MODE_2, OUTPUT_MODE_50};
enum {
    PIN_1, PIN_2, PIN_3,
    PIN_4, PIN_5, PIN_6,
    PIN_7, PIN_8, PIN_9,
    PIN_10, PIN_11, PIN_12,
    PIN_13
};
enum {ANALOG_MODE, FLOATING_INPUT, INPUT_PUPD, RESERVED};
enum {GPOPP, GPOOD, AFOPP, AFOOD};

typedef struct gpio {
   uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} gpio;

typedef struct rcc {
   uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR;
} rcc;

extern void _esram (void);
extern uint32_t _sdata, _edata, _sbss, _ebss, _etext;
static inline void enable_bank (uint32_t pin);
static inline void gpio_set_mode (uint32_t pin, uint32_t mode, uint32_t CNF);
static inline void gpio_set (uint32_t pin);
static inline void gpio_reset (uint32_t pin);
__attribute__((noreturn))void _reset (void);

__attribute__((section(".isr_vector"))) void (*const isr_vectors[CORTEX_INTERRUPTS + NVIC_CHANNELS]) (void) = {
_esram, _reset
};

int main (void)
{
    uint32_t pin = PIN('C', PIN_13);
    enable_bank(pin);
    gpio_set_mode(pin, OUTPUT_MODE_50, GPOPP);
    gpio_reset(pin);
    for (;;) (void) 0;
    return 0;
}

__attribute__((noreturn))void _reset (void)
{
    //move .data to sram first
    for (uint32_t *loc = &_sdata, *data = &_etext; loc < &_sdata; loc++)
    {
        *loc = *data;
        data++;
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
    uint32_t slot = (bank & 7) + 2;
    reg_address->APB2ENR &= ~(uint32_t)(1 << slot);
    reg_address->APB2ENR |= (uint32_t)(1 << (slot));
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

    reg_address->BSRR = 1 << slot;
}

static inline void gpio_reset (uint32_t pin){
    uint32_t pin_number = PINNO(pin);
    uint32_t bank = PINBANK(pin);
    uint32_t slot = (pin_number & 15) + 16;
    gpio *reg_address = REGISTERS(bank);

    reg_address->BSRR = 1 << slot;
}
