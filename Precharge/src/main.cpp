#include "stm32c0xx.h"

//prescalar confing & TIM1IRQ is handled by this;
#define FREQUENCY 500000UL

//Global Variables
volatile uint32_t msTicks = 0;
volatile uint16_t last_capture = 0;
volatile uint32_t frequency = 0;
volatile uint32_t overflow_count = 0;
    //volatile uint32_t inspect_var = 0;

//SysTick Interrupt Handler
extern "C" void SysTick_Handler(void) {
    msTicks++;
}

extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) {
        overflow_count++;
        TIM1->SR &= ~TIM_SR_UIF; // Clear flag
    }
}

extern "C" void TIM1_CC_IRQHandler(void) {
    /*
    This handler is entered 
    */
    // Everything else stays almost the same, just change TIM2 to TIM1
    static uint16_t firstTimeFlag = 0;
    if (TIM1->SR & TIM_SR_CC1IF) {
        uint16_t current_capture = TIM1->CCR1;
        uint16_t diff = 0; // if diff is 0 then no freq calculation saving clockcs
        if (firstTimeFlag != 0){
            diff = current_capture - last_capture;
        }
        else{
            firstTimeFlag = 1;
        }

        if (diff > 0) {
            frequency = FREQUENCY / diff; // ok if this is 100 clock cycles, then this ISR is 12.5uS long
        }

        last_capture = current_capture;
        
        //TIM1->SR = ~TIM_SR_CC1IF; //clears loop condition flag, resetting IRQ
        // Apparently overflow flag might also be cleared so we have to solely reset the CC1IF 
        TIM1->SR &= ~TIM_SR_CC1IF;
    }
}

//Delay Function
void delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
        // Wait For Interrupt
        __WFI(); 
    }
}

// 8MHz External Oscillator
void SystemClock_Config_HSE(void) {
    // 1. Enable HSE and wait until ready
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // 2. Set Flash Latency to 0 (at 8MHz, we don't need wait states)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;

    // 3. Select HSE as System Clock
    RCC->CFGR &= ~RCC_CFGR_SW; // Clear bits
    RCC->CFGR |= RCC_CFGR_SW_0; // 01: HSE selected as system clock
    
    // 4. Wait for switch to take effect
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_0);

    SystemCoreClockUpdate(); // Updates global variable to 8000000
}

void Timer_Input_Init(void) {
    // enable GPIOA and TIM1 Clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // TIM1 is on APB2

    // set PA5 to Alternate Function Mode 10
    GPIOA->MODER &= ~GPIO_MODER_MODE5;
    GPIOA->MODER |= GPIO_MODER_MODE5_1;

    // set PA5 Alternate Function to AF5 TIM1_CH1
    // AFR[0] is AFRL PA5 uses bits 20-23.
    //  use '5' here for AF5.
    GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL5_Pos);
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos); 
}

void TIM1_Config(void) {
    // set TIM1EN register, turns on tim1 clock
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

    // we calculated min sampling frequency of 8Hz with prescalar 16 (15 + 1). 
    TIM1->PSC = (8000000/FREQUENCY) - 1;           

    //autoreload register max value (16bit = 0xffff), triggers interrupt and goes to 0 after arr ==0xff
    TIM1->ARR = 0xFFFF;

    //clear capture-compare mode register 1 
    TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;

    //set channel 1 to input
    TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;
    

    //Filter not needed
    //TIM1->CCMR1 &= ~TIM_CCMR1_IC1F; 
    
    // rising edge capture mode CC1P=0, CC1NP=0
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    
    // channel 1 capture enabled; on every rising edge, TIM1 CCR1 = TIM1 CNT
    TIM1->CCER |= TIM_CCER_CC1E;

    // Enable Capture/Compare Interrupt 
    TIM1->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);

    //start timer
    TIM1->CR1 |= TIM_CR1_CEN;

    //enable  TIM1 Capture Compare interrupts
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn); //also enable the IRQ for update interrupt flag (Overflow)
}

// --- Main ---
int main() {
    SystemClock_Config_HSE(); // Set to 8MHz External Crystal
    Timer_Input_Init();       // Configure PA5 as AF2 (TIM2_CH1)
    TIM1_Config();
    SysTick_Config(SystemCoreClock / 1000);
    uint32_t counter = 0;
    uint32_t past = 0;

    while (1) {
        if (msTicks - past > 5000){
            counter++;
            past = msTicks;
        }

    }
}