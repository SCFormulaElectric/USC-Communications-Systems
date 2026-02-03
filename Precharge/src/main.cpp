#include "stm32c0xx.h"

//Global Variables
volatile uint32_t msTicks = 0;
volatile uint32_t last_capture = 0;
volatile uint32_t frequency = 0;
    //volatile uint32_t inspect_var = 0;

//SysTick Interrupt Handler
extern "C" void SysTick_Handler(void) {
    msTicks++;
}

extern "C" void TIM1_CC_IRQHandler(void) {
    /*
    This handler is entered 
    */
    // Everything else stays almost the same, just change TIM2 to TIM1
    if (TIM1->SR & TIM_SR_CC1IF) {
        uint32_t current_capture = TIM1->CCR1;
        uint32_t diff = current_capture - last_capture;

        if (diff > 0) {
            frequency = 8000000 / diff;
        }

        last_capture = current_capture;
        TIM1->SR = ~TIM_SR_CC1IF;
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
    // 1. Enable GPIOA and TIM1 Clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // TIM1 is on APB2

    // 2. Set PA5 to Alternate Function Mode (10)
    GPIOA->MODER &= ~GPIO_MODER_MODE5;
    GPIOA->MODER |= GPIO_MODER_MODE5_1;

    // 3. Set PA5 Alternate Function to AF5 (TIM1_CH1)
    // AFR[0] is AFRL. PA5 uses bits 20-23.
    // We use '5' here for AF5.
    GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL5_Pos);
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos); 
}

void TIM1_Config(void) {
    // 1. TIM1 is on APB2, so ensure the clock is enabled
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

    // 2. Set Prescaler and Auto-Reload
    // At 8MHz, a 16-bit timer overflows every 8.19ms (65536 / 8,000,000)
    // this has undefined overflow behavior.
    TIM1->PSC = 0;           
    TIM1->ARR = 0xFFFF;      // 16-bit limit for TIM1

    // 3. Configure Channel 1 as Input (CC1S = 01)
    TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;

    // 4. Input Filter and Prescaler
    // Setting IC1F to 0000 (No filter) for maximum responsiveness at 15kHz
    TIM1->CCMR1 &= ~TIM_CCMR1_IC1F; 
    
    // 5. Detection on Rising Edge (CC1P=0, CC1NP=0)
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    
    // 6. Enable Capture on Channel 1
    TIM1->CCER |= TIM_CCER_CC1E;

    // 7. Enable Capture/Compare Interrupt and Start Timer
    TIM1->DIER |= TIM_DIER_CC1IE;
    TIM1->CR1 |= TIM_CR1_CEN;

    // 8. Enable the specific TIM1 Capture Compare IRQ
    NVIC_EnableIRQ(TIM1_CC_IRQn);
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