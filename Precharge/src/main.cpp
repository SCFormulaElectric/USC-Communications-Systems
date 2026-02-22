#include "stm32c0xx.h"

//prescalar confing & TIM1IRQ is handled by this;
//TIMER FREQUENCY. SYSCLK IS AT CONSTANT 8MHz
#define FREQUENCY 500000UL

// POST-AIR Global Variables
volatile uint32_t msTicks_post = 0;
volatile uint32_t frequency_post = 0;
volatile uint32_t overflow_count_post = 0;

// POST-AIR - Volatile variables that are only global for GDB/testing
volatile uint32_t current_capture_32_post = 0;
volatile uint16_t diff_post = 0;
volatile uint32_t last_capture_32_post = 0;
static uint8_t is_initialized_post = 0;
    //volatile uint32_t inspect_var = 0;

// // PRE-AIR - Global variables
// volatile uint32_t msTicks_pre = 0;
// volatile uint32_t frequency_pre = 0;
// volatile uint32_t overflow_count_pre = 0;

// // PRE-AIR - Volatile variables that are only global for GDB/testing
// volatile uint32_t current_capture_32_pre = 0;
// volatile uint16_t diff_pre = 0;
// volatile uint32_t last_capture_32_pre = 0;
// static uint8_t is_initialized_pre = 0;


//SysTick Interrupt Handler
extern "C" void SysTick_Handler(void) {
    msTicks_post++;
}

extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) {
        overflow_count_post++;
        TIM1->SR &= ~TIM_SR_UIF; // Clear flag
    }
}


extern "C" void TIM1_CC_IRQHandler(void) {
    /*
    This handler is entered 
    */
    // Everything else stays almost the same, just change TIM2 to TIM1
    uint32_t ovf = overflow_count_post;
    uint16_t cap = TIM1->CCR1;

    if ((TIM1->SR & TIM_SR_UIF) && cap < 0x8000) {
        ovf++;
    }
    current_capture_32_post = (ovf << 16) | cap;

    if (is_initialized_post) {
            //period between rising edges ((T+Tn) - T) = Tn
            diff_post = current_capture_32_post - last_capture_32_post;

            if (diff_post > 0) {
                // you can 
                frequency_post = FREQUENCY / diff_post; 
            }
    }
    else {
            // This handles case where only 1 rising edge has occured and diff calc is undf
            is_initialized_post = 1; 
        }
    last_capture_32_post = current_capture_32_post;
    TIM1->SR &= ~TIM_SR_CC1IF;
}


// extern "C" void TIM17_IRQHandler(void) {
//     /*
//     This handler is entered 
//     */
//     if (TIM17->SR & TIM_SR_CC1IF) {
//         uint32_t ovf = overflow_count_pre;
//         uint16_t cap = TIM17->CCR1;

//         if ((TIM17->SR & TIM_SR_UIF) && cap < 0x8000) {
//             ovf++;
//         }
//         current_capture_32_pre = (ovf << 16) | cap;

//         if (is_initialized_pre) {
//                 //period between rising edges ((T+Tn) - T) = Tn
//                 diff_post = current_capture_32_pre - last_capture_32_pre;

//                 if (diff_pre > 0) {
//                     // you can 
//                     frequency_pre = FREQUENCY / diff_pre; 
//                 }
//         }
//         else {
//                 // This handles case where only 1 rising edge has occured and diff calc is undf
//                 is_initialized_pre = 1; 
//             }
//         last_capture_32_pre = current_capture_32_pre;
//         TIM17->SR &= ~TIM_SR_CC1IF;
//     }

//     if (TIM17->SR & TIM_SR_UIF) {
//         overflow_count_pre++;
//         TIM17->SR &= ~TIM_SR_UIF; // Clear flag
//     }

// }


//Delay Function
void delay_ms(uint32_t ms) {
    uint32_t start = msTicks_post;
    while ((msTicks_post - start) < ms) {
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
    // enable GPIOA and TIM1 and TIM17 Clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // TIM1 is on APB2
    //RCC->APBENR2 |= RCC_APBENR2_TIM17EN; // TIM17

    // we set PA0 [instead ofpa6] to timer1 (af mode)
    GPIOA->MODER &= ~GPIO_MODER_MODE0; //PA6 
    GPIOA->MODER |= GPIO_MODER_MODE0_1; ///*!< /*!< 0x00000002 */ */ from c011xx.h

    // changed PA0 to  Alternate 
    GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL0_Pos);
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL0_Pos); 

    // // set PA7 to Alternate Function Mode 10 [PRE AIR]
    // GPIOA->MODER &= ~GPIO_MODER_MODE7; //PA7 
    // GPIOA->MODER |= GPIO_MODER_MODE7_1; ///*!< 0x00002000 */ from c011xx.h

    // // set PA7 Alternate Function to AF2 TIM17_CH1
    // GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL7_Pos);
    // GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL7_Pos); 

    //Before we tested with pull down, now external 3.3V pull up so remove.
    //GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk); 
    //GPIOA->PUPDR |= (2U << GPIO_PUPDR_PUPD0_Pos);
    // GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk); 
    // GPIOA->PUPDR |= (2U << GPIO_PUPDR_PUPD7_Pos);
}

void TIM1_Config(void) {
    //FOR GDB / DEBUGGING stop timers during debugging to allow for step-throuh
    DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM1_STOP;

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
    
    //digital input filter

    TIM1->CCMR1 &= ~TIM_CCMR1_IC1F;

    TIM1->CCMR1 |= (7U << TIM_CCMR1_IC1F_Pos); // set 0111 for 4uS filter at 8MHz clock
    

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

// void TIM17_Config(void) {
//     // set TIM17EN register, turns on tim17 clock
//     RCC->APBENR2 |= RCC_APBENR2_TIM17EN;

//     // we calculated min sampling frequency of 8Hz with prescalar 16 (15 + 1). 
//     TIM17->PSC = (8000000/FREQUENCY) - 1;           

//     //autoreload register max value (16bit = 0xffff), triggers interrupt and goes to 0 after arr ==0xff
//     TIM17->ARR = 0xFFFF;

//     //clear capture-compare mode register 1 
//     TIM17->CCMR1 &= ~TIM_CCMR1_CC1S;

//     //set channel 1 to input
//     TIM17->CCMR1 |= TIM_CCMR1_CC1S_0;
    

//     //Filter not needed
//     //TIM17->CCMR1 &= ~TIM_CCMR1_IC1F; 
    
//     // rising edge capture mode CC1P=0, CC1NP=0
//     TIM17->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    
//     // channel 1 capture enabled; on every rising edge, TIM17 CCR1 = TIM17 CNT
//     TIM17->CCER |= TIM_CCER_CC1E;

//     // Enable Capture/Compare Interrupt 
//     TIM17->DIER |= (TIM_DIER_UIE | TIM_DIER_CC2IE);   // use interrupt 2

//     //start timer
//     TIM17->CR1 |= TIM_CR1_CEN;

//     //enable  TIM17 Capture Compare interrupts
//     NVIC_EnableIRQ(TIM17_IRQn);
//     // NVIC_EnableIRQ(TIM17_BRK_UP_TRG_COM_IRQn); //also enable the IRQ for update interrupt flag (Overflow)    
// }

// --- Main ---
int main() {
    SystemClock_Config_HSE(); // Set to 8MHz External Crystal
    Timer_Input_Init();       // Configure PA6 as AF2 (TIM1_CH1), PA7 as TIM17
    TIM1_Config();
    // TIM17_Config();
    SysTick_Config(SystemCoreClock / 1000);
    uint32_t counter_post = 0;
    uint32_t past_post = 0;
    // uint32_t counter_pre = 0;
    // uint32_t past_pre = 0;

    while (1) {
        if (msTicks_post - past_post > 5000){
            counter_post++;
            past_post = msTicks_post;
        }
        // if (msTicks_pre - past_pre > 5000){
        //     counter_pre++;
        //     past_pre = msTicks_pre;
        // }
    }
}