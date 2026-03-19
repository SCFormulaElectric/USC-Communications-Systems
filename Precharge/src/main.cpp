#include "stm32c0xx.h"
//SUSPECT ISSUE WITH STATE MACHINE IS ELAPSED TIME, going through PUI -> PUI ->PUI, NO VOLTAGE APPLIED (so i guess )
//prescalar confing & TIM1IRQ is handled by this;
//TIMER FREQUENCY. SYSCLK IS AT CONSTANT 8MHz
#define FREQUENCY 500000UL

// POST-AIR Global Variables
volatile uint32_t msTicks = 0;
volatile uint32_t overflow_count_post = 0;
volatile uint32_t overflow_count_pre = 0;

// DEBUG ONLY - global for GDB visibility
volatile uint32_t current_capture_32_post = 0;
volatile uint16_t diff_post = 0;
volatile uint32_t last_capture_32_post = 0;
static volatile uint8_t is_initialized_post = 0;

volatile uint32_t current_capture_32_pre = 0;
volatile uint16_t diff_pre = 0;
volatile uint32_t last_capture_32_pre = 0;
static volatile uint8_t is_initialized_pre = 0;


//SysTick Interrupt Handler
extern "C" void SysTick_Handler(void) {
    msTicks++;
}

extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) {
        overflow_count_post++;
        TIM1->SR = ~TIM_SR_UIF;
    }
}


extern "C" void TIM1_CC_IRQHandler(void) {

    /*
    This handler is entered 
    */
    // Everything else stays almost the same, just change TIM2 to TIM1
    if (TIM1->SR & TIM_SR_CC1IF){
    uint32_t ovf = overflow_count_post;
    uint16_t cap = TIM1->CCR1;

    if ((TIM1->SR & TIM_SR_UIF) && cap < 0x8000) {
        ovf++;
    }
    current_capture_32_post = (ovf << 16) | cap;

    if (is_initialized_post) {
            //period between rising edges ((T+Tn) - T) = Tn
            diff_post = current_capture_32_post - last_capture_32_post;
    }
    else {
            // This handles case where only 1 rising edge has occured and diff calc is undf
            is_initialized_post = 1; 
        }
    last_capture_32_post = current_capture_32_post;
    TIM1->SR = ~TIM_SR_CC1IF;
    }
}


 extern "C" void TIM17_IRQHandler(void) {
     /*
     This handler is entered 
     */
     if (TIM17->SR & TIM_SR_CC1IF) {
         uint32_t ovf = overflow_count_pre;
         uint16_t cap = TIM17->CCR1;

         if ((TIM17->SR & TIM_SR_UIF) && cap < 0x8000) {
             ovf++;
         }
         current_capture_32_pre = (ovf << 16) | cap;

         if (is_initialized_pre) {
                 //period between rising edges ((T+Tn) - T) = Tn
                 diff_pre = current_capture_32_pre - last_capture_32_pre;

         }
         else {
                 // This handles case where only 1 rising edge has occured and diff calc is undf
                 is_initialized_pre = 1; 
             }
         last_capture_32_pre = current_capture_32_pre;
         
         TIM17->SR = ~TIM_SR_CC1IF;
         
     }

     if (TIM17->SR & TIM_SR_UIF) {
         overflow_count_pre++;
         
         TIM17->SR = ~TIM_SR_UIF;
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
    // enable GPIOA and TIM1 and TIM17 Clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // TIM1 is on APB2
    RCC->APBENR2 |= RCC_APBENR2_TIM17EN; // TIM17

    // we set PA0 [instead ofpa6] to timer1 (af mode)
    GPIOA->MODER &= ~GPIO_MODER_MODE0; //PA6 
    GPIOA->MODER |= GPIO_MODER_MODE0_1; ///*!< /*!< 0x00000002 */ */ from c011xx.h

    // changed PA0 to  Alternate 
    GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL0_Pos);
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL0_Pos); 

    // // set PA7 to Alternate Function Mode 10 [PRE AIR]
    GPIOA->MODER &= ~GPIO_MODER_MODE7; //PA7 
    GPIOA->MODER |= GPIO_MODER_MODE7_1; ///*!< 0x00002000 */ from c011xx.h

    // // set PA7 Alternate Function to AF5 TIM17_CH1
    GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL7_Pos);
    GPIOA->AFR[0] |= (5U << GPIO_AFRL_AFSEL7_Pos);

    //Before we tested with pull down, now external 3.3V pull up so remove.
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk); 
     GPIOA->PUPDR |= (2U << GPIO_PUPDR_PUPD7_Pos);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk); 
    GPIOA->PUPDR |= (1U << GPIO_PUPDR_PUPD0_Pos);
}

void TIM1_Config(void) {
    //FOR GDB / DEBUGGING stop timers during debugging to allow for step-throuh
    DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM1_STOP;
    DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM17_STOP;

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
    
    // rising edge capture mode CC1P=0, CC1NP=0 switched to falling mode
    TIM1->CCER |= TIM_CCER_CC1P;
    
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

void TIM17_Config(void) {
//     // set TIM17EN register, turns on tim17 clock
       RCC->APBENR2 |= RCC_APBENR2_TIM17EN;

//     // we calculated min sampling frequency of 8Hz with prescalar 16 (15 + 1). 
       TIM17->PSC = (8000000/FREQUENCY) - 1;           

//     //autoreload register max value (16bit = 0xffff), triggers interrupt and goes to 0 after arr ==0xff
       TIM17->ARR = 0xFFFF;

//     //clear capture-compare mode register 1 
       TIM17->CCMR1 &= ~TIM_CCMR1_CC1S;

//     //set channel 1 to input
       TIM17->CCMR1 |= TIM_CCMR1_CC1S_0;

       // rising edge capture mode CC1P=0, CC1NP=0 switched to falling edge
       TIM17->CCER |= TIM_CCER_CC1P;
    
//     // channel 1 capture enabled; on every rising edge, TIM17 CCR1 = TIM17 CNT
       TIM17->CCER |= TIM_CCER_CC1E;

     // Enable Capture/Compare Interrupt 
       TIM17->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);
//     //start timer
       TIM17->CR1 |= TIM_CR1_CEN;

//     //enable  TIM17 Capture Compare interrupts
       NVIC_EnableIRQ(TIM17_IRQn);
 }

 void TIM3_Config(void){
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    TIM3->ARR = 0xFFFF;
    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
 }

 void TIM14_Config(void){
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;  // correct
    (void)RCC->APBENR2;                    // stall
    TIM14->CR1 &= ~TIM_CR1_CEN;
    TIM14->PSC = 7999;
    TIM14->ARR = 0xFFFF;
    TIM14->CNT = 0;
    TIM14->EGR |= TIM_EGR_UG;
    TIM14->SR &= ~TIM_SR_UIF;
    TIM14->CR1 |= TIM_CR1_CEN;
 }

 void Delay_uS(uint16_t us){
    TIM3->CNT = 0;
    uint32_t target = us * 8;
    while (TIM3->CNT < target);
 }
//Other gpio configs
void Configure_GPIOA_Pins(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // enable clock source
    
    (void)RCC->IOPENR; //strall

    GPIOA->MODER &= ~((3U << (1 * 2)) |   // Clear PA1 bits [3:2] (output) Precharge Relay
                      (3U << (2 * 2)) |   // Clear PA2 bits [5:4] (input) CHG signal
                      (3U << (3 * 2)) |   // Clear PA3 bits [7:6] (input) SDC
                      (3U << (5 * 2)) |   // Clear PA5 bits [11:10] (output) UART TX
                      (3U << (11 * 2)));  // Clear PA11 bits [23:22] (output) AIR Relay

    GPIOA->MODER |=  ((1U << (1 * 2)) |   // Set PA1 as Output
                      (1U << (5 * 2)) | //(1U << (5 * 2)) |   // Set PA5 as Output [not setting as output for UART]
                      (1U << (11 * 2)));  // Set PA11 as Output
    //GPIOA->BSRR = GPIO_BSRR_BS5; //set high
}

void uartTx(uint8_t data){
    __disable_irq();
    GPIOA->BSRR = GPIO_BSRR_BR5;
    Delay_uS(102);
    for (int i = 0; i < 8; i++){
        if (data & (1 << i)){
            GPIOA->BSRR = GPIO_BSRR_BS5;
        }
        else{
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        Delay_uS(102);
    }
    GPIOA->BSRR = GPIO_BSRR_BS5;
    Delay_uS(102);
    __enable_irq();

}
// NOTE: only handles 0-9999
void digitUartTx(int number){
    char thousands = number /1000 + '0';
    char hundreds = (number/100) % 10 + '0';
    char tens = (number/10) % 10 + '0';
    char ones = number % 10 + '0';
    uartTx(thousands);
    uartTx(hundreds);
    uartTx(tens);
    uartTx(ones);
}

typedef enum {
    STATE_IDLE,
    STATE_PRECHARGING,
    STATE_SAFE,
    STATE_UNSAFE
    } precharge_state_t;

// --- Main ---
int main() {
    SystemClock_Config_HSE(); // Set to 8MHz External Crystal
    Timer_Input_Init();       // Configure PA6 as AF2 (TIM1_CH1), PA7 as TIM17
    TIM1_Config();
    TIM17_Config();
    Configure_GPIOA_Pins();
    SysTick_Config(SystemCoreClock / 1000);
    TIM3_Config();
    TIM14_Config();

    
    uint32_t localDiff_pre = 0;
    uint32_t localDiff_post = 0;




    uint32_t localFreq_pre = 0;
    uint32_t localFreq_post = 0;
    uint16_t is_pa3_high = 0;
    uint16_t is_pa2_high = 0;
    //-------------------------------------



    uint16_t start_time = 0;
    uint16_t elapsed_time = 0;
    uint32_t uartTestDealy = 0;

    precharge_state_t system_state = STATE_IDLE;
    //set precharge fault low: 
    GPIOA->BSRR = (1U <<(5 + 16));
    GPIOA->BSRR = (1U <<(1 + 16)); //no trickle
    GPIOA->BSRR = (1U <<(11 + 16)); // make sure glv not connected to air relay

    // Ratio thresholds (tune experimentally)

    //
    uint32_t ratio_percent = 0;

    while (1) {

        //gather inputs
        __disable_irq();
        localDiff_pre = diff_pre;
        localDiff_post = diff_post;
        __enable_irq();

        is_pa3_high = (GPIOA->IDR & (1U << 3)) ? 1 : 0; //CHG,for now we won't deal with CHG, treating ELCON charging & Motor Controller Discharging the Same
        is_pa2_high = (GPIOA->IDR & (1U << 2)) ? 1 : 0; // SDC

        if (localDiff_pre > 0){
            localFreq_pre = FREQUENCY / localDiff_pre;
        }
        else{
            localFreq_pre = 0;
        }

        if (localDiff_post > 0){
            localFreq_post = FREQUENCY / localDiff_post;
        }
        else{
            localFreq_post = 0;

        }


        //logic start-------------------------------------
        if (localFreq_pre > 0) {
            ratio_percent = (localFreq_post * 100) / localFreq_pre; // this gives percentage to 2 decimals e.g.
        } else {
            ratio_percent = 0;
        }
        //state machine
        //if (msTicks - uartTestDealy > 500){
        //    uartTestDealy = msTicks;
        //    uartTx('F');
        //    uartTx('R');
        //    uartTx(' ');
        //    digitUartTx(localFreq_pre);
        //    uartTx(' ');
        //    uartTx(' ');
        //    uartTx('F');
        //    uartTx('O');
        //    uartTx(' ');
        //    digitUartTx(localFreq_post);
        //    uartTx('\r');
        //    uartTx('\n');
        //}

        if (!is_pa3_high){
            system_state = STATE_SAFE;
        }
        switch (system_state) {
            case STATE_IDLE:

                elapsed_time = 0;
                

                //Set outputs... BSRR in stm32 allows for atomic operations by avoiding RMW. So setting is (1 << x) & resetting is (1 << (x+16)), the 32bit register is halved and the top is clear
                GPIOA->BSRR = (1U <<(1 + 16)); //PA1 is Precharge resistor relay enable. Setting this low disconnects GLV- from AIR+_en
                GPIOA->BSRR = (1U << (11 + 16)); //PA11 is AIR relay. ONLY SET IN SAFE
                GPIOA->BSRR = (1U << (5 + 16)); // -> PRECHARGE ERROR SIGNAL

                if (is_pa2_high){ //[most recently, sdc confirmed to be noninverted unlike last year] if SDC is not HIGH, then we are ok to try and precharge 
                    system_state = STATE_PRECHARGING;
                    start_time = TIM14->CNT;  // capture TIM14 snapshot on entry
                    GPIOA->BSRR = (1U <<(1));  // if SDC chillin, we start tricklin
                }
                break;

            case STATE_PRECHARGING:
                //GPIOA->BSRR = (1U << (5 + 0)); //let's just do this for debugging

                elapsed_time = TIM14->CNT - start_time;
                //digitUartTx(elapsed_time);

                if (!is_pa2_high) {
                    system_state = STATE_IDLE;
                }


                if (ratio_percent >= 89){
                    if (elapsed_time < 1000)
                        system_state = STATE_UNSAFE;
                    else if ((elapsed_time >1000) && (elapsed_time <= 2000))
                        system_state = STATE_SAFE;
                }
                else{
                    if (elapsed_time > 2000){
                        system_state = STATE_UNSAFE;
                    }
                }
                break;
            case STATE_SAFE: //here we close air then open relay to avoid any drops in votlage
                GPIOA->BSRR = (1U << (11)); //CLOSE AIR RELAY  [set to 1, nmos high, GLV connected] 
                GPIOA->BSRR = (1U <<(1+16)); //Precharge relay [clear to 0, nmos low, GLV disconnected]
                GPIOA->BSRR = (1U << (5 + 16)); 

                break;

            case STATE_UNSAFE:

                GPIOA->BSRR = (1U <<(1 + 16)); //PA1 is Precharge resistor relay enable. Setting this low disconnects GLV- from AIR+_en
                GPIOA->BSRR = (1U << (11 + 16)); //OPEN AIR RELAY
                GPIOA->BSRR = GPIO_BSRR_BS5; //PRECHARGE FAULT WHEN WE've determined that we're unsafe
                break;
        }
    }
}