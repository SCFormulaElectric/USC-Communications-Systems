#include "stm32c0xx.h"

//Global Variables
volatile uint32_t msTicks = 0;
    //volatile uint32_t inspect_var = 0;

//SysTick Interrupt Handler
extern "C" void SysTick_Handler(void) {
    msTicks++;
}

//Delay Function
void delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
        // Wait For Interrupt
        __WFI(); 
    }
}

// 48MHz
void SystemClock_Config(void) {
    //Set Flash Latency to 1 Wait State to accomdate 48MHZ clk
    //old code set flash register to 

    //(gdb) print /x *0x40022000
    //$7 = 0x4060
    //FLASH->ACR |= FLASH_ACR_LATENCY;  

    //testing thiss one and it works = 0x4060
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | 0x1;

    //check
    while ((FLASH->ACR & FLASH_ACR_LATENCY) == 0);

    //    Clear HSIDIV bits [13:11] -> 000 means Div/1 (48MHz)
    RCC->CR &= ~RCC_CR_HSIDIV; 
    
    //Wait for HSI to be ready
    while (!(RCC->CR & RCC_CR_HSIRDY));

    //Update SystemCoreClock variable
    SystemCoreClockUpdate(); 

    //SysTick for 1ms interrupts
    SysTick_Config(SystemCoreClock / 1000);
}

// --- Main ---
int main() {
    SystemClock_Config(); // Sets 48MHz

    // Enable GPIOA Clock
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    
    // clear mode bits for PA5Output
    GPIOA->MODER &= ~GPIO_MODER_MODE5;
    // Set mode to 01 GP Output
    GPIOA->MODER |= GPIO_MODER_MODE5_0; 

    while (1) {
        // Toggle PA5 
        GPIOA->ODR ^= GPIO_ODR_OD5; 
        
        // Delay 500ms
        delay_ms(500);              
    }
}