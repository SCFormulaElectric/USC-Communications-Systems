#include "stm32c0xx.h"

#define SYSTEM_FREQ 8000000UL
#define TARGET_FREQ 500000UL // Timer frequency


class FrequencyMonitor {
private:
    TIM_TypeDef* _timer;        // defined in stm32c011xx.h
    IRQn_Type _irq_cc;          // ISR name
    IRQn_Type _irq_update;      // Overflow ISR name
    

    volatile uint32_t _overflow_count;
    volatile uint32_t _current_capture;
    volatile uint32_t _last_capture;
    volatile uint32_t _frequency;
    volatile bool _is_initialized;

public:
    // member initialized list constructor
    FrequencyMonitor(TIM_TypeDef* timer, IRQn_Type irq_cc, IRQn_Type irq_update) 
        : _timer(timer), _irq_cc(irq_cc), _irq_update(irq_update),
          _overflow_count(0), _current_capture(0), _last_capture(0), 
          _frequency(0), _is_initialized(false) {}

    
    void init() {
        //enable tim1 clock
        if (_timer == TIM1) RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

        //enable tim2 clk
        if (_timer == TIM17) RCC->APBENR2 |= RCC_APBENR2_TIM17EN;

        // set prescalar of timer attribute 
        _timer->PSC = (SYSTEM_FREQ / TARGET_FREQ) - 1;

        //set autoreload to max 16bit value
        _timer->ARR = 0xFFFF;

        // /clear capture-compare mode register 1 
        _timer->CCMR1 &= ~TIM_CCMR1_CC1S;
        _timer->CCMR1 |= TIM_CCMR1_CC1S_0; // chn1 input
        
        _timer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // rising edge capture mode CC1P=0, CC1NP=0
        // Enable Capture
        _timer->CCER |= TIM_CCER_CC1E;     

        // Enable Capture/compare interrupt 
        _timer->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);
        _timer->CR1 |= TIM_CR1_CEN;

        // NVIC Enable
        NVIC_EnableIRQ(_irq_cc);
        if (_irq_update != _irq_cc) {
            NVIC_EnableIRQ(_irq_update);
        }
    }

    
    void handleOverflow() {
        if (_timer->SR & TIM_SR_UIF) {
            _overflow_count++;
            _timer->SR &= ~TIM_SR_UIF; 
        }
    }

    
    void handleCapture() {
        if (_timer->SR & TIM_SR_CC1IF) {
            uint32_t ovf = _overflow_count;
            uint16_t cap = _timer->CCR1;

            // if after entry we overflow then account for it 
            if ((_timer->SR & TIM_SR_UIF) && cap < 0x8000) {
                ovf++;
            }
            //tim is 16-bit so ovf * 2^16 == ovf << 16
            _current_capture = (ovf << 16) | cap;

            if (_is_initialized) {
                uint32_t diff = _current_capture - _last_capture;
                if (diff > 0) {
                    _frequency = TARGET_FREQ / diff;
                }
            } else {
                _is_initialized = true;
            }
            
            _last_capture = _current_capture;
            _timer->SR &= ~TIM_SR_CC1IF; 
        }
    }

    //since freq is private then ew must have a public function to return it
    uint32_t getFrequency() const { return _frequency; }
};


FrequencyMonitor monitorPost(TIM1, TIM1_CC_IRQn, TIM1_BRK_UP_TRG_COM_IRQn);
FrequencyMonitor monitorPre(TIM17, TIM17_IRQn, TIM17_IRQn); // TIM17  has 1 IRQ

volatile uint32_t msTicks = 0;

//gotta hook the functions into C interrupt routines so eabi compiler sees it
extern "C" {

    void SysTick_Handler(void) {
        msTicks++;
    }


    void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
        monitorPost.handleOverflow();
    }


    void TIM1_CC_IRQHandler(void) {
        monitorPost.handleCapture();
    }


    void TIM17_IRQHandler(void) {
        // im not sure what having only 1 interrupt handler will change as we update over flow in a diff one in tim1
        monitorPre.handleOverflow();
        monitorPre.handleCapture();
    }
}



void SystemClock_Config_HSE(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_0;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_0);
    SystemCoreClockUpdate();
}

void GPIO_Init(void) { //you can input all other pin out here. e.g. PRECHG EN, etc.
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // PA6 TIM1CH1  AF2 5
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE6) | GPIO_MODER_MODE6_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << GPIO_AFRL_AFSEL6_Pos)) | (5 << GPIO_AFRL_AFSEL6_Pos);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD6) | (2 << GPIO_PUPDR_PUPD6_Pos);

    // PA7 tim17ch1 AF2 2
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE7) | GPIO_MODER_MODE7_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << GPIO_AFRL_AFSEL7_Pos)) | (2 << GPIO_AFRL_AFSEL7_Pos);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD7) | (2 << GPIO_PUPDR_PUPD7_Pos);
}

// --- Main ---

int main() {
    SystemClock_Config_HSE();
    GPIO_Init();
    SysTick_Config(SystemCoreClock / 1000);

    // Initialize the Class instances
    monitorPost.init();
    monitorPre.init();

    uint32_t past = 0;

    while (1) {
        if (msTicks - past > 1000) { // Check every 100ms
            past = msTicks;
            
            // Read frequencies cleanly
            uint32_t freq_post = monitorPost.getFrequency();
            uint32_t freq_pre = monitorPre.getFrequency();

            // Do something with freq_post and freq_pre...
            (void)freq_post;
            (void)freq_pre;
        }
        __WFI();
    }
}