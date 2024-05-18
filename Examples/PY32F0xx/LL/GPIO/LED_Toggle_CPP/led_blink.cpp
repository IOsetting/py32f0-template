#include "main.h"
#include "led_blink.hpp"

struct Led
{
    explicit Led(GPIO_TypeDef* t_port, uint32_t t_pin)
        : port(t_port)
        , pin(t_pin)
    {
    }

    void on()
    {
        LL_GPIO_SetOutputPin(port, pin);
    }

    void off()
    {
        LL_GPIO_ResetOutputPin(port, pin);
    }

    void toggle()
    {
        LL_GPIO_TogglePin(port, pin);
    }

private:
    GPIO_TypeDef* port;
    uint32_t pin;
};

void LED_Blink(void)
{
    Led led(GPIOA, LL_GPIO_PIN_0);

    while (true)
    {
        led.toggle();
        LL_mDelay(1000);
    }
}