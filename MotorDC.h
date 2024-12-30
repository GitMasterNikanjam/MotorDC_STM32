#pragma once

// ##############################################################################################
// MCU Select:

#include "mcu_select.h"

/*
    If there is not exist mcu_select.h at beside of this header file, Create it and put this bellow following content. 
    Then select your desired MCU that want work with.
*/
// ----------------------------------------------------------------
// mcu_select.h file:

// Define the target MCU family here
// Uncomment the desired MCU family definition below:

// #define STM32F1
// #define STM32F4
// #define STM32H7

// ----------------------------------------------------------------

// ##################################################################################
// Include libraries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"      // HAL library for STM32F1 series
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"      // HAL library for STM32F4 series
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"      // HAL library for STM32H7 series
#else
#error "Unsupported MCU family. Please define a valid target (e.g., STM32F1, STM32F4, STM32H7)."
#endif

#include <string>       // Include the standard string library for error handling and messages
#include <math.h>

// ##################################################################################
// Defin Global Macros:


// ######################################################################################
// Define Classes:

/**
 * @class MotorDC
 * @brief The class for advanced Motor DC control.
 * @note - This class provide PWM, Direction and Enable signal for control Motor DC driver.
 * @note - The default PWM frequency is 1000Hz.
 */
class MotorDC
{
public:

    /// @brief Stores the last error message encountered by the object.
    std::string errorMessage;

    /**
     * @struct ParametersStructure
     * @brief Parameters structure.
     * @note Any changes to these parameters require object re-initialization afterward.
     */
    struct ParametersStructure
    {
        /**
         * @brief Hal timer handle pointer. eg: htim1, htim2, ...
         * @note - Set this timer handler carefully because the object generate PWM by this handler.
         * 
         * @note - The Timer handler must be configured and initialized before and outside of the object.
         * 
         * @note - The PWM frequency can be set dynamically using the setPwmFrequency() method.
         */
        TIM_HandleTypeDef *TIMER_HANDLE;

        /**
         * @brief MotorDC direction control GPIO port. 
         * @note - Only if the DC motor driver has direction control pin, this parameter need to be set; otherwise, leave it.
         */
        GPIO_TypeDef *DIR_GPIO_PORT;

        /**
         * @brief MotorDC enable control GPIO port. 
         * @note - Only if the DC motor driver has enable control pin, this parameter need to be set; otherwise, leave it.
         */
        GPIO_TypeDef *ENA_GPIO_PORT;
        
        /**
         * @brief MotorDC direction control GPIO pin. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
         * @note - Only if the DC motor driver has direction control pin, this parameter need to be set; otherwise, leave it.
         */
        uint16_t DIR_GPIO_PIN;

        /**
         * @brief MotorDC enable control GPIO pin. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
         * @note - Only if the DC motor driver has enable control pin, this parameter need to be set; otherwise, leave it.
         */
        uint16_t ENA_GPIO_PIN;

        /**
         * @brief MotorDC enable control GPIO pin active mode. 
         * @note - A value of 0 means motor enabled when enable pin set to 0 digital value. (Default value)
         * @note - A value of 1 means motor enabled when enable pin set to 1 digital value.
         */
        uint8_t ENA_ACTIVE_MODE;

        /**
         * @brief The Timer base clock frequency. [Hz]
         * @note This value must equal the specified peripheral clock frequency; otherwise, the MotorDC object will not generate the correct PWM signal.
         */
        uint32_t CLOCK_FREQUENCY;

        /**
         * @brief Timer channel number for PWM output. this value can be: 1, 2, 3 or 4.
         */
        uint8_t CHANNEL_NUM;

        /**
         * @brief Minimum allowable dutycycle. [%]
         * @note A value of 0 means it is disabled.
         */
        float DUTYCYCLE_MIN;

        /**
         * @brief Maximum allowable dutycycle. [%]
         * @note A value of 0 means it is disabled.
         */
        float DUTYCYCLE_MAX;

        /**
         * @brief The desired PWM Frequency. [Hz]
         * @note - Its value is acceptable between: 1000Hz to 20000Hz
         * 
         * @note - Default value is 1000 Hz.
         *  */ 
        uint32_t PWM_Frequency;

        /**
         * @brief MotorDC direction control polarity. A value of 0 or 1 acceptable for it.
         * @note - A value of 0 means normal direction. The output digital value of the direction pin will be 0 in normal direction.
         * @note - A value of 1 means reverse direction. The output digital value of the direction pin will be 1 in normal direction.
         */
        uint8_t DIR_POL;

    }parameters;

    /**
     * @struct ValuesStructure
     * @brief Values Structure
     */
    struct ValuesStructure
    {
        /**
         * @brief Output dutycycle. [%]
         * @note Its range value is 0% to +100% 
         */
        float dutyCycle;

        /**
         * @brief Output direction value. -1, 0 and 1
         * @note - A value of 0 means stop condition.
         * @note - A value of 1 means normal direction condition.
         * @note - A value of -1 means reverse direction condition.
         */
        int8_t dir;

        /**
         * @brief Timer PWM CCR value. 
         */
        uint32_t PWM;

        /**
         * @brief Enable/disable condition. false: disabled, true: enabled.
         */
        bool enable;
    }value;

    /**
     * @brief Constructor. Init some variables and parameters.
     * @note - The default PWM frequency is 1000Hz.
     */
    MotorDC();

    /**
     * @brief Init object. check parameters validation. Init some variables.
     * @return true if succeeded.
     * @note This method automatically sets the timer prescaler register and auto-reload register. It is also possible to change their previously set values.
     * @warning The Timer handler must be initialized before Servo.init(). Otherwise, PWM generation may not work correctly.
     */
    bool init(void);

    /**
     * @brief Set PWM to zero and stop pwm generation.
     * @return true if succeeded.
     */
    bool clear(void);

    /**
     * @brief Set PWM frequency. [Hz]
     * @note - It is possible that set pwm frequency after/before init() or start().
     * 
     * @note - Its value is acceptable between: 1000Hz to 20000Hz
     * 
     * @note - Default value is 1000 Hz.
     * @return true if succeeded.
     */
    bool setPwmFrequency(uint32_t value = 1000);

    /// @brief Get the PWM dutycycle signal control resolution. [%]
    float getResolution(void) {return _resolution;};

    /**
     * @brief Set Current PWM dutycycle. [%]
     * @param dutyCycle: PWM dutycycle value. [%]
     * @note dutyCycle value range be from -100% to +100%
     */
    void write(float dutyCycle);             

    /**
     * @brief Set value of enable digital pin for anable of motor driver.
     */
    void enable(void);

    /**
     * @brief Set value of enable digital pin for disable of motor driver.
     */
    void disable(void);                   

private:

    /**
     * @brief PWM period time. [us]
     * @note It is calculated from 1/PWM_FREQUENCY.
     */
    float _period;

    /**
     * @brief PWM dutycycle signal control resolution. [%]
     */
    float _resolution;

    /**
     * @brief Timer channel address value for Servo object. eg: TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 or TIM_CHANNEL_4
     */
    uint32_t _channelAddress;

    /// @brief The init flag is true if initialization succeeded. If it is false, the Servo object needs to be initialized correctly.
    bool _InitFlag;

    /**
     * @brief GPIO_PinState value for enabled of motor driver.
     */
    GPIO_PinState _enable;

    /**
     * @brief GPIO_PinState value for disabled of motor driver.
     */
    GPIO_PinState _disable;

    /**
     * @brief GPIO_PinState value for direction control of motor driver.
     */
    GPIO_PinState _normalDirection;

    /**
     * @brief GPIO_PinState value for direction control of motor driver.
     */
    GPIO_PinState _reverseDirection;

    /**
     * @brief check parameters validation.
     * @return true if succeeded.
     */
    bool _checkParameters(void);

    /**
     * @brief Enable RCC GPIO PORT for certain port.
     */
    void RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *GPIO_PORT);
};




