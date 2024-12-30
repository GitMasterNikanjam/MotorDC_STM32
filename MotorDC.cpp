#include "MotorDC.h"


MotorDC::MotorDC()
{
    parameters.CHANNEL_NUM = 0;
    parameters.CLOCK_FREQUENCY = 0;
    parameters.DUTYCYCLE_MAX = 0;
    parameters.DUTYCYCLE_MIN = 0;
    parameters.PWM_Frequency = 1000;
    parameters.TIMER_HANDLE = NULL;
    parameters.DIR_GPIO_PORT = nullptr;
    parameters.DIR_GPIO_PIN = 0;
    parameters.ENA_GPIO_PORT = nullptr;
    parameters.ENA_GPIO_PIN = 0;
    parameters.ENA_ACTIVE_MODE = 0;

    clear();
}

bool MotorDC::init(void)
{
    if(setPwmFrequency(parameters.PWM_Frequency) == false)
    {
        return false;
    }

    if(_checkParameters() == false)
    {
        return false;
    }

    _period = (1000000.0 / (float)parameters.PWM_Frequency);

    switch(parameters.CHANNEL_NUM)
    {
        case 1:
            _channelAddress = TIM_CHANNEL_1;
        break;
        case 2:
            _channelAddress = TIM_CHANNEL_2;
        break;
        case 3:
            _channelAddress = TIM_CHANNEL_3;
        break;
        case 4:
            _channelAddress = TIM_CHANNEL_4;
        break;
        default:
            errorMessage = "Error MotorDC: Channel number is not valid.";
            return false;
    }

    if(HAL_TIM_PWM_Start(parameters.TIMER_HANDLE, _channelAddress) != HAL_OK)
    {
        errorMessage = "Error MotorDC: HAL_TIM_PWM_Start() is not succeeded.";
        return false;
    }

    write(0);

    if(parameters.ENA_ACTIVE_MODE == 0)
    {
        _enable = GPIO_PIN_RESET;
        _disable = GPIO_PIN_SET;
    }
    else
    {
        _enable = GPIO_PIN_SET;
        _disable = GPIO_PIN_RESET;
    }

    if(parameters.DIR_POL == 0)
    {
        _normalDirection = GPIO_PIN_RESET;
        _reverseDirection = GPIO_PIN_SET;
    }
    else
    {
        _normalDirection = GPIO_PIN_SET;
        _reverseDirection = GPIO_PIN_RESET;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(parameters.DIR_GPIO_PORT != nullptr)
    {
        RCC_GPIO_CLK_ENABLE(parameters.DIR_GPIO_PORT);

        GPIO_InitStruct.Pin = parameters.DIR_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(parameters.DIR_GPIO_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(parameters.DIR_GPIO_PORT, parameters.DIR_GPIO_PIN, GPIO_PIN_RESET);
    }

    GPIO_InitStruct = {0};

    if(parameters.ENA_GPIO_PORT != nullptr)
    {
        RCC_GPIO_CLK_ENABLE(parameters.ENA_GPIO_PORT);

        GPIO_InitStruct.Pin = parameters.ENA_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(parameters.ENA_GPIO_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(parameters.ENA_GPIO_PORT, parameters.ENA_GPIO_PIN, _disable);
    }

    _InitFlag = true;
    return true;
}

bool MotorDC::clear(void)
{
    if(_InitFlag == false)
    {
        errorMessage = "Error Servo: init() must be finished before clear().";
        return false;
    }

    write(0);

    if(HAL_TIM_PWM_Stop(parameters.TIMER_HANDLE, _channelAddress) != HAL_OK)
    {
        errorMessage = "Error Servo: HAL_TIM_PWM_Stop() is not succeeded.";
        return false;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(parameters.DIR_GPIO_PORT != nullptr)
    {   
        GPIO_InitStruct.Pin = parameters.DIR_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(parameters.DIR_GPIO_PORT, &GPIO_InitStruct);
    }

    GPIO_InitStruct = {0};

    if(parameters.ENA_GPIO_PORT != nullptr)
    {
        GPIO_InitStruct.Pin = parameters.ENA_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(parameters.ENA_GPIO_PORT, &GPIO_InitStruct);
    }
    
    _InitFlag = false;
    _channelAddress = 0;
    _period = 0;
    _resolution = 0;

    value.dir = 0;
    value.dutyCycle = 0;
    value.enable = false;
    value.PWM = 0;

    return true;   
}

bool MotorDC::setPwmFrequency(uint32_t value)
{
    bool state = ((parameters.PWM_Frequency >= 1000) || (parameters.PWM_Frequency <= 20000));

    if(state == false)
    {
        errorMessage = "Error MotorDC: PWM frequency is not correct value. Please select a value at range of 1000Hz to 20000Hz";
        return false;
    }

    float ARR = 1000 - 1;
    float PSC = ((float)parameters.CLOCK_FREQUENCY / ((float)value * (ARR + 1))) - 1;

    if(PSC < 0) 
    {
        PSC = 0;
    }
    else
    {
        PSC = round(PSC);
    }

    ARR = ((float)parameters.CLOCK_FREQUENCY / ((float)value * (PSC + 1))) - 1;

    if(ARR < 0)
    {
        ARR = 0;
    }
    else if(ARR > 65535)
    {
        ARR = 65535;
    }
    else
    {
        ARR = round(ARR);
    }

    parameters.TIMER_HANDLE->Instance->PSC = (uint32_t)PSC;

    parameters.TIMER_HANDLE->Instance->ARR = (uint32_t)ARR;

    parameters.PWM_Frequency = ((float)parameters.CLOCK_FREQUENCY / ((ARR + 1) * (PSC + 1)));

    _period = (1000000.0 / (float)parameters.PWM_Frequency);
    _resolution = _period / (float)parameters.TIMER_HANDLE->Instance->ARR;

    return true;
}

void MotorDC::write(float dutyCycle)
{
    if(_InitFlag == false)
    {
        return;
    }

    if(dutyCycle > 100.0)
    {
        dutyCycle = 100;
    }
    else if(dutyCycle < -100)
    {
        dutyCycle = -100;
    }

    float abs_dutycycle = abs(dutyCycle);
    GPIO_PinState dir;

    if(dutyCycle > 0)
    {
        value.dir = 1;
        dir = _normalDirection;
    }
    else if(dutyCycle < 0)
    {
        value.dir = -1;
        dir = _reverseDirection;
    }
    else
    {
        value.dir = 0;
        dir = _normalDirection;
    }

    if(parameters.DUTYCYCLE_MAX > 0)
    {
        if(abs_dutycycle > parameters.DUTYCYCLE_MAX)
        {
            abs_dutycycle = parameters.DUTYCYCLE_MAX;
        }
    }

    if( (parameters.DUTYCYCLE_MIN > 0) && (abs_dutycycle != 0))
    {
        if(abs_dutycycle < parameters.DUTYCYCLE_MIN)
        {
            abs_dutycycle = parameters.DUTYCYCLE_MIN;
        }
    }

    value.PWM = (abs_dutycycle / 100.0) * (parameters.TIMER_HANDLE->Instance->ARR + 1);

    if(value.PWM != 0)
    {
        value.PWM = value.PWM - 1;
    }
    
    __HAL_TIM_SET_COMPARE(parameters.TIMER_HANDLE, _channelAddress, value.PWM);

    if(parameters.DIR_GPIO_PORT != nullptr)
    {
        HAL_GPIO_WritePin(parameters.DIR_GPIO_PORT, parameters.DIR_GPIO_PIN, dir);
    } 
}

void MotorDC::enable(void)
{
    if(parameters.ENA_GPIO_PORT != nullptr)
    {
        HAL_GPIO_WritePin(parameters.DIR_GPIO_PORT, parameters.DIR_GPIO_PIN, _enable);
    } 
}

void MotorDC::disable(void)
{
    if(parameters.ENA_GPIO_PORT != nullptr)
    {
        HAL_GPIO_WritePin(parameters.DIR_GPIO_PORT, parameters.DIR_GPIO_PIN, _disable);
    } 
}

bool MotorDC::_checkParameters(void)
{
    bool state = (parameters.CHANNEL_NUM >= 1) && (parameters.CHANNEL_NUM <= 4) &&
                 (parameters.CLOCK_FREQUENCY > 0) && 
                 (parameters.TIMER_HANDLE != NULL) && 
                 ((parameters.PWM_Frequency >= 1000) || (parameters.PWM_Frequency <= 20000)) &&
                 (parameters.ENA_ACTIVE_MODE <= 1) &&
                 (parameters.DUTYCYCLE_MAX >= 0) && (parameters.DUTYCYCLE_MIN >= 0);

    if(state == false)
    {
        errorMessage = "Error Servo: One or some parameters are not correct.";
        return false;
    }

    if(parameters.DUTYCYCLE_MAX > 0)
    {
        if(parameters.DUTYCYCLE_MAX <= parameters.DUTYCYCLE_MIN)
        {
            errorMessage = "Error Servo:- MAX_PULSE_WIDTH can not be smaller or equal MIN_PULS_WIDTH.";
            return false;
        }
    }

    if(parameters.DUTYCYCLE_MIN > 0)
    {
        if(parameters.DUTYCYCLE_MIN >= (parameters.TIMER_HANDLE->Instance->ARR + 1))
        {
            errorMessage = "Error Servo: Timer handle auto reload value can not be less than MIN_PULSE_WIDTH.";
            return false;
        }
    }

    if(parameters.DUTYCYCLE_MAX > 0)
    {
        if(parameters.DUTYCYCLE_MAX >= (parameters.TIMER_HANDLE->Instance->ARR + 1))
        {
            errorMessage = "Error Servo: Timer handle auto reload value can not be less than MAX_PULSE_WIDTH.";
            return false;
        }
    }

    return true;
}

void MotorDC::RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *GPIO_PORT)
{
    if(GPIO_PORT == nullptr)
    {
        return;
    }

    #ifdef GPIOA
    if(GPIO_PORT == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOB
    else if (GPIO_PORT == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOC
    else if (GPIO_PORT == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOD
    else if (GPIO_PORT == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOE
    else if (GPIO_PORT == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOF
    else if (GPIO_PORT == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOG
    else if (GPIO_PORT == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOH
    else if (GPIO_PORT == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOI
    else if (GPIO_PORT == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
    #endif
}