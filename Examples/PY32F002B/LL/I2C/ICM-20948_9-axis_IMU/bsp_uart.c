#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "bsp_uart.h"

const char BSP_UartHex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void BSP_UartTxChar(char ch)
{
    LL_USART_TransmitData8(USART1, ch);
    while (!LL_USART_IsActiveFlag_TC(USART1));
    LL_USART_ClearFlag_TC(USART1);
}

void BSP_UartTxHex(uint8_t hex)
{
    BSP_UartTxChar(BSP_UartHex[(hex >> 4) & 0x0F]);
    BSP_UartTxChar(BSP_UartHex[hex & 0x0F]);
}

void BSP_UartTxHexs(uint8_t *hex, uint8_t size)
{
    while (size--)
    {
        BSP_UartTxHex(*(hex + size));
    }
}

void BSP_UartTxString(char *str)
{
    while (*str) BSP_UartTxChar(*str++);
}

void BSP_UartTxLine(char *str, uint8_t hex)
{
    while (*str) BSP_UartTxChar(*str++);
    BSP_UartTxHex(hex);
    BSP_UartTxString("\r\n");
}

void BSP_UartTxInt(int32_t num, uint8_t len)
{
    uint8_t i = 0, negative = 0, buff[16];

    if (num == 0)
    {
        buff[i++] = '0';
    }
    else
    {
        if (num < 0)
        {
            negative = 1;
            num = -num;
        }
        while (num > 0)
        {
            buff[i++] = (num % 10) + '0';
            num /= 10;
        }
        buff[i++] = negative ? '-' : ' ';
    }
    // add leading spaces
    if (i < len)
    {
        while (i < len) buff[i++] = ' ';
    }
    while (i--) BSP_UartTxChar(buff[i]);
}

/**
  * @brief  UART GPIO Config, 115200 8-N-1
  */
void BSP_UartReconfig(uint32_t baudRate)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

    /* USART Init */
    LL_USART_SetBaudRate(USART1, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudRate);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_Enable(USART1);
    LL_USART_ClearFlag_TC(USART1);
    // PA6: USART1 TX
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_1);
    // PA7: USART1 RX
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_3);
}


#if defined (__GNUC__) && !defined (__clang__)
    #define GETCHAR_PROTOTYPE __attribute__((weak)) int __io_getchar (void)
    #define PUTCHAR_PROTOTYPE __attribute__((weak)) int __io_putchar(int ch)
#else
    #define GETCHAR_PROTOTYPE int fgetc(FILE * f)
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
    /* Send a byte to USART */
    LL_USART_TransmitData8(USART1, ch);
    while (!LL_USART_IsActiveFlag_TC(USART1));
    LL_USART_ClearFlag_TC(USART1);

    return (ch);
}

GETCHAR_PROTOTYPE
{
    int ch;
    while (!LL_USART_IsActiveFlag_RXNE(USART1));
    ch = LL_USART_ReceiveData8(USART1);
    return (ch);
}

#if defined (__GNUC__) && !defined (__clang__)
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        __io_putchar(*ptr++);
    }
    return len;
}
#endif

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        *ptr++ = __io_getchar();
    }
    return len;
}

__attribute__((weak)) int _isatty(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

__attribute__((weak)) int _close(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    errno = EBADF;
    return -1;
}

__attribute__((weak)) int _lseek(int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;

    errno = EBADF;
    return -1;
}

__attribute__((weak)) int _fstat(int fd, struct stat *st)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return 0;
}

__attribute__((weak)) int _getpid(void)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int _kill(pid_t pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = ENOSYS;
    return -1;
}
