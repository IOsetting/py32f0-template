#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "py32f0xx_bsp_printf.h"

const char HEX_TABLE[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void BSP_UART_TxChar(char ch)
{
    LL_USART_TransmitData8(DEBUG_USART, ch);
    while (!LL_USART_IsActiveFlag_TC(DEBUG_USART));
    LL_USART_ClearFlag_TC(DEBUG_USART);
}

void BSP_UART_TxHex8(uint8_t hex)
{
    BSP_UART_TxChar(HEX_TABLE[(hex >> 4) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[hex & 0x0F]);
}

void BSP_UART_TxHex16(uint16_t hex)
{
    BSP_UART_TxChar(HEX_TABLE[(hex >> 12) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 8) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 4) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[hex & 0xF]);
}

void BSP_UART_TxHex32(uint32_t hex)
{
    BSP_UART_TxChar(HEX_TABLE[(hex >> 28) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 24) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 20) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 16) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 12) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 8) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[(hex >> 4) & 0x0F]);
    BSP_UART_TxChar(HEX_TABLE[hex & 0xF]);
}

void BSP_UART_TxString(char *str)
{
    while (*str) BSP_UART_TxChar(*str++);
}

/**
  * @brief  DEBUG_USART GPIO Config,Mode Config,115200 8-N-1
  * @param  None
  * @retval None
  */
void BSP_USART_Config(uint32_t baudRate)
{
    DEBUG_USART_CLK_ENABLE();

    /* USART Init */
    LL_USART_SetBaudRate(DEBUG_USART, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudRate);
    LL_USART_SetDataWidth(DEBUG_USART, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(DEBUG_USART, LL_USART_STOPBITS_1);
    LL_USART_SetParity(DEBUG_USART, LL_USART_PARITY_NONE);
    LL_USART_SetHWFlowCtrl(DEBUG_USART, LL_USART_HWCONTROL_NONE);
    LL_USART_SetTransferDirection(DEBUG_USART, LL_USART_DIRECTION_TX_RX);
    LL_USART_Enable(DEBUG_USART);
    LL_USART_ClearFlag_TC(DEBUG_USART);

    /**USART GPIO Configuration
    PA2     ------> USART1_TX
    PA3     ------> USART1_RX
    */
    DEBUG_USART_RX_GPIO_CLK_ENABLE();
    DEBUG_USART_TX_GPIO_CLK_ENABLE();

    LL_GPIO_SetPinMode(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_PIN, DEBUG_USART_TX_AF);

    LL_GPIO_SetPinMode(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_PIN, DEBUG_USART_RX_AF);

#if defined (__GNUC__) && !defined (__clang__)
    // To avoid io buffer
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
#endif
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
    LL_USART_TransmitData8(DEBUG_USART, ch);
    while (!LL_USART_IsActiveFlag_TC(DEBUG_USART));
    LL_USART_ClearFlag_TC(DEBUG_USART);

    return (ch);
}

GETCHAR_PROTOTYPE
{
    int ch;
    while (!LL_USART_IsActiveFlag_RXNE(DEBUG_USART));
    ch = LL_USART_ReceiveData8(DEBUG_USART);
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
