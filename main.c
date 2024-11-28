#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"

// LoRa Configuration
#define SPI_BASE          SSI0_BASE
#define LORA_CS_PORT      GPIO_PORTA_BASE
#define LORA_CS_PIN       GPIO_PIN_3

// LoRa Registers
#define REG_OP_MODE       0x01
#define REG_FRF_MSB       0x06
#define REG_FRF_MID       0x07
#define REG_FRF_LSB       0x08
#define REG_PAYLOAD_LENGTH 0x22
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO          0x00
#define REG_FIFO_TX_BASE  0x0E
#define REG_IRQ_FLAGS     0x12
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG3 0x26

// Function Prototypes
void SPI_Init(void);
void UART_Init(void);
void LoRa_Init(void);
void LoRa_SendPacket(const char* message);
void LoRa_WriteRegister(uint8_t reg, uint8_t value);
uint8_t LoRa_ReadRegister(uint8_t reg);
void LoRa_Select(void);
void LoRa_Deselect(void);
void UART_ReadString(char* buffer, int maxLength);

int main(void)
{
    // Set up the system clock to 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SPI_Init();
    UART_Init();
    LoRa_Init();

    char message[100];

    while (1)
    {
        // Read input from UART
        UART_ReadString(message, sizeof(message));

        // Transmit the packet through LoRa
        LoRa_SendPacket(message);
        int i;
        // Print message over UART for debugging
        UARTCharPut(UART0_BASE, '\n');
        for (i = 0; message[i] != '\0'; i++) {
            UARTCharPut(UART0_BASE, message[i]);
        }

        // Optional delay between transmissions
        SysCtlDelay(SysCtlClockGet() / 3); // 1-second delay at 50MHz
    }
}

void SPI_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinTypeGPIOOutput(LORA_CS_PORT, LORA_CS_PIN);
    LoRa_Deselect();

    SSIConfigSetExpClk(SPI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 4000000, 8);
    SSIEnable(SPI_BASE);
}

void UART_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void LoRa_Init(void)
{
    LoRa_WriteRegister(REG_OP_MODE, 0x80); // Sleep mode, LoRa mode

    LoRa_WriteRegister(REG_FRF_MSB, 0x6C); // 433 MHz (MSB)
    LoRa_WriteRegister(REG_FRF_MID, 0x80); // MID
    LoRa_WriteRegister(REG_FRF_LSB, 0x00); // LSB

    LoRa_WriteRegister(REG_FIFO_TX_BASE, 0x00); // Set FIFO base address
    LoRa_WriteRegister(REG_FIFO_ADDR_PTR, 0x00); // Set FIFO pointer
    LoRa_WriteRegister(REG_PAYLOAD_LENGTH, 0xFF); // Max payload

    LoRa_WriteRegister(REG_MODEM_CONFIG1, 0x72); // Bandwidth 125 kHz, CR 4/5
    LoRa_WriteRegister(REG_MODEM_CONFIG2, 0x74); // SF7, CRC on
    LoRa_WriteRegister(REG_MODEM_CONFIG3, 0x04); // Low data rate optimization off
}

void UART_ReadString(char* buffer, int maxLength)
{
    int index = 0;
    char c;
    while (index < maxLength - 1) {
        c = UARTCharGet(UART0_BASE); // Blocking call, waits for data
        if (c == '\r') { // End of string
            break;
        }
        buffer[index++] = c;
        UARTCharPut(UART0_BASE, c); // Echo the character
    }
    buffer[index] = '\0'; // Null-terminate the string
}

void LoRa_SendPacket(const char* message)
{
    // Clear IRQ flags to reset TxDone
    LoRa_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    // Reset FIFO pointer
    LoRa_WriteRegister(REG_FIFO_ADDR_PTR, 0x00);

    // Write message to FIFO
    LoRa_Select();
    SSIDataPut(SPI_BASE, REG_FIFO | 0x80); // FIFO write command

    while (*message) {
        SSIDataPut(SPI_BASE, *message++);
        while (SSIBusy(SPI_BASE));
    }

    LoRa_Deselect();

    // Start transmission
    LoRa_WriteRegister(REG_OP_MODE, 0x83); // Transmit mode

    // Wait for TxDone (polling IRQ flags)
    while ((LoRa_ReadRegister(REG_IRQ_FLAGS) & 0x08) == 0);

    // Clear IRQ flags again after TxDone
    LoRa_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    // Back to standby mode for the next transmission
    LoRa_WriteRegister(REG_OP_MODE, 0x81); // Standby mode
}

void LoRa_WriteRegister(uint8_t reg, uint8_t value)
{
    LoRa_Select();
    SSIDataPut(SPI_BASE, reg | 0x80);
    while (SSIBusy(SPI_BASE));
    SSIDataPut(SPI_BASE, value);
    while (SSIBusy(SPI_BASE));
    LoRa_Deselect();
}

uint8_t LoRa_ReadRegister(uint8_t reg)
{
    uint32_t data;
    LoRa_Select();
    SSIDataPut(SPI_BASE, reg & 0x7F); // Read command

    while (SSIBusy(SPI_BASE));
    SSIDataGet(SPI_BASE, &data);
    LoRa_Deselect();

    return data & 0xFF;
}

void LoRa_Select(void)
{
    GPIOPinWrite(LORA_CS_PORT, LORA_CS_PIN, 0);
}

void LoRa_Deselect(void)
{
    GPIOPinWrite(LORA_CS_PORT, LORA_CS_PIN, LORA_CS_PIN);
}



