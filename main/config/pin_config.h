#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/* ================= ENCODER ================= */
#define JGB37_ENC_A_GPIO 35 // Đã đảo chân để bánh quay tới ra RPM dương
#define JGB37_ENC_B_GPIO 34

/* ================= TB67H450FNG ================= */
#define TB67_IN1_GPIO 33 // PWM1
#define TB67_IN2_GPIO 32 // PWM2

/* ================= MG90S ================= */
#define MG90S_PWM_GPIO 25

/* ================= TCRT5000 ARRAY ================= */
#define TCRT_SEL1_GPIO 19
#define TCRT_SEL2_GPIO 18
#define TCRT_SEL3_GPIO 17
#define TCRT_ADC_GPIO 36

/* ================= VL53L0X ================= */
#define VL53L0X_SDA_GPIO 16
#define VL53L0X_SCL_GPIO 4

/* ================= SPI ================= */
#define SPI_SCK_GPIO 18
#define SPI_MISO_GPIO 19
#define SPI_MOSI_GPIO 23
#define SPI_CS_GPIO 5

/* ================= STATUS LED ================= */
#define LED_RED_GPIO 27
#define LED_GREEN_GPIO 26

/* ================= UART ================= */
#define UART_RX_GPIO 3
#define UART_TX_GPIO 1

#define BOOT_GPIO 0

#endif