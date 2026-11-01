#include <Arduino.h>

// Clock setup for the Octopus Pro's 25 MHz crystal. Replaces the Nucleo-H723ZG variant's
// SystemClock_Config (declared weak), which expects an 8 MHz clock from the ST-Link.
//
//   25 MHz HSE / 5 = 5 MHz  ->  x160 = 800 MHz VCO  ->  /2 = 400 MHz CPU
//   AHB 200 MHz, APB1..4 100 MHz, USB from the internal 48 MHz oscillator (HSI48),
//   trimmed against the USB host's 1 kHz start-of-frame by the CRS.
//
// 400 MHz is below the chip's 550 MHz maximum on purpose: plenty for step generation, with
// relaxed flash timing. The INFO command prints the resulting frequency.

// The HAL computes every clock frequency (millis, delays, baud rates) from HSE_VALUE.
static_assert(HSE_VALUE == 25000000U, "HSE_VALUE must match the Octopus Pro's 25 MHz crystal");

extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {};
  RCC_ClkInitTypeDef clk = {};
  RCC_PeriphCLKInitTypeDef periph = {};
  RCC_CRSInitTypeDef crs = {};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  osc.HSEState = RCC_HSE_ON;
  osc.HSI48State = RCC_HSI48_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 5;
  osc.PLL.PLLN = 160;
  osc.PLL.PLLP = 2;
  osc.PLL.PLLQ = 8;
  osc.PLL.PLLR = 2;
  osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;  // 4-8 MHz PLL input
  osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  osc.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider = RCC_HCLK_DIV2;
  clk.APB3CLKDivider = RCC_APB3_DIV2;
  clk.APB1CLKDivider = RCC_APB1_DIV2;
  clk.APB2CLKDivider = RCC_APB2_DIV2;
  clk.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_CRS_CLK_ENABLE();
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB1;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  crs.ErrorLimitValue = 34;
  crs.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&crs);

  HAL_PWREx_EnableUSBVoltageDetector();
}
