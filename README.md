## Features

- **MCU:** STM32L0 (STM32L051xx) on a custom PCB
- **Very low power** operation using LPTIM and low-frequency clock sources
- **Multiple PWM generation modes** with EMC in mind:
  - `LowEMI-Stop` – LPTIM clocked from LSE/LSI, MCU in STOP mode, edge-aligned PWM, no dithering
  - `LowEMI-Sleep` – low-power PWM with MCU in SLEEP mode and standard timers
  - `CA-Base` – basic **center-aligned** PWM (timer counts up/down, pulse centered in period)
  - `CA-PRBS` – center-aligned PWM with pseudo-random modulation (PRBS / LFSR) to spread EMI
  - `CA-DMA` – PWM with DMA-driven update (triangle / shaped pattern for spectrum spreading)
  - `AutoTune-Freq` – automatic search for a PWM frequency that minimizes supply current / EMI
- **Automotive-style use case:**
