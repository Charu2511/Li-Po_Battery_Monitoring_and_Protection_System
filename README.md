# Li-Po_Battery_Monitoring_and_Protection_System

An Arduino Mega 2560-based embedded system for real-time monitoring, State of Charge (SOC) estimation, energy tracking, and automatic protection of a 3S Lithium-Polymer (Li-Po) battery.

## Overview

This project presents a low-cost battery monitoring and protection platform designed to continuously evaluate the operating condition of a 3S Li-Po battery (11.1 V, 1.55 Ah).

Unlike conventional voltage-only battery indicators, the system combines battery voltage and load current measurements with C-rate-dependent discharge curve interpolation to estimate the remaining battery capacity.

The system monitors essential electrical parameters, including voltage, current, instantaneous power, consumed energy, and operating time. It also incorporates multiple visual indicators and an automatic relay-based protection mechanism to disconnect the load during overcurrent and low-capacity conditions.

## Key Features

- **Real-Time Monitoring:** Continuous measurement of battery voltage and load current.
- **Power and Energy Calculation:** Calculation of instantaneous power and accumulated energy consumption.
- **Load-Dependent SOC Estimation:** Estimation of battery State of Charge using C-rate-based discharge curve interpolation.
- **Multiple SOC Indicators:** Numerical SOC display and LM3914-based LED level indication.
- **Current Status Indication:** RGB LED indication across six predefined current operating conditions.
- **Overcurrent Protection:** Automatic load disconnection at or above 1.30 A with a latched protection mechanism.
- **Low-Capacity Protection:** Automatic load disconnection when estimated remaining SOC reaches approximately 12%.
- **Serial Monitoring:** Transmission of measured and calculated operating parameters for monitoring and analysis.

## System Specifications

| Parameter | Specification |
|---|---|
| Battery Type | Lithium-Polymer (Li-Po) |
| Battery Configuration | 3S |
| Nominal Battery Voltage | 11.1 V |
| Rated Capacity | 1.55 Ah |
| Microcontroller | Arduino Mega 2560 |
| Current Sensor | ACS712-30A |
| Voltage Measurement | Analog Voltage Sensor |
| Overcurrent Threshold | 1.30 A |
| Low-Capacity Threshold | 1.364 Ah Used |
| Minimum Usable SOC | Approximately 12% |
| Communication | Serial (UART) |

## Hardware Components

| Component | Function |
|---|---|
| Arduino Mega 2560 | Main processing and control unit |
| 3S Li-Po Battery | Power source |
| ACS712-30A | Load current measurement |
| Voltage Sensor Module | Battery voltage measurement |
| LCD Display | Voltage, current, power, energy, and elapsed-time display |
| 4-Digit 7-Segment Display | Numerical SOC indication |
| LM3914 Driver IC | LED-based battery-level indication |
| Common-Anode RGB LED | Current operating status indication |
| Relay Module | Automatic load disconnection |
| RC Low-Pass Filter | PWM-to-DC signal conversion for LM3914 |
| LEDs and Passive Components | Visual indication and signal conditioning |

## Working Methodology

The system operates through four primary functional stages:

### 1. Voltage and Current Measurement

Battery voltage and load current are measured using an analog voltage sensor and an ACS712 current sensor, respectively.

- Battery voltage is acquired through analog input A0.
- Load current is acquired through analog input A1.
- The ACS712 zero-current output is calibrated during startup.
- Averaging, deadband handling, and smoothing techniques are applied to reduce measurement noise.

### 2. Power and Energy Calculation

Instantaneous power is calculated from the measured battery voltage and load current.

**Instantaneous Power:**

P = V × I

Accumulated energy is calculated using the actual time interval between successive measurements.

**Consumed Energy:**

ΔE = (P × Δt) / 3600

Where:
- P = Instantaneous power (W)
- Δt = Sampling interval (seconds)
- ΔE = Consumed energy (Wh)

The total operating time is also accumulated and displayed in hours, minutes, and seconds.

### 3. State of Charge (SOC) Estimation

The system employs a C-rate-dependent discharge curve interpolation method to estimate the battery's used capacity and remaining SOC.

The estimation process involves:

1. Calculating the battery C-rate from the measured load current.
2. Selecting the two surrounding C-rate columns from the predefined battery discharge table.
3. Interpolating the discharge characteristics for the calculated C-rate.
4. Comparing the measured battery voltage with the interpolated discharge curve.
5. Estimating the consumed battery capacity (Ah).
6. Calculating the remaining battery SOC.

**C-rate:**

C-rate = I / Battery Capacity (Ah)

**State of Charge:**

SOC (%) = [(Capacity − Used Capacity) / Capacity] × 100

The numerical SOC is displayed on a dedicated four-digit display.

Additionally, the estimated SOC is mapped to a PWM signal (0–255), which passes through an external RC low-pass filter to provide a proportional DC input to the LM3914 LED driver.

### 4. Battery Protection and Status Indication

The system incorporates current-dependent RGB indication and automatic relay control.

**RGB LED Operating Conditions**

| Load Current | RGB Indication |
|---|---|
| I < 0.25 A | OFF |
| 0.25 A ≤ I < 0.60 A | Green |
| 0.60 A ≤ I < 0.80 A | Yellow |
| 0.80 A ≤ I < 1.00 A | Orange |
| 1.00 A ≤ I < 1.30 A | Slow Red Blinking |
| I ≥ 1.30 A | Fast Red Blinking + Relay Cutoff |

**Protection Mechanisms**

| Protection Condition | System Response |
|---|---|
| Overcurrent (≥ 1.30 A) | Relay opens and the fault is latched until reset or power cycle. |
| Low Battery Capacity (≥ 1.364 Ah Used) | Relay opens and low-capacity warning is activated. |
| Battery Voltage ≤ 11.0 V during recovery | Load remains disconnected. |
| Battery Voltage > 11.0 V during recovery | Load can reconnect if current is ≤ 0.25 A and no overcurrent latch is active. |

## Software and Development Tools

- **Arduino IDE:** Embedded programming and microcontroller implementation.
- **C/C++:** Embedded system programming.
- **Proteus:** Circuit design and hardware simulation.
- **MATLAB / Spreadsheet Tools:** Discharge curve analysis and interpolation verification.

## Repository Structure

```text
Li-Po-Battery-Monitoring-and-Protection/
│
├── README.md
│
├── Arduino_Code/
│   └── battery_monitoring_system.ino
│
├── Circuit_Diagram/
│   ├── circuit_diagram.png
│   └── pcb_layout.png
│
├── Documentation/
│   └── project_report.pdf
│
└── Images/
    └── hardware_implementation.jpg
```

*Note: The directory structure is a suggested organization for this repository.*

## Implementation and Testing

The prototype integrates the sensing, processing, display, indication, and protection subsystems using an Arduino Mega 2560.

The implemented functions include:

- Voltage and current acquisition with sensor calibration.
- Power and energy calculations.
- C-rate-dependent SOC estimation.
- Numerical and LED-based SOC indication.
- Current-dependent RGB status indication.
- Automatic overcurrent and low-capacity relay protection.
- Serial transmission of operating parameters and protection status.

For further validation, sensor readings and protection thresholds should be compared against reference instruments under controlled operating conditions.

## Future Improvements

The following improvements are proposed for future development:

- Individual cell-voltage monitoring for all three battery cells.
- Temperature sensing and over-temperature protection.
- Experimental validation and optimization of battery discharge curves.
- SOC hysteresis and timed relay recovery logic.
- Comparison with coulomb counting and model-based SOC estimation methods.
- Non-volatile data storage for energy consumption and fault history.
- Integration of a dedicated Battery Management System (BMS) with cell balancing.
- Development of a compact custom PCB for practical deployment.

## Safety Note

This project is an academic prototype intended for battery monitoring and protection demonstrations. Its voltage- and C-rate-based SOC estimation depends on the accuracy of the predefined discharge characteristics and sensor calibration.

It should not be considered a replacement for a dedicated, certified Battery Management System (BMS). Appropriate cell-level protection, fusing, and safe battery-handling practices are necessary for practical deployment.

## Author

**Tanzila Sultana**

Department of Electrical & Electronic Engineering  
Rajshahi University of Engineering & Technology (RUET)  
Bangladesh

**Project:** Li-Po Battery Monitoring and Protection System  
**Course:** EEE 4002 – Capstone Project Design II

---

*Developed as an undergraduate capstone project at RUET.*otection_System
