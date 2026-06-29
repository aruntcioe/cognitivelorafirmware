# Cognitive LoRa Firmware

AI-assisted cognitive radio LPWAN firmware and simulation assets for LoRa communication in the 433 MHz band.

## Overview

This repository contains:

- **Transmitter firmware** (`cr_transmitter/`) for sending application data and receiving control commands.
- **Receiver firmware** (`cr_receiver/`) for collecting link metrics, running AI-based decisions, and sending adaptation commands.
- **Simulation code** (`simulation/`) for LoRa physical-layer emulation and dataset generation.
- **Hardware connection references** (`hardware-connection/`) including images and wiring documentation.

The project uses a **dual-plane architecture**:

- **Data plane** at ~433 MHz for continuous payload traffic.
- **Control plane** at 445 MHz for cognitive/adaptive commands (channel hop, SF/CR adaptation).

## Repository Structure

- `cr_transmitter/cr_transmitter.ino` – main transmitter node firmware.
- `cr_transmitter/transmitterv2.ino` – alternate/duplicate transmitter sketch.
- `cr_receiver/cr_receiver.ino` – main receiver/gateway node firmware.
- `cr_receiver/receiverv2.ino` – alternate receiver sketch revision.
- `simulation/LoRaPHY.m` – LoRa PHY implementation used by simulation scripts.
- `simulation/fuyalsimulation.m` – bottom-up physical simulation and dataset generator.
- `hardware-connection/` – wiring spreadsheet and reference images.

## Key Features

- ESP32 + SX1278 based LoRa transmitter/receiver implementation.
- Runtime radio reconfiguration via control channel:
  - Channel hopping
  - Spreading Factor (SF) adaptation
  - Coding Rate (CR) adaptation
- Feature-window based link monitoring on receiver side:
  - RSSI / SNR statistics
  - Packet loss rate (PLR)
  - CRC failure tracking
  - Time-on-air (ToA)
- MATLAB simulation pipeline for generating labeled physical-layer dataset samples.

## Hardware

The firmware targets a setup with:

- **ESP32**
- **Two SX1278 modules per node**
  - One for data plane (~433 MHz)
  - One for control plane (445 MHz)

See `hardware-connection/` for wiring references and images.

## Software & Tooling

- **Arduino / PlatformIO compatible C++ firmware** (uses `RadioLib` and `SPI`)
- **MATLAB/Octave-style scripts** for PHY simulation and dataset generation

## Getting Started

### 1) Firmware setup

1. Install Arduino IDE or PlatformIO.
2. Install required libraries:
   - `RadioLib`
   - `SPI` (included in Arduino core)
3. Open and configure:
   - `cr_transmitter/cr_transmitter.ino`
   - `cr_receiver/cr_receiver.ino`
4. Verify pin definitions for your exact ESP32 board and SX1278 wiring.
5. Flash transmitter and receiver nodes.

### 2) Simulation setup

1. Open MATLAB.
2. Ensure both files are accessible:
   - `simulation/LoRaPHY.m`
   - `simulation/fuyalsimulation.m`
3. Run `fuyalsimulation.m`.
4. Generated dataset is exported as CSV (per script output filename).

## Notes

- This repository is an active prototype/research project.
- Some sketches include work-in-progress sections and likely require cleanup/refinement before production deployment.
- Frequency and parameter values should be validated against local regulations and your hardware front-end.

## License

No license file is currently present in this repository.
