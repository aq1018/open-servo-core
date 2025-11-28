# open-servo-core

Democratize Robotics For Everyone

## Overview

This project aims to create a drop-in replacement control board for MG90-class servos that adds cascade control, current sensing, and a DYNAMIXEL-style serial protocol. The goal is to make robotics more accessible by turning a $2.50 servo into a $5 smart actuator (vs $30+ for commercial alternatives). Built with bare-metal Rust on STM32/CH32V microcontrollers.

## Project Status

🚧 **Early Development** - Not ready for general use

- ✅ **Working**: Basic PID control, PWM motor drive, current sensing, ADC readings
- 🔄 **In Progress**: Cascade control loops, UART protocol implementation  
- 📋 **Planned**: System identification, auto-tuning, multi-servo bus communication

## Project Structure

```
firmware/open-servo-core/  - Rust firmware for STM32F301
hardware/
  servo-dev-board/         - Development board for firmware testing  
  encoder-board/           - Quadrature IR sensor board for system ID
  motor-mount/             - Test fixtures and mechanical designs
  common/                  - Shared KiCad symbols & footprints
```

## Development

- **Firmware**: Bare-metal Rust using PAC-level drivers, debugged with probe-rs and defmt
- **Hardware**: KiCad designs with JLCPCB production files available
- **Target MCU**: STM32F301 (current), CH32V003 (planned)
- **Control**: Cascaded loops - current → velocity → position

## Goals

- Implement cascade control with inner current, middle velocity, and outer position loops
- Create a DYNAMIXEL-inspired protocol for controlling multiple servos on one bus
- Add system identification and auto-tuning capabilities
- Keep total cost under $5 per modified servo
- Require no mechanical modifications to the servo

## Contributing

This is experimental research code in active development. Issues and discussions are welcome. Check the hardware folders for KiCad designs and the firmware folder for the Rust implementation.

## License

MIT License - See [LICENSE](LICENSE) file for details