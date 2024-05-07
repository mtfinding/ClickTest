# Automated Continuity Testing Device

## Overview

This repository contains the finalized source-code and hardware design documents of an automated hardware-tester. The projects was created as part of bachelor-thesis and is specifically designed for the Nucleo L432KC development board in combination with a propriatory harwdare expander used at my uni FHTW)  
## Features

- **Automated Continuity Testing**: The device automates the continuity testing process, reducing manual effort and increasing efficiency.
- **Terminal Interface**: A 115200baud UART terminal is available to visulaize test-results and select from certain testing-options.
- **Quick-Test Button**: A physical button that can be mapped to various test-actions via terminal commands.

## System overview

The system consists of a single PCBA project which can be broken apart in all necessary components with only little manual THT soldering required.
The core components are: 
- 

![hw_overview](https://github.com/[mtfinding]/[ClickTest]/img/[main]/click_overview.png?raw=true)

