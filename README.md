The system is controlled by an Arduino ATmega2560 Microcontroller (MCU), which integrates all sensors and actuators, and receives input from a remote controller through a transceiver module. A power supply provides power to both the motor and the MCU through a motor driver voltage regulator. A fail-safe feature to cut the string is implemented through a linear actuator with a razor blade attached to the tip. 

System inputs and outputs include,
Inputs:
Transceiver
LiDAR Range Finder
Current Sensor
Power Supply
Outputs:
DC Motor
Linear Actuator

Specific components have been chosen that easily integrate with this MCU utilizing either a 5 Volt or 3.3 Volt logic, and all components are powered through the motor driver voltage regulator or the MCU itself. Weight is a major consideration for the selection of specific components to minimize interference with drone flight. All electronic components, including the motor and power supply, weigh a total of 528 grams.

Electronic Components List:
MCU: ATMega2560 (37 g)
Transceiver: SX1280 (29 g)
Range Finder: LiDAR Lite V3 HP (LLV3HP) (34 g)
Current Sensor: INA219 (3 g)
Power Supply: TalentCell 12V 3000mAh Li-ion Battery Pack  (190 g)
DC Motor: RS555-EN 12V Brushed DC Motor (150 g)
Motor Driver: BTS7960 Motor Driver (66 g)
Linear Actuator: Actuonix PQ12R (19 g)

The system can also be independently controlled through a remote controller that communicates with the MCU through the transceiver
