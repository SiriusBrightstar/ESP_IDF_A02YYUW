# A02YYUW IP67 Rated Ultrasonic Distance Sensor

## Specifications

- Operating Voltage:                  3.3~5V
- standby Current:                    ≤5mA
- Average Current:                    ≤8mA
- Blind Zone Distance:                30mm
- Ranging Distance for Flat Object:   30-4500mm
- Output:                             UART
- Response Time:                      100ms
- Sensing Angle:                      60°
- Probe Center Frequency:             40K±1.0K
- Operating Temperature:              -15~60℃
- Protection Rate:                    IP67

Wiki Link: [DFRobot](https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311)

## Usage

There are 2 modes supported:

- Read Time data:
  - Response Time 100ms
  - Pass `A02YYUW_PIN_SELECT_REALTIME` in `a02yyuw_init` function
- Processed Data:
  - Response Time 100-300ms
  - This data is more steady
  - Pass `A02YYUW_PIN_SELECT_PROCESSED` in `a02yyuw_init` function

## Output

```log
I (415) MAIN: Distance: 1262mm
I (2515) MAIN: Distance: 1254mm
```
