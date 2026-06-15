## Nexys 4 DDR / XC7A100T-CSG324 parking lot main board
## Top module: parking_main_board_top

## 100 MHz clock
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { clk }]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { clk }]

## CPU reset button, active low
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } [get_ports { reset_n }]

## LEDs for debug/status
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { led[5] }]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { led[6] }]
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { led[7] }]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports { led[8] }]
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports { led[9] }]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports { led[10] }]
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports { led[11] }]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports { led[12] }]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports { led[13] }]
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { led[14] }]
set_property -dict { PACKAGE_PIN V11 IOSTANDARD LVCMOS33 } [get_ports { led[15] }]

## PMOD JA: RC522 RFID module
## JA1 -> SDA/SS, JA2 -> SCK, JA3 -> MOSI, JA4 -> MISO, JA7 -> RST
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports { rc522_ss_n }]
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports { rc522_sck }]
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { rc522_mosi }]
set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS33 } [get_ports { rc522_miso }]
set_property -dict { PACKAGE_PIN D17 IOSTANDARD LVCMOS33 } [get_ports { rc522_rst }]

## PMOD JB: LCD I2C and ultrasonic sensor
## JB1 -> LCD SDA, JB2 -> LCD SCL, JB3 -> ultrasonic TRIG, JB4 -> ultrasonic ECHO
set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports { lcd_sda }]
set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS33 } [get_ports { lcd_scl }]
set_property PULLUP true [get_ports { lcd_sda }]
set_property PULLUP true [get_ports { lcd_scl }]
set_property -dict { PACKAGE_PIN G16 IOSTANDARD LVCMOS33 } [get_ports { ultrasonic_trig }]
set_property -dict { PACKAGE_PIN H14 IOSTANDARD LVCMOS33 } [get_ports { ultrasonic_echo }]

## PMOD JC: two servos and buzzer
## JC1 -> entry servo signal, JC2 -> exit servo signal, JC3 -> buzzer signal
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { entry_servo_pwm }]
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 } [get_ports { exit_servo_pwm }]
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { buzzer }]

## PMOD JD: MAX7219 8x8 LED matrix
## JD1 -> DIN, JD2 -> CS/LOAD, JD3 -> CLK
set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 } [get_ports { matrix_din }]
set_property -dict { PACKAGE_PIN H1 IOSTANDARD LVCMOS33 } [get_ports { matrix_cs }]
set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 } [get_ports { matrix_sclk }]
