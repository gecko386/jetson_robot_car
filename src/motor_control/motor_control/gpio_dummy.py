#GPIO Dummy

CLARA_AGX_XAVIER = 'CLARA_AGX_XAVIER'
JETSON_NX = 'JETSON_NX'
JETSON_XAVIER = 'JETSON_XAVIER'
JETSON_TX2 = 'JETSON_TX2'
JETSON_TX1 = 'JETSON_TX1'
JETSON_NANO = 'JETSON_NANO'
JETSON_TX2_NX='JETSON_TX2_NX'

# These arrays contain tuples of all the relevant GPIO data for each Jetson
# Platform. The fields are:
# - Linux GPIO pin number (within chip, not global),
#   (map from chip GPIO count to value, to cater for different numbering schemes)
# - Linux exported GPIO name,
#   (map from chip GPIO count to value, to cater for different naming schemes)
#   (entries omitted if exported filename is gpio%i)
# - GPIO chip sysfs directory
# - Pin number (BOARD mode)
# - Pin number (BCM mode)
# - Pin name (CVM mode)
# - Pin name (TEGRA_SOC mode)
# - PWM chip sysfs directory
# - PWM ID within PWM chip
# The values are used to generate dictionaries that map the corresponding pin
# mode numbers to the Linux GPIO pin number and GPIO chip directory

CLARA_AGX_XAVIER_PIN_DEFS = [
    ({224: 134, 169: 106}, {169:  'PQ.06'}, "2200000.gpio", 7, 4, 'MCLK05', 'SOC_GPIO42', None, None),
    ({224: 140, 169: 112}, {169:  'PR.04'}, "2200000.gpio", 11, 17, 'UART1_RTS', 'UART1_RTS', None, None),
    ({224:  63, 169:  51}, {169:  'PH.07'}, "2200000.gpio", 12, 18, 'I2S2_CLK', 'DAP2_SCLK', None, None),
    ({224: 124, 169:  96}, {169:  'PP.04'}, "2200000.gpio", 13, 27, 'GPIO32', 'SOC_GPIO04', None, None),
    # Older versions of L4T don't enable this PWM controller in DT, so this PWM
    # channel may not be available.
    ({224: 105, 169:  84}, {169:  'PN.01'}, "2200000.gpio", 15, 22, 'GPIO27', 'SOC_GPIO54', '3280000.pwm', 0),
    ({ 40:   8,  30:   8}, { 30: 'PBB.00'}, "c2f0000.gpio", 16, 23, 'GPIO8', 'CAN1_STB', None, None),
    ({224:  56, 169:  44}, {169:  'PH.00'}, "2200000.gpio", 18, 24, 'GPIO35', 'SOC_GPIO12', '32c0000.pwm', 0),
    ({224: 205, 169: 162}, {169:  'PZ.05'}, "2200000.gpio", 19, 10, 'SPI1_MOSI', 'SPI1_MOSI', None, None),
    ({224: 204, 169: 161}, {169:  'PZ.04'}, "2200000.gpio", 21, 9, 'SPI1_MISO', 'SPI1_MISO', None, None),
    ({224: 129, 169: 101}, {169:  'PQ.01'}, "2200000.gpio", 22, 25, 'GPIO17', 'SOC_GPIO21', None, None),
    ({224: 203, 169: 160}, {169:  'PZ.03'}, "2200000.gpio", 23, 11, 'SPI1_CLK', 'SPI1_SCK', None, None),
    ({224: 206, 169: 163}, {169:  'PZ.06'}, "2200000.gpio", 24, 8, 'SPI1_CS0_N', 'SPI1_CS0_N', None, None),
    ({224: 207, 169: 164}, {169:  'PZ.07'}, "2200000.gpio", 26, 7, 'SPI1_CS1_N', 'SPI1_CS1_N', None, None),
    ({ 40:   3,  30:   3}, { 30: 'PAA.03'}, "c2f0000.gpio", 29, 5, 'CAN0_DIN', 'CAN0_DIN', None, None),
    ({ 40:   2,  30:   2}, { 30: 'PAA.02'}, "c2f0000.gpio", 31, 6, 'CAN0_DOUT', 'CAN0_DOUT', None, None),
    ({ 40:   9,  30:   9}, { 30: 'PBB.01'}, "c2f0000.gpio", 32, 12, 'GPIO9', 'CAN1_EN', None, None),
    ({ 40:   0,  30:   0}, { 30: 'PAA.00'}, "c2f0000.gpio", 33, 13, 'CAN1_DOUT', 'CAN1_DOUT', None, None),
    ({224:  66, 169:  54}, {169:  'PI.02'}, "2200000.gpio", 35, 19, 'I2S2_FS', 'DAP2_FS', None, None),
    # Input-only (due to base board)
    ({224: 141, 169: 113}, {169:  'PR.05'}, "2200000.gpio", 36, 16, 'UART1_CTS', 'UART1_CTS', None, None),
    ({ 40:   1,  30:   1}, { 30: 'PAA.01'}, "c2f0000.gpio", 37, 26, 'CAN1_DIN', 'CAN1_DIN', None, None),
    ({224:  65, 169:  53}, {169:  'PI.01'}, "2200000.gpio", 38, 20, 'I2S2_DIN', 'DAP2_DIN', None, None),
    ({224:  64, 169:  52}, {169:  'PI.00'}, "2200000.gpio", 40, 21, 'I2S2_DOUT', 'DAP2_DOUT', None, None)
]
compats_clara_agx_xavier = (
    'nvidia,e3900-0000+p2888-0004',
)

JETSON_NX_PIN_DEFS = [
    ({224: 148, 169: 118}, {169:  'PS.04'}, "2200000.gpio", 7, 4, 'GPIO09', 'AUD_MCLK', None, None),
    ({224: 140, 169: 112}, {169:  'PR.04'}, "2200000.gpio", 11, 17, 'UART1_RTS', 'UART1_RTS', None, None),
    ({224: 157, 169: 127}, {169:  'PT.05'}, "2200000.gpio", 12, 18, 'I2S0_SCLK', 'DAP5_SCLK', None, None),
    ({224: 192, 169: 149}, {169:  'PY.00'}, "2200000.gpio", 13, 27, 'SPI1_SCK', 'SPI3_SCK', None, None),
    ({ 40:  20,  30:  16}, { 30: 'PCC.04'}, "c2f0000.gpio", 15, 22, 'GPIO12', 'TOUCH_CLK', None, None),
    ({224: 196, 169: 153}, {169:  'PY.04'}, "2200000.gpio", 16, 23, 'SPI1_CS1', 'SPI3_CS1_N', None, None),
    ({224: 195, 169: 152}, {169:  'PY.03'}, "2200000.gpio", 18, 24, 'SPI1_CS0', 'SPI3_CS0_N', None, None),
    ({224: 205, 169: 162}, {169:  'PZ.05'}, "2200000.gpio", 19, 10, 'SPI0_MOSI', 'SPI1_MOSI', None, None),
    ({224: 204, 169: 161}, {169:  'PZ.04'}, "2200000.gpio", 21, 9, 'SPI0_MISO', 'SPI1_MISO', None, None),
    ({224: 193, 169: 150}, {169:  'PY.01'}, "2200000.gpio", 22, 25, 'SPI1_MISO', 'SPI3_MISO', None, None),
    ({224: 203, 169: 160}, {169:  'PZ.03'}, "2200000.gpio", 23, 11, 'SPI0_SCK', 'SPI1_SCK', None, None),
    ({224: 206, 169: 163}, {169:  'PZ.06'}, "2200000.gpio", 24, 8, 'SPI0_CS0', 'SPI1_CS0_N', None, None),
    ({224: 207, 169: 164}, {169:  'PZ.07'}, "2200000.gpio", 26, 7, 'SPI0_CS1', 'SPI1_CS1_N', None, None),
    ({224: 133, 169: 105}, {169:  'PQ.05'}, "2200000.gpio", 29, 5, 'GPIO01', 'SOC_GPIO41', None, None),
    ({224: 134, 169: 106}, {169:  'PQ.06'}, "2200000.gpio", 31, 6, 'GPIO11', 'SOC_GPIO42', None, None),
    ({224: 136, 169: 108}, {169:  'PR.00'}, "2200000.gpio", 32, 12, 'GPIO07', 'SOC_GPIO44', '32f0000.pwm', 0),
    ({224: 105, 169:  84}, {169:  'PN.01'}, "2200000.gpio", 33, 13, 'GPIO13', 'SOC_GPIO54', '3280000.pwm', 0),
    ({224: 160, 169: 130}, {169:  'PU.00'}, "2200000.gpio", 35, 19, 'I2S0_FS', 'DAP5_FS', None, None),
    ({224: 141, 169: 113}, {169:  'PR.05'}, "2200000.gpio", 36, 16, 'UART1_CTS', 'UART1_CTS', None, None),
    ({224: 194, 169: 151}, {169:  'PY.02'}, "2200000.gpio", 37, 26, 'SPI1_MOSI', 'SPI3_MOSI', None, None),
    ({224: 159, 169: 129}, {169:  'PT.07'}, "2200000.gpio", 38, 20, 'I2S0_DIN', 'DAP5_DIN', None, None),
    ({224: 158, 169: 128}, {169:  'PT.06'}, "2200000.gpio", 40, 21, 'I2S0_DOUT', 'DAP5_DOUT', None, None)
]
compats_nx = (
    'nvidia,p3509-0000+p3668-0000',
    'nvidia,p3509-0000+p3668-0001',
    'nvidia,p3449-0000+p3668-0000',
    'nvidia,p3449-0000+p3668-0001',
)

JETSON_XAVIER_PIN_DEFS = [
    ({224: 134, 169: 106}, {169:  'PQ.06'}, "2200000.gpio", 7, 4, 'MCLK05', 'SOC_GPIO42', None, None),
    ({224: 140, 169: 112}, {169:  'PR.04'}, "2200000.gpio", 11, 17, 'UART1_RTS', 'UART1_RTS', None, None),
    ({224:  63, 169:  51}, {169:  'PH.07'}, "2200000.gpio", 12, 18, 'I2S2_CLK', 'DAP2_SCLK', None, None),
    ({224: 136, 169: 108}, {169:  'PR.00'}, "2200000.gpio", 13, 27, 'PWM01', 'SOC_GPIO44', '32f0000.pwm', 0),
    # Older versions of L4T don'Pt enable this PWM controller in DT, so this PWM
    # channel may not be available.
    ({224: 105, 169:  84}, {169:  'PN.01'}, "2200000.gpio", 15, 22, 'GPIO27', 'SOC_GPIO54', '3280000.pwm', 0),
    ({ 40:   8,  30:   8}, { 30: 'PBB.00'}, "c2f0000.gpio", 16, 23, 'GPIO8', 'CAN1_STB', None, None),
    ({224:  56, 169:  44}, {169:  'PH.00'}, "2200000.gpio", 18, 24, 'GPIO35', 'SOC_GPIO12', '32c0000.pwm', 0),
    ({224: 205, 169: 162}, {169:  'PZ.05'}, "2200000.gpio", 19, 10, 'SPI1_MOSI', 'SPI1_MOSI', None, None),
    ({224: 204, 169: 161}, {169:  'PZ.04'}, "2200000.gpio", 21, 9, 'SPI1_MISO', 'SPI1_MISO', None, None),
    ({224: 129, 169: 101}, {169:  'PQ.01'}, "2200000.gpio", 22, 25, 'GPIO17', 'SOC_GPIO21', None, None),
    ({224: 203, 169: 160}, {169:  'PZ.03'}, "2200000.gpio", 23, 11, 'SPI1_CLK', 'SPI1_SCK', None, None),
    ({224: 206, 169: 163}, {169:  'PZ.06'}, "2200000.gpio", 24, 8, 'SPI1_CS0_N', 'SPI1_CS0_N', None, None),
    ({224: 207, 169: 164}, {169:  'PZ.07'}, "2200000.gpio", 26, 7, 'SPI1_CS1_N', 'SPI1_CS1_N', None, None),
    ({ 40:   3,  30:   3}, { 30: 'PAA.03'}, "c2f0000.gpio", 29, 5, 'CAN0_DIN', 'CAN0_DIN', None, None),
    ({ 40:   2,  30:   2}, { 30: 'PAA.02'}, "c2f0000.gpio", 31, 6, 'CAN0_DOUT', 'CAN0_DOUT', None, None),
    ({ 40:   9,  30:   9}, { 30: 'PBB.01'}, "c2f0000.gpio", 32, 12, 'GPIO9', 'CAN1_EN', None, None),
    ({ 40:   0,  30:   0}, { 30: 'PAA.00'}, "c2f0000.gpio", 33, 13, 'CAN1_DOUT', 'CAN1_DOUT', None, None),
    ({224:  66, 169:  54}, {169:  'PI.02'}, "2200000.gpio", 35, 19, 'I2S2_FS', 'DAP2_FS', None, None),
    # Input-only (due to base board)
    ({224: 141, 169: 113}, {169:  'PR.05'}, "2200000.gpio", 36, 16, 'UART1_CTS', 'UART1_CTS', None, None),
    ({ 40:   1,  30:   1}, { 30: 'PAA.01'}, "c2f0000.gpio", 37, 26, 'CAN1_DIN', 'CAN1_DIN', None, None),
    ({224:  65, 169:  53}, {169:  'PI.01'}, "2200000.gpio", 38, 20, 'I2S2_DIN', 'DAP2_DIN', None, None),
    ({224:  64, 169:  52}, {169:  'PI.00'}, "2200000.gpio", 40, 21, 'I2S2_DOUT', 'DAP2_DOUT', None, None)
]
compats_xavier = (
    'nvidia,p2972-0000',
    'nvidia,p2972-0006',
    'nvidia,jetson-xavier',
    'nvidia,galen-industrial',
    'nvidia,jetson-xavier-industrial',
)

JETSON_TX2_NX_PIN_DEFS = [
    ({192:  76, 140: 66}, {140:  'PJ.04'}, "2200000.gpio", 7, 4, 'GPIO09', 'AUD_MCLK', None, None),
    ({64: 28, 47: 23}, {47:  'PW.04'}, "c2f0000.gpio", 11, 17, 'UART1_RTS', 'UART3_RTS', None, None),
    ({192:  72, 140: 62}, {140:  'PJ.00'}, "2200000.gpio", 12, 18, 'I2S0_SCLK', 'DAP1_SCLK', None, None),
    ({64:  17, 47: 12}, {47:  'PV.01'}, "c2f0000.gpio", 13, 27, 'SPI1_SCK', 'GPIO_SEN1', None, None),
    ({192:  18,  140: 16}, {140: 'PC.02'}, "2200000.gpio", 15, 22, 'GPIO12', 'DAP2_DOUT', None, None),
    ({192: 19, 140: 17}, {140:  'PC.03'}, "2200000.gpio", 16, 23, 'SPI1_CS1', 'DAP2_DIN', None, None),
    ({64: 20, 47: 15}, {47:  'PV.04'}, "c2f0000.gpio", 18, 24, 'SPI1_CS0', 'GPIO_SEN4', None, None),
    ({192: 58, 140:  49}, {140:  'PH.02'}, "2200000.gpio", 19, 10, 'SPI0_MOSI', 'GPIO_WAN7', None, None),
    ({192: 57, 140: 48}, {140:  'PH.01'}, "2200000.gpio", 21, 9, 'SPI0_MISO', 'GPIO_WAN6', None, None),
    ({64: 18, 47: 13}, {47:  'PV.02'}, "c2f0000.gpio", 22, 25, 'SPI1_MISO', 'GPIO_SEN2', None, None),
    ({192: 56, 140:  47}, {140:  'PH.00'}, "2200000.gpio", 23, 11, 'SPI1_CLK', 'GPIO_WAN5', None, None),
    ({192: 59, 140: 50}, {140:  'PH.03'}, "2200000.gpio", 24, 8, 'SPI0_CS0', 'GPIO_WAN8', None, None),
    ({192: 163, 140: 130}, {140:  'PY.03'}, "2200000.gpio", 26, 7, 'SPI0_CS1', 'GPIO_MDM4', None, None),
    ({192: 105, 140: 86}, {140:  'PN.01'}, "2200000.gpio", 29, 5, 'GPIO01', 'GPIO_CAM2', None, None),
    ({64: 50, 47: 41}, {47:  'PEE.02'}, "c2f0000.gpio", 31, 6, 'GPIO11', 'TOUCH_CLK', None, None),
    ({64: 8, 47: 5}, {47:  'PU.00'}, "c2f0000.gpio", 32, 12, 'GPIO07', 'GPIO_DIS0', '3280000.pwm', 0),
    ({64: 13, 47: 10}, {47:  'PU.05'}, "c2f0000.gpio", 33, 13, 'GPIO13', 'GPIO_DIS5', '32a0000.pwm', 0),
    ({192: 75, 140: 65}, {140:  'PJ.03'}, "2200000.gpio", 35, 19, 'I2S0_FS', 'DAP1_FS', None, None),
    ({64: 29, 47: 24}, {47:  'PW.05'}, "c2f0000.gpio", 36, 16, 'UART1_CTS', 'UART3_CTS', None, None),
    ({64: 19, 47: 14}, {47:  'PV.03'}, "c2f0000.gpio", 37, 26, 'SPI1_MOSI', 'GPIO_SEN3', None, None),
    ({192: 74, 140: 64}, {140:  'PJ.02'}, "2200000.gpio", 38, 20, 'I2S0_DIN', 'DAP1_DIN', None, None),
    ({192:  73, 140:  63}, {140:  'PJ.01'}, "2200000.gpio", 40, 21, 'I2S0_DOUT', 'DAP1_DOUT', None, None)
]
compats_tx2_nx = (
    'nvidia,p3509-0000+p3636-0001',
)

JETSON_TX2_PIN_DEFS = [
    ({192:  76, 140:  66}, {140:  'PJ.04'}, "2200000.gpio", 7, 4, 'PAUDIO_MCLK', 'AUD_MCLK', None, None),
    # Output-only (due to base board)
    ({192: 146, 140: 117}, {140:  'PT.02'}, "2200000.gpio", 11, 17, 'PUART0_RTS', 'UART1_RTS', None, None),
    ({192:  72, 140:  62}, {140:  'PJ.00'}, "2200000.gpio", 12, 18, 'PI2S0_CLK', 'DAP1_SCLK', None, None),
    ({192:  77, 140:  67}, {140:  'PJ.05'}, "2200000.gpio", 13, 27, 'PGPIO20_AUD_INT', 'GPIO_AUD0', None, None),
    (                  15,              {}, "3160000.i2c/i2c-0/0-0074", 15, 22, 'GPIO_EXP_P17', 'GPIO_EXP_P17', None, None),
    # Input-only (due to module):
    ({ 64:  40,  47:  31}, { 47: 'PAA.00'}, "c2f0000.gpio", 16, 23, 'AO_DMIC_IN_DAT', 'CAN_GPIO0', None, None),
    ({192: 161, 140: 128}, {140:  'PY.01'}, "2200000.gpio", 18, 24, 'GPIO16_MDM_WAKE_AP', 'GPIO_MDM2', None, None),
    ({192: 109, 140:  90}, {140:  'PN.05'}, "2200000.gpio", 19, 10, 'SPI1_MOSI', 'GPIO_CAM6', None, None),
    ({192: 108, 140:  89}, {140:  'PN.04'}, "2200000.gpio", 21, 9, 'SPI1_MISO', 'GPIO_CAM5', None, None),
    (                  14,              {}, "3160000.i2c/i2c-0/0-0074", 22, 25, 'GPIO_EXP_P16', 'GPIO_EXP_P16', None, None),
    ({192: 107, 140:  88}, {140:  'PN.03'}, "2200000.gpio", 23, 11, 'SPI1_CLK', 'GPIO_CAM4', None, None),
    ({192: 110, 140:  91}, {140:  'PN.06'}, "2200000.gpio", 24, 8, 'SPI1_CS0', 'GPIO_CAM7', None, None),
    # Board pin 26 is not available on this board
    ({192:  78, 140:  68}, {140:  'PJ.06'}, "2200000.gpio", 29, 5, 'GPIO19_AUD_RST', 'GPIO_AUD1', None, None),
    ({ 64:  42,  47:  33}, { 47: 'PAA.02'}, "c2f0000.gpio", 31, 6, 'GPIO9_MOTION_INT', 'CAN_GPIO2', None, None),
    # Output-only (due to module):
    ({ 64:  41,  47:  32}, { 47: 'PAA.01'}, "c2f0000.gpio", 32, 12, 'AO_DMIC_IN_CLK', 'CAN_GPIO1', None, None),
    ({192:  69, 140:  59}, {140:  'PI.05'}, "2200000.gpio", 33, 13, 'GPIO11_AP_WAKE_BT', 'GPIO_PQ5', None, None),
    ({192:  75, 140:  65}, {140:  'PJ.03'}, "2200000.gpio", 35, 19, 'I2S0_LRCLK', 'DAP1_FS', None, None),
    # Input-only (due to base board) IF NVIDIA debug card NOT plugged in
    # Output-only (due to base board) IF NVIDIA debug card plugged in
    ({192: 147, 140: 118}, {140:  'PT.03'}, "2200000.gpio", 36, 16, 'UART0_CTS', 'UART1_CTS', None, None),
    ({192:  68, 140:  58}, {140:  'PI.04'}, "2200000.gpio", 37, 26, 'GPIO8_ALS_PROX_INT', 'GPIO_PQ4', None, None),
    ({192:  74, 140:  64}, {140:  'PJ.02'}, "2200000.gpio", 38, 20, 'I2S0_SDIN', 'DAP1_DIN', None, None),
    ({192:  73, 140:  63}, {140:  'PJ.01'}, "2200000.gpio", 40, 21, 'I2S0_SDOUT', 'DAP1_DOUT', None, None)
]
compats_tx2 = (
    'nvidia,p2771-0000',
    'nvidia,p2771-0888',
    'nvidia,p3489-0000',
    'nvidia,lightning',
    'nvidia,quill',
    'nvidia,storm',
)

JETSON_TX1_PIN_DEFS = [
    (216, {}, "6000d000.gpio", 7, 4, 'AUDIO_MCLK', 'AUD_MCLK', None, None),
    # Output-only (due to base board)
    (162, {}, "6000d000.gpio", 11, 17, 'UART0_RTS', 'UART1_RTS', None, None),
    (11, {}, "6000d000.gpio", 12, 18, 'I2S0_CLK', 'DAP1_SCLK', None, None),
    (38, {}, "6000d000.gpio", 13, 27, 'GPIO20_AUD_INT', 'GPIO_PE6', None, None),
    (15, {}, "7000c400.i2c/i2c-1/1-0074", 15, 22, 'GPIO_EXP_P17', 'GPIO_EXP_P17', None, None),
    (37, {}, "6000d000.gpio", 16, 23, 'AO_DMIC_IN_DAT', 'DMIC3_DAT', None, None),
    (184, {}, "6000d000.gpio", 18, 24, 'GPIO16_MDM_WAKE_AP', 'MODEM_WAKE_AP', None, None),
    (16, {}, "6000d000.gpio", 19, 10, 'SPI1_MOSI', 'SPI1_MOSI', None, None),
    (17, {}, "6000d000.gpio", 21, 9, 'SPI1_MISO', 'SPI1_MISO', None, None),
    (14, {}, "7000c400.i2c/i2c-1/1-0074", 22, 25, 'GPIO_EXP_P16', 'GPIO_EXP_P16', None, None),
    (18, {}, "6000d000.gpio", 23, 11, 'SPI1_CLK', 'SPI1_SCK', None, None),
    (19, {}, "6000d000.gpio", 24, 8, 'SPI1_CS0', 'SPI1_CS0', None, None),
    (20, {}, "6000d000.gpio", 26, 7, 'SPI1_CS1', 'SPI1_CS1', None, None),
    (219, {}, "6000d000.gpio", 29, 5, 'GPIO19_AUD_RST', 'GPIO_X1_AUD', None, None),
    (186, {}, "6000d000.gpio", 31, 6, 'GPIO9_MOTION_INT', 'MOTION_INT', None, None),
    (36, {}, "6000d000.gpio", 32, 12, 'AO_DMIC_IN_CLK', 'DMIC3_CLK', None, None),
    (63, {}, "6000d000.gpio", 33, 13, 'GPIO11_AP_WAKE_BT', 'AP_WAKE_NFC', None, None),
    (8, {}, "6000d000.gpio", 35, 19, 'I2S0_LRCLK', 'DAP1_FS', None, None),
    # Input-only (due to base board) IF NVIDIA debug card NOT plugged in
    # Input-only (due to base board) (always reads fixed value) IF NVIDIA debug card plugged in
    (163, {}, "6000d000.gpio", 36, 16, 'UART0_CTS', 'UART1_CTS', None, None),
    (187, {}, "6000d000.gpio", 37, 26, 'GPIO8_ALS_PROX_INT', 'ALS_PROX_INT', None, None),
    (9, {}, "6000d000.gpio", 38, 20, 'I2S0_SDIN', 'DAP1_DIN', None, None),
    (10, {}, "6000d000.gpio", 40, 21, 'I2S0_SDOUT', 'DAP1_DOUT', None, None)
]
compats_tx1 = (
    'nvidia,p2371-2180',
    'nvidia,jetson-cv',
)

JETSON_NANO_PIN_DEFS = [
    (216, {}, "6000d000.gpio", 7, 4, 'GPIO9', 'AUD_MCLK', None, None),
    (50, {}, "6000d000.gpio", 11, 17, 'UART1_RTS', 'UART2_RTS', None, None),
    (79, {}, "6000d000.gpio", 12, 18, 'I2S0_SCLK', 'DAP4_SCLK', None, None),
    (14, {}, "6000d000.gpio", 13, 27, 'SPI1_SCK', 'SPI2_SCK', None, None),
    (194, {}, "6000d000.gpio", 15, 22, 'GPIO12', 'LCD_TE', None, None),
    (232, {}, "6000d000.gpio", 16, 23, 'SPI1_CS1', 'SPI2_CS1', None, None),
    (15, {}, "6000d000.gpio", 18, 24, 'SPI1_CS0', 'SPI2_CS0', None, None),
    (16, {}, "6000d000.gpio", 19, 10, 'SPI0_MOSI', 'SPI1_MOSI', None, None),
    (17, {}, "6000d000.gpio", 21, 9, 'SPI0_MISO', 'SPI1_MISO', None, None),
    (13, {}, "6000d000.gpio", 22, 25, 'SPI1_MISO', 'SPI2_MISO', None, None),
    (18, {}, "6000d000.gpio", 23, 11, 'SPI0_SCK', 'SPI1_SCK', None, None),
    (19, {}, "6000d000.gpio", 24, 8, 'SPI0_CS0', 'SPI1_CS0', None, None),
    (20, {}, "6000d000.gpio", 26, 7, 'SPI0_CS1', 'SPI1_CS1', None, None),
    (149, {}, "6000d000.gpio", 29, 5, 'GPIO01', 'CAM_AF_EN', None, None),
    (200, {}, "6000d000.gpio", 31, 6, 'GPIO11', 'GPIO_PZ0', None, None),
    # Older versions of L4T have a DT bug which instantiates a bogus device
    # which prevents this library from using this PWM channel.
    (168, {}, "6000d000.gpio", 32, 12, 'GPIO07', 'LCD_BL_PW', '7000a000.pwm', 0),
    (38, {}, "6000d000.gpio", 33, 13, 'GPIO13', 'GPIO_PE6', '7000a000.pwm', 2),
    (76, {}, "6000d000.gpio", 35, 19, 'I2S0_FS', 'DAP4_FS', None, None),
    (51, {}, "6000d000.gpio", 36, 16, 'UART1_CTS', 'UART2_CTS', None, None),
    (12, {}, "6000d000.gpio", 37, 26, 'SPI1_MOSI', 'SPI2_MOSI', None, None),
    (77, {}, "6000d000.gpio", 38, 20, 'I2S0_DIN', 'DAP4_DIN', None, None),
    (78, {}, "6000d000.gpio", 40, 21, 'I2S0_DOUT', 'DAP4_DOUT', None, None)
]
compats_nano = (
    'nvidia,p3450-0000',
    'nvidia,p3450-0002',
    'nvidia,jetson-nano',
)

jetson_gpio_data = {
    CLARA_AGX_XAVIER: (
        CLARA_AGX_XAVIER_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '16384M',
            'REVISION': 'Unknown',
            'TYPE': 'CLARA_AGX_XAVIER',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM Carmel'
        }
    ),
    JETSON_NX: (
        JETSON_NX_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '16384M',
            'REVISION': 'Unknown',
            'TYPE': 'Jetson NX',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM Carmel'
        }
    ),
    JETSON_XAVIER: (
        JETSON_XAVIER_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '16384M',
            'REVISION': 'Unknown',
            'TYPE': 'Jetson Xavier',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM Carmel'
        }
    ),
    JETSON_TX2_NX: (
        JETSON_TX2_NX_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '4096M',
            'REVISION': 'Unknown',
            'TYPE': 'Jetson TX2 NX',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM A57 + Denver'
        }
    ),
    JETSON_TX2: (
        JETSON_TX2_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '8192M',
            'REVISION': 'Unknown',
            'TYPE': 'Jetson TX2',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM A57 + Denver'
        }
    ),
    JETSON_TX1: (
        JETSON_TX1_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '4096M',
            'REVISION': 'Unknown',
            'TYPE': 'Jetson TX1',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM A57'
        }
    ),
    JETSON_NANO: (
        JETSON_NANO_PIN_DEFS,
        {
            'P1_REVISION': 1,
            'RAM': '4096M',
            'REVISION': 'Unknown',
            'TYPE': 'Jetson Nano',
            'MANUFACTURER': 'NVIDIA',
            'PROCESSOR': 'ARM A57'
        }
    ),
}



def get_data():

    model = JETSON_NX
    pin_defs, jetson_info = jetson_gpio_data[model]

    return model, jetson_info

# Pin Numbering Modes
BOARD = 10
BCM = 11
TEGRA_SOC = 1000
CVM = 1001

# The constants and their offsets are implemented to prevent HIGH from being
# used in place of other variables (ie. HIGH and RISING should not be
# interchangeable)

# Pull up/down options
_PUD_OFFSET = 20
PUD_OFF = 0 + _PUD_OFFSET
PUD_DOWN = 1 + _PUD_OFFSET
PUD_UP = 2 + _PUD_OFFSET

HIGH = 1
LOW = 0

# Edge possibilities
# These values (with _EDGE_OFFSET subtracted) must match gpio_event.py:*_EDGE
_EDGE_OFFSET = 30
RISING = 1 + _EDGE_OFFSET
FALLING = 2 + _EDGE_OFFSET
BOTH = 3 + _EDGE_OFFSET

# GPIO directions. UNKNOWN constant is for gpios that are not yet setup
UNKNOWN = -1
OUT = 0
IN = 1
HARD_PWM = 43


model, JETSON_INFO = get_data()
RPI_INFO = JETSON_INFO

# Dictionary objects used as lookup tables for pin to linux gpio mapping

_gpio_warnings = True
_gpio_mode = None


# Function used to enable/disable warnings during setup and cleanup.
# Param -> state is a bool
def setwarnings(state):
    global _gpio_warnings
    _gpio_warnings = bool(state)


# Function used to set the pin mumbering mode. Possible mode values are BOARD,
# BCM, TEGRA_SOC and CVM
def setmode(mode):
    global _gpio_mode

    # check if a different mode has been set
    if _gpio_mode and mode != _gpio_mode:
        raise ValueError("A different mode has already been set!")

    mode_map = {
        BOARD: 'BOARD',
        BCM: 'BCM',
        CVM: 'CVM',
        TEGRA_SOC: 'TEGRA_SOC',
    }

    # check if mode parameter is valid
    if mode not in mode_map:
        raise ValueError("An invalid mode was passed to setmode()!")

    _gpio_mode = mode


# Function used to get the currently set pin numbering mode
def getmode():
    return _gpio_mode


# Mutable class to represent a default function argument.
# See https://stackoverflow.com/a/57628817/2767322
class _Default:
    def __init__(self, val):
        self.val = val


# Function used to setup individual pins or lists/tuples of pins as
# Input or Output. Param channels must an integer or list/tuple of integers,
# direction must be IN or OUT, pull_up_down must be PUD_OFF, PUD_UP or
# PUD_DOWN and is only valid when direction in IN, initial must be HIGH or LOW
# and is only valid when direction is OUT
def setup(channels, direction, pull_up_down=_Default(PUD_OFF), initial=None):
    print("set up channels: "+str(channels) + "as "+str(direction) )


# Function used to cleanup channels at the end of the program.
# The param channel can be an integer or list/tuple of integers specifying the
# channels to be cleaned up. If no channel is provided, all channels are
# cleaned
def cleanup(channel=None):
    print("clean up channels")


# Function used to return the current value of the specified channel.
# Function returns either HIGH or LOW
def input(channel):
    print("read from channel: "+str(channel))
    return 1


# Function used to set a value to a channel or list/tuple of channels.
# Parameter channels must be an integer or list/tuple of integers.
# Values must be either HIGH or LOW or list/tuple
# of HIGH and LOW with the same length as the channels list/tuple
def output(channels, values):
    print("write values: "+str(values)+" to channels: "+str(channels))


# Function used to check if an event occurred on the specified channel.
# Param channel must be an integer.
# This function return True or False
def event_detected(channel):
    print("event detected on channel: "+str(channel))
    return True

# Function used to add a callback function to channel, after it has been
# registered for events using add_event_detect()
def add_event_callback(channel, callback):
    print("add callback on channel: "+str(channel))


# Function used to add threaded event detection for a specified gpio channel.
# Param gpio must be an integer specifying the channel, edge must be RISING,
# FALLING or BOTH. A callback function to be called when the event is detected
# and an integer bounctime in milliseconds can be optionally provided
def add_event_detect(channel, edge, callback=None, bouncetime=None):
    print("add event detect on channel: "+str(channel))


# Function used to remove event detection for channel
def remove_event_detect(channel):
    print("remove event detect on channel: "+str(channel))


# Function used to perform a blocking wait until the specified edge
# is detected for the param channel. Channel must be an integer and edge must
# be either RISING, FALLING or BOTH.
# bouncetime in milliseconds and timeout in millseconds can optionally be
# provided
def wait_for_edge(channel, edge, bouncetime=None, timeout=None):
    print("wait for edge on channel: "+str(channel))
    return channel


# Function used to check the currently set function of the channel specified.
# Param channel must be an integers. The function returns either IN, OUT,
# or UNKNOWN
def gpio_function(channel):
    print("get func on channel: "+str(channel))
    return UNKNOWN


class PWM(object):
    def __init__(self, channel, frequency_hz):
        print("create PWM on channel: "+str(channel)+" at frequency: "+str(frequency_hz))

    def __del__(self):
        pass

    def start(self, duty_cycle_percent):
        print("Start PWM")

    def ChangeFrequency(self, frequency_hz):
        print("change PWM frequency: "+str(frequency_hz))

    def ChangeDutyCycle(self, duty_cycle_percent):
        print("change PWM duty cycle: "+str(duty_cycle_percent))

    def stop(self):
        print("Stop PWM")

