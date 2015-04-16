# ME135 Project: Foosball
Man in the middle to convert UART commands to I2C and PWM outputs.

Current command format (Hex):

    [header] [motor0] [motor1] [motor2] [motor3] [servo0]...[servo3]
    FFFFFFFF 00006400 00006400 00009CFF 00006400 0000 3C00 7800 B400

Motor values and servo positions are little endian.

UART is 115200 baud.

Grizzly addresses are 0x09, 0x0A, 0x0B, 0x0E respectively.

