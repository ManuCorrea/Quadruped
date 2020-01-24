 // look in pins.pcf for all the pin names on the TinyFPGA BX board
module top (
    input CLK,    // 16MHz clock
    input PIN_1, //Motor 1
    input PIN_2,
    input PIN_3, //Motor 2
    input PIN_4,
    input PIN_5, //Motor 3
    input PIN_6,
    input PIN_7, //Motor 4
    input PIN_8,
    input PIN_9, //Motor 5
    input PIN_10,
    input PIN_11, //Motor 6
    input PIN_12,
    input PIN_13, //Motor 7
    input PIN_14,
    input PIN_15, //Motor 8
    input PIN_16,
    input PIN_17, //Motor 9
    input PIN_18,
    input PIN_19, //Motor 10
    input PIN_20,
    input PIN_21, //Motor 11
    input PIN_22,
    input PIN_23, //Motor 12
    input PIN_24,
    output USBPU  // USB pull-up resistor
);
    // drive USB pull-up resistor to '0' to disable USB
    assign USBPU = 0;

    // wires for each counter output
    wire [4:0] counter1;
    wire [4:0] counter2;
    wire [4:0] counter3;
    wire [4:0] counter4;
    wire [4:0] counter5;
    wire [4:0] counter6;
    wire [4:0] counter7;
    wire [4:0] counter8;
    wire [4:0] counter9;
    wire [4:0] counter10;
    wire [4:0] counter11;
    wire [4:0] counter12;

    // initalization of each module
    rotaryEncoderCounter C1(CLK, PIN_1, PIN_2, counter1);
    rotaryEncoderCounter C2(CLK, PIN_3, PIN_4, counter2);
    rotaryEncoderCounter C3(CLK, PIN_5, PIN_6, counter3);
    rotaryEncoderCounter C4(CLK, PIN_7, PIN_8, counter4);
    rotaryEncoderCounter C5(CLK, PIN_9, PIN_10, counter5);
    rotaryEncoderCounter C6(CLK, PIN_11, PIN_12, counter6);
    rotaryEncoderCounter C7(CLK, PIN_13, PIN_14, counter7);
    rotaryEncoderCounter C8(CLK, PIN_15, PIN_16, counter8);
    rotaryEncoderCounter C9(CLK, PIN_17, PIN_18, counter9);
    rotaryEncoderCounter C10(CLK, PIN_19, PIN_20, counter10);
    rotaryEncoderCounter C11(CLK, PIN_21, PIN_22, counter11);
    rotaryEncoderCounter C12(CLK, PIN_23, PIN_24, counter12);
endmodule
