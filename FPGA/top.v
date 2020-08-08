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
    input PIN_26, // pin at the bottom
    input PIN_27, // Tx pin at the bottom
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

    wire baud_tick;
    baud_gen GENERATOR(CLK, baud_tick); // 115200 serial

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

    reg[4:0] dataSerial;
    reg transmit;
    reg [4:0]position=4'd0;
    reg bit_trans;
    assign PIN_27 = bit_trans;

    reg [4:0]motorNum = 4'd0;

    // TODO: implement motor data select
    always @(posedge baud_tick) begin
      case(motorNum)
        0:dataSerial = counter1;
        1:dataSerial = counter2;
        2:dataSerial = counter3;
        3:dataSerial = counter4;
        4:dataSerial = counter5;
        5:dataSerial = counter6;
        6:dataSerial = counter7;
        7:dataSerial = counter8;
        8:dataSerial = counter9;
        9:dataSerial = counter10;
        10:dataSerial = counter11;
        11:begin dataSerial = counter12; dataSerial=0; end
      endcase

      if (PIN_26 == 1'b1 || transmit == 1) begin // If pin 26 HIGH then we transmit
            transmit <= 1;                       // by pin 27 Tx
            case(position)
              0:bit_trans <= 0; // startbit
              1:bit_trans <= dataSerial[position]; //b1
              2:bit_trans <= dataSerial[position];
              3:bit_trans <= dataSerial[position];
              4:bit_trans <= dataSerial[position];
              5:bit_trans <= dataSerial[position];
              6:bit_trans <= dataSerial[position];
              7:bit_trans <= dataSerial[position];
              8:bit_trans <= dataSerial[position]; //b7
              9:bit_trans <= 1; //stop
              10: transmit <= 0;
              default : bit_trans <= 1;

            endcase
            position <= position + 4'd1;
        end
      end

endmodule
