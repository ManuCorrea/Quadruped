module rotaryEncoderCounter(input CLK, A, B,
                            output [4:0] counterWire);

  reg [4:0] counter;
  // Internal variables
  reg [1:0] A_State;
  reg [1:0] A_Last_State;
  assign counterWire = counter;

  always @(posedge CLK) begin
      A_State <= A;
      if (A_Last_State != A_State) begin
        if (A_State != B) begin
          counter <= counter - 1; //CCW
        end
        else begin
          counter <= counter + 1; // CW
        end
      end
      A_Last_State <= A_State;

  end

endmodule
