module baud_gen(input CLK, output wire baud_clk); // 115200 bauds
  reg[13:0] acc;
  always @ (posedge CLK) begin
    acc <= acc[12:0] + 59;
  end

  assign baud_clk = acc[13];
endmodule
