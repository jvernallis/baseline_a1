`include "mips_core.svh"

module branch_target_buffer#(
    parameter INDEX_WIDTH = 4,
    parameter ASSOCIATIVITY = 2
)(
    input  clk,    // Clock
    input  rst_n,  // Synchronous reset active low
    input we_bp,
    
);

endmodule