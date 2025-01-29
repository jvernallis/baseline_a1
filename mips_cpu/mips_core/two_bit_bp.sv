`include "mips_core.svh"

module two_bit_bp#(
    parameter INDEX_WIDTH = 4,
    parameter ADDR_WIDTH = 26
)(
    input  clk,         // Clock
    input  rst_n,       // Synchronous reset active low
    input we_bp,        //when dwcode resoves that instr was a branch
    input valid_branch, //from decode; if the pranch presdiction was correct
    input logic[ADDR_WIDTH-1:0] write_pc,//from decode, current_pc
    output logic pred,

    pc_ifc.in i_pc_next
    
    //counter signals
);
localparam COUNTER_WIDTH =2;

logic [INDEX_WIDTH-1:0] index; 
logic [INDEX_WIDTH-1:0] index_write; 
logic [COUNTER_WIDTH-1:0] current_counter;

assign index = i_pc_next.pc[INDEX_WIDTH-1:0];
assign current_counter = counter_regs[index];
assign index_write = write_pc[INDEX_WIDTH-1:0];

always_comb begin
    pred = counter_regs[index][1];
end

logic [COUNTER_WIDTH-1:0] counter_regs[INDEX_WIDTH-1:0];

always_ff@(posedge clk)
begin
    if(~rst_n) begin
        for (int i=0; i<INDEX_WIDTH;i++)begin
            counter_regs[i] <= 'b10;
        end
    end
	 else begin
		if(we_bp) begin
        unique case({current_counter,valid_branch})
            'b000: begin
                counter_regs[index_write] <= 'b00;
            end
            'b001:begin
                counter_regs[index_write] <= 'b01;
            end
            'b010: begin
                counter_regs[index_write] <= 'b00;
            end
            'b011:begin
                counter_regs[index_write] <= 'b10;
            end
            'b100: begin
                counter_regs[index_write] <= 'b01;
            end
            'b101:begin
                counter_regs[index_write] <= 'b11;
            end
            'b110: begin
                counter_regs[index_write] <= 'b10;
            end
            'b111:begin
                counter_regs[index_write] <= 'b11;
            end
        endcase 
    end
	end
end

endmodule