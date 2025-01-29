`include "mips_core.svh"

module g_share#(
    parameter INDEX_WIDTH = 4,
    parameter ADDR_WIDTH = 26
)(
    input  clk,         // Clock
    input  rst_n,       // Synchronous reset active low
    input we_bp,        //when decode resoves that instr was a branch
    input valid_branch, //from decode; if the pranch prediction was correct
    input fb_pred,      //from decode : prediction that was made
    input logic[ADDR_WIDTH-1:0] write_pc,//from decode, current_pc
    output logic pred,

    pc_ifc.in i_pc_next
    
    //counter signals
);
logic global_history [INDEX_WIDTH-1:0];

ghr #(
    .GHR_DEPTH(INDEX_WIDTH)
)GHR (
    .clk,
    .rst_n,
    .we_ghr(we_bp),
    .branch_taken(fb_pred),
    .global_history_out(global_history)
);

localparam COUNTER_WIDTH =2;

logic [INDEX_WIDTH-1:0] index; 
logic [INDEX_WIDTH-1:0] index_write; 
logic [COUNTER_WIDTH-1:0] current_counter;
logic counter_dir;

assign current_counter = counter_regs[index];
assign index_write = write_pc[INDEX_WIDTH-1:0];

always_comb begin
    //xor to fold down history and pc to index length
    for (int i=0; i<INDEX_WIDTH;i++)begin
        index = i_pc_next.pc[i] ^ global_history[i];
    end
    //prediction is reader form msb of counter
    pred = counter_regs[index][1];
    //direction of increment for the counter(based on the branch prediction and the valifity of prediction)
    //from decode
    counter_dir = ~(valid_branch ^ fb_pred);
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
        unique case({current_counter,counter_dir})
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

module ghr#(
    parameter GHR_DEPTH = 4
)(
    input  clk,                   // Clock
    input  rst_n,                 // Synchronous reset active low
    input logic we_ghr,           //we from ghr will be the same as the we_bp
    input logic branch_taken,     //if the branch was taken(from decode)
    output logic global_history_out[GHR_DEPTH-1:0]
);
logic global_history_reg [GHR_DEPTH-1:0];

always_ff@(posedge clk)begin
    if(~rst_n) begin
        for (int i=0; i<GHR_DEPTH;i++)begin
            global_history_reg[i] <= 'b0;
        end
    end
    else begin
        if(we_ghr)begin
            for (int i=1; i<GHR_DEPTH;i++)begin
                global_history_reg[i] <= global_history_reg[i-1];
            end
            global_history_reg[0] <= branch_taken;
        end
        else begin
            global_history_reg <= global_history_reg;
        end
    end
end

assign global_history_out = global_history_reg;

endmodule
