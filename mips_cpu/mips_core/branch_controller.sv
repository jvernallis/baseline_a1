/*
 * branch_controller.sv
 * Author: Zinsser Zhang
 * Last Revision: 04/08/2018
 *
 * branch_controller predicts from PC provided by fetch stage.
 * As branch predictions cannot cause hazards, there is no link to the hazard controller.
 * 
 * See wiki page "Branch and Jump" for details.
 */
`include "mips_core.svh"

module branch_controller (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	pc_ifc.in if_pc, //IF's next_pc
	branch_prediction_ifc.out if_branch_prediction,

	// Feedback
	pc_ifc.in dec_pc,
	branch_resolution_ifc.in dec_branch_resolved
);
	logic branch_outcome; // PC of branch destination
	
	branch_target_buffer BTB (
		.clk, .rst_n,

		.we_btb			(dec_branch_resolved.valid), //From d2e register
		.w_target		(dec_branch_resolved.target), //From d2e register
		.hit_out		(if_branch_prediction.valid),
		.r_target 		(if_branch_prediction.target),
		.i_req_pc		(if_pc.pc),
		.i_pc_next 		(branch_outcome) //Will be computed
	);

	// Predictor interface changes: prediction will be binary logic; target not provided.
	// Change the following line to switch predictor
	// branch_predictor_2bit PREDICTOR (
	branch_predictor_always_not_taken PREDICTOR (
		.clk, .rst_n,

		// .i_req_valid     (request_prediction),
		.i_req_pc        (if_pc.pc),
		// .i_req_target    (dec_branch_decoded.target),
		.o_req_prediction(if_branch_prediction.prediction),

		.i_fb_valid      (dec_branch_resolved.valid),
		.i_fb_pc         (dec_pc.pc),
		.i_fb_prediction (dec_branch_resolved.prediction),
		.i_fb_outcome    (dec_branch_resolved.outcome)
	);

	always_comb
	begin
		
		branch_outcome = 
			(if_branch_prediction.valid & (if_branch_prediction.prediction == TAKEN)) 
			? if_branch_prediction.target 
			: dec_pc.pc + `ADDR_WIDTH'd8;
	end

	// always @(posedge clk) begin
	// 	$display("Branch testing - now going to predict the signal with PC %h", if_pc.pc);
	// end

endmodule

module branch_predictor_always_not_taken (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	// input logic i_req_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	// input logic [`ADDR_WIDTH - 1 : 0] i_req_target,
	output mips_core_pkg::BranchOutcome o_req_prediction,

	// Feedback
	input logic i_fb_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
	input mips_core_pkg::BranchOutcome i_fb_prediction,
	input mips_core_pkg::BranchOutcome i_fb_outcome
);

	always_comb
	begin
		o_req_prediction = NOT_TAKEN;
	end

endmodule

module branch_predictor_2bit (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	// input logic i_req_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	// input logic [`ADDR_WIDTH - 1 : 0] i_req_target,
	output mips_core_pkg::BranchOutcome o_req_prediction,

	// Feedback
	input logic i_fb_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
	input mips_core_pkg::BranchOutcome i_fb_prediction,
	input mips_core_pkg::BranchOutcome i_fb_outcome
);

	logic [1:0] counter;

	task incr;
		begin
			if (counter != 2'b11)
				counter <= counter + 2'b01;
		end
	endtask

	task decr;
		begin
			if (counter != 2'b00)
				counter <= counter - 2'b01;
		end
	endtask

	always_ff @(posedge clk)
	begin
		if(~rst_n)
		begin
			counter <= 2'b01;	// Weakly not taken
		end
		else
		begin
			if (i_fb_valid)
			begin
				case (i_fb_outcome)
					NOT_TAKEN: decr();
					TAKEN:     incr();
				endcase
			end
		end
	end

	always_comb
	begin
		o_req_prediction = counter[1] ? TAKEN : NOT_TAKEN;
	end

endmodule

module branch_target_buffer #(
    parameter INDEX_WIDTH = 4,
    parameter ASSOCIATIVITY = 2,
	parameter ADDR_WIDTH = 26)(
   // General signals
    input  clk,    // Clock
    input  rst_n,  // Synchronous reset active low
    input  we_btb,
	
    input  logic [`ADDR_WIDTH - 1 : 0] w_target,
    output logic hit_out,
    output logic [`ADDR_WIDTH - 1 : 0] r_target,

    // Request
    input  logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	input  logic [`ADDR_WIDTH - 1 : 0] i_pc_next
);
    localparam TAG_WIDTH = `ADDR_WIDTH - INDEX_WIDTH;
    localparam DEPTH = 1 << INDEX_WIDTH;
    
    logic [TAG_WIDTH - 1 : 0] i_tag;
	logic [INDEX_WIDTH - 1 : 0] i_index;
	logic [INDEX_WIDTH - 1 : 0] i_index_next;

    //signals for least recently used associative logic
    logic r_select_way;
    logic w1_select_way;
	logic w2_select_way;
	logic [DEPTH - 1 : 0] lru_rp;

    assign {i_tag, i_index} = i_req_pc[`ADDR_WIDTH - 1 : 2];
	assign i_index_next = i_pc_next[INDEX_WIDTH-1:0];
    // targetbank signals
	logic targetbank_we[ASSOCIATIVITY];
	logic [`ADDR_WIDTH - 1 : 0] targetbank_wdata;
	logic [INDEX_WIDTH - 1 : 0] targetbank_waddr;
	logic [INDEX_WIDTH - 1 : 0] targetbank_raddr;
    logic [`ADDR_WIDTH - 1 : 0] targetbank_rdata[ASSOCIATIVITY];

    //generate target banks
    genvar g;
	generate
	
		for (g=0; g< ASSOCIATIVITY; g++)
		begin : targetbanks
			cache_bank #(
				.DATA_WIDTH (`ADDR_WIDTH),
				.ADDR_WIDTH (INDEX_WIDTH)
			) targetbank (
				.clk,
				.i_we (targetbank_we[g]),
				.i_wdata(targetbank_wdata),
				.i_waddr(targetbank_waddr),
				.i_raddr(targetbank_raddr),

				.o_rdata(targetbank_rdata[g])
			);
		end

	endgenerate

   // tagbank signals 
    logic tagbank_we[ASSOCIATIVITY];
	logic [TAG_WIDTH - 1 : 0] tagbank_wdata;
	logic [INDEX_WIDTH - 1 : 0] tagbank_waddr;
	logic [INDEX_WIDTH - 1 : 0] tagbank_raddr;
	logic [TAG_WIDTH - 1 : 0] tagbank_rdata[ASSOCIATIVITY];
	
	//generate tag banks
	genvar w;
	generate
		for (w=0; w< ASSOCIATIVITY; w++)
		begin: tagbanks
			cache_bank #(
				.DATA_WIDTH (TAG_WIDTH),
				.ADDR_WIDTH (INDEX_WIDTH)
			) tagbank (
				.clk,
				.i_we    (tagbank_we[w]),
				.i_wdata (tagbank_wdata),
				.i_waddr (tagbank_waddr),
				.i_raddr (tagbank_raddr),

				.o_rdata (tagbank_rdata[w])
			);
		end

	endgenerate

	// Valid bits
	logic [DEPTH - 1 : 0] valid_bits[ASSOCIATIVITY];
   // btb hit
	logic hit;

    always_comb
    begin
		//change hit condition when more ways are added
		hit = ( ((i_tag == tagbank_rdata[0]) & valid_bits[0][i_index])
				  |	((i_tag == tagbank_rdata[1]) & valid_bits[1][i_index]));
       //change if statment when more ways are added
      if (hit)
		begin
			if (i_tag == tagbank_rdata[0])
			begin
				r_select_way = 'b0;
			end
			else
			begin
				r_select_way = 'b1;
			end
		end
		else
		begin
			r_select_way = lru_rp[i_index];
		end
    end

    //tagbank connections
    logic [INDEX_WIDTH - 1 : 0]w1_index;
    logic [TAG_WIDTH - 1 : 0]w1_tag_data;
	logic [INDEX_WIDTH - 1 : 0]w2_index;
    logic [TAG_WIDTH - 1 : 0]w2_tag_data;
    always_comb
	begin
        tagbank_wdata = w2_tag_data;
        tagbank_waddr = w2_index;
		tagbank_we[0] = ~w2_select_way ? we_btb :1'b0;
        tagbank_we[1] = w2_select_way ? we_btb :1'b0;
		  
		tagbank_raddr = i_index_next;
	end

    //targetbank connections
    logic [ADDR_WIDTH - 1 : 0]w_target_data;
    always_comb
	begin
        targetbank_wdata = w_target_data;
        targetbank_waddr = w2_index;
		targetbank_we[0] = ~w2_select_way ? we_btb :1'b0;
        targetbank_we[1] = w2_select_way ? we_btb :1'b0;
		  
		targetbank_raddr = i_index_next;
	end
    //read outputs
    always_comb
	begin
		hit_out = hit;
		r_target = targetbank_rdata[r_select_way];
	end
	
	
    always_ff @(posedge clk)
	begin
		if(~rst_n)
		begin
			for (int i=0; i<DEPTH;i++)
				lru_rp[i] <= 0;
			for (int i=0; i<ASSOCIATIVITY;i++)
				valid_bits[i] <= '0;
		end
		else
		begin
            lru_rp[i_index] <= ~r_select_way;
            if(~hit)
            begin
				//data for write locked in for miss(will write 2 cycles after miss and branch confirm)
                w1_tag_data <= i_tag;
                 
                w1_index <= i_index;
                w1_select_way <= r_select_way;
            end
				//set valid bit on a write(will be valid cycle after write)
				valid_bits[w2_index][w2_select_way] <=we_btb;
				//data for write (will write cycle after miss and branch confirm)
				w2_tag_data <= w1_tag_data;  
				w2_index <= w1_index;
				w2_select_way <= w1_select_way;
				//target from decode sent to write
				w_target_data <= w_target;
      end
	end
endmodule

module branch_resolver (
	decoder_output_ifc.in i_decoded,
	reg_file_output_ifc.in i_reg_data,

	// Feedback
	branch_resolution_ifc.out o_branch_resolution
);
	always_comb
	begin
	if (i_decoded.valid)
		begin
			// FIXME: What if these registers get edited in the previous instruction? Do we need forwarding?
			logic signed [`DATA_WIDTH - 1 : 0] op1 =	i_reg_data.rs_data;
			logic signed [`DATA_WIDTH - 1 : 0] op2 =  	i_decoded.uses_immediate
				? i_decoded.immediate
				: i_reg_data.rt_data;

			o_branch_resolution.target = i_decoded.branch_target;

			if(i_decoded.is_jump | i_decoded.is_branch_jump) begin
				o_branch_resolution.valid = 1'b1;
				o_branch_resolution.outcome = TAKEN; //Default to taken for jumps - branches get corrected later
			end else begin
				o_branch_resolution.valid = 1'b0;
				o_branch_resolution.outcome = NOT_TAKEN;
			end

			case(i_decoded.alu_ctl)
				ALUCTL_BA:   o_branch_resolution.outcome = TAKEN;
				ALUCTL_BEQ:  o_branch_resolution.outcome = op1 == op2     ? TAKEN : NOT_TAKEN;
				ALUCTL_BNE:  o_branch_resolution.outcome = op1 != op2     ? TAKEN : NOT_TAKEN;
				ALUCTL_BLEZ: o_branch_resolution.outcome = op1 <= signed'(0) ? TAKEN : NOT_TAKEN;
				ALUCTL_BGTZ: o_branch_resolution.outcome = op1 > signed'(0)  ? TAKEN : NOT_TAKEN;
				ALUCTL_BGEZ: o_branch_resolution.outcome = op1 >= signed'(0) ? TAKEN : NOT_TAKEN;
				ALUCTL_BLTZ: o_branch_resolution.outcome = op1 < signed'(0)  ? TAKEN : NOT_TAKEN;
				default: begin
					//Not a branch - do nothing
				end
			endcase
			// if(i_decoded.alu_ctl == ALUCTL_BNE & !o_branch_resolution.outcome)
			// 	$display("Branch decode of BNE computed to %d; op1 = %h, op2 = %h. Valid bit is %b", o_branch_resolution.outcome, op1, op2, o_branch_resolution.valid);

			// if(o_branch_resolution.valid == 1'b1)
			// 	$display("Branch decode decided that the instruction with control %d was a branch", i_decoded.alu_ctl);
		end
	end
endmodule
