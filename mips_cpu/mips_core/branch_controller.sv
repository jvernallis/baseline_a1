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

	// Stall
	hazard_control_ifc.in i_hc,

	// Request
	pc_ifc.in if_pc, //IF's next_pc
	branch_prediction_ifc.out if_branch_prediction,

	// Feedback
	pc_ifc.in dec_pc,
	branch_resolution_ifc.in dec_branch_resolved
);
	logic [`ADDR_WIDTH - 1 : 0] branch_outcome; // PC of branch destination
	
	branch_target_buffer BTB (
		.clk, .rst_n,

		.we_btb			(dec_branch_resolved.valid), 
		.w_target		(dec_branch_resolved.target),
		.r_target 		(if_branch_prediction.target),
		.hit_out		(if_branch_prediction.valid),
		
		.i_req_pc		(if_pc.pc),
		.i_pc_next 		(branch_outcome), //Will be computed
		.i_fb_pc		(dec_pc.pc)
	);

	// Predictor interface changes: prediction will be binary logic; target not provided.
	// Change the following line to switch predictor
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
		if(!i_hc.stall)
		begin
			branch_outcome = 
				(if_branch_prediction.valid & (if_branch_prediction.prediction == TAKEN)) 
				? if_branch_prediction.target 
				: if_pc.pc + `ADDR_WIDTH'd4;
		end
		else
		begin
			branch_outcome = if_pc.pc + `ADDR_WIDTH'd4;
		end
	end
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
		o_req_prediction = TAKEN;
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

module branch_predictor_gshare (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Request
	input logic i_req_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	input logic [`ADDR_WIDTH - 1 : 0] i_req_target,
	output mips_core_pkg::BranchOutcome o_req_prediction,

	// Feedback
	input logic i_fb_valid,
	input logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
	input mips_core_pkg::BranchOutcome i_fb_prediction,
	input mips_core_pkg::BranchOutcome i_fb_outcome
);
localparam COUNTER_WIDTH = 2;
localparam INDEX_WIDTH = 16;

logic [INDEX_WIDTH-1:0] index_read; 
logic [INDEX_WIDTH-1:0] index_write; 
logic [INDEX_WIDTH-1:0] ghr;
// logic [INDEX_WIDTH-1:0] old_ghr;
logic [COUNTER_WIDTH-1:0] current_counter;

// logic [1:0] counter;

logic [COUNTER_WIDTH-1:0] counter_regs[2**INDEX_WIDTH];

	task incr;
		begin
			if (counter_regs[index_write] != 2'b11)
				counter_regs[index_write] <= counter_regs[index_write] + 2'b01;
		end
	endtask

	task decr;
		begin
			if (counter_regs[index_write] != 2'b00)
				counter_regs[index_write] <= counter_regs[index_write] - 2'b01;
		end
	endtask


	always_ff @(posedge clk)
	begin
		if(~rst_n)
		begin
			for (int i=0; i<2**INDEX_WIDTH; i++) 
			begin
			counter_regs[i] <= 2'b10; //Default: weakly taken
			end
		end
		else
		begin
		index_write <= index_read;

		if (i_fb_valid)
		begin

				case (i_fb_outcome)
					NOT_TAKEN: decr();
					TAKEN:     incr();
				endcase

				for (int i=1; i<INDEX_WIDTH; i++)
				begin
					ghr[i] <= ghr[i-1];
				end
			ghr[0] <= i_fb_outcome;
		end
		else begin
			ghr <= ghr;
		end
		end
	end

	always_comb
	begin

		index_read = i_req_pc[INDEX_WIDTH-1:0] ^ ghr;
		o_req_prediction = counter_regs[index_read][1] ? TAKEN : NOT_TAKEN;
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
    output logic [`ADDR_WIDTH - 1 : 0] r_target,
	output logic hit_out,

    // Request
    input  logic [`ADDR_WIDTH - 1 : 0] i_req_pc,
	input  logic [`ADDR_WIDTH - 1 : 0] i_pc_next,
	input  logic [`ADDR_WIDTH - 1 : 0] i_fb_pc,
);
    localparam TAG_WIDTH = `ADDR_WIDTH - INDEX_WIDTH - 2;
    localparam DEPTH = 1 << INDEX_WIDTH;
    
    logic [TAG_WIDTH - 1 : 0] i_tag;
	logic [INDEX_WIDTH - 1 : 0] i_index;
	logic [INDEX_WIDTH - 1 : 0] i_index_next;

	logic [TAG_WIDTH - 1 : 0] w_tag;
	logic [INDEX_WIDTH - 1 : 0] w_index;

	logic [`ADDR_WIDTH - 1 : 0] last_i_pc;
	logic [`ADDR_WIDTH - 1 : 0] last_pc_next;

    //signals for least recently used associative logic
	logic r_select_way;
	logic w_select_way;
	
	// btb hit
	logic hit;
	logic w_hit;
	
	logic [DEPTH - 1 : 0] lru_rp;

    assign {i_tag, i_index} = i_req_pc[`ADDR_WIDTH - 1 : 2];
	assign {w_tag, w_index} = i_fb_pc[`ADDR_WIDTH - 1 : 2];
	assign i_index_next = i_pc_next[INDEX_WIDTH - 1 + 2 : 2];
    // databank signals
	logic databank_we[ASSOCIATIVITY];
	logic [`ADDR_WIDTH - 1 : 0] databank_wdata;
	logic [INDEX_WIDTH - 1 : 0] databank_waddr;
	logic [INDEX_WIDTH - 1 : 0] databank_raddr;
    logic [`ADDR_WIDTH - 1 : 0] databank_rdata[ASSOCIATIVITY];

    //generate data banks
    genvar g;
	generate
		for (g=0; g< ASSOCIATIVITY; g++)
		begin : databanks
			cache_bank #(
				.DATA_WIDTH (`ADDR_WIDTH),
				.ADDR_WIDTH (INDEX_WIDTH)
			) databank (
				.clk,
				.i_we (databank_we[g]),
				.i_wdata(databank_wdata),
				.i_waddr(databank_waddr),
				.i_raddr(databank_raddr),

				.o_rdata(databank_rdata[g])
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
		for (w=0; w < ASSOCIATIVITY; w++)
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
   
    always_comb
    begin
		//change hit condition when more ways are added
		hit = ( ((i_tag == tagbank_rdata[0]) & valid_bits[0][i_index])
			  | ((i_tag == tagbank_rdata[1]) & valid_bits[1][i_index]));

		hit &= (i_req_pc == last_pc_next);
       
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
		// else
		// begin
		// 	select_way = lru_rp[i_index];
		// end

		w_hit = ( ((w_tag == tagbank_rdata[0]) & valid_bits[0][w_index])
				| ((w_tag == tagbank_rdata[1]) & valid_bits[1][w_index]));
		w_hit &= (last_i_pc == i_req_pc);
		if (w_hit)
		begin
			if (w_tag == tagbank_rdata[0])
			begin
				w_select_way = 'b0;
			end
			else
			begin
				w_select_way = 'b1;
			end
		end
		else
		begin
			w_select_way = lru_rp[w_index];
		end
    end

    //tagbank connections
    always_comb
	begin
        tagbank_wdata = w_tag;
        tagbank_waddr = w_index;
		tagbank_we[w_select_way] = we_btb;
		tagbank_we[~w_select_way] = '0;
		tagbank_raddr = i_index_next;
	end

    //databank connections
    always_comb
	begin
        databank_wdata = w_target;
        databank_waddr = w_index;
		databank_we[w_select_way] = we_btb;
        databank_we[~w_select_way] = '0;
		databank_raddr = i_index_next;
	end

    //read outputs
    always_comb
	begin
		hit_out = hit;
		r_target = databank_rdata[r_select_way];
	end
	
    always_ff @(posedge clk)
	begin
		last_pc_next <= i_pc_next;
		last_i_pc <= i_req_pc;
		if(~rst_n)
		begin
			for (int i=0; i<DEPTH;i++)
				lru_rp[i] <= 0;
			for (int i=0; i<ASSOCIATIVITY;i++)
				valid_bits[i] <= '0;
		end
		else
		begin
			if(hit)
				lru_rp[i_index] <= ~r_select_way;
            // if(~hit)
            // begin
			// 	//data for write locked in for miss(will write 2 cycles after miss and branch confirm)
            //     w1_tag_data <= i_tag;
            //     w1_index <= i_index;
            //     w1_select_way <= r_select_way;
            // end
			// 	//set valid bit on a write(will be valid cycle after write)
			// 	valid_bits[w2_index][w2_select_way] <= we_btb;
			// 	//data for write (will write cycle after miss and branch confirm)
			// 	w2_tag_data <= w1_tag_data;  
			// 	w2_index <= w1_index;
			// 	w2_select_way <= w1_select_way;
			// 	//target from decode sent to write
			// 	w_target_data <= w_target;


		if(we_btb)
		begin
			valid_bits[w_select_way][w_index] <= '1;
			// if(w_hit)
			// 	$display("Write hit with w_tag = %h and w_index = %h", w_tag, w_index);
			// $display("Write enable with input pc %h, target %h", i_fb_pc, w_target);

			if(i_fb_pc == 12'hACC)
			begin
				// $display("Writing to the cache for branch ACC, target is %h", w_target);
				// $display("Writing index %h and tag %h", tagbank_waddr, tagbank_wdata);
			end

			// if(i_fb_pc == 12'hAC8)
			// 	$display("Error: writing to cache for branch AC8. This shouldn't ever happen. Target is %h", w_target);
		end
		if(hit)
		begin
			//$display("Cache read hit for PC %h, the target was %h", i_req_pc, r_target);
			if(i_req_pc == 12'hACC)
				// $display("Hit the cache for branch ACC, target is %h", r_target);

			if(i_req_pc == 12'hAC8)
			begin
				$display("Error: hit the cache for branch AC8. This shouldn't ever happen. Target is %h", r_target);
				$display("When this happened, index was %h and tag was %h. Last pc next was %h, last pc was %h", i_index, i_tag, last_pc_next, last_i_pc);
				$display("Sanity check: PC was %h, next PC is %h, current index is %h, next index is %h", i_req_pc, i_pc_next, i_index, i_index_next);
			end
		end

		if(i_req_pc == 12'hACC)
		begin
			//$display("At PC %h, the hit status is %d. Tag was %h and index was %h for a last PC of %h and a last next PC of %h", i_req_pc, hit, i_index, i_tag, last_i_pc, last_pc_next);
		end

		if(i_pc_next == 12'hACC)
		begin
			// if(hit)
			// begin
			// 	$display("Error: We should not be here, hit is 1.");
			// end
			// $display("At ACC's read in data/tag banks. The read index is %h", i_index_next);
			// $display("Databanks are currently %h and %h", databank_rdata[0], databank_rdata[1]);
			// $display("Tagbanks are currently %h and %h", tagbank_rdata[0], tagbank_rdata[1]);
			// $display("i_req_pc is %h", i_req_pc);
		end
      end
	end
endmodule

module branch_resolver (
	pc_ifc.in i_pc,
	decoder_output_ifc.in i_decoded,
	reg_file_output_ifc.in i_reg_data,

	// Feedback
	branch_resolution_ifc.out o_branch_resolution
);
	always_comb
	begin
	if (i_decoded.valid)
		begin
			logic signed [`DATA_WIDTH - 1 : 0] op1 =	i_reg_data.rs_data;
			logic signed [`DATA_WIDTH - 1 : 0] op2 =  	i_decoded.uses_immediate
				? i_decoded.immediate
				: i_reg_data.rt_data;

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

			if(o_branch_resolution.valid & o_branch_resolution.outcome)
				o_branch_resolution.target = i_decoded.branch_target;
			else
				o_branch_resolution.target = i_pc.pc + `ADDR_WIDTH'd4;
		end
	end
endmodule
