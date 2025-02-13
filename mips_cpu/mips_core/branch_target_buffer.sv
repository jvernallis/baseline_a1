`include "mips_core.svh"

module branch_target_buffer (
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
    pc_ifc.in i_pc_current,
	pc_ifc.in i_pc_next
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

    assign {i_tag, i_index} = i_pc_current.pc[`ADDR_WIDTH - 1 : 2];
	assign i_index_next = i_pc_next.pc[INDEX_WIDTH-1:0];
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
