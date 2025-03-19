/*
 * i_cache.sv
 * Author: Zinsser Zhang 
 * Revision : Sankara 			
 * Last Revision: 04/04/2023
 *
 * This is a direct-mapped instruction cache. Line size and depth (number of
 * lines) are set via INDEX_WIDTH and BLOCK_OFFSET_WIDTH parameters. Notice that
 * line size means number of words (each consist of 32 bit) in a line. Because
 * all addresses in mips_core are 26 byte addresses, so the sum of TAG_WIDTH,
 * INDEX_WIDTH and BLOCK_OFFSET_WIDTH is `ADDR_WIDTH - 2.
 *
 * Typical line sizes are from 2 words to 8 words. The memory interfaces only
 * support up to 8 words line size.
 *
 * Because we need a hit latency of 1 cycle, we need an asynchronous read port,
 * i.e. data is ready during the same cycle when address is calculated. However,
 * SRAMs only support synchronous read, i.e. data is ready the cycle after the
 * address is calculated. Due to this conflict, we need to read from the banks
 * on the clock edge at the beginning of the cycle. As a result, we need both
 * the registered version of address and a non-registered version of address
 * (which will effectively be registered in SRAM).
 *
 * See wiki page "Synchronous Caches" for details.
 */
 `include "mips_core.svh"

 module i_cache #(
	parameter NUM_THREADS = 2,
	 parameter INDEX_WIDTH = 6, // 1 KB Cahe size 
	 parameter BLOCK_OFFSET_WIDTH = 2,
	 parameter BUFFER_LEN = 8,
	 parameter ASSOCIATIVITY = 2
	 )(
	 // General signals
	 input clk,    // Clock
	 input rst_n,  // Synchronous reset active low
	 
	 thread_control_ifc.in i_tc,
 
	 // Request
	 pc_ifc.in i_pc_current,
	 pc_ifc.in i_pc_next,
 
	 // Response
	 cache_output_ifc.out out,
 
	 // Memory interface
	 axi_read_address.master mem_read_address[BUFFER_LEN],
	 axi_read_data.master mem_read_data[BUFFER_LEN]
 );
 localparam TAG_WIDTH = `ADDR_WIDTH - INDEX_WIDTH - BLOCK_OFFSET_WIDTH - 2;
 localparam LINE_SIZE = 1 << BLOCK_OFFSET_WIDTH;
 localparam DEPTH = 1 << INDEX_WIDTH;

 // Check if the parameters are set correctly
 generate
	if(TAG_WIDTH <= 0 || LINE_SIZE > 16)
	begin
		INVALID_I_CACHE_PARAM invalid_i_cache_param ();
	end
endgenerate

 // Parsing
 logic [TAG_WIDTH - 1 : 0] i_tag;
 logic [INDEX_WIDTH - 1 : 0] i_index;
 logic [BLOCK_OFFSET_WIDTH - 1 : 0] i_block_offset;

 logic [INDEX_WIDTH - 1 : 0] i_index_next;
 logic sb_hit;
 logic [`DATA_WIDTH-1:0] sb_rdata[LINE_SIZE];

 assign {i_tag, i_index, i_block_offset} = i_pc_current.pc[`ADDR_WIDTH - 1 : 2];
 assign i_index_next = i_pc_next.pc[BLOCK_OFFSET_WIDTH + 2 +: INDEX_WIDTH];
 // Above line uses +: slice, a feature of SystemVerilog
 // See https://stackoverflow.com/questions/18067571

 // States
 enum logic [2:0] {
	 STATE_READY,            // Ready for incoming requests
	 STATE_REFILL_REQUEST,   // Sending out memory read request
	 STATE_REFILL_DATA       // Loads a cache line from memory
 } state, next_state;

 // Registers for flushing and refilling
 logic [INDEX_WIDTH - 1:0] r_index;
 logic [TAG_WIDTH - 1:0] r_tag;

 // databank signals
 logic [LINE_SIZE - 1 : 0] databank_select;
 logic [LINE_SIZE - 1 : 0] databank_we[ASSOCIATIVITY];
 logic [`DATA_WIDTH - 1 : 0] databank_wdata[LINE_SIZE];
 logic [INDEX_WIDTH - 1 : 0] databank_waddr;
 logic [INDEX_WIDTH - 1 : 0] databank_raddr;
 logic [`DATA_WIDTH - 1 : 0] databank_rdata [ASSOCIATIVITY][LINE_SIZE];

 logic select_way;
 logic r_select_way;
 logic [DEPTH - 1 : 0] lru_rp;

 // databanks
 genvar g,w;
 generate
	 for (g = 0; g < LINE_SIZE; g++)
	 begin : datasets
		 for (w=0; w< ASSOCIATIVITY; w++)
		 begin : databanks
			 cache_bank #(
				 .DATA_WIDTH (`DATA_WIDTH),
				 .ADDR_WIDTH (INDEX_WIDTH)
			 ) databank (
				 .clk,
				 .i_we (databank_we[w][g]),
				 .i_wdata(databank_wdata[g]),
				 .i_waddr(databank_waddr),
				 .i_raddr(databank_raddr),

				 .o_rdata(databank_rdata[w][g])
			 );
		 end
	 end
 endgenerate

 // tagbank signals
 logic tagbank_we[ASSOCIATIVITY];
 logic [TAG_WIDTH - 1 : 0] tagbank_wdata;
 logic [INDEX_WIDTH - 1 : 0] tagbank_waddr;
 logic [INDEX_WIDTH - 1 : 0] tagbank_raddr;
 logic [TAG_WIDTH - 1 : 0] tagbank_rdata[ASSOCIATIVITY];

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
 logic [DEPTH - 1 : 0] valid_bits[NUM_THREADS][ASSOCIATIVITY];


 // Intermediate signals
 logic hit, miss, tag_hit;
 logic last_refill_word;

 always_comb
 begin
	 tag_hit = ( ((i_tag == tagbank_rdata[0]) & valid_bits[i_tc.thread_id][0][i_index])
			   |	((i_tag == tagbank_rdata[1]) & valid_bits[i_tc.thread_id][1][i_index]));
	 hit = (tag_hit)
		 & (state == STATE_READY);
	 miss = ~hit;
 
	 if (hit)
	 begin
		 if (i_tag == tagbank_rdata[0])
		 begin
			 select_way = 'b0;
		 end
		 else 
		 begin
			 select_way = 'b1;
		 end
	 end
	 else if (miss)
	 begin
		 select_way = lru_rp[i_index];
	 end
	 else
	 begin
		 select_way = 'b0;
	 end
 
 end

 always_comb
 begin
	 for (int i=0; i<ASSOCIATIVITY;i++)
		 databank_we[i] = '0;
	 if (sb_hit)				// We are refilling data
		 databank_we[r_select_way] = {LINE_SIZE{sb_hit & (state == STATE_REFILL_DATA)}};
 end

 always_comb
 begin

		 databank_wdata = sb_rdata;
		 databank_waddr = r_index;
		 if (next_state == STATE_READY)
			 databank_raddr = i_index_next;
		 else
			 databank_raddr = r_index;
 end

 always_comb
 begin
	 tagbank_we[r_select_way] = sb_hit & (state == STATE_REFILL_DATA);
	 tagbank_we[~r_select_way] = '0;
	 tagbank_wdata = r_tag;
	 tagbank_waddr = r_index;
	 tagbank_raddr = i_index_next;
 end

 always_comb
 begin
	 out.valid = hit;
	 out.data = databank_rdata[select_way][i_block_offset];
 end

 always_comb
 begin
	 next_state = state;
	 unique case (state)
		 STATE_READY:
			 if (miss)
				next_state = STATE_REFILL_DATA;
		 STATE_REFILL_DATA:
			 if (sb_hit)
				 next_state = STATE_READY;
	 endcase
 end

 always_ff @(posedge clk)
 begin
	 if(~rst_n)
	 begin
		 state <= STATE_READY;
		 databank_select <= 1;
		 for (int i=0; i<ASSOCIATIVITY;i++)
			 valid_bits[i_tc.thread_id][i] <= '0;
		 for (int i=0; i<DEPTH;i++)
			 lru_rp[i] <= 0;
	 end
	 else
	 begin
		 state <= next_state;

		 case (state)
			 STATE_READY:
			 begin
				 if (miss)
				 begin
					 r_tag <= i_tag;
					 r_index <= i_index;
					 r_select_way <= select_way;
				 end
					 lru_rp[i_index] <= ~select_way;
			 end
			 STATE_REFILL_DATA:
			 begin
				 if (sb_hit)
				 begin
					 valid_bits[i_tc.thread_id][r_select_way][r_index] <= 1'b1;
				 end
			 end
		 endcase
	 end
 end
 stream_buffer#(
	.BLOCK_OFFSET_WIDTH(BLOCK_OFFSET_WIDTH),
	.LINE_SIZE(LINE_SIZE),
	.BUFF_DATA_WIDTH(`DATA_WIDTH),
	.BUFFER_LEN(BUFFER_LEN),
	.MEMID(0)
)SB_G(
	.clk,
	.rst_n(rst_n),
	.thread_id(i_tc.thread_id),
	.current_addr({r_tag,r_index,2'b0}),
	.cache_miss(miss),
	.miss_valid(STATE_REFILL_DATA == state),
	.hit_out(sb_hit),
	.sb_rdata(sb_rdata),
	.mem_read_address,
	.mem_read_data
);
 endmodule
 
 