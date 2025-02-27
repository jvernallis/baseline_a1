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
	 parameter INDEX_WIDTH = 6, // 1 KB Cahe size 
	 parameter BLOCK_OFFSET_WIDTH = 2,
	 parameter BUFFER_LEN = 8
	 )(
	 // General signals
	 input clk,    // Clock
	 input rst_n,  // Synchronous reset active low
 
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
	 logic [`ADDR_WIDTH-1:0]curr,next;
	 assign curr = i_pc_current.pc[`ADDR_WIDTH - 1 : 0];
	 assign next = i_pc_next.pc[`ADDR_WIDTH - 1 : 0];
	 assign {i_tag, i_index, i_block_offset} = i_pc_current.pc[`ADDR_WIDTH - 1 : 2];
	 assign i_index_next = i_pc_next.pc[BLOCK_OFFSET_WIDTH + 2 +: INDEX_WIDTH];
	 // Above line uses +: slice, a feature of SystemVerilog
	 // See https://stackoverflow.com/questions/18067571
 
	 // States
	 enum logic[1:0] {
		 STATE_READY,            // Ready for incoming requests
		 STATE_REFILL_REQUEST,   // Sending out a memory read request
		 STATE_REFILL_DATA       // Missing on a read
	 } state, next_state;
 
	 // Registers for refilling
	 logic [INDEX_WIDTH - 1:0] r_index;
	 logic [TAG_WIDTH - 1:0] r_tag;
 
	 // databank signals
	 //logic [LINE_SIZE - 1 : 0] databank_select;
	 logic [LINE_SIZE - 1 : 0] databank_we;
	 logic [`DATA_WIDTH - 1 : 0] databank_wdata[LINE_SIZE];
	 logic [INDEX_WIDTH - 1 : 0] databank_waddr;
	 logic [INDEX_WIDTH - 1 : 0] databank_raddr;
	 logic [`DATA_WIDTH - 1 : 0] databank_rdata [LINE_SIZE];
 
	 // databanks
	 genvar g;
	 generate
		 for (g = 0; g < LINE_SIZE; g++)
		 begin : databanks
			 cache_bank #(
				 .DATA_WIDTH (`DATA_WIDTH),
				 .ADDR_WIDTH (INDEX_WIDTH)
			 ) databank (
				 .clk,
				 .i_we (databank_we[g]),
				 .i_wdata(databank_wdata[g]),
				 .i_waddr(databank_waddr),
				 .i_raddr(databank_raddr),
 
				 .o_rdata(databank_rdata[g])
			 );
		 end
	 endgenerate
 
	 // tagbank signals
	 logic tagbank_we;
	 logic [TAG_WIDTH - 1 : 0] tagbank_wdata;
	 logic [INDEX_WIDTH - 1 : 0] tagbank_waddr;
	 logic [INDEX_WIDTH - 1 : 0] tagbank_raddr;
	 logic [TAG_WIDTH - 1 : 0] tagbank_rdata;
 
	 cache_bank #(
		 .DATA_WIDTH (TAG_WIDTH),
		 .ADDR_WIDTH (INDEX_WIDTH)
	 ) tagbank (
		 .clk,
		 .i_we    (tagbank_we),
		 .i_wdata (tagbank_wdata),
		 .i_waddr (tagbank_waddr),
		 .i_raddr (tagbank_raddr),
 
		 .o_rdata (tagbank_rdata)
	 );
 
	 // Valid bits
	 logic [DEPTH  - 1 : 0] valid_bits;
 
	 // Intermediate signals
	 logic hit, miss,sb_hit;
	 logic [`DATA_WIDTH-1:0] sb_rdata[LINE_SIZE];
	 logic last_refill_word;
	 logic v;
 
	 always_comb
	 begin
		v = valid_bits[i_index];
		 hit = v
			 & (i_tag == tagbank_rdata)
			 & (state == STATE_READY);
		 miss = ~hit;
	 end
 
	 always_comb
	 begin
 
		 databank_we = {LINE_SIZE{sb_hit & (state == STATE_REFILL_DATA)}};
 
		 databank_wdata = sb_rdata;
		 databank_waddr = r_index;
		 databank_raddr = i_index_next;
	 end
 
	 always_comb
	 begin
		 tagbank_we = sb_hit & (state == STATE_REFILL_DATA);
		 tagbank_wdata = r_tag;
		 tagbank_waddr = r_index;
		 tagbank_raddr = i_index_next;
	 end
 
	 always_comb
	 begin
		 out.valid = hit;
		 out.data = databank_rdata[i_block_offset];
	 end
	logic eq;
	 always_comb
	 begin
		 //eq = (i_pc_current.pc == i_pc_next.pc);
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
			 valid_bits <= '0;
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
					 end
				 end
				 STATE_REFILL_DATA:
				 begin
					 if (sb_hit)
					 begin
						 valid_bits[r_index] <= sb_hit;
					 end
				 end
			 endcase
		 end
	 end
	 stream_buffer#(
		 .BLOCK_OFFSET_WIDTH(BLOCK_OFFSET_WIDTH),
		 .LINE_SIZE(LINE_SIZE),
		 .BUFF_DATA_WIDTH(`ADDR_WIDTH),
		 .BUFFER_LEN(BUFFER_LEN)
	 )SB(
		 .clk,
		 .rst_n,
		 .current_addr({r_tag,r_index,2'b0}),
		 .cache_miss(miss),
		 .miss_valid(STATE_REFILL_DATA == state),
		 .hit_out(sb_hit),
		 .sb_rdata(sb_rdata),
		 .mem_read_address,
		 .mem_read_data
	 );
 endmodule
 
 