module sb_cell#(
    parameter BLOCK_OFFSET_WIDTH = 2,
    parameter LINE_SIZE = 4,
    parameter BUFF_DATA_WIDTH = `ADDR_WIDTH,
    parameter SB_ID = 0,
    parameter MEM_ID 
)(
    input logic clk,
    input rst_n,
    input logic thread_id,
    input logic [`ADDR_WIDTH-3:0] cell_addr,
    input logic cell_enable,
    input logic cell_stale,
    output logic [`DATA_WIDTH-1:0] cell_rdata[LINE_SIZE],
    output logic [`ADDR_WIDTH-1:0] cell_rtag,
    output logic available,
    output logic ar_valid,

    axi_read_address.master mem_read_address,
    axi_read_data.master mem_read_data
);
    // States
    enum logic[1:0] {
        STATE_READY,            // Ready for incoming requests
        STATE_REFILL_REQUEST,   // Sending out a memory read request
        STATE_REFILL_DATA,       // Missing on a read
        STATE_STALE
    } state, next_state;
logic [3:0]memid;
assign memid = MEM_ID + SB_ID ;
logic last_refill_word;

logic [LINE_SIZE - 1 : 0] databank_select;
logic [LINE_SIZE - 1 : 0] databank_we;
logic [`DATA_WIDTH - 1 : 0] databank_wdata;
logic [`DATA_WIDTH - 1 : 0] databank_rdata [LINE_SIZE];

genvar g;
    generate
        for (g = 0; g < LINE_SIZE; g++)
        begin : databanks
            cache_bank #(
                .DATA_WIDTH (`DATA_WIDTH),
                .ADDR_WIDTH (1)
            ) databank (
                .clk,
                .i_we (databank_we[g]),
                .i_wdata(databank_wdata),
                .i_waddr('b0),
                .i_raddr('b0),

                .o_rdata(databank_rdata[g])
            );
        end
    endgenerate

logic [`ADDR_WIDTH - 3 : 0] tagbank_rdata;

cache_bank #(
    .DATA_WIDTH (`DATA_WIDTH),
    .ADDR_WIDTH ($clog2(1))
) tagbank (
    .clk,
    .i_we    (cell_enable),
    .i_wdata (cell_addr),
    .i_waddr ('b0),
    .i_raddr ('b0),
    
    .o_rdata (tagbank_rdata)
);
always_comb
begin
    last_refill_word = databank_select[LINE_SIZE - 1]
        & mem_read_data.RVALID;
end

logic[`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] mem_addr;
always_comb
    begin
        mem_addr = (cell_addr);
        mem_read_address.ARADDR = {mem_addr,
            {BLOCK_OFFSET_WIDTH + 2{1'b0}}};
        // Experimental: Set memory address MSB to thread ID
		mem_read_address.ARADDR = {thread_id, mem_read_address.ARADDR[`ADDR_WIDTH - 2 : 0]};
        mem_read_address.ARLEN = LINE_SIZE;
        mem_read_address.ARVALID = state == STATE_REFILL_REQUEST;
        mem_read_address.ARID = memid;

        // Always ready to consume data
        mem_read_data.RREADY = 1'b1;
    end

    always_comb
    begin
        if (mem_read_data.RVALID)
            databank_we = databank_select;
        else
            databank_we = '0;

        databank_wdata = mem_read_data.RDATA;
    end
//output logic
always_comb
    begin
        cell_rdata = databank_rdata;
        cell_rtag = tagbank_rdata;
        available = last_refill_word & (state != STATE_STALE);
        ar_valid = mem_read_address.ARVALID;
    end
always_comb
    begin
        next_state = state;
        unique case (state)
            //only come out of READY when both cache and stream buffer miss
            STATE_READY:
                next_state =  (cell_enable) ? STATE_REFILL_REQUEST : STATE_READY; 
            STATE_REFILL_REQUEST:begin
                if (mem_read_address.ARREADY)
                    next_state = STATE_REFILL_DATA;
            end
            STATE_REFILL_DATA: begin
                if(last_refill_word)begin
                    next_state = STATE_READY;
                end
                else if(cell_stale)begin
                        next_state = STATE_STALE;
                end
            end
            STATE_STALE: begin
                //return to a ready state after the last stale data is recieved
                if(mem_read_data.RVALID & mem_read_data.RLAST)
                    next_state = STATE_READY;
                end
        endcase
        
    end
    always_ff @(posedge clk)
    begin
        if(~rst_n)
        begin
            state <= STATE_READY;
            databank_select <= 1;
        end
        else
        begin
            state <= next_state;        
            case (state)
                STATE_READY:
                begin
                    databank_select <=databank_select;
                end
                STATE_REFILL_REQUEST:
                begin
                    databank_select <=databank_select;
                end
                STATE_REFILL_DATA:
                begin
                    if (mem_read_data.RVALID)
                    begin
                        databank_select <= {databank_select[LINE_SIZE - 2 : 0],
                            databank_select[LINE_SIZE - 1]};
                    end
                end
                STATE_STALE:
                begin
                    databank_select <= 1;
                end
            endcase
        end
    end
endmodule