
module stream_buffer#(
    parameter BLOCK_OFFSET_WIDTH = 2,
    parameter LINE_SIZE = 4,
    parameter BUFF_DATA_WIDTH = `ADDR_WIDTH,
    parameter MEMID = 0,
    parameter BUFFER_LEN = 8

    
)(
    input logic clk,
    input rst_n,
    input logic [`ADDR_WIDTH-3:0] current_addr,  //cache access address
    input logic cache_miss,
    input logic miss_valid,
    output logic hit_out,                       //if there is a hit in the stream buffer    
    output logic [`DATA_WIDTH-1:0] sb_rdata[LINE_SIZE],

    axi_read_address.master mem_read_address[BUFFER_LEN],
     axi_read_data.master mem_read_data[BUFFER_LEN]
);
    // States
    enum logic[1:0] {
        STATE_READY,            // Ready for incoming requests
        STATE_REFILL_REQUEST,       // Missing on a read
        FETCH_CHECK
    } state, next_state;

// available bits
logic [BUFFER_LEN - 1 : 0] available_bits;
// Intermediate signals
logic hit, miss;
//buffer pointers for FIFO managmnet
logic[`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0] buff_tail,buff_head;
//logic that indicates that the stream buffer and cache both missed
logic all_miss;
//regeter to hold the value of the address to write from
logic [`ADDR_WIDTH-BLOCK_OFFSET_WIDTH-3:0]current_addr_reg; 

logic [BUFFER_LEN - 1 : 0] databank_raddr;
logic [`DATA_WIDTH - 1 : 0] databank_rdata [BUFFER_LEN][LINE_SIZE];
logic  databank_available[BUFFER_LEN];
logic [BUFFER_LEN-1:0] ar_valid;
logic [`ADDR_WIDTH -1 : 0] tagbank_rdata[BUFFER_LEN];

genvar g;
    generate
        for (g = 0; g < BUFFER_LEN; g++)
        begin : sb_cells
            sb_cell #(
                .SB_ID(g),
                .MEM_ID(0)
            ) databank (
                .clk,
                .rst_n,
                .cell_addr(current_addr_reg + buff_tail),
                .cell_enable((state == STATE_REFILL_REQUEST)&&(buff_tail[$clog2(BUFFER_LEN)-1:0] == g)),
                .cell_stale((state == STATE_READY) & all_miss),
                .cell_rdata(databank_rdata[g]),
                .cell_rtag(tagbank_rdata[g]),
                .available(databank_available[g]),
                .ar_valid(ar_valid[g]),
                .mem_read_address(mem_read_address[g]),
                .mem_read_data(mem_read_data[g])
            );
        end
    endgenerate

logic [`DATA_WIDTH-1:0] a;
always_comb
begin
    a = tagbank_rdata[buff_head[$clog2(BUFFER_LEN)-1:0]];
    hit = available_bits[buff_head[$clog2(BUFFER_LEN)-1:0]]
        & (current_addr[`ADDR_WIDTH-3:2] == a);

    miss = ~hit;
    all_miss = miss & cache_miss_reg & miss_valid;
end


always_comb
begin
    databank_raddr = buff_head[$clog2(BUFFER_LEN)-1:0];
end

//output logic
always_comb
    begin
        hit_out = hit;
        sb_rdata = databank_rdata[databank_raddr];
    end
//state logic
    logic tail_at_end;
always_comb
    begin
        next_state = state;
        tail_at_end = ((buff_tail - buff_head) == BUFFER_LEN);
        unique case (state)
            //only come out of READY when both cache and stream buffer miss
            STATE_READY:
                next_state =  (all_miss | ~tail_at_end) ? STATE_REFILL_REQUEST : STATE_READY; 
            STATE_REFILL_REQUEST: begin
                if(ar_valid[buff_tail[$clog2(BUFFER_LEN)-1:0]])
                    next_state = FETCH_CHECK;
                end
            FETCH_CHECK:begin
                //if the head has valid data return to ready
                if(available_bits[buff_head[$clog2(BUFFER_LEN)-1:0]] & tail_at_end) begin
                    next_state = STATE_READY;
                end
                if(~tail_at_end) begin
                    next_state = STATE_REFILL_REQUEST;
                end
            end
        endcase
    end
    logic cache_miss_reg;
    always_ff @(posedge clk)
    begin
        if(~rst_n)
        begin
            state <= STATE_READY;
            available_bits <= '0;
            buff_head <= 'b0;
            buff_tail <= 'b0;
            cache_miss_reg <= 'b0;
        end
        else
        begin
            state <= next_state;
            cache_miss_reg <= cache_miss;

            if(databank_available.or()) begin
                for(int i = 0; i < BUFFER_LEN;i++)begin
                    available_bits[i] <= (databank_available[i]) ? 'b1:available_bits[i];
                end
            end
            else if(hit)begin
                buff_head <= buff_head +'b1;
                available_bits[buff_head[$clog2(BUFFER_LEN)-1:0]] <= 'b0;
            end    

            if ((ar_valid[buff_tail[$clog2(BUFFER_LEN)-1:0]]) & (~tail_at_end))
            begin
                buff_tail <= buff_tail+'b1;
            end
               
            case (state)
                STATE_READY:
                begin
                   if (all_miss)
                       begin
                           current_addr_reg <= {current_addr[`ADDR_WIDTH - 3 : BLOCK_OFFSET_WIDTH]};
                           available_bits <= '0;
                           buff_head <= 'b0;
                           buff_tail <= 'b0;
                       end
                       else 
                       begin
                           current_addr_reg <= current_addr_reg;
                       end
                end
                STATE_REFILL_REQUEST,FETCH_CHECK:
                begin
                end
            endcase
        end
    end
endmodule