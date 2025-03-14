module psudo_lru #(
    parameter ASSOCIATIVITY = 4
)(
    input clk,
    input rst_n,
    input lru_en,
    input logic [$clog2(ASSOCIATIVITY)-1:0] select_way,
    output logic [$clog2(ASSOCIATIVITY)-1:0] lru_rp
);

logic br_1, br_2l, br_2r, en_last;
always_ff@(posedge clk) begin
    if(~rst_n) begin
        br_1 <= 'b0;
        br_2l <= 'b0;
        br_2r <= 'b0;
        en_last <='b0;
    end
    else begin
        en_last <= lru_en;
        if(lru_en &(en_last!=lru_en))begin
            if(select_way[1])begin
                br_1 <= 'b0;
                br_2l <= br_2l;
                br_2r <= ~br_2r;
            end
            else begin
                br_1 <= 'b1;
                br_2l <= ~br_2l;
                br_2r <= br_2r;
            end
        end
        else begin
            br_1 <= br_1;
            br_2l <= br_2l;
            br_2r <= br_2r;
        end
    end
end

assign lru_rp =br_1 ?{br_1,br_2r}:{br_1,br_2l};
endmodule