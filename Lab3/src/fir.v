module fir 
#(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)(
    // AXI4-Lite接口
    output wire                     awready,
    output wire                     wready,
    input  wire                     awvalid,
    input  wire [pADDR_WIDTH-1:0]   awaddr,
    input  wire                     wvalid,
    input  wire [pDATA_WIDTH-1:0]   wdata,
    output wire                     arready,
    input  wire                     rready,
    input  wire                     arvalid,
    input  wire [pADDR_WIDTH-1:0]   araddr,
    output wire                     rvalid,
    output wire [pDATA_WIDTH-1:0]   rdata,    
    
    // 流数据接口
    input  wire                     ss_tvalid, 
    input  wire [pDATA_WIDTH-1:0]   ss_tdata, 
    input  wire                     ss_tlast, 
    output wire                     ss_tready, 
    input  wire                     sm_tready, 
    output wire                     sm_tvalid, 
    output wire [pDATA_WIDTH-1:0]   sm_tdata, 
    output wire                     sm_tlast, 
    
    // BRAM接口
    output wire [3:0]               tap_WE,
    output wire                     tap_EN,
    output wire [pDATA_WIDTH-1:0]   tap_Di,
    output wire [pADDR_WIDTH-1:0]   tap_A,
    input  wire [pDATA_WIDTH-1:0]   tap_Do,

    output wire [3:0]               data_WE,
    output wire                     data_EN,
    output wire [pDATA_WIDTH-1:0]   data_Di,
    output wire [pADDR_WIDTH-1:0]   data_A,
    input  wire [pDATA_WIDTH-1:0]   data_Do,

    input  wire                     axis_clk,
    input  wire                     axis_rst_n
);

    // 状态定义
    localparam [1:0] IDLE  = 2'b00;
    localparam [1:0] START = 2'b01;
    localparam [1:0] DONE  = 2'b10;

    // 内部信号
    reg [1:0] current_state, next_state;
    reg [pADDR_WIDTH-1:0] tap_addr;
    reg [pDATA_WIDTH-1:0] length_reg;
    reg [2:0] ap_ctrl; // {ap_start, ap_done, ap_idle}
    
    // FIR计算相关
    reg signed [pDATA_WIDTH-1:0] mult_result;
    reg signed [pDATA_WIDTH-1:0] fir_accum;
    reg [pDATA_WIDTH-1:0] input_reg;
    reg [pADDR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [6:0] cycle_counter;
    reg [9:0] data_counter;

    // AXI-Lite控制信号
    wire axi_aw_ready = (current_state == IDLE) && awvalid && wvalid;
    wire axi_ar_ready = arvalid && !rvalid;

    // 状态转换逻辑
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    always @(*) begin
        next_state = current_state;
        case(current_state)
            IDLE:   if (ap_ctrl[0]) next_state = START;
            START:  if (data_counter == length_reg-1 && sm_tready) next_state = DONE;
            DONE:   next_state = IDLE;
        endcase
    end

    // AXI-Lite写通道
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ap_ctrl <= 3'b100; // 初始化为idle状态
            length_reg <= 0;
        end else begin
            if (axi_aw_ready) begin
                case(awaddr)
                    12'h00: ap_ctrl[0] <= wdata[0]; // ap_start
                    12'h10: length_reg <= wdata;
                endcase
            end
            // 状态更新
            ap_ctrl[1] <= (current_state == DONE); // ap_done
            ap_ctrl[2] <= (current_state == IDLE); // ap_idle
        end
    end

    // AXI-Lite读通道
    reg [pDATA_WIDTH-1:0] read_data;
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            read_data <= 0;
        end else if (arvalid && axi_ar_ready) begin
            case(araddr)
                12'h00: read_data <= {29'd0, ap_ctrl};
                12'h10: read_data <= length_reg;
                default: read_data <= tap_Do;
            endcase
        end
    end

    // BRAM控制逻辑
    assign tap_WE = (awaddr >= 12'h20 && axi_aw_ready) ? 4'b1111 : 4'b0;
    assign tap_Di = wdata;
    assign tap_EN = 1'b1;
    assign data_WE = (current_state == START && cycle_counter == Tape_Num) ? 4'b1111 : 4'b0;
    assign data_Di = input_reg;
    assign data_EN = 1'b1;

    // 数据通路
    always @(posedge axis_clk) begin
        if (ss_tvalid && ss_tready)
            input_reg <= ss_tdata;
    end

    // FIR计算逻辑
    always @(posedge axis_clk) begin
        if (current_state == START) begin
            if (cycle_counter < Tape_Num) begin
                mult_result <= input_reg * tap_Do;
                fir_accum <= fir_accum + mult_result;
            end else begin
                fir_accum <= 0;
            end
        end
    end

    // 地址生成逻辑
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            case(current_state)
                IDLE: begin
                    wr_ptr <= 0;
                    rd_ptr <= 0;
                end
                START: begin
                    if (cycle_counter == 0) begin
                        wr_ptr <= wr_ptr + 1;
                        rd_ptr <= rd_ptr + 1;
                    end
                end
            endcase
        end
    end

    // 计数器逻辑
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            cycle_counter <= 0;
            data_counter <= 0;
        end else begin
            if (current_state == START) begin
                if (cycle_counter < Tape_Num)
                    cycle_counter <= cycle_counter + 1;
                else
                    cycle_counter <= 0;

                if (sm_tvalid && sm_tready)
                    data_counter <= data_counter + 1;
            end else begin
                cycle_counter <= 0;
                data_counter <= 0;
            end
        end
    end

    // 输出逻辑
    assign sm_tvalid = (cycle_counter == Tape_Num);
    assign sm_tdata = fir_accum;
    assign sm_tlast = (data_counter == length_reg-1);
    assign ss_tready = (current_state == START && cycle_counter == 0);

    // BRAM地址分配
    assign tap_A = (current_state == IDLE) ? (awaddr - 12'h20) :
                   (current_state == START) ? cycle_counter :
                   (araddr - 12'h20);
    assign data_A = (current_state == IDLE) ? {rd_ptr, 2'b00} :
                    (current_state == START) ? {wr_ptr, 2'b00} : 0;

    // AXI-Lite响应信号
    assign awready = axi_aw_ready;
    assign wready = axi_aw_ready;
    assign arready = axi_ar_ready;
    assign rvalid = (read_data != 0);
    assign rdata = read_data;

endmodule
