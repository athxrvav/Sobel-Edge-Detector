`timescale 1ns / 1ps

module sobel_edge_detector #(
    parameter IMG_WIDTH  = 8,      // Configurable width
    parameter IMG_HEIGHT = 8,      // Configurable height
    parameter THRESHOLD  = 30      // Configurable edge threshold
)(
    input wire clk,
    input wire reset,
    input wire start,
    output reg done
);

    localparam TOTAL_PIXELS = IMG_WIDTH * IMG_HEIGHT;

    // MEMORY DECLARATIONS (1D flattened arrays)
    reg [23:0] input_mem [0:TOTAL_PIXELS-1]; // 24-bit RGB inputs
    reg [7:0]  gray_mem  [0:TOTAL_PIXELS-1]; // 8-bit Grayscale internally generated
    reg        edge_mem  [0:TOTAL_PIXELS-1]; // 1-bit Edge mapping output

    // FSM STATES
    localparam IDLE  = 3'd0,
               LOAD  = 3'd1,
               GRAY  = 3'd2,
               SOBEL = 3'd3,
               DUMP  = 3'd4,
               DONE  = 3'd5;
               
    reg [2:0] state;

    // INTERNAL VARIABLES
    integer pix_count;
    integer row_cnt;
    integer col_cnt;

    // Grayscale Conversion Variables
    reg [7:0] r_val, g_val, b_val;
    reg [17:0] gray_full; 

    // Sobel 3x3 Window Variables (9-bit to prevent overflow before signs)
    reg [8:0] P00, P01, P02; 
    reg [8:0] P10, P11, P12; 
    reg [8:0] P20, P21, P22; 

    // Gradient Calculation Variables
    reg signed [10:0] Gx, Gy;
    reg [10:0] abs_Gx, abs_Gy;
    reg [11:0] magnitude;

    // TASK: LOAD 3x3 PIXEL WINDOW
    // Automatically handles boundary pixels by padding with 0.
    // Address calculation formula used: (row * width) + column
    task automatic load_window;
        input integer c_row;
        input integer c_col;
        begin
            // Top Row (c_row - 1)
            P00 = (c_row > 0 && c_col > 0)             ? {1'b0, gray_mem[(c_row-1)*IMG_WIDTH + (c_col-1)]} : 9'd0;
            P01 = (c_row > 0)                          ? {1'b0, gray_mem[(c_row-1)*IMG_WIDTH + c_col]}     : 9'd0;
            P02 = (c_row > 0 && c_col < IMG_WIDTH-1)   ? {1'b0, gray_mem[(c_row-1)*IMG_WIDTH + (c_col+1)]} : 9'd0;
            
            // Middle Row (c_row)
            P10 = (c_col > 0)                          ? {1'b0, gray_mem[c_row*IMG_WIDTH + (c_col-1)]}     : 9'd0;
            P11 =                                        {1'b0, gray_mem[c_row*IMG_WIDTH + c_col]};
            P12 = (c_col < IMG_WIDTH-1)                ? {1'b0, gray_mem[c_row*IMG_WIDTH + (c_col+1)]}     : 9'd0;
            
            // Bottom Row (c_row + 1)
            P20 = (c_row < IMG_HEIGHT-1 && c_col > 0)           ? {1'b0, gray_mem[(c_row+1)*IMG_WIDTH + (c_col-1)]} : 9'd0;
            P21 = (c_row < IMG_HEIGHT-1)                        ? {1'b0, gray_mem[(c_row+1)*IMG_WIDTH + c_col]}     : 9'd0;
            P22 = (c_row < IMG_HEIGHT-1 && c_col < IMG_WIDTH-1) ? {1'b0, gray_mem[(c_row+1)*IMG_WIDTH + (c_col+1)]} : 9'd0;
        end
    endtask

    // MAIN STATE MACHINE
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state     <= IDLE;
            done      <= 1'b0;
            pix_count <= 0;
            row_cnt   <= 0;
            col_cnt   <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) state <= LOAD;
                end
                
                LOAD: begin
                    // Read initial image in HEX format into the memory
                    $readmemh("image_rgb.hex", input_mem);
                    pix_count <= 0;
                    state     <= GRAY;
                end
                
                GRAY: begin
                    // Extract RGB channels 
                    // [23:16]=Red, [15:8]=Green, [7:0]=Blue
                    r_val = input_mem[pix_count][23:16];
                    g_val = input_mem[pix_count][15:8];
                    b_val = input_mem[pix_count][7:0];
                    
                    // Hardware friendly RGB -> Gray Conversion
                    // Math: gray = (306*R + 601*G + 117*B) / 1024
                    gray_full = (306 * r_val) + (601 * g_val) + (117 * b_val);
                    gray_mem[pix_count] = gray_full[17:10]; // Extracts integer part (division by 1024)
                    
                    if (pix_count == TOTAL_PIXELS - 1) begin
                        row_cnt <= 0;
                        col_cnt <= 0;
                        state   <= SOBEL;
                    end else begin
                        pix_count <= pix_count + 1;
                    end
                end
                
                SOBEL: begin
                    // Load the 3x3 window around current (row, col)
                    load_window(row_cnt, col_cnt);
                    
                    // Hardware Convolution: using shift '<< 1' instead of multiplying by 2
                    // Gx Kernel (Horizontal Gradient)
                    Gx = (P02 + (P12 << 1) + P22) - (P00 + (P10 << 1) + P20);
                    
                    // Gy Kernel (Vertical Gradient)
                    Gy = (P20 + (P21 << 1) + P22) - (P00 + (P01 << 1) + P02);
                    
                    // Calculate Absolute Values (Using 2's complement if MSB is 1)
                    abs_Gx = (Gx[10]) ? (~Gx + 1) : Gx;
                    abs_Gy = (Gy[10]) ? (~Gy + 1) : Gy;
                    
                    // Magnitude Approximation (Manhattan Distance instead of Euclidean)
                    magnitude = abs_Gx + abs_Gy;
                    
                    // Thresholding
                    edge_mem[row_cnt * IMG_WIDTH + col_cnt] = (magnitude > THRESHOLD) ? 1'b1 : 1'b0;
                    
                    // Iterate through image
                    if (col_cnt == IMG_WIDTH - 1) begin
                        col_cnt <= 0;
                        if (row_cnt == IMG_HEIGHT - 1) begin
                            state <= DUMP;
                        end else begin
                            row_cnt <= row_cnt + 1;
                        end
                    end else begin
                        col_cnt <= col_cnt + 1;
                    end
                end
                
                DUMP: begin
                     $writememb("edge_output.bin", edge_mem); // Optional verification
                    state <= DONE;
                end
                
                DONE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end

endmodule