`timescale 1ns / 1ps
`default_nettype none

module parking_lot_top (
    input  wire        clk,
    input  wire        reset_n,

    input  wire        ultrasonic_echo,
    output wire        ultrasonic_trig,

    output wire        entry_servo_pwm,
    output wire        exit_servo_pwm,
    output wire        buzzer,

    output wire        rc522_ss_n,
    output wire        rc522_sck,
    output wire        rc522_mosi,
    input  wire        rc522_miso,
    output wire        rc522_rst,

    inout  wire        lcd_sda,
    inout  wire        lcd_scl,

    output wire        matrix_din,
    output wire        matrix_cs,
    output wire        matrix_sclk,

    output wire [15:0] led
);

    localparam integer PARK_CAPACITY = 4;
    localparam [2:0]   PARK_CAPACITY_VALUE = 3'd4;

    wire [11:0] distance_cm;
    wire        distance_valid;
    wire        car_near;
    wire        ultrasonic_timeout;

    ultrasonic_park_sensor u_ultrasonic (
        .clk(clk),
        .reset_n(reset_n),
        .echo(ultrasonic_echo),
        .trig(ultrasonic_trig),
        .distance_cm(distance_cm),
        .distance_valid(distance_valid),
        .near_limit(car_near),
        .timeout(ultrasonic_timeout)
    );

    wire entry_idle;
    wire entry_busy;
    wire entry_opening;
    wire entry_closing;
    reg  entry_start;

    parking_servo_gate #(
        .CLOSED_PULSE_CYCLES(50_000),
        .OPEN_PULSE_CYCLES(150_000)
    ) u_entry_gate (
        .clk(clk),
        .reset_n(reset_n),
        .start(entry_start),
        .pwm(entry_servo_pwm),
        .idle(entry_idle),
        .busy(entry_busy),
        .opening(entry_opening),
        .closing(entry_closing)
    );

    wire exit_idle;
    wire exit_busy;
    wire exit_opening;
    wire exit_closing;
    reg  exit_start;

    parking_servo_gate #(
        .CLOSED_PULSE_CYCLES(50_000),
        .OPEN_PULSE_CYCLES(150_000)
    ) u_exit_gate (
        .clk(clk),
        .reset_n(reset_n),
        .start(exit_start),
        .pwm(exit_servo_pwm),
        .idle(exit_idle),
        .busy(exit_busy),
        .opening(exit_opening),
        .closing(exit_closing)
    );

    parking_buzzer u_buzzer (
        .clk(clk),
        .reset_n(reset_n),
        .enable(entry_closing | exit_closing),
        .buzzer(buzzer)
    );

    wire        card_present;
    wire        card_pulse;
    wire [15:0] rfid_debug;

    rc522_card_detector u_rc522 (
        .clk(clk),
        .reset_n(reset_n),
        .debug_leds(rfid_debug),
        .card_present(card_present),
        .card_pulse(card_pulse),
        .rc522_ss_n(rc522_ss_n),
        .rc522_sck(rc522_sck),
        .rc522_mosi(rc522_mosi),
        .rc522_miso(rc522_miso),
        .rc522_rst(rc522_rst)
    );

    reg [2:0] spaces_left;
    reg       car_armed;
    reg       card_armed;

    wire entry_request = car_near && car_armed && !entry_busy && (spaces_left != 3'd0);
    wire exit_request  = card_pulse && card_armed && !exit_busy;

    always @(posedge clk) begin
        if (!reset_n) begin
            spaces_left <= PARK_CAPACITY_VALUE;
            car_armed   <= 1'b1;
            card_armed  <= 1'b1;
            entry_start <= 1'b0;
            exit_start  <= 1'b0;
        end else begin
            entry_start <= entry_request;
            exit_start  <= exit_request;

            if (!car_near) begin
                car_armed <= 1'b1;
            end else if (entry_request) begin
                car_armed <= 1'b0;
            end

            if (!card_present) begin
                card_armed <= 1'b1;
            end else if (exit_request) begin
                card_armed <= 1'b0;
            end

            case ({entry_request, exit_request})
                2'b10: begin
                    spaces_left <= spaces_left - 3'd1;
                end
                2'b01: begin
                    if (spaces_left < PARK_CAPACITY_VALUE) begin
                        spaces_left <= spaces_left + 3'd1;
                    end
                end
                default: begin
                    spaces_left <= spaces_left;
                end
            endcase
        end
    end

    lcd_i2c_parking u_lcd (
        .clk(clk),
        .reset_n(reset_n),
        .spaces_left(spaces_left),
        .i2c_sda(lcd_sda),
        .i2c_scl(lcd_scl)
    );

    wire [1:0] matrix_mode =
        entry_opening ? 2'd1 :
        entry_closing ? 2'd2 :
        2'd0;

    max7219_parking_display u_matrix (
        .clk(clk),
        .reset_n(reset_n),
        .mode(matrix_mode),
        .din(matrix_din),
        .cs(matrix_cs),
        .sclk(matrix_sclk)
    );

    assign led[2:0]  = spaces_left;
    assign led[3]    = car_near;
    assign led[4]    = entry_busy;
    assign led[5]    = exit_busy;
    assign led[6]    = card_present;
    assign led[7]    = ultrasonic_timeout;
    assign led[15:8] = rfid_debug[15:8];

endmodule

module parking_servo_gate #(
    parameter integer CLK_FREQ            = 100_000_000,
    parameter integer PWM_PERIOD_CYCLES   = 2_000_000,
    parameter integer CLOSED_PULSE_CYCLES = 50_000,
    parameter integer OPEN_PULSE_CYCLES   = 150_000,
    parameter integer MOVE_CYCLES         = 50_000_000,
    parameter integer OPEN_HOLD_CYCLES    = 150_000_000
) (
    input  wire clk,
    input  wire reset_n,
    input  wire start,
    output reg  pwm,
    output wire idle,
    output wire busy,
    output wire opening,
    output wire closing
);

    localparam [1:0] S_CLOSED  = 2'd0;
    localparam [1:0] S_OPENING = 2'd1;
    localparam [1:0] S_HOLD    = 2'd2;
    localparam [1:0] S_CLOSING = 2'd3;

    reg [1:0]  state;
    reg [31:0] state_count;
    reg [20:0] pwm_count;

    wire [31:0] active_pulse =
        (state == S_OPENING || state == S_HOLD) ? OPEN_PULSE_CYCLES : CLOSED_PULSE_CYCLES;

    assign idle    = (state == S_CLOSED);
    assign busy    = (state != S_CLOSED);
    assign opening = (state == S_OPENING);
    assign closing = (state == S_CLOSING);

    always @(posedge clk) begin
        if (!reset_n) begin
            pwm_count <= 21'd0;
            pwm       <= 1'b0;
        end else begin
            if (pwm_count >= PWM_PERIOD_CYCLES - 1) begin
                pwm_count <= 21'd0;
            end else begin
                pwm_count <= pwm_count + 21'd1;
            end

            pwm <= (pwm_count < active_pulse[20:0]);
        end
    end

    always @(posedge clk) begin
        if (!reset_n) begin
            state       <= S_CLOSED;
            state_count <= 32'd0;
        end else begin
            case (state)
                S_CLOSED: begin
                    state_count <= 32'd0;
                    if (start) begin
                        state <= S_OPENING;
                    end
                end

                S_OPENING: begin
                    if (state_count >= MOVE_CYCLES - 1) begin
                        state       <= S_HOLD;
                        state_count <= 32'd0;
                    end else begin
                        state_count <= state_count + 32'd1;
                    end
                end

                S_HOLD: begin
                    if (state_count >= OPEN_HOLD_CYCLES - 1) begin
                        state       <= S_CLOSING;
                        state_count <= 32'd0;
                    end else begin
                        state_count <= state_count + 32'd1;
                    end
                end

                S_CLOSING: begin
                    if (state_count >= MOVE_CYCLES - 1) begin
                        state       <= S_CLOSED;
                        state_count <= 32'd0;
                    end else begin
                        state_count <= state_count + 32'd1;
                    end
                end

                default: begin
                    state       <= S_CLOSED;
                    state_count <= 32'd0;
                end
            endcase
        end
    end

endmodule

module parking_buzzer (
    input  wire clk,
    input  wire reset_n,
    input  wire enable,
    output reg  buzzer
);

    always @(posedge clk) begin
        if (!reset_n) begin
            buzzer <= 1'b0;
        end else begin
            buzzer <= enable;
        end
    end

endmodule

module ultrasonic_park_sensor #(
    parameter integer CLK_FREQ       = 100_000_000,
    parameter integer TRIG_INTERVAL  = 6_000_000,
    parameter integer TRIG_PULSE     = 1_000,
    parameter integer CYCLES_PER_CM  = 5_800,
    parameter integer TIMEOUT_CYCLES = 3_000_000,
    parameter integer NEAR_CM        = 15
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        echo,
    output reg         trig,
    output reg  [11:0] distance_cm,
    output reg         distance_valid,
    output reg         near_limit,
    output reg         timeout
);

    reg [22:0] trig_count;
    reg        echo_meta;
    reg        echo_sync;
    reg        echo_prev;
    reg        measuring;
    reg [21:0] echo_count;
    reg [1:0]  near_hits;
    reg [1:0]  far_hits;

    wire echo_rise = echo_sync && !echo_prev;
    wire echo_fall = !echo_sync && echo_prev;
    wire [11:0] measured_cm = echo_count / CYCLES_PER_CM;

    always @(posedge clk) begin
        if (!reset_n) begin
            trig_count <= 23'd0;
            trig       <= 1'b0;
        end else begin
            if (trig_count >= TRIG_INTERVAL - 1) begin
                trig_count <= 23'd0;
            end else begin
                trig_count <= trig_count + 23'd1;
            end

            trig <= (trig_count < TRIG_PULSE);
        end
    end

    always @(posedge clk) begin
        if (!reset_n) begin
            echo_meta      <= 1'b0;
            echo_sync      <= 1'b0;
            echo_prev      <= 1'b0;
            measuring      <= 1'b0;
            echo_count     <= 22'd0;
            distance_cm    <= 12'd999;
            distance_valid <= 1'b0;
            near_limit     <= 1'b0;
            timeout        <= 1'b0;
            near_hits      <= 2'd0;
            far_hits       <= 2'd0;
        end else begin
            echo_meta      <= echo;
            echo_sync      <= echo_meta;
            echo_prev      <= echo_sync;
            distance_valid <= 1'b0;

            if (echo_rise) begin
                measuring  <= 1'b1;
                echo_count <= 22'd0;
                timeout    <= 1'b0;
            end else if (measuring && echo_sync) begin
                if (echo_count >= TIMEOUT_CYCLES) begin
                    measuring      <= 1'b0;
                    echo_count     <= 22'd0;
                    distance_cm    <= 12'd999;
                    distance_valid <= 1'b1;
                    timeout        <= 1'b1;
                    near_hits      <= 2'd0;
                    if (far_hits != 2'd3) begin
                        far_hits <= far_hits + 2'd1;
                    end
                    if (far_hits >= 2'd1) begin
                        near_limit <= 1'b0;
                    end
                end else begin
                    echo_count <= echo_count + 22'd1;
                end
            end

            if (measuring && echo_fall) begin
                measuring      <= 1'b0;
                echo_count     <= 22'd0;
                distance_cm    <= measured_cm;
                distance_valid <= 1'b1;
                timeout        <= 1'b0;

                if (measured_cm <= NEAR_CM) begin
                    far_hits <= 2'd0;
                    if (near_hits != 2'd3) begin
                        near_hits <= near_hits + 2'd1;
                    end
                    if (near_hits >= 2'd1) begin
                        near_limit <= 1'b1;
                    end
                end else begin
                    near_hits <= 2'd0;
                    if (far_hits != 2'd3) begin
                        far_hits <= far_hits + 2'd1;
                    end
                    if (far_hits >= 2'd1) begin
                        near_limit <= 1'b0;
                    end
                end
            end
        end
    end

endmodule

module max7219_parking_display (
    input  wire       clk,
    input  wire       reset_n,
    input  wire [1:0] mode,
    output reg        din,
    output reg        cs,
    output reg        sclk
);

    reg [15:0] clk_div;
    reg        spi_tick;

    always @(posedge clk) begin
        if (!reset_n) begin
            clk_div  <= 16'd0;
            spi_tick <= 1'b0;
        end else if (clk_div == 16'd499) begin
            clk_div  <= 16'd0;
            spi_tick <= 1'b1;
        end else begin
            clk_div  <= clk_div + 16'd1;
            spi_tick <= 1'b0;
        end
    end

    function [7:0] matrix_row;
        input [1:0] display_mode;
        input [3:0] row;
        begin
            case (display_mode)
                2'd1: begin
                    case (row)
                        4'd1: matrix_row = 8'b00011000;
                        4'd2: matrix_row = 8'b00111100;
                        4'd3: matrix_row = 8'b01111110;
                        4'd4: matrix_row = 8'b00011000;
                        4'd5: matrix_row = 8'b00011000;
                        4'd6: matrix_row = 8'b00011000;
                        4'd7: matrix_row = 8'b00011000;
                        4'd8: matrix_row = 8'b00011000;
                        default: matrix_row = 8'h00;
                    endcase
                end

                2'd2: begin
                    case (row)
                        4'd1: matrix_row = 8'b00011000;
                        4'd2: matrix_row = 8'b00011000;
                        4'd3: matrix_row = 8'b00011000;
                        4'd4: matrix_row = 8'b00011000;
                        4'd5: matrix_row = 8'b00011000;
                        4'd6: matrix_row = 8'b01111110;
                        4'd7: matrix_row = 8'b00111100;
                        4'd8: matrix_row = 8'b00011000;
                        default: matrix_row = 8'h00;
                    endcase
                end

                default: begin
                    case (row)
                        4'd1: matrix_row = 8'b11111000;
                        4'd2: matrix_row = 8'b10000100;
                        4'd3: matrix_row = 8'b10000100;
                        4'd4: matrix_row = 8'b11111000;
                        4'd5: matrix_row = 8'b10000000;
                        4'd6: matrix_row = 8'b10000000;
                        4'd7: matrix_row = 8'b10000000;
                        4'd8: matrix_row = 8'b10000000;
                        default: matrix_row = 8'h00;
                    endcase
                end
            endcase
        end
    endfunction

    function [15:0] matrix_command;
        input [3:0] index;
        input [1:0] display_mode;
        reg [3:0] row_addr;
        begin
            case (index)
                4'd0: matrix_command = 16'h0900;
                4'd1: matrix_command = 16'h0A03;
                4'd2: matrix_command = 16'h0B07;
                4'd3: matrix_command = 16'h0C01;
                4'd4: matrix_command = 16'h0F00;
                default: begin
                    row_addr = index - 4'd4;
                    matrix_command = {row_addr, matrix_row(display_mode, row_addr)};
                end
            endcase
        end
    endfunction

    localparam [2:0] M_IDLE     = 3'd0;
    localparam [2:0] M_LOAD_CMD = 3'd1;
    localparam [2:0] M_CLK_LOW  = 3'd2;
    localparam [2:0] M_CLK_HIGH = 3'd3;
    localparam [2:0] M_NEXT_BIT = 3'd4;
    localparam [2:0] M_LATCH    = 3'd5;
    localparam [2:0] M_NEXT_CMD = 3'd6;
    localparam [2:0] M_DONE     = 3'd7;

    reg [2:0]  state;
    reg [3:0]  cmd_index;
    reg [4:0]  bit_index;
    reg [15:0] shift_reg;
    reg [1:0]  latched_mode;

    always @(posedge clk) begin
        if (!reset_n) begin
            state        <= M_IDLE;
            cmd_index    <= 4'd0;
            bit_index    <= 5'd0;
            shift_reg    <= 16'd0;
            latched_mode <= 2'd3;
            din          <= 1'b0;
            cs           <= 1'b1;
            sclk         <= 1'b0;
        end else if (spi_tick) begin
            case (state)
                M_IDLE: begin
                    din          <= 1'b0;
                    cs           <= 1'b1;
                    sclk         <= 1'b0;
                    cmd_index    <= 4'd0;
                    latched_mode <= mode;
                    state        <= M_LOAD_CMD;
                end

                M_LOAD_CMD: begin
                    shift_reg <= matrix_command(cmd_index, latched_mode);
                    bit_index <= 5'd15;
                    cs        <= 1'b0;
                    sclk      <= 1'b0;
                    state     <= M_CLK_LOW;
                end

                M_CLK_LOW: begin
                    sclk  <= 1'b0;
                    din   <= shift_reg[bit_index];
                    state <= M_CLK_HIGH;
                end

                M_CLK_HIGH: begin
                    sclk  <= 1'b1;
                    state <= M_NEXT_BIT;
                end

                M_NEXT_BIT: begin
                    sclk <= 1'b0;
                    if (bit_index == 5'd0) begin
                        state <= M_LATCH;
                    end else begin
                        bit_index <= bit_index - 5'd1;
                        state     <= M_CLK_LOW;
                    end
                end

                M_LATCH: begin
                    cs    <= 1'b1;
                    state <= M_NEXT_CMD;
                end

                M_NEXT_CMD: begin
                    if (cmd_index == 4'd12) begin
                        state <= M_DONE;
                    end else begin
                        cmd_index <= cmd_index + 4'd1;
                        state     <= M_LOAD_CMD;
                    end
                end

                M_DONE: begin
                    cs   <= 1'b1;
                    sclk <= 1'b0;
                    din  <= 1'b0;
                    if (mode != latched_mode) begin
                        cmd_index    <= 4'd0;
                        latched_mode <= mode;
                        state        <= M_LOAD_CMD;
                    end
                end

                default: begin
                    state <= M_IDLE;
                end
            endcase
        end
    end

endmodule

module lcd_i2c_parking #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer I2C_FREQ = 100_000,
    parameter [6:0]   LCD_ADDR = 7'h27
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire [2:0] spaces_left,
    inout  wire       i2c_sda,
    inout  wire       i2c_scl
);

    localparam BL = 1'b1;
    localparam integer DIVIDER = CLK_FREQ / (I2C_FREQ * 4);

    reg sda_low;
    reg scl_low;

    assign i2c_sda = sda_low ? 1'b0 : 1'bz;
    assign i2c_scl = scl_low ? 1'b0 : 1'bz;

    reg [15:0] div_cnt;
    reg        i2c_tick;

    always @(posedge clk) begin
        if (!reset_n) begin
            div_cnt  <= 16'd0;
            i2c_tick <= 1'b0;
        end else if (div_cnt == DIVIDER - 1) begin
            div_cnt  <= 16'd0;
            i2c_tick <= 1'b1;
        end else begin
            div_cnt  <= div_cnt + 16'd1;
            i2c_tick <= 1'b0;
        end
    end

    reg        pcf_req;
    reg [7:0]  pcf_data;
    reg        pcf_busy;
    reg        pcf_done;

    reg [7:0]  tx_data;
    reg [7:0]  tx_byte;
    reg [3:0]  bit_cnt;
    reg [5:0]  i2c_state;

    localparam [5:0] I_IDLE       = 6'd0;
    localparam [5:0] I_START_1    = 6'd1;
    localparam [5:0] I_START_2    = 6'd2;
    localparam [5:0] I_ADDR_BIT_0 = 6'd3;
    localparam [5:0] I_ADDR_BIT_1 = 6'd4;
    localparam [5:0] I_ADDR_BIT_2 = 6'd5;
    localparam [5:0] I_ADDR_ACK_0 = 6'd6;
    localparam [5:0] I_ADDR_ACK_1 = 6'd7;
    localparam [5:0] I_ADDR_ACK_2 = 6'd8;
    localparam [5:0] I_DATA_BIT_0 = 6'd9;
    localparam [5:0] I_DATA_BIT_1 = 6'd10;
    localparam [5:0] I_DATA_BIT_2 = 6'd11;
    localparam [5:0] I_DATA_ACK_0 = 6'd12;
    localparam [5:0] I_DATA_ACK_1 = 6'd13;
    localparam [5:0] I_DATA_ACK_2 = 6'd14;
    localparam [5:0] I_STOP_1     = 6'd15;
    localparam [5:0] I_STOP_2     = 6'd16;
    localparam [5:0] I_DONE       = 6'd17;

    always @(posedge clk) begin
        if (!reset_n) begin
            i2c_state <= I_IDLE;
            pcf_busy  <= 1'b0;
            pcf_done  <= 1'b0;
            sda_low   <= 1'b0;
            scl_low   <= 1'b0;
            tx_data   <= 8'd0;
            tx_byte   <= 8'd0;
            bit_cnt   <= 4'd0;
        end else begin
            pcf_done <= 1'b0;

            if (i2c_state == I_IDLE) begin
                sda_low  <= 1'b0;
                scl_low  <= 1'b0;
                pcf_busy <= 1'b0;

                if (pcf_req) begin
                    pcf_busy  <= 1'b1;
                    tx_data   <= pcf_data;
                    tx_byte   <= {LCD_ADDR, 1'b0};
                    bit_cnt   <= 4'd7;
                    i2c_state <= I_START_1;
                end
            end else if (i2c_tick) begin
                case (i2c_state)
                    I_START_1: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b1;
                        i2c_state <= I_START_2;
                    end

                    I_START_2: begin
                        scl_low   <= 1'b1;
                        i2c_state <= I_ADDR_BIT_0;
                    end

                    I_ADDR_BIT_0: begin
                        scl_low   <= 1'b1;
                        sda_low   <= ~tx_byte[bit_cnt];
                        i2c_state <= I_ADDR_BIT_1;
                    end

                    I_ADDR_BIT_1: begin
                        scl_low   <= 1'b0;
                        i2c_state <= I_ADDR_BIT_2;
                    end

                    I_ADDR_BIT_2: begin
                        scl_low <= 1'b1;
                        if (bit_cnt == 4'd0) begin
                            i2c_state <= I_ADDR_ACK_0;
                        end else begin
                            bit_cnt   <= bit_cnt - 4'd1;
                            i2c_state <= I_ADDR_BIT_0;
                        end
                    end

                    I_ADDR_ACK_0: begin
                        sda_low   <= 1'b0;
                        scl_low   <= 1'b1;
                        i2c_state <= I_ADDR_ACK_1;
                    end

                    I_ADDR_ACK_1: begin
                        scl_low   <= 1'b0;
                        i2c_state <= I_ADDR_ACK_2;
                    end

                    I_ADDR_ACK_2: begin
                        scl_low   <= 1'b1;
                        tx_byte   <= tx_data;
                        bit_cnt   <= 4'd7;
                        i2c_state <= I_DATA_BIT_0;
                    end

                    I_DATA_BIT_0: begin
                        scl_low   <= 1'b1;
                        sda_low   <= ~tx_byte[bit_cnt];
                        i2c_state <= I_DATA_BIT_1;
                    end

                    I_DATA_BIT_1: begin
                        scl_low   <= 1'b0;
                        i2c_state <= I_DATA_BIT_2;
                    end

                    I_DATA_BIT_2: begin
                        scl_low <= 1'b1;
                        if (bit_cnt == 4'd0) begin
                            i2c_state <= I_DATA_ACK_0;
                        end else begin
                            bit_cnt   <= bit_cnt - 4'd1;
                            i2c_state <= I_DATA_BIT_0;
                        end
                    end

                    I_DATA_ACK_0: begin
                        sda_low   <= 1'b0;
                        scl_low   <= 1'b1;
                        i2c_state <= I_DATA_ACK_1;
                    end

                    I_DATA_ACK_1: begin
                        scl_low   <= 1'b0;
                        i2c_state <= I_DATA_ACK_2;
                    end

                    I_DATA_ACK_2: begin
                        scl_low   <= 1'b1;
                        i2c_state <= I_STOP_1;
                    end

                    I_STOP_1: begin
                        sda_low   <= 1'b1;
                        scl_low   <= 1'b0;
                        i2c_state <= I_STOP_2;
                    end

                    I_STOP_2: begin
                        sda_low   <= 1'b0;
                        scl_low   <= 1'b0;
                        i2c_state <= I_DONE;
                    end

                    I_DONE: begin
                        pcf_done  <= 1'b1;
                        pcf_busy  <= 1'b0;
                        i2c_state <= I_IDLE;
                    end

                    default: begin
                        i2c_state <= I_IDLE;
                    end
                endcase
            end
        end
    end

    reg        lcd_req;
    reg [7:0]  lcd_byte;
    reg        lcd_rs;
    reg        lcd_busy;
    reg        lcd_done;

    reg [5:0]  lcd_state;
    reg [7:0]  high_en;
    reg [7:0]  high_dis;
    reg [7:0]  low_en;
    reg [7:0]  low_dis;
    reg [31:0] wait_cnt;

    localparam [5:0] L_IDLE       = 6'd0;
    localparam [5:0] L_SEND_H_EN  = 6'd1;
    localparam [5:0] L_WAIT_H_EN  = 6'd2;
    localparam [5:0] L_SEND_H_DIS = 6'd3;
    localparam [5:0] L_WAIT_H_DIS = 6'd4;
    localparam [5:0] L_SEND_L_EN  = 6'd5;
    localparam [5:0] L_WAIT_L_EN  = 6'd6;
    localparam [5:0] L_SEND_L_DIS = 6'd7;
    localparam [5:0] L_WAIT_L_DIS = 6'd8;
    localparam [5:0] L_DELAY      = 6'd9;
    localparam [5:0] L_DONE       = 6'd10;

    function [7:0] make_pcf;
        input [3:0] nibble;
        input       rs;
        input       en;
        begin
            make_pcf = {nibble[3], nibble[2], nibble[1], nibble[0], BL, en, 1'b0, rs};
        end
    endfunction

    always @(posedge clk) begin
        if (!reset_n) begin
            lcd_state <= L_IDLE;
            lcd_busy  <= 1'b0;
            lcd_done  <= 1'b0;
            wait_cnt  <= 32'd0;
            pcf_req   <= 1'b0;
            pcf_data  <= 8'd0;
            high_en   <= 8'd0;
            high_dis  <= 8'd0;
            low_en    <= 8'd0;
            low_dis   <= 8'd0;
        end else begin
            lcd_done <= 1'b0;
            pcf_req  <= 1'b0;

            case (lcd_state)
                L_IDLE: begin
                    lcd_busy <= 1'b0;

                    if (lcd_req) begin
                        lcd_busy <= 1'b1;

                        high_en  <= make_pcf(lcd_byte[7:4], lcd_rs, 1'b1);
                        high_dis <= make_pcf(lcd_byte[7:4], lcd_rs, 1'b0);
                        low_en   <= make_pcf(lcd_byte[3:0], lcd_rs, 1'b1);
                        low_dis  <= make_pcf(lcd_byte[3:0], lcd_rs, 1'b0);

                        lcd_state <= L_SEND_H_EN;
                    end
                end

                L_SEND_H_EN: begin
                    if (!pcf_busy) begin
                        pcf_data  <= high_en;
                        pcf_req   <= 1'b1;
                        lcd_state <= L_WAIT_H_EN;
                    end
                end

                L_WAIT_H_EN: begin
                    if (pcf_done) begin
                        lcd_state <= L_SEND_H_DIS;
                    end
                end

                L_SEND_H_DIS: begin
                    if (!pcf_busy) begin
                        pcf_data  <= high_dis;
                        pcf_req   <= 1'b1;
                        lcd_state <= L_WAIT_H_DIS;
                    end
                end

                L_WAIT_H_DIS: begin
                    if (pcf_done) begin
                        lcd_state <= L_SEND_L_EN;
                    end
                end

                L_SEND_L_EN: begin
                    if (!pcf_busy) begin
                        pcf_data  <= low_en;
                        pcf_req   <= 1'b1;
                        lcd_state <= L_WAIT_L_EN;
                    end
                end

                L_WAIT_L_EN: begin
                    if (pcf_done) begin
                        lcd_state <= L_SEND_L_DIS;
                    end
                end

                L_SEND_L_DIS: begin
                    if (!pcf_busy) begin
                        pcf_data  <= low_dis;
                        pcf_req   <= 1'b1;
                        lcd_state <= L_WAIT_L_DIS;
                    end
                end

                L_WAIT_L_DIS: begin
                    if (pcf_done) begin
                        wait_cnt  <= 32'd0;
                        lcd_state <= L_DELAY;
                    end
                end

                L_DELAY: begin
                    if (wait_cnt >= 32'd300_000) begin
                        lcd_state <= L_DONE;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                L_DONE: begin
                    lcd_done  <= 1'b1;
                    lcd_busy  <= 1'b0;
                    lcd_state <= L_IDLE;
                end

                default: begin
                    lcd_state <= L_IDLE;
                end
            endcase
        end
    end

    function [7:0] line1_char;
        input [4:0] i;
        begin
            case (i)
                5'd0:  line1_char = "W";
                5'd1:  line1_char = "E";
                5'd2:  line1_char = "L";
                5'd3:  line1_char = "C";
                5'd4:  line1_char = "O";
                5'd5:  line1_char = "M";
                5'd6:  line1_char = "E";
                default: line1_char = " ";
            endcase
        end
    endfunction

    function [7:0] line2_char;
        input [4:0] i;
        input [2:0] spaces;
        begin
            case (i)
                5'd0:  line2_char = "S";
                5'd1:  line2_char = "P";
                5'd2:  line2_char = "A";
                5'd3:  line2_char = "C";
                5'd4:  line2_char = "E";
                5'd5:  line2_char = " ";
                5'd6:  line2_char = "L";
                5'd7:  line2_char = "E";
                5'd8:  line2_char = "F";
                5'd9:  line2_char = "T";
                5'd10: line2_char = ":";
                5'd11: line2_char = " ";
                5'd12: line2_char = 8'h30 + {5'd0, spaces};
                default: line2_char = " ";
            endcase
        end
    endfunction

    reg [7:0]  main_state;
    reg [31:0] delay_cnt;
    reg [26:0] refresh_cnt;
    reg [4:0]  char_idx;
    reg [2:0]  last_spaces;

    localparam [7:0] S_POWER_DELAY = 8'd0;
    localparam [7:0] S_INIT_33     = 8'd1;
    localparam [7:0] S_INIT_32     = 8'd2;
    localparam [7:0] S_FUNC        = 8'd3;
    localparam [7:0] S_DISPLAY     = 8'd4;
    localparam [7:0] S_CLEAR       = 8'd5;
    localparam [7:0] S_ENTRY       = 8'd6;
    localparam [7:0] S_LINE1_ADDR  = 8'd7;
    localparam [7:0] S_LINE1_WRITE = 8'd8;
    localparam [7:0] S_LINE2_ADDR  = 8'd9;
    localparam [7:0] S_LINE2_WRITE = 8'd10;
    localparam [7:0] S_IDLE        = 8'd11;

    always @(posedge clk) begin
        if (!reset_n) begin
            lcd_req      <= 1'b0;
            lcd_byte     <= 8'd0;
            lcd_rs       <= 1'b0;
            main_state   <= S_POWER_DELAY;
            delay_cnt    <= 32'd0;
            refresh_cnt  <= 27'd0;
            char_idx     <= 5'd0;
            last_spaces  <= 3'd7;
        end else begin
            lcd_req <= 1'b0;

            case (main_state)
                S_POWER_DELAY: begin
                    if (delay_cnt >= 32'd5_000_000) begin
                        delay_cnt  <= 32'd0;
                        main_state <= S_INIT_33;
                    end else begin
                        delay_cnt <= delay_cnt + 32'd1;
                    end
                end

                S_INIT_33: begin
                    if (!lcd_busy) begin
                        lcd_byte   <= 8'h33;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        main_state <= S_INIT_32;
                    end
                end

                S_INIT_32: begin
                    if (lcd_done) begin
                        lcd_byte   <= 8'h32;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        main_state <= S_FUNC;
                    end
                end

                S_FUNC: begin
                    if (lcd_done) begin
                        lcd_byte   <= 8'h28;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        main_state <= S_DISPLAY;
                    end
                end

                S_DISPLAY: begin
                    if (lcd_done) begin
                        lcd_byte   <= 8'h0C;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        main_state <= S_CLEAR;
                    end
                end

                S_CLEAR: begin
                    if (lcd_done) begin
                        lcd_byte   <= 8'h01;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        main_state <= S_ENTRY;
                    end
                end

                S_ENTRY: begin
                    if (lcd_done) begin
                        lcd_byte   <= 8'h06;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        main_state <= S_LINE1_ADDR;
                    end
                end

                S_LINE1_ADDR: begin
                    if (lcd_done) begin
                        lcd_byte   <= 8'h80;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        char_idx   <= 5'd0;
                        main_state <= S_LINE1_WRITE;
                    end
                end

                S_LINE1_WRITE: begin
                    if (lcd_done) begin
                        if (char_idx < 5'd16) begin
                            lcd_byte <= line1_char(char_idx);
                            lcd_rs   <= 1'b1;
                            lcd_req  <= 1'b1;
                            char_idx <= char_idx + 5'd1;
                        end else begin
                            main_state <= S_LINE2_ADDR;
                        end
                    end
                end

                S_LINE2_ADDR: begin
                    if (!lcd_busy) begin
                        lcd_byte   <= 8'hC0;
                        lcd_rs     <= 1'b0;
                        lcd_req    <= 1'b1;
                        char_idx   <= 5'd0;
                        main_state <= S_LINE2_WRITE;
                    end
                end

                S_LINE2_WRITE: begin
                    if (lcd_done) begin
                        if (char_idx < 5'd16) begin
                            lcd_byte <= line2_char(char_idx, spaces_left);
                            lcd_rs   <= 1'b1;
                            lcd_req  <= 1'b1;
                            char_idx <= char_idx + 5'd1;
                        end else begin
                            last_spaces <= spaces_left;
                            refresh_cnt <= 27'd0;
                            main_state  <= S_IDLE;
                        end
                    end
                end

                S_IDLE: begin
                    if (refresh_cnt >= 27'd100_000_000 || spaces_left != last_spaces) begin
                        refresh_cnt <= 27'd0;
                        main_state  <= S_LINE2_ADDR;
                    end else begin
                        refresh_cnt <= refresh_cnt + 27'd1;
                    end
                end

                default: begin
                    main_state <= S_POWER_DELAY;
                end
            endcase
        end
    end

endmodule

module rc522_card_detector (
    input  wire        clk,
    input  wire        reset_n,

    output wire [15:0] debug_leds,
    output reg         card_present,
    output reg         card_pulse,

    output reg         rc522_ss_n,
    output reg         rc522_sck,
    output wire        rc522_mosi,
    input  wire        rc522_miso,
    output reg         rc522_rst
);

    localparam [7:0] CommandReg      = 8'h01;
    localparam [7:0] ComIrqReg       = 8'h04;
    localparam [7:0] ErrorReg        = 8'h06;
    localparam [7:0] FIFODataReg     = 8'h09;
    localparam [7:0] FIFOLevelReg    = 8'h0A;
    localparam [7:0] BitFramingReg   = 8'h0D;
    localparam [7:0] ModeReg         = 8'h11;
    localparam [7:0] TxControlReg    = 8'h14;
    localparam [7:0] TxASKReg        = 8'h15;
    localparam [7:0] TModeReg        = 8'h2A;
    localparam [7:0] TPrescalerReg   = 8'h2B;
    localparam [7:0] TReloadRegH     = 8'h2C;
    localparam [7:0] TReloadRegL     = 8'h2D;
    localparam [7:0] VersionReg      = 8'h37;

    localparam [7:0] PCD_IDLE        = 8'h00;
    localparam [7:0] PCD_TRANSCEIVE  = 8'h0C;
    localparam [7:0] PICC_REQA       = 8'h26;

    localparam integer SPI_HALF_PERIOD    = 100;
    localparam integer RESET_LOW_CYCLES   = 100_000;
    localparam integer RESET_WAIT_CYCLES  = 1_000_000;
    localparam integer POLL_DELAY_CYCLES  = 10_000_000;
    localparam integer IRQ_POLL_LIMIT     = 3000;

    reg [7:0] version;
    reg       version_ok;
    reg       init_done;
    reg       antenna_ok;
    reg       error_flag;
    reg       poll_toggle;
    reg       heartbeat;
    reg       rx_irq_seen;

    assign debug_leds[7:0]  = version;
    assign debug_leds[8]    = version_ok;
    assign debug_leds[9]    = init_done;
    assign debug_leds[10]   = antenna_ok;
    assign debug_leds[11]   = card_present;
    assign debug_leds[12]   = error_flag;
    assign debug_leds[13]   = poll_toggle;
    assign debug_leds[14]   = heartbeat;
    assign debug_leds[15]   = rx_irq_seen;

    reg [31:0] heartbeat_cnt;

    always @(posedge clk) begin
        if (!reset_n) begin
            heartbeat_cnt <= 32'd0;
            heartbeat     <= 1'b0;
        end else if (heartbeat_cnt >= 32'd49_999_999) begin
            heartbeat_cnt <= 32'd0;
            heartbeat     <= ~heartbeat;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 32'd1;
        end
    end

    reg        spi_start;
    reg        spi_busy;
    reg        spi_done;
    reg        spi_read;
    reg [7:0]  spi_addr;
    reg [7:0]  spi_wdata;
    reg [7:0]  spi_rdata;
    reg [15:0] spi_tx_shift;
    reg [15:0] spi_rx_shift;
    reg [4:0]  spi_bit_count;
    reg [31:0] spi_div_count;

    assign rc522_mosi = spi_tx_shift[15];

    wire [7:0] spi_addr_byte =
        (((spi_addr << 1) & 8'h7E) | (spi_read ? 8'h80 : 8'h00));

    always @(posedge clk) begin
        if (!reset_n) begin
            spi_busy      <= 1'b0;
            spi_done      <= 1'b0;
            spi_tx_shift  <= 16'd0;
            spi_rx_shift  <= 16'd0;
            spi_bit_count <= 5'd0;
            spi_div_count <= 32'd0;
            spi_rdata     <= 8'd0;
            rc522_ss_n    <= 1'b1;
            rc522_sck     <= 1'b0;
        end else begin
            spi_done <= 1'b0;

            if (spi_start && !spi_busy) begin
                spi_busy      <= 1'b1;
                spi_done      <= 1'b0;
                spi_bit_count <= 5'd0;
                spi_div_count <= 32'd0;
                spi_rx_shift  <= 16'd0;
                rc522_ss_n    <= 1'b0;
                rc522_sck     <= 1'b0;

                if (spi_read) begin
                    spi_tx_shift <= {spi_addr_byte, 8'h00};
                end else begin
                    spi_tx_shift <= {spi_addr_byte, spi_wdata};
                end
            end else if (spi_busy) begin
                if (spi_div_count >= SPI_HALF_PERIOD - 1) begin
                    spi_div_count <= 32'd0;

                    if (rc522_sck == 1'b0) begin
                        rc522_sck    <= 1'b1;
                        spi_rx_shift <= {spi_rx_shift[14:0], rc522_miso};
                    end else begin
                        rc522_sck    <= 1'b0;
                        spi_tx_shift <= {spi_tx_shift[14:0], 1'b0};

                        if (spi_bit_count == 5'd15) begin
                            spi_rdata  <= spi_rx_shift[7:0];
                            rc522_ss_n <= 1'b1;
                            rc522_sck  <= 1'b0;
                            spi_busy   <= 1'b0;
                            spi_done   <= 1'b1;
                        end else begin
                            spi_bit_count <= spi_bit_count + 5'd1;
                        end
                    end
                end else begin
                    spi_div_count <= spi_div_count + 32'd1;
                end
            end
        end
    end

    localparam [7:0]
        S_RESET_LOW       = 8'd0,
        S_RESET_WAIT      = 8'd1,
        S_PREP_OP         = 8'd2,
        S_START_OP        = 8'd3,
        S_WAIT_OP         = 8'd4,
        S_READ_VERSION    = 8'd10,
        S_AFTER_VERSION   = 8'd11,
        S_INIT_1          = 8'd20,
        S_INIT_2          = 8'd21,
        S_INIT_3          = 8'd22,
        S_INIT_4          = 8'd23,
        S_INIT_5          = 8'd24,
        S_INIT_6          = 8'd25,
        S_READ_TXCTRL     = 8'd26,
        S_WRITE_TXCTRL    = 8'd27,
        S_CONFIRM_TXCTRL  = 8'd28,
        S_INIT_DONE       = 8'd29,
        S_POLL_DELAY      = 8'd40,
        S_REQA_BF_7       = 8'd50,
        S_REQA_IDLE       = 8'd51,
        S_REQA_IRQ_CLR    = 8'd52,
        S_REQA_FIFO_CLR   = 8'd53,
        S_REQA_FIFO_DATA  = 8'd54,
        S_REQA_TRANSCEIVE = 8'd55,
        S_REQA_STARTSEND  = 8'd56,
        S_REQA_READ_IRQ   = 8'd57,
        S_REQA_CHECK_IRQ  = 8'd58,
        S_REQA_CLEARSTART = 8'd59,
        S_REQA_READ_ERR   = 8'd60,
        S_REQA_CHECK_ERR  = 8'd61,
        S_REQA_READ_LEVEL = 8'd62,
        S_REQA_CHECK_LVL  = 8'd63,
        S_REQA_DRAIN_1    = 8'd64,
        S_REQA_DRAIN_2    = 8'd65;

    reg [7:0]  state;
    reg [7:0]  return_state;
    reg [31:0] wait_count;
    reg [15:0] irq_poll_count;
    reg [7:0]  last_read;

    reg        req_read;
    reg [7:0]  req_addr;
    reg [7:0]  req_wdata;
    reg [7:0]  req_return_state;

    always @(posedge clk) begin
        if (!reset_n) begin
            state            <= S_RESET_LOW;
            return_state     <= S_RESET_LOW;
            wait_count       <= 32'd0;
            irq_poll_count   <= 16'd0;
            last_read        <= 8'd0;
            spi_start        <= 1'b0;
            spi_read         <= 1'b0;
            spi_addr         <= 8'd0;
            spi_wdata        <= 8'd0;
            req_read         <= 1'b0;
            req_addr         <= 8'd0;
            req_wdata        <= 8'd0;
            req_return_state <= S_RESET_LOW;
            rc522_rst        <= 1'b0;
            version          <= 8'd0;
            version_ok       <= 1'b0;
            init_done        <= 1'b0;
            antenna_ok       <= 1'b0;
            card_present     <= 1'b0;
            card_pulse       <= 1'b0;
            error_flag       <= 1'b0;
            poll_toggle      <= 1'b0;
            rx_irq_seen      <= 1'b0;
        end else begin
            spi_start  <= 1'b0;
            card_pulse <= 1'b0;

            case (state)
                S_RESET_LOW: begin
                    rc522_rst <= 1'b0;
                    if (wait_count >= RESET_LOW_CYCLES) begin
                        wait_count <= 32'd0;
                        state      <= S_RESET_WAIT;
                    end else begin
                        wait_count <= wait_count + 32'd1;
                    end
                end

                S_RESET_WAIT: begin
                    rc522_rst <= 1'b1;
                    if (wait_count >= RESET_WAIT_CYCLES) begin
                        wait_count <= 32'd0;
                        state      <= S_READ_VERSION;
                    end else begin
                        wait_count <= wait_count + 32'd1;
                    end
                end

                S_PREP_OP: begin
                    spi_read     <= req_read;
                    spi_addr     <= req_addr;
                    spi_wdata    <= req_wdata;
                    return_state <= req_return_state;
                    state        <= S_START_OP;
                end

                S_START_OP: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        state     <= S_WAIT_OP;
                    end
                end

                S_WAIT_OP: begin
                    if (spi_done) begin
                        last_read <= spi_rdata;
                        state     <= return_state;
                    end
                end

                S_READ_VERSION: begin
                    req_read         <= 1'b1;
                    req_addr         <= VersionReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_AFTER_VERSION;
                    state            <= S_PREP_OP;
                end

                S_AFTER_VERSION: begin
                    version <= last_read;
                    if ((last_read == 8'h91) || (last_read == 8'h92) || (last_read == 8'hB2)) begin
                        version_ok <= 1'b1;
                        error_flag <= 1'b0;
                        state      <= S_INIT_1;
                    end else begin
                        version_ok   <= 1'b0;
                        error_flag   <= 1'b1;
                        card_present <= 1'b0;
                        state        <= S_POLL_DELAY;
                    end
                end

                S_INIT_1: begin
                    req_read         <= 1'b0;
                    req_addr         <= TModeReg;
                    req_wdata        <= 8'h8D;
                    req_return_state <= S_INIT_2;
                    state            <= S_PREP_OP;
                end

                S_INIT_2: begin
                    req_read         <= 1'b0;
                    req_addr         <= TPrescalerReg;
                    req_wdata        <= 8'h3E;
                    req_return_state <= S_INIT_3;
                    state            <= S_PREP_OP;
                end

                S_INIT_3: begin
                    req_read         <= 1'b0;
                    req_addr         <= TReloadRegL;
                    req_wdata        <= 8'd30;
                    req_return_state <= S_INIT_4;
                    state            <= S_PREP_OP;
                end

                S_INIT_4: begin
                    req_read         <= 1'b0;
                    req_addr         <= TReloadRegH;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_INIT_5;
                    state            <= S_PREP_OP;
                end

                S_INIT_5: begin
                    req_read         <= 1'b0;
                    req_addr         <= TxASKReg;
                    req_wdata        <= 8'h40;
                    req_return_state <= S_INIT_6;
                    state            <= S_PREP_OP;
                end

                S_INIT_6: begin
                    req_read         <= 1'b0;
                    req_addr         <= ModeReg;
                    req_wdata        <= 8'h3D;
                    req_return_state <= S_READ_TXCTRL;
                    state            <= S_PREP_OP;
                end

                S_READ_TXCTRL: begin
                    req_read         <= 1'b1;
                    req_addr         <= TxControlReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_WRITE_TXCTRL;
                    state            <= S_PREP_OP;
                end

                S_WRITE_TXCTRL: begin
                    req_read         <= 1'b0;
                    req_addr         <= TxControlReg;
                    req_wdata        <= last_read | 8'h03;
                    req_return_state <= S_CONFIRM_TXCTRL;
                    state            <= S_PREP_OP;
                end

                S_CONFIRM_TXCTRL: begin
                    req_read         <= 1'b1;
                    req_addr         <= TxControlReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_INIT_DONE;
                    state            <= S_PREP_OP;
                end

                S_INIT_DONE: begin
                    init_done <= 1'b1;
                    if ((last_read & 8'h03) == 8'h03) begin
                        antenna_ok <= 1'b1;
                    end else begin
                        antenna_ok <= 1'b0;
                        error_flag <= 1'b1;
                    end
                    wait_count <= 32'd0;
                    state      <= S_POLL_DELAY;
                end

                S_POLL_DELAY: begin
                    if (wait_count >= POLL_DELAY_CYCLES) begin
                        wait_count     <= 32'd0;
                        irq_poll_count <= 16'd0;
                        if (version_ok) begin
                            poll_toggle <= ~poll_toggle;
                            state       <= S_REQA_BF_7;
                        end else begin
                            state <= S_READ_VERSION;
                        end
                    end else begin
                        wait_count <= wait_count + 32'd1;
                    end
                end

                S_REQA_BF_7: begin
                    req_read         <= 1'b0;
                    req_addr         <= BitFramingReg;
                    req_wdata        <= 8'h07;
                    req_return_state <= S_REQA_IDLE;
                    state            <= S_PREP_OP;
                end

                S_REQA_IDLE: begin
                    req_read         <= 1'b0;
                    req_addr         <= CommandReg;
                    req_wdata        <= PCD_IDLE;
                    req_return_state <= S_REQA_IRQ_CLR;
                    state            <= S_PREP_OP;
                end

                S_REQA_IRQ_CLR: begin
                    req_read         <= 1'b0;
                    req_addr         <= ComIrqReg;
                    req_wdata        <= 8'h7F;
                    req_return_state <= S_REQA_FIFO_CLR;
                    state            <= S_PREP_OP;
                end

                S_REQA_FIFO_CLR: begin
                    req_read         <= 1'b0;
                    req_addr         <= FIFOLevelReg;
                    req_wdata        <= 8'h80;
                    req_return_state <= S_REQA_FIFO_DATA;
                    state            <= S_PREP_OP;
                end

                S_REQA_FIFO_DATA: begin
                    req_read         <= 1'b0;
                    req_addr         <= FIFODataReg;
                    req_wdata        <= PICC_REQA;
                    req_return_state <= S_REQA_TRANSCEIVE;
                    state            <= S_PREP_OP;
                end

                S_REQA_TRANSCEIVE: begin
                    req_read         <= 1'b0;
                    req_addr         <= CommandReg;
                    req_wdata        <= PCD_TRANSCEIVE;
                    req_return_state <= S_REQA_STARTSEND;
                    state            <= S_PREP_OP;
                end

                S_REQA_STARTSEND: begin
                    req_read         <= 1'b0;
                    req_addr         <= BitFramingReg;
                    req_wdata        <= 8'h87;
                    req_return_state <= S_REQA_READ_IRQ;
                    state            <= S_PREP_OP;
                end

                S_REQA_READ_IRQ: begin
                    req_read         <= 1'b1;
                    req_addr         <= ComIrqReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_REQA_CHECK_IRQ;
                    state            <= S_PREP_OP;
                end

                S_REQA_CHECK_IRQ: begin
                    if ((last_read & 8'h20) != 8'h00) begin
                        rx_irq_seen <= 1'b1;
                        state       <= S_REQA_CLEARSTART;
                    end else if ((last_read & 8'h01) != 8'h00) begin
                        card_present <= 1'b0;
                        state        <= S_POLL_DELAY;
                    end else if (irq_poll_count >= IRQ_POLL_LIMIT) begin
                        card_present <= 1'b0;
                        state        <= S_POLL_DELAY;
                    end else begin
                        irq_poll_count <= irq_poll_count + 16'd1;
                        state          <= S_REQA_READ_IRQ;
                    end
                end

                S_REQA_CLEARSTART: begin
                    req_read         <= 1'b0;
                    req_addr         <= BitFramingReg;
                    req_wdata        <= 8'h07;
                    req_return_state <= S_REQA_READ_ERR;
                    state            <= S_PREP_OP;
                end

                S_REQA_READ_ERR: begin
                    req_read         <= 1'b1;
                    req_addr         <= ErrorReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_REQA_CHECK_ERR;
                    state            <= S_PREP_OP;
                end

                S_REQA_CHECK_ERR: begin
                    if ((last_read & 8'h13) != 8'h00) begin
                        error_flag   <= 1'b1;
                        card_present <= 1'b0;
                        state        <= S_POLL_DELAY;
                    end else begin
                        state <= S_REQA_READ_LEVEL;
                    end
                end

                S_REQA_READ_LEVEL: begin
                    req_read         <= 1'b1;
                    req_addr         <= FIFOLevelReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_REQA_CHECK_LVL;
                    state            <= S_PREP_OP;
                end

                S_REQA_CHECK_LVL: begin
                    if (last_read >= 8'd2) begin
                        if (!card_present) begin
                            card_pulse <= 1'b1;
                        end
                        card_present <= 1'b1;
                        error_flag   <= 1'b0;
                        state        <= S_REQA_DRAIN_1;
                    end else begin
                        card_present <= 1'b0;
                        state        <= S_POLL_DELAY;
                    end
                end

                S_REQA_DRAIN_1: begin
                    req_read         <= 1'b1;
                    req_addr         <= FIFODataReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_REQA_DRAIN_2;
                    state            <= S_PREP_OP;
                end

                S_REQA_DRAIN_2: begin
                    req_read         <= 1'b1;
                    req_addr         <= FIFODataReg;
                    req_wdata        <= 8'h00;
                    req_return_state <= S_POLL_DELAY;
                    state            <= S_PREP_OP;
                end

                default: begin
                    state <= S_RESET_LOW;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
