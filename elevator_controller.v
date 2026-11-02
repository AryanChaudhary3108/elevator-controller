`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// 4-Floor Elevator Controller - Vivado simulation version
// ---------------------------------------------------------------------------
// Floors are numbered 1 to 4.
//   req[0] = request for floor 1
//   req[1] = request for floor 2
//   req[2] = request for floor 3
//   req[3] = request for floor 4
//
// How it works (SCAN / direction priority):
//  * Requests are latched into `pending` when a req bit is high.
//  * IDLE is the decision state. Every clock it checks:
//      - a request on the current floor -> open the door
//      - otherwise a request above          -> move up
//      - otherwise a request below          -> move down
//      - otherwise                          -> stay idle
//  * While moving up, the lift stops at every requested floor on the way
//    up, and only reverses when no request above remains. Same for down.
//    A request behind it is therefore queued and served after the
//    current direction run finishes - it does NOT make the lift zig-zag.
//  * DOOR holds the door open for DOOR_CYCLES clocks. If door_obstruct
//    is high, the timer restarts, so the door stays open.
//  * emergency forces the lift to stop (motor off, door closed) and
//    keeps the pending requests. When emergency goes low, it resumes.
//
// Timing parameters are in clock cycles, not seconds, so the simulation
// stays short. With a 10 ns clock, TRAVEL_CYCLES = 10 is 100 ns per
// floor and DOOR_CYCLES = 15 is 150 ns of door-open time.
// For a real board you would add a clock divider and use seconds here.
// ---------------------------------------------------------------------------

module elevator_controller #(
    parameter TRAVEL_CYCLES = 10,   // clocks to travel one floor
    parameter DOOR_CYCLES   = 15    // clocks the door stays open
)(
    input  wire       clk,
    input  wire       rst,          // active high, asynchronous
    input  wire [3:0] req,          // floor requests, bit 0 = floor 1
    input  wire       emergency,    // high = stop everything
    input  wire       door_obstruct,// high = hold the door open
    output wire [2:0] current_floor,// 1 to 4
    output wire [1:0] direction,    // 00 idle, 01 up, 10 down
    output wire       motor_up,
    output wire       motor_down,
    output wire       door_open,
    output wire [3:0] pending_out
);

    // States
    localparam IDLE      = 3'd0;
    localparam MOVE_UP   = 3'd1;
    localparam MOVE_DOWN = 3'd2;
    localparam DOOR      = 3'd3;
    localparam EMERG     = 3'd4;

    reg [2:0]  state;
    reg [2:0]  floor_reg;           // 1 to 4
    reg [3:0]  pending;             // latched, not-yet-served requests
    reg [15:0] travel_cnt;
    reg [15:0] door_cnt;

    // Requests latched this cycle (old pending OR new req inputs)
    wire [3:0] pending_latched = pending | req;

    // One-hot bit for the floor the lift is on now
    reg [3:0] floor_bit;
    always @* begin
        case (floor_reg)
            3'd1:    floor_bit = 4'b0001;
            3'd2:    floor_bit = 4'b0010;
            3'd3:    floor_bit = 4'b0100;
            3'd4:    floor_bit = 4'b1000;
            default: floor_bit = 4'b0001;
        endcase
    end

    // Is there a (latched) request above / below the current floor?
    // The current floor itself is excluded here on purpose.
    reg any_above;
    reg any_below;
    always @* begin
        case (floor_reg)
            3'd1: begin
                any_above = |pending_latched[3:1];
                any_below = 1'b0;
            end
            3'd2: begin
                any_above = |pending_latched[3:2];
                any_below = pending_latched[0];
            end
            3'd3: begin
                any_above = pending_latched[3];
                any_below = |pending_latched[1:0];
            end
            3'd4: begin
                any_above = 1'b0;
                any_below = |pending_latched[2:0];
            end
            default: begin
                any_above = 1'b0;
                any_below = 1'b0;
            end
        endcase
    end

    // Outputs are a direct decode of the state - no extra registers.
    assign current_floor = floor_reg;
    assign direction     = (state == MOVE_UP)   ? 2'b01 :
                           (state == MOVE_DOWN) ? 2'b10 : 2'b00;
    assign motor_up      = (state == MOVE_UP);
    assign motor_down    = (state == MOVE_DOWN);
    assign door_open     = (state == DOOR);
    assign pending_out   = pending;

    // Main FSM - one always block, so there are no multiple drivers.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= IDLE;
            floor_reg  <= 3'd1;
            pending    <= 4'b0000;
            travel_cnt <= 16'd0;
            door_cnt   <= 16'd0;
        end
        else if (emergency) begin
            // Stop, keep pending requests, wait for emergency to clear.
            state      <= EMERG;
            travel_cnt <= 16'd0;
            door_cnt   <= 16'd0;
        end
        else begin
            case (state)
                EMERG: begin
                    // emergency is low here (see the branch above)
                    state   <= IDLE;
                    pending <= pending_latched;
                end

                IDLE: begin
                    // Decision state, checked every clock.
                    pending <= pending_latched;
                    if (pending_latched & floor_bit) begin
                        // Request on this floor: serve it, open the door.
                        // This assignment is last, so it wins over the
                        // one above it.
                        pending  <= pending_latched & ~floor_bit;
                        state    <= DOOR;
                        door_cnt <= 16'd0;
                    end
                    else if (any_above) begin
                        state      <= MOVE_UP;
                        travel_cnt <= 16'd0;
                    end
                    else if (any_below) begin
                        state      <= MOVE_DOWN;
                        travel_cnt <= 16'd0;
                    end
                    else begin
                        state <= IDLE;
                    end
                end

                MOVE_UP: begin
                    pending <= pending_latched;
                    if (travel_cnt == TRAVEL_CYCLES - 1) begin
                        // Arrived at the next floor. Go back to IDLE and
                        // let IDLE decide: door (if requested here),
                        // keep going up, or stop.
                        travel_cnt <= 16'd0;
                        floor_reg  <= floor_reg + 3'd1;
                        state      <= IDLE;
                    end
                    else begin
                        travel_cnt <= travel_cnt + 16'd1;
                    end
                end

                MOVE_DOWN: begin
                    pending <= pending_latched;
                    if (travel_cnt == TRAVEL_CYCLES - 1) begin
                        travel_cnt <= 16'd0;
                        floor_reg  <= floor_reg - 3'd1;
                        state      <= IDLE;
                    end
                    else begin
                        travel_cnt <= travel_cnt + 16'd1;
                    end
                end

                DOOR: begin
                    // Do NOT re-queue the floor being served: a request
                    // held high while the door is open would otherwise
                    // be latched again after it was just cleared, causing
                    // a duplicate door opening at the same floor.
                    // This is the Test 12 fix.
                    pending <= pending_latched & ~floor_bit;
                    if (door_obstruct) begin
                        // Hold the door open, restart the timer.
                        door_cnt <= 16'd0;
                    end
                    else if (door_cnt == DOOR_CYCLES - 1) begin
                        door_cnt <= 16'd0;
                        state    <= IDLE;
                    end
                    else begin
                        door_cnt <= door_cnt + 16'd1;
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
