`timescale 1ns / 1ps
// Expanded testbench for the 4-floor elevator controller.
// Prints PASS / FAIL in the Tcl console during simulation.

module elevator_tb;

    reg        clk = 1'b0;
    reg        rst = 1'b1;
    reg  [3:0] req = 4'b0000;
    reg        emergency = 1'b0;
    reg        door_obstruct = 1'b0;
    integer    i;

    wire [2:0] current_floor;
    wire [1:0] direction;
    wire       motor_up;
    wire       motor_down;
    wire       door_open;
    wire [3:0] pending_out;

    elevator_controller #(
        .TRAVEL_CYCLES(10),
        .DOOR_CYCLES(15)
    ) dut (
        .clk(clk),
        .rst(rst),
        .req(req),
        .emergency(emergency),
        .door_obstruct(door_obstruct),
        .current_floor(current_floor),
        .direction(direction),
        .motor_up(motor_up),
        .motor_down(motor_down),
        .door_open(door_open),
        .pending_out(pending_out)
    );

    always #5 clk = ~clk;

    always @(posedge door_open) begin
        $display("[%0t ns] Door opened at floor %0d", $time, current_floor);
    end

    task pulse_req;
        input [3:0] r;
        begin
            req = r;
            @(posedge clk);
            @(posedge clk);
            req = 4'b0000;
        end
    endtask

    task wait_door_at;
        input [2:0] exp_floor;
        integer timeout;
        begin
            timeout = 0;
            while (door_open !== 1'b1 && timeout < 2000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (door_open !== 1'b1)
                $display("FAIL: door never opened, expected floor %0d", exp_floor);
            else if (current_floor !== exp_floor)
                $display("FAIL: door opened at floor %0d, expected %0d",
                         current_floor, exp_floor);
            else
                $display("PASS: door opened at floor %0d", exp_floor);

            timeout = 0;
            while (door_open !== 1'b0 && timeout < 2000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            @(posedge clk);
            @(posedge clk);
        end
    endtask

    initial begin
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        rst = 1'b0;
        @(posedge clk);
        @(posedge clk);
        if (current_floor !== 3'd1)
            $display("FAIL: after reset floor is %0d, expected 1", current_floor);
        else
            $display("PASS: after reset, lift at floor 1, idle");

        $display("\nTest 1: request floor 4 from floor 1");
        pulse_req(4'b1000);
        wait_door_at(3'd4);

        $display("\nTest 2: from floor 4, request floors 2 and 1");
        pulse_req(4'b0011);
        wait_door_at(3'd2);
        wait_door_at(3'd1);

        $display("\nTest 3: request floor 3, then floor 1 while moving up");
        pulse_req(4'b0100);
        wait (motor_up);
        @(posedge clk);
        @(posedge clk);
        pulse_req(4'b0001);
        wait_door_at(3'd3);
        wait_door_at(3'd1);

        $display("\nTest 4: request current floor (floor 1)");
        pulse_req(4'b0001);
        wait_door_at(3'd1);
        if (current_floor !== 3'd1)
            $display("FAIL: lift moved on a same-floor request");

        $display("\nTest 5: emergency stop while moving up to floor 4");
        pulse_req(4'b1000);
        wait (motor_up);
        emergency = 1'b1;
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        if (motor_up || motor_down)
            $display("FAIL: motor still running during emergency");
        else
            $display("PASS: motor stopped during emergency");
        emergency = 1'b0;
        wait_door_at(3'd4);

        $display("\nTest 6: up intermediate stop, floors 2 and 4");
        pulse_req(4'b0001);
        wait_door_at(3'd1);
        pulse_req(4'b1010);
        wait_door_at(3'd2);
        wait_door_at(3'd4);

        $display("\nTest 7: opposite priority from floor 2, request 4 and 1");
        pulse_req(4'b0010);
        wait_door_at(3'd2);
        pulse_req(4'b1001);
        wait_door_at(3'd4);
        wait_door_at(3'd1);

        $display("\nTest 8: request floor 4 while door is open at floor 2");
        pulse_req(4'b0010);
        wait (door_open);
        pulse_req(4'b1000);
        wait (!door_open);
        @(posedge clk);
        @(posedge clk);
        wait_door_at(3'd4);

        $display("\nTest 9: door obstruct holds door open at floor 3");
        pulse_req(4'b0100);
        wait (door_open);
        door_obstruct = 1'b1;
        for (i = 0; i < 30; i = i + 1) begin
            @(posedge clk);
        end
        if (door_open !== 1'b1)
            $display("FAIL: door closed during obstruct");
        else
            $display("PASS: door held open during obstruct");
        door_obstruct = 1'b0;
        wait (!door_open);

        $display("\nTest 10: reset mid-trip to floor 4");
        pulse_req(4'b1000);
        wait (motor_up);
        rst = 1'b1;
        @(posedge clk);
        @(posedge clk);
        rst = 1'b0;
        @(posedge clk);
        if (current_floor !== 3'd1 || motor_up || motor_down ||
            door_open || pending_out !== 4'b0000)
            $display("FAIL: reset did not return lift to floor 1 idle, floor=%0d pending=%b",
                     current_floor, pending_out);
        else
            $display("PASS: reset returned lift to floor 1 idle, pending cleared");

        $display("\nTest 11: emergency stop while moving down to floor 1");
        pulse_req(4'b1000);
        wait_door_at(3'd4);
        pulse_req(4'b0001);
        wait (motor_down);
        emergency = 1'b1;
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        if (motor_up || motor_down)
            $display("FAIL: motor still running during emergency down");
        else
            $display("PASS: motor stopped during emergency down");
        emergency = 1'b0;
        wait_door_at(3'd1);

        $display("\nTest 12: request all floors together, must serve 1, 2, 3, 4");
        pulse_req(4'b1111);
        wait_door_at(3'd1);
        wait_door_at(3'd2);
        wait_door_at(3'd3);
        wait_door_at(3'd4);

        $display("\nTest 13: emergency during door open at floor 4, then idle");
        pulse_req(4'b1000);
        wait (door_open);
        emergency = 1'b1;
        @(posedge clk);
        @(posedge clk);
        if (door_open || motor_up || motor_down)
            $display("FAIL: door/motor not stopped by emergency");
        else
            $display("PASS: emergency closed door and stopped motor");
        emergency = 1'b0;
        @(posedge clk);
        @(posedge clk);
        for (i = 0; i < 50; i = i + 1) begin
            @(posedge clk);
        end
        if (current_floor !== 3'd4 || motor_up || motor_down ||
            door_open || pending_out !== 4'b0000)
            $display("FAIL: lift not stable idle at floor 4");
        else
            $display("PASS: lift stable idle at floor 4, no spurious request");

        $display("\nAll elevator tests finished.");
        $finish;
    end

endmodule
