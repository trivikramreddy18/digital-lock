
# Two-Key Digital Lock with Debouncing: RTL-to-GDSII Project Report


## 1. Project Overview

This project implements a digital combination lock that is operated with two push-buttons, `key0` (the "0" key) and `key1` (the "1" key). The lock unlocks when the correct four-press code is entered. The design follows the complete RTL-to-GDSII flow, including Verilog RTL design, simulation, synthesis, gate-level simulation, physical design, sign-off checks, timing analysis, and final GDSII layout generation.

The design uses standard cells only, without macros, memories, SRAM/ROM, IP blocks, or analog components. It targets a 100 µm × 100 µm die using the SkyWater Sky130 `sky130_fd_sc_hd` process.

**Project summary:** The lock takes a clock, an active-low reset, and two raw (asynchronous, bouncy) button inputs. Each button passes through its own synchronizer and debouncer, which produces a clean one-cycle press pulse. A five-state finite state machine (FSM) compares each press with the stored code and drives an `unlock` output and a 3-bit `progress` output.

## 2. Real-World Application

**Problem:** Mechanical push-buttons do not produce clean signals. A single press can generate several rapid on/off transitions (contact bounce), and the button is not synchronized to any clock. A lock that reads such an input directly can count one press as many, or can be advanced by electrical noise. The lock must also fail safe: a wrong key must never leave it partly advanced, and it must never unlock without the exact code.

**Users and place of use:** This type of controller can be used in cabinet and locker locks, equipment-room access panels, hobby and embedded security projects, and other places where a small, low-cost keypad lock is sufficient.

**Why dedicated hardware:** The security rules are implemented directly in the FSM logic. The unlock output is decoded from one state only, and every wrong press, every simultaneous press of both keys, and every reset returns the FSM to the start state. Because the logic is a few dozen standard cells, it can be placed on a chip alongside other functions at negligible cost.

**Example scenario:** The code is `1, 1, 0, 1`. A user presses `key1` and the lock moves to progress 1. The user presses `key1` again (progress 2), then `key0` (progress 3), then `key1`, and the `unlock` output goes high (progress 4). Pressing any key afterwards relocks the lock. If the user instead presses a wrong key at any step, for example `key0` as the very first press, the progress returns to 0 and the user must start again.

**Scope:** The project implements a fixed-code, two-key lock with input synchronization and debouncing. The lock-actuator driver, a physical keypad, programmable codes, and attempt-lockout features are outside the current scope.

## 3. Specification

### Inputs and Outputs

| Signal | Direction | Width | Purpose |
|---|---|---|---|
| `clk` | Input | 1 | Clock for the debouncers and FSM |
| `rst_n` | Input | 1 | Asynchronous, active-low reset |
| `key0` | Input | 1 | Raw push-button for key "0" |
| `key1` | Input | 1 | Raw push-button for key "1" |
| `unlock` | Output | 1 | High while the lock is open |
| `progress` | Output | 3 | FSM state (0 to 4), useful for debug LEDs |

The top-level module has four input bits and four output bits, eight port bits in total.

### Parameters

- `DEBOUNCE_CYCLES = 4`: number of clock cycles an input must remain changed before it is accepted.
- `CODE = 4'b1011`: the code, where `CODE[0]` is the first key pressed. The code is therefore `1, 1, 0, 1`.

Both values are parameters and can be changed before synthesis.

### Functional Requirements

1. After reset, the lock must be locked and `progress` must be 0.
2. The lock unlocks only after the four code keys are pressed in the correct order.
3. A wrong key at any position returns the FSM to state 0.
4. Pressing both keys together counts as a wrong key.
5. Glitches shorter than the debounce time must be ignored.
6. A button held down for a long time must count as exactly one press.
7. While unlocked, any press relocks the lock.
8. Asserting reset while unlocked must relock the lock immediately.

## 4. Design and RTL

The design consists of two instances of a `debouncer` module (one per key) and a `digital_lock` FSM that consumes their press pulses.

### Debouncer

Each debouncer has the following structure:

- A two-flop synchronizer (`s1`, `s2`) moves the asynchronous button into the clock domain.
- A `stable` register holds the last accepted value of the input.
- A counter `cnt` counts how many consecutive cycles `s2` differs from `stable`. If the input returns to the stable value, the counter clears, so short glitches are discarded.
- When the input has differed for `DEBOUNCE_CYCLES` cycles, `stable` takes the new value and `press` is registered high for one cycle, but only for a rising edge (`press <= done && s2`). Releasing the button updates `stable` without generating a press.

The counter width is `CW = $clog2(DEBOUNCE_CYCLES + 1)`, which is 3 bits for the default value. From a clean button edge, the press pulse appears about six clock edges later (two synchronizer stages, four debounce cycles).

### FSM States

| State | `progress` | Meaning | Expected key | On correct key | On wrong key |
|---|---|---|---|---|---|
| S0 | `000` | Locked, no keys entered | `1` (`CODE[0]`) | S1 | S0 |
| S1 | `001` | First key correct | `1` (`CODE[1]`) | S2 | S0 |
| S2 | `010` | Second key correct | `0` (`CODE[2]`) | S3 | S0 |
| S3 | `011` | Third key correct | `1` (`CODE[3]`) | S4 | S0 |
| S4 | `100` | Unlocked (`unlock = 1`) | Any key | S0 | S0 |

The state advances only on a press pulse (`any_press = p0 | p1`). A press is correct when both keys were not pressed in the same cycle and the pressed key matches the code bit for the current state:

`correct = !both && (key_val == CODE[state[1:0]])`

where `key_val = p1`, so a `key0` press has value 0 and a `key1` press has value 1. The `unlock` output is simply `(state == 4)`, and `progress` is the state register itself.

### Reset Operation

The reset is asynchronous and active-low. When `rst_n = 0`, all debouncer registers and the state register are cleared immediately, without waiting for a clock edge. The lock is therefore locked and `progress` is 0 at once.

### RTL Code (`digital_lock.v`)

```verilog
// Digital lock with 2 keys (key0 = "0", key1 = "1"), 4-press code.
// Macro-free: only parameters/localparams are used.

module debouncer #(
    parameter DEBOUNCE_CYCLES = 4          // clock cycles input must be stable
) (
    input  wire clk,
    input  wire rst_n,
    input  wire din,                       // raw, asynchronous button
    output reg  press                      // 1-cycle pulse per clean press
);
    localparam CW = $clog2(DEBOUNCE_CYCLES + 1);
    localparam [CW-1:0] LIMIT = DEBOUNCE_CYCLES - 1;

    reg s1, s2, stable;
    reg [CW-1:0] cnt;

    wire changed = (s2 != stable);
    wire done    = changed && (cnt == LIMIT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 1'b0; s2 <= 1'b0; stable <= 1'b0;
            cnt <= {CW{1'b0}}; press <= 1'b0;
        end else begin
            s1 <= din;                     // 2-flop synchronizer
            s2 <= s1;
            press <= done && s2;           // rising edge only
            if (!changed)   cnt <= {CW{1'b0}};
            else if (done) begin
                cnt <= {CW{1'b0}};
                stable <= s2;
            end else        cnt <= cnt + 1'b1;
        end
    end
endmodule

module digital_lock #(
    parameter DEBOUNCE_CYCLES = 4,
    parameter [3:0] CODE = 4'b1011         // CODE[0] is the first key pressed
) (
    input  wire clk,
    input  wire rst_n,
    input  wire key0,
    input  wire key1,
    output wire unlock,
    output wire [2:0] progress             // state, handy for debug LEDs
);
    localparam [2:0] S_OPEN = 3'd4;

    wire p0, p1;
    debouncer #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_db0 (clk, rst_n, key0, p0);
    debouncer #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_db1 (clk, rst_n, key1, p1);

    reg [2:0] state;

    wire any_press = p0 | p1;
    wire both      = p0 & p1;
    wire key_val   = p1;
    wire correct   = !both && (key_val == CODE[state[1:0]]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= 3'd0;
        else if (any_press) begin
            if (state == S_OPEN)       state <= 3'd0;      // any press relocks
            else if (correct)          state <= state + 3'd1;
            else                       state <= 3'd0;      // wrong key: restart
        end
    end

    assign unlock   = (state == S_OPEN);
    assign progress = state;
endmodule
```

### Synthesized Structure (Yosys Schematic)

The Yosys schematic shows the two debouncer instances feeding the FSM logic. The inputs `key0` and `key1` each go to a debouncer (`u_db0`, `u_db1`, both parameterized with `DEBOUNCE_CYCLES = 4`). Their press outputs `p0` and `p1` produce `any_press` (an OR gate) and `both` (an AND gate), and `key_val` is `p1`.

<img width="1280" alt="digital_lock_schematic_inputs" src="images/fig01_schematic_inputs.jpg" />

*Figure 1: Yosys schematic, input side. Two debouncer instances produce `p0` and `p1`, from which `any_press`, `both`, and `key_val` are derived.*

The second half of the schematic shows the 3-bit `state` register (an asynchronous-reset flip-flop), the code lookup that indexes the constant `4'b1011` with `state[1:0]`, the `correct` signal, the incrementer (`state + 1`), the multiplexers that select between increment and reset to zero, and the comparator `state == 3'b100` that drives `unlock`.

<img width="1280" alt="digital_lock_schematic_fsm" src="images/fig02_schematic_fsm.jpg" />

*Figure 2: Yosys schematic, FSM side. The `$shiftx` cell implements the `CODE[state[1:0]]` lookup, `$add` is the state increment, and `$eq` with `3'b100` produces `unlock`.*

## 5. RTL Simulation and Verification

A self-checking Verilog testbench generates a 10-unit clock, applies reset, and drives the two keys through ten groups of tests. A reference model (`ref_state`) tracks the expected FSM state. After each press, the DUT's `progress` and `unlock` outputs are compared with the reference model (a scoreboard). Each button press in the testbench includes contact bounce on both press and release, and the stable hold time (`HOLD = 12` cycles) is longer than `DEBOUNCE_CYCLES`.

The testbench checks the following:

| Test | What it verifies |
|---|---|
| T1 | Reset values: locked, `progress = 0`, and no X on outputs |
| T2 | Correct code unlocks; any press then relocks |
| T3 | A wrong key at each of the four positions returns to state 0 |
| T4 | Both keys together, mid-sequence and while unlocked, return to state 0 |
| T5 | Glitches of 1, 2, and 3 cycles are ignored, including mid-sequence |
| T6 | A 200-cycle hold counts as exactly one press |
| T7 | Reset mid-sequence, reset during debounce (no stray press), and asynchronous reset while unlocked |
| T8 | Three repeated unlock/relock cycles |
| T9 | 20 random wrong four-key sequences never unlock |
| T10 | 100 random presses compared against the reference model |

The same testbench file is used for RTL and gate-level simulation. The `GLS` macro only changes the name of the waveform dump file (`digital_lock.vcd` for RTL, `gls_wave.vcd` for GLS).

### RTL Simulation Results

- Checks run: **[N]**, errors: **0** (replace with the "Checks run" line from your simulation output).
- The terminal ends with `ALL TESTS PASSED`.
- GTKWave reports a simulation end time of 32,800,000 ps (about 32.8 µs, roughly 3,280 clock cycles).

The waveforms show `progress` stepping `000 → 001 → 010 → 011 → 100`, with `unlock` high only in state `100`. The following press returns `progress` to `000`. The `key0` and `key1` traces show the contact bounce before each stable press, and the `p0`/`p1` pulses appear only once the debounce time has elapsed. The debouncer internals (`s2`, `stable`, `press`) are also visible in the first capture.

<img width="1280" alt="rtl_wave_debouncer_internals" src="images/fig03_rtl_wave_internals.jpg" />

*Figure 3: RTL waveform with debouncer internals (`s2`, `stable`, `press`). Each bouncing button produces exactly one `press` pulse, and `progress` advances once per correct press.*

<img width="1280" alt="rtl_wave_unlock_relock" src="images/fig04_rtl_wave_unlock.jpg" />

*Figure 4: Full unlock sequence with debouncer internals. After the fourth correct press (`1, 1, 0, 1`), `progress` reaches `100` and `unlock` goes high. The next press (`key0`) returns `progress` to `000` and relocks. Time units follow the simulator's `timescale`; one clock period is 10 time units.*

The same behaviour is visible at the testbench level, where only the DUT ports are shown: `key1`, `key1`, `key0`, `key1` raise `unlock`, a later `key0` press relocks, and the following reset pulse (`rst_n` low) starts the next test.

<img width="1280" alt="rtl_wave_testbench_level" src="images/fig05_rtl_wave_testbench.jpg" />

*Figure 5: Testbench-level RTL waveform (`tb_digital_lock`) showing `clk`, `rst_n`, `key0`, `key1`, `unlock`, and `progress[2:0]`.*

### Testbench Code (`tb_digital_lock.v`)

```verilog
// Self-checking testbench for digital_lock (macro-free, no preprocessor directives).
// Uses DUT default parameters (DEBOUNCE_CYCLES=4, CODE=4'b1011 -> keys 1,1,0,1) so the
// SAME file runs for RTL simulation and gate-level simulation.
//
// A reference model (ref_state) tracks the expected FSM state; after every press
// the DUT outputs are compared with it (scoreboard).
//
// Coverage: reset values, correct code, relock, wrong key at every position,
// both keys together, glitches shorter than the debounce time, long hold,
// contact bounce, reset mid-sequence / during debounce / while unlocked,
// repeated unlock, random wrong sequences, random press stream, X-check.
module tb_digital_lock;
    localparam [3:0] CODE_TB = 4'b1011;
    localparam integer HOLD  = 12;           // stable hold cycles (> DEBOUNCE_CYCLES)

    reg clk = 0, rst_n = 0, key0 = 0, key1 = 0;
    wire unlock;
    wire [2:0] progress;

    integer errors = 0, checks = 0, verbose = 1;
    integer ref_state = 0, seed = 12345;
    integer i, j, k;
    reg r;
    reg [3:0] seq;

    digital_lock dut (
        .clk(clk), .rst_n(rst_n), .key0(key0), .key1(key1),
        .unlock(unlock), .progress(progress));

    always #5 clk = ~clk;

    initial begin                               // watchdog
        #20000000;
        $display("TIMEOUT");
        $finish;
    end

    // ---------------- helpers ----------------
    task cycles(input integer n);
        begin repeat (n) @(negedge clk); end
    endtask

    task drive(input which, input v);
        begin if (which) key1 = v; else key0 = v; end
    endtask

    task check(input ok, input [255:0] name);
        begin
            checks = checks + 1;
            if (!ok) begin
                errors = errors + 1;
                $display("FAIL: %0s (unlock=%b progress=%0d ref=%0d)", name, unlock, progress, ref_state);
            end else if (verbose)
                $display("PASS: %0s", name);
        end
    endtask

    task check_state(input [255:0] name);
        begin
            check((progress === ref_state[2:0]) && (unlock === (ref_state == 4)), name);
        end
    endtask

    task ref_update(input kv);
        begin
            if (ref_state == 4)               ref_state = 0;
            else if (kv == CODE_TB[ref_state]) ref_state = ref_state + 1;
            else                               ref_state = 0;
        end
    endtask

    task do_reset;
        begin
            rst_n = 0; cycles(3); rst_n = 1; cycles(3);
            ref_state = 0;
        end
    endtask

    // One clean press with contact bounce on press and on release.
    task press(input which);
        begin
            drive(which, 1); cycles(1);
            drive(which, 0); cycles(1);
            drive(which, 1); cycles(2);
            drive(which, 0); cycles(1);
            drive(which, 1); cycles(HOLD);       // stable press
            drive(which, 0); cycles(1);
            drive(which, 1); cycles(1);          // release bounce
            drive(which, 0); cycles(HOLD);
        end
    endtask

    task press_chk(input which);
        begin
            press(which);
            ref_update(which);
            check_state("state matches reference after press");
        end
    endtask

    task press_both;
        begin
            key0 = 1; key1 = 1; cycles(HOLD);
            key0 = 0; key1 = 0; cycles(HOLD);
            ref_state = 0;                       // both keys together = wrong key
        end
    endtask

    task glitch(input which, input integer len);
        begin
            drive(which, 1); cycles(len);
            drive(which, 0); cycles(HOLD);
        end
    endtask

    task enter_code;
        integer m;
        begin
            for (m = 0; m < 4; m = m + 1) press_chk(CODE_TB[m]);
        end
    endtask

    // ---------------- tests ----------------
    initial begin
        `ifdef GLS
             $dumpfile("gls_wave.vcd");
        `else
            $dumpfile("digital_lock.vcd");
    `endif
        $dumpvars(0, tb_digital_lock);
        cycles(3); rst_n = 1; cycles(3);

        // T1 reset values and no X on outputs
        check((progress === 3'd0) && (unlock === 1'b0), "T1 reset values: locked, progress 0");
        check((^{unlock, progress}) !== 1'bx, "T1 no X on outputs");

        // T2 correct code unlocks, any press relocks
        enter_code;
        check(unlock === 1'b1, "T2 correct code unlocks");
        press_chk(0);
        check(unlock === 1'b0, "T2 any press relocks");

        // T3 wrong key at every position
        for (k = 0; k < 4; k = k + 1) begin
            do_reset;
            for (j = 0; j < k; j = j + 1) press_chk(CODE_TB[j]);
            press_chk(~CODE_TB[k]);
            check((progress === 3'd0) && (unlock === 1'b0), "T3 wrong key returns to state 0");
        end

        // T4 both keys together (mid-sequence and while unlocked)
        do_reset; press_chk(1); press_chk(1);
        press_both;
        check((progress === 3'd0) && (unlock === 1'b0), "T4 both keys mid-sequence -> state 0");
        do_reset; enter_code; press_both;
        check((progress === 3'd0) && (unlock === 1'b0), "T4 both keys while unlocked -> relock");

        // T5 glitches shorter than debounce time are ignored
        do_reset;
        glitch(1, 1); check_state("T5 1-cycle glitch ignored");
        glitch(1, 2); check_state("T5 2-cycle glitch ignored");
        glitch(0, 3); check_state("T5 3-cycle glitch ignored");
        press_chk(1); press_chk(1);
        glitch(0, 2); check_state("T5 glitch mid-sequence ignored");

        // T6 long hold counts as exactly one press
        do_reset;
        drive(1, 1); cycles(200); drive(1, 0); cycles(HOLD);
        ref_update(1);
        check_state("T6 long hold = one press");
        check(progress === 3'd1, "T6 progress is exactly 1");

        // T7 reset mid-sequence, during debounce, and while unlocked
        do_reset; press_chk(1); press_chk(1);
        check(progress === 3'd2, "T7 progress 2 before reset");
        do_reset;
        check(progress === 3'd0, "T7 reset mid-sequence clears progress");
        press_chk(0); press_chk(1);
        check(unlock === 1'b0, "T7 remaining keys alone do not unlock");
        do_reset;
        drive(1, 1); cycles(2); rst_n = 0; cycles(2); drive(1, 0); cycles(2);
        rst_n = 1; cycles(HOLD); ref_state = 0;
        check_state("T7 reset during debounce leaves no stray press");
        do_reset; enter_code;
        check(unlock === 1'b1, "T7 unlocked before reset");
        rst_n = 0; cycles(2);
        check(unlock === 1'b0, "T7 async reset relocks immediately");
        rst_n = 1; cycles(3); ref_state = 0;

        // T8 repeated unlock / relock
        for (i = 0; i < 3; i = i + 1) begin
            enter_code;
            check(unlock === 1'b1, "T8 unlock");
            press_chk(1);
            check(unlock === 1'b0, "T8 relock");
        end

        // T9 random wrong 4-key sequences never unlock
        verbose = 0;
        for (i = 0; i < 20; i = i + 1) begin
            do_reset;
            seq = $random(seed);
            if (seq == CODE_TB) seq = ~seq;
            for (j = 0; j < 4; j = j + 1) press_chk(seq[j]);
            check(unlock === 1'b0, "T9 random wrong sequence never unlocks");
        end
        verbose = 1;
        $display("PASS: T9 random wrong sequences done (20 sequences)");

        // T10 random press stream against reference model
        verbose = 0;
        do_reset;
        for (i = 0; i < 100; i = i + 1) begin
            r = $random(seed);
            press_chk(r);
        end
        verbose = 1;
        $display("PASS: T10 random stream done (100 presses)");

        $display("Checks run: %0d, errors: %0d", checks, errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end
endmodule
```

## 6. Synthesis

The RTL was synthesized using Yosys inside the OpenLane flow, targeting the `sky130_fd_sc_hd` library. The synthesized netlist is `digital_lock_synth.v`.

### Synthesis Results

| Item | Value |
|---|---:|
| Cells counted by OpenLane synthesis | 66 |
| Flip-flop type | `dfrtp_1` (asynchronous active-low reset) |
| Flip-flops expected from RTL | 19 |
| Memories | 0 |
| Ports (bits) | 8 |

The 19 flip-flops come from the RTL: each debouncer has `s1`, `s2`, `stable`, a 3-bit counter, and `press` (8 flip-flops each, 16 in total), and the FSM has a 3-bit state register. Reset-capable flip-flops (`dfrtp_1`) are used because the reset is asynchronous.

The netlist contains these standard-cell types: `a211oi`, `a21oi`, `a31oi`, `clkinv`, `dfrtp`, `lpflow_inputiso1p`, `lpflow_isobufsrc`, `mux2`, `nand3b`, `nor2`, `nor3b`, `o2111ai`, `o211ai`, `o22ai`, `xnor2`, and `xor2`. The XOR/XNOR cells implement the `s2 != stable` comparison and the counter increment, and the AOI/OAI cells implement the terminal-count comparison and FSM next-state logic.

The terminal listing of these cell types is shown in Figure 6 in the next section.

## 7. Gate-Level Simulation (GLS)

The synthesized netlist was simulated with the Sky130 cell models in functional mode using Icarus Verilog, with the macros `GLS`, `FUNCTIONAL`, and a unit delay defined at compile time. The Sky130 `primitives.v` and `sky130_fd_sc_hd.v` models were compiled together with `digital_lock_synth.v` and the same testbench. Because the testbench does not reference internal RTL registers, no testbench changes were needed.

### GLS Results

- Checks run: **[N]**, errors: **0** (replace with the GLS terminal output).
- The GLS waveform (`gls_wave.vcd`) shows the same behaviour as RTL: `progress` steps `000 → 001 → 010 → 011 → 100`, `unlock` rises in state `100`, and the next press and the reset pulse return `progress` to `000`.
- The `dut` hierarchy in the GLS waveform exposes the synthesized nets (`_33_`, `_34_`, ...) alongside the named signals `key_val`, `p0`, `p1`, and `state[2:0]`.

The compile step below uses the Sky130 cell models together with the synthesized netlist. The `grep` output lists the standard-cell types used by `digital_lock_synth.v`.

<img width="1280" alt="gls_compile_terminal" src="images/fig06_gls_compile.jpg" />

*Figure 6: GLS compile commands and the list of Sky130 cell types used by the synthesized netlist.*

The GLS waveform below was generated from the synthesized netlist with the same testbench.

<img width="1280" alt="gls_waveform" src="images/fig07_gls_wave.jpg" />

*Figure 7: Gate-level simulation waveform. The observed sequence matches the RTL waveform.*

## 8. Pre-Layout Timing Analysis

Timing was checked on the synthesized netlist using OpenSTA with an ideal clock (script `test_sta.tcl`, constraints in `digital_lock.sdc`).

### Pre-Layout STA Results

| Metric | Value |
|---|---:|
| WNS | 0.00 ns |
| TNS | 0.00 ns |
| Setup slack (reported path) | +23.33 ns (MET) |
| Worst hold slack (reported path) | +0.13 ns (MET) |
| Clock uncertainty | 0.20 ns |

The setup report shows a data required time of 24.73 ns, which implies a clock period of about 25 ns with 0.20 ns of uncertainty (**confirm against `digital_lock.sdc`**).

The reported hold path runs from one `dfrtp_1` flip-flop to another. The clock-to-Q delay is 0.29 ns, so data arrives at 0.29 ns. The required time is 0.16 ns (0.20 ns clock uncertainty minus a library hold time of 0.04 ns), which gives 0.13 ns of hold slack.

These results use an ideal clock and do not include routed wire delays.

<img width="1280" alt="prelayout_sta" src="images/fig08_prelayout_sta.jpg" />

*Figure 8: Pre-layout OpenSTA report showing positive setup and hold slack and WNS/TNS of 0.00.*

## 9. Physical Design

The physical design was performed using OpenLane in its Docker environment with the Sky130A PDK. The flow includes synthesis, floorplanning, power-grid generation, placement, clock-tree synthesis, routing, Magic DRC, Netgen LVS, antenna checking, electrical rule checking (CVC), and GDSII generation.

### Main Configuration

| Setting | Value |
|---|---|
| Design | `digital_lock` |
| Clock port | `clk` |
| Clock period (OpenLane `CLOCK_PERIOD`) | 10 ns |
| Die area | 100 µm × 100 µm (0.01 mm²) |
| Synthesis strategy | AREA 0 |
| Synthesis max fanout | 5 |
| Cell library | `sky130_fd_sc_hd` |
| Macros | None |

### Main Physical-Design Results

| Item | Result |
|---|---|
| Flow status | Completed without fatal errors |
| Total runtime | About 2 min 45 s to 2 min 49 s |
| Die area | 0.01 mm² |
| Placement utilization (OpenDP) | 12% |
| Cells counted by OpenLane | 66 |
| Tap cells | 90 |
| End caps | 56 |
| Routed wire length | 1602 µm |
| Vias | 491 |
| Routing layer 2 usage | 3.8% |
| Routing layer 3 usage | 3.47% |
| TritonRoute violations | 0 |
| Magic DRC violations | 0 |
| LVS errors | 0 |
| Antenna violations | 0 pins, 0 nets |
| CVC errors | 0 |

The OpenLane summary CSV records `-1` for the KLayout violation count, meaning that check was not run. The utilization is low because the 100 µm × 100 µm die has an area of 10,000 µm², while the logic is much smaller.

<img width="1280" alt="signoff_summary" src="images/fig09_signoff_summary.jpg" />

*Figure 9: OpenLane final summary showing zero CVC errors, zero Magic DRC violations, zero LVS errors, and zero antenna violations.*

<img width="1280" alt="final_summary_csv" src="images/fig10_summary_csv.jpg" />

*Figure 10: `final_summary_report.csv` generated by the flow.*

### 9.1 Floorplanning

Floorplanning establishes the die and core dimensions, creates standard-cell rows, positions I/O pins, and builds the power grid.

The die was configured as 100 µm × 100 µm (the power grid and standard-cell rows are visible in Figure 11). The eight port bits (`clk`, `rst_n`, `key0`, `key1`, `unlock`, and `progress[2:0]`) were placed around the die edges. The flow added 90 tap cells and 56 end caps.

**Result:** The floorplan was accepted, and the flow continued.

### 9.2 Placement

Placement assigns physical positions to standard cells. Global placement estimates suitable locations, while detailed placement legalizes the cells and removes overlaps.

The reported placement utilization was 12%. The logic cells form a cluster near the middle of the die. Other rows contain physical-only cells, including tap cells and end caps.

**Result:** The cells were placed without overlap, and substantial free space remained.

<img width="1280" alt="placement_layout" src="images/fig11_placement.jpg" />

*Figure 11: Placed layout. The logic cluster sits in the centre of the die between the power-grid stripes; the blue horizontal lines are power rails and the vertical hatched stripes are the power grid.*

### 9.3 Clock-Tree Synthesis (CTS)

Clock-tree synthesis builds the network that distributes the clock signal to the flip-flops while controlling clock skew and insertion delay.

The design has 19 flip-flops, so its clock network is small. Before CTS, the timing analysis assumes an ideal clock. After CTS, the post-route analysis uses the propagated clock.

**Result:** The clock tree was built, and the typical-corner timing requirements were met (WNS and TNS of 0.00 ns in the OpenLane summary).

### 9.4 Routing

Routing creates the metal wires and vias that connect standard cells, the clock network, and the power grid.

Global routing estimates routing paths, while detailed routing uses TritonRoute to generate the actual interconnections. The design completed detailed routing with zero reported routing violations. The total routed wire length was 1602 µm, with 491 vias.

**Result:** All nets were routed without reported violations, and the routed layout proceeded to GDSII generation (`digital_lock_100x100.gds`).

<img width="1280" alt="routed_layout" src="images/fig12_routed_layout.jpg" />

*Figure 12: Final routed layout in Magic, showing standard-cell rows, power rails and stripes, and signal routing.*

<img width="1280" alt="magic_layout_render" src="images/fig13_magic_render.jpg" />

*Figure 13: Magic layout render with the pin labels `clk`, `key0`, `unlock`, `progress[0]`, `progress[1]`, and `progress[2]` visible around the boundary.*

## 10. Post-Route Timing Analysis

After routing, OpenSTA was run across 18 process-voltage-temperature (PVT) corners of the Sky130 library (script `run_all_sta.sh`, one report per corner), and the worst negative slack (WNS) and total negative slack (TNS) were plotted for each corner (`plot_wns.py`, `plot_tns.py`).

### Typical Corner Results

| Metric | Value |
|---|---:|
| Corner | TT, 25 °C, 1.8 V (`tt_025C_1v80`) |
| WNS | 0.00 ns |
| TNS | 0.00 ns |
| Worst setup slack | **[fill from `sta_tt_025C_1v80.rpt`]** |
| Worst hold slack | **[fill from `sta_tt_025C_1v80.rpt`]** |

The OpenLane summary also reports WNS and TNS of 0.0 for the flow's own typical-corner analysis.

### Results Across All Corners

WNS and TNS are zero at 14 of the 18 corners. Four corners show negative slack. The values below are read from the plots and are approximate.

| Corner | Approx. WNS (ns) | Approx. TNS (ns) |
|---|---:|---:|
| `ss_n40C_1v44` | −0.3 | −0.3 |
| `ss_n40C_1v40` | −1.9 | −1.9 |
| `ss_n40C_1v35` | −5.0 | −5.0 |
| `ss_n40C_1v28` | −12.8 | −12.8 |
| All other 14 corners | 0.0 | 0.0 |

Observations:

- All four failing corners are slow-process, −40 °C corners with supply voltage of 1.44 V or lower, far below the nominal 1.8 V supply. Slack degrades steadily as the supply falls from 1.44 V to 1.28 V.
- The slow corner `ss_n40C_1v60` and its `ccsnoise` variant, and all slow corners at 1.76 V and above, show no violations.
- `ss_100C_1v40` is clean while `ss_n40C_1v40` fails. At low supply voltages, cells can be slower at cold temperature than at hot temperature (temperature inversion), which is consistent with this result.
- TNS equals WNS at each failing corner, which suggests that a single path dominates the violation. Check the corresponding `.rpt` files to identify it.
- Fast corners all pass setup and hold.

<img width="1280" alt="wns_plot" src="images/fig14_wns_plot.jpg" />

*Figure 14: WNS by library corner.*

<img width="1280" alt="tns_plot" src="images/fig15_tns_plot.jpg" />

*Figure 15: TNS by library corner.*

### Understanding the Timing Metrics

- **WNS:** Worst negative setup slack.
- **TNS:** Sum of negative setup slacks.
- **Setup slack:** The difference between the required data arrival time and the actual arrival time.
- **Hold slack:** The margin by which the data path satisfies the hold-time requirement.

Zero WNS and TNS indicate no reported setup violations at that corner. Negative values indicate that at least one path misses its setup requirement there.

## 11. Summary of Results

| Stage | Result |
|---|---|
| RTL simulation | **[N]** checks, 0 errors, `ALL TESTS PASSED` |
| Synthesis | 66 cells counted by OpenLane, 19 flip-flops expected from RTL |
| Gate-level simulation | **[N]** checks, 0 errors, same waveform as RTL |
| Pre-layout timing | WNS 0.00 ns, setup +23.33 ns, hold +0.13 ns |
| Floorplan | 100 µm × 100 µm die |
| Placement utilization | 12% |
| Placement, CTS, routing | Completed, zero routing violations |
| DRC / LVS / antenna / CVC | 0 / 0 / 0 / 0 |
| Post-route typical timing | WNS 0.00 ns, TNS 0.00 ns |
| Post-route multi-corner timing | Clean at 14 of 18 corners; setup violations at four `ss_n40C` corners of 1.44 V and below |
| Output | GDSII file generated (`digital_lock_100x100.gds`) |

### Submission Contents

The project folder, archived as `digital_lock_submission.zip`, contains the RTL (`digital_lock.v`), testbench (`digital_lock_tb.v`), synthesized netlist (`digital_lock_synth.v`), SDC and STA scripts (`digital_lock.sdc`, `test_sta.tcl`, `run_all_sta.sh`), 18 per-corner STA reports, the WNS/TNS plot scripts and images, the simulation outputs (`digital_lock.vcd`, `lock_sim`, `gls_sim`), the schematic files, the layout renders, `final_summary_report.csv`, and the final GDSII in `pd_results/`.

<img width="1280" alt="submission_zip_part1" src="images/fig16_submission_zip_1.jpg" />

*Figure 16: Creating `digital_lock_submission.zip` from the `digital_lock_project` folder (first part of the file list).*

<img width="1280" alt="submission_zip_part2" src="images/fig17_submission_zip_2.jpg" />

*Figure 17: End of the zip listing, including the schematic image, remaining STA reports, `final_summary_report.csv`, and the `gls_sim` output.*

## 12. Limitations and Future Work

**Timing at low-voltage corners.** Four slow, cold, low-voltage corners (1.28 V to 1.44 V) show setup violations. The design is intended for the nominal 1.8 V supply, so these corners are outside the expected operating range. If low-voltage operation is required, the clock period would need to be relaxed or the critical path restructured. The multi-corner analysis also reuses parasitics extracted at the typical corner, so the corner results are estimates rather than a complete sign-off.

**Debounce time.** With `DEBOUNCE_CYCLES = 4` and a 10 ns clock, the debounce time is only 40 ns. Real push-buttons bounce for milliseconds, so practical hardware needs a larger parameter value or a clock divider that produces a slow enable tick.

**Security.** The code is fixed at synthesis time. A four-key binary code has only 16 combinations, and there is no attempt counter or lockout, so it can be found by trial. The `progress` output reveals how many keys are correct, so it should not be exposed outside a debug setting. The lock also stays open until a key is pressed, with no automatic relock timer.

**Simultaneous presses.** The `both` check only sees two presses whose debounced pulses fall in the same clock cycle. Two presses that are a few cycles apart are treated as two separate presses.

**Reset.** Reset assertion is asynchronous and takes effect immediately, but reset deassertion is not synchronized to the clock. A reset synchronizer would be added in a system where the reset release can occur near a clock edge.

**Verification and flow.**
- Gate-level simulation is functional with unit delays, with no SDF back-annotation.
- Power was not analyzed.
- The KLayout check was not run (`-1` in the summary CSV).
- The Magic layout was used as the reference for the reported DRC, LVS, and antenna checks.

### Future Enhancements

1. Make the code programmable and store it in registers.
2. Add an attempt counter with a temporary lockout after repeated failures.
3. Add an automatic relock timer.
4. Remove or gate the `progress` output in production.
5. Add a clock divider so the debounce time covers real button bounce.
6. Add a reset synchronizer.
7. Extend to a keypad with more keys and a longer code.
8. Re-run implementation with a relaxed clock constraint and check the low-voltage corners.

## 13. Conclusion

A two-key digital lock with synchronizers, debouncers, and a five-state code-checking FSM was designed in synthesizable Verilog and verified with a self-checking testbench in both RTL and gate-level simulation. The testbench covers reset behaviour, the correct code, wrong keys at each position, simultaneous key presses, glitches, long holds, resets at several points, and random sequences.

The design was synthesized with the Sky130 standard-cell library and implemented on a 100 µm × 100 µm die using OpenLane. The physical-design flow completed with clean reported DRC, LVS, antenna, and CVC checks. Timing was met before layout and at the typical corner after routing, and the multi-corner analysis showed clean results at 14 of 18 corners, with setup violations only at four cold, low-voltage corners well below the nominal supply. The flow generated the final GDSII layout.

Overall, the project demonstrates the complete RTL-to-GDSII methodology, including RTL design, simulation, synthesis, gate-level verification, static timing analysis, floorplanning, placement, clock-tree synthesis, routing, physical verification, and layout generation.

---

**End of Report**
