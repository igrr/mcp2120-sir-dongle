from nmigen import *
from nmigen.build import *
from nmigen.utils import bits_for
from nmigen.cli import main_parser, main_runner
from nmigen.back.pysim import *

try:
    from nmigen_boards.icestick import *

    ONLY_SIM = 0
except ImportError:
    ONLY_SIM = 1

BAUD_RATES = [
    (9600, 0x14),
    (115200, 0),
    (19200, 0x13),
    (38400, 0x12),
    (57600, 0x11),
]
BAUD_COMMIT = 0x11


class IrDAConverter(Elaboratable):
    def __init__(self, freq):  # type: (int) -> None
        min_baud_rate = 9600
        max_baud_rate = 115200
        default_baud_rate = 9600
        self.freq = freq
        baud_rate_div_width = bits_for(freq // min_baud_rate)
        self.baud_rate_div_default = freq // default_baud_rate
        self.baud_rate_div = Signal(baud_rate_div_width, reset=self.baud_rate_div_default)

        self.baud_rate_val = Signal(bits_for(max_baud_rate), reset=default_baud_rate)
        self.baud_rate_idx = Signal(3, reset=0)
        self.baud_rate_update_done = Signal()
        self.baud_rate_update = Signal()
        self.baud_rate_mode_prev = Signal(2)

        self.uart_rx = Signal()
        self.uart_rx_bit = Signal(bits_for(10))
        self.uart_rx_div_cnt = Signal(8)
        self.uart_rx_symbol_act = Signal(reset=0)

        self.uart_rx_time_per_sub_bit = Signal(16, reset=freq * 16 // (default_baud_rate * 16))
        self.uart_rx_time = Signal(32, reset=0)
        self.uart_rx_next_sub_bit_time = Signal(32, reset=0)

        self.uart_rx_pos = Signal(bits_for(16 * 10))
        self.uart_rx_shaped = Signal()
        self.uart_rx_ctrl_word = Signal(8)

        self.rx_bit_time = Signal(32, reset=0)
        self.irda_rx = Signal(reset=1)
        self.irda_rx_pulse_timeout = Signal(baud_rate_div_width)
        self.irda_rx_shaped = Signal(reset=1)

        self.baud_rate_mode = Signal()

    def elaborate(self, platform):  # type: (ICEStickPlatform) -> Module
        m = Module()

        # IrDA -> UART
        # Convert every pulse into a UART bit
        irda_rx_prev = Signal()
        irda_rx_pulse_start = Signal()

        m.d.sync += [
            irda_rx_prev.eq(self.irda_rx),
            irda_rx_pulse_start.eq(~self.irda_rx & irda_rx_prev),
        ]

        with m.If(irda_rx_pulse_start & ~self.uart_rx_symbol_act):
            m.d.sync += [
                self.irda_rx_shaped.eq(0),
                self.irda_rx_pulse_timeout.eq(self.baud_rate_div),
            ]
        with m.Elif(self.irda_rx_pulse_timeout == 0):
            m.d.sync += self.irda_rx_shaped.eq(1)
        with m.Else():
            m.d.sync += self.irda_rx_pulse_timeout.eq(self.irda_rx_pulse_timeout - 1)

        # UART -> IrDA
        # Detect the start bit.
        uart_rx_prev = Signal(reset=1)
        uart_rx_start = Signal()
        uart_rx_ctrl_word_update = Signal()
        next_sub_bit_flag = Signal()
        next_time = Signal(32)

        m.d.sync += [
            uart_rx_prev.eq(self.uart_rx),
            uart_rx_start.eq((~self.uart_rx) & uart_rx_prev & (~self.uart_rx_symbol_act)),
        ]
        m.d.comb += [
            next_time.eq(self.uart_rx_time + 16),
            next_sub_bit_flag.eq(self.uart_rx_symbol_act & (next_time > self.uart_rx_next_sub_bit_time))
        ]

        with m.If(uart_rx_start):
            m.d.sync += [
                self.uart_rx_pos.eq(0),
                self.uart_rx_symbol_act.eq(1),
                self.uart_rx_time.eq(0),
                self.uart_rx_next_sub_bit_time.eq(self.uart_rx_time_per_sub_bit)
            ]
        with m.Elif(self.uart_rx_symbol_act):
            m.d.sync += [
                self.uart_rx_time.eq(next_time)
            ]

        m.d.sync += uart_rx_ctrl_word_update.eq(0)

        uart_rx_sub_bit = Signal(4)
        uart_rx_start_bit = Signal(1)
        uart_rx_stop_bit = Signal(1)
        uart_rx_symbol_end = Signal(1)
        m.d.comb += [
            uart_rx_sub_bit.eq(self.uart_rx_pos[0:4]),
            self.uart_rx_bit.eq(self.uart_rx_pos[4:]),
            uart_rx_start_bit.eq(self.uart_rx_bit == 0),
            uart_rx_stop_bit.eq(self.uart_rx_bit == 9),
            uart_rx_symbol_end.eq(self.uart_rx_bit == 10),
        ]

        with m.If(next_sub_bit_flag):
            m.d.sync += [
                self.uart_rx_pos.eq(self.uart_rx_pos + 1),
                self.uart_rx_next_sub_bit_time.eq(self.uart_rx_next_sub_bit_time + self.uart_rx_time_per_sub_bit)
            ]
            with m.If(uart_rx_stop_bit):
                # End of symbol
                m.d.sync += self.uart_rx_symbol_act.eq(0)
            with m.Elif(uart_rx_sub_bit == 8 - 1):
                # Middle of a bit
                with m.If(~self.uart_rx):
                    # Output the pulse, if the bit is 0
                    m.d.sync += self.uart_rx_shaped.eq(1)
                with m.If((~uart_rx_start_bit) & (~uart_rx_stop_bit)):
                    # Update the control word (excluding the start bit)
                    m.d.sync += self.uart_rx_ctrl_word.eq(Cat(self.uart_rx_ctrl_word[1:], self.uart_rx))
                    m.d.sync += uart_rx_ctrl_word_update.eq(1)
            with m.Elif(uart_rx_sub_bit == 11 - 1):
                # Clear the output pulse
                m.d.sync += self.uart_rx_shaped.eq(0)

        with m.If(self.baud_rate_mode_prev == 0b10):
            m.d.sync += [
                self.uart_rx_ctrl_word.eq(0x14),
                self.baud_rate_div.eq(Const(self.freq // 9600)),
                self.uart_rx_time_per_sub_bit.eq(Const(self.freq * 16 // (9600 * 16))),
                self.baud_rate_val.eq(Const(9600)),
                self.baud_rate_idx.eq(Const(0)),
            ]

        m.d.sync += [
            self.baud_rate_mode_prev.eq(Cat(self.baud_rate_mode_prev[1], self.baud_rate_mode)),
            self.baud_rate_update_done.eq(0),
        ]
        m.d.comb += [
            self.baud_rate_update.eq(self.baud_rate_mode_prev == 0b01)
        ]

        with m.If(self.baud_rate_update):
            i = 0
            for br, c in BAUD_RATES:
                with m.If(self.uart_rx_ctrl_word[0:8] == c):
                    m.d.sync += [
                        self.baud_rate_div.eq(Const(self.freq // br)),
                        self.uart_rx_time_per_sub_bit.eq(Const(self.freq * 16 // (br * 16))),
                        self.baud_rate_val.eq(Const(br)),
                        self.baud_rate_idx.eq(Const(i)),
                        self.baud_rate_update_done.eq(1)
                    ]
                i += 1

        return m


class IrDADongle(Elaboratable):
    def __init__(self, freq):  # type: (int) -> None

        self.irda_converter = IrDAConverter(freq)

        self.uart_dtr = Signal()
        self.uart_rts = Signal()

        self.nrst = Signal()
        self.baud_rate_mode = Signal()

    def elaborate(self, platform):  # type: (ICEStickPlatform) -> Module
        m = Module()

        m.submodules.irda_converter = ResetInserter(self.nrst)(self.irda_converter)

        if platform:
            uart = platform.request("uart")
            irda = platform.request("irda")

            m.d.sync += [
                self.irda_converter.irda_rx.eq(irda.rx.i),
                self.irda_converter.uart_rx.eq(uart.rx.i),
                self.uart_dtr.eq(uart.dtr.i),
                self.uart_rts.eq(uart.rts.i),
                uart.tx.o.eq(self.irda_converter.irda_rx_shaped),
                irda.tx.o.eq(self.irda_converter.uart_rx_shaped),
                irda.en.o.eq(1),
                self.irda_converter.baud_rate_mode.eq(self.baud_rate_mode)
            ]

            m.d.sync += [
                self.nrst.eq((self.uart_dtr & self.uart_rts)),
                self.baud_rate_mode.eq((~self.uart_dtr) & self.uart_rts)
            ]

            platform.add_resources([
                Resource("debug", 0, Pins("62  61  60  56  48  47  45  44", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS")),
            ])
            dbg = platform.request("debug", 0)
            m.d.comb += [
                dbg.o[0].eq(self.irda_converter.irda_rx),
                dbg.o[1].eq(self.irda_converter.uart_rx),
                dbg.o[2].eq(self.irda_converter.irda_rx_shaped),
                dbg.o[3].eq(self.irda_converter.baud_rate_mode),
                dbg.o[4].eq(self.irda_converter.baud_rate_update_done),
                dbg.o[5:8].eq(self.irda_converter.baud_rate_idx),
            ]

        return m


class IrDADongeTB(Elaboratable):
    def __init__(self, freq):
        pass

    def elaborate(self, platform):
        pass


if __name__ == "__main__":
    parser = main_parser()
    args = parser.parse_args()
    if not ONLY_SIM and args.action == "generate":
        p = ICEStickPlatform()
        p.build(IrDADongle(int(p.default_clk_frequency)), do_program=True)
    elif ONLY_SIM or args.action == "simulate":
        freq = 12000000
        dut = IrDADongle(freq)
        sim = Simulator(dut)
        sim.add_clock(1.0 / freq)


        def out_byte(b, baud):
            yield dut.irda_converter.uart_rx.eq(0)
            yield Delay(1 / baud)
            for i in range(8):
                yield dut.irda_converter.uart_rx.eq((b >> i) & 1)
                yield Delay(1 / baud)
            yield dut.irda_converter.uart_rx.eq(1)
            yield Delay(1 / baud)


        def process():
            yield dut.irda_converter.uart_rx.eq(1)
            yield dut.nrst.eq(1)
            yield dut.irda_converter.baud_rate_mode.eq(0)
            yield Delay(2/freq)
            yield dut.nrst.eq(0)
            yield Delay(2/freq)
            baud = 9600
            yield dut.irda_converter.baud_rate_mode.eq(1)
            yield Delay(4 / freq)
            yield dut.irda_converter.baud_rate_mode.eq(0)
            yield Delay(4 / freq)
            yield from out_byte(0x5a, baud)
            yield Delay(2/freq)
            yield dut.irda_converter.baud_rate_mode.eq(1)
            yield Delay(2/freq)
            yield from out_byte(0x00, baud)
            yield Delay(2/freq)
            yield dut.irda_converter.baud_rate_mode.eq(0)
            baud = 115200
            yield Delay(2/freq)
            yield from out_byte(0x55, baud)
            yield Delay(2/freq)
            yield dut.irda_converter.baud_rate_mode.eq(1)
            yield Delay(4 / freq)
            baud = 9600
            yield from out_byte(0x14, baud)
            yield Delay(4 / freq)
            yield dut.irda_converter.baud_rate_mode.eq(0)
            yield Delay(4 / freq)
            yield from out_byte(0x55, baud)


        sim.add_process(process)
        with sim.write_vcd(vcd_file=args.vcd_file, gtkw_file=args.gtkw_file):
            sim.run_until(6 * 11 / (9600), run_passive=True)
