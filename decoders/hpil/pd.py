import sigrokdecode as srd
import math
from functools import reduce

class Decoder(srd.Decoder):
    """
    Decodes HP-IL pulses.

    https://www.hpl.hp.com/hpjournal/pdfs/IssuePDFs/1983-01.pdf

    Input hpil0,hpil1 signals are assumed to be inverted
    (2 LM393 comparators in the inverting configuration, i.e. "inactive high",
    instead of the shmiddt triggers in the orginal circuit)
    and represent >0 and <0 parts of a single pulse.
    """

    api_version = 3
    id = 'hpil'
    name = 'HP-IL'
    longname = 'HP-IL'
    desc = 'HP-IL interface decoder.'
    tags = ['Embedded/industrial']
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['hpil']
    channels = (
        {'id': 'hpil0', 'name': 'HPIL 0', 'desc': 'HPIL+ inv'},
        {'id': 'hpil1', 'name': 'HPIL 1', 'desc': 'HPIL- inv'},
    )
    binary = (
        ('bits', 'HP-IL bits'),
    )
    annotations = (
        ('bits', 'HP-IL bits'),
        ('msg_class', 'HP-IL message class'),
        ('msg_control', 'HP-IL message control bits'),
        ('msg_payload', 'HP-IL message payload'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('msg_class', 'Class', (1,3,)),
        ('msg_frame', 'Frame', (2,)),
    )

    def __init__(self, **kwargs):
        self.reset()

    def metadata(self, key, value):
        None

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def reset(self):
        self.state = 'START'
        self.ss = self.es = None
        self.pulse_width = None
        self.num_decoded_bits = None
        self.message = []

    def putx(self, data):
        self.put(self.ss, self.es, self.out_ann, data)

    def putp(self, data):
        self.put(self.ss, self.es, self.out_python, data)

    def putb(self, data):
        self.put(self.ss, self.es, self.out_binary, data)

    def message_bit(self, b):
        """
        Save a single message bit.
        (start, stop, bit)
        """
        self.message.append((self.ss, self.es, b))

    def one_pulse(self):
        """
        Reads a single pulse.

        :return (start, end, -1) when a negative pulse is detected,
                (start, end, +1) when a positive pulse is detected,
                (start, end, None) otherwise
        """
        hpil0, hpil1 = self.wait([{0: 'h', 1: 'f'}, {1: 'h', 0: 'f'}])

        start = self.samplenum

        if hpil0 == 0:
            self.wait([{0: 'r', 1: 'h'}])
            self.wait([{0: 'h', 1: 'f'}])
            self.wait([{0: 'h', 1: 'r'}])

            return (start, self.samplenum, -1)

        elif hpil1 ==0:
            self.wait([{1: 'r', 0: 'h'}])
            self.wait([{1: 'h', 0: 'f'}])
            self.wait([{1: 'h', 0: 'r'}])

            return (start, self.samplenum, 1)

        else:
            return (start, self.samplenum, None)

    def one_pause(self):
        """
        Reads a single pause.
        Expects it to roughly match self.pulse_width

        :return (start, end, Bool)
        """

        start = self.samplenum
        # Wait for either pin falling while trying to skip approx "pause duration" number of samples
        to_skip = math.floor(self.pulse_width * 0.85)
        pins = self.wait([{'skip': to_skip}, {1: 'f'}, {0: 'f'}])

        if self.matched[0]:
            # this was an actual pause, no other pin changes
            return (start, self.samplenum, True)
        else:
            # one of the pins changed too soon, this is not a pause
            return (start, self.samplenum, False)

    def one_bit(self):
        """
        Reads a single bit that is not encoded as a "start bit".

        :return (start, end, 0/1 bit) when successful
                (start, end, None) otherwise
        """

        s, e, p = self.one_pause()
        ps, pe, pp = self.one_pulse()

        if p and (not (pp is None)):
            return (s, pe, 1 if pp > 0 else 0)
        else:
            return (s, pe, None)


    def handle_start(self):
        self.message = []

        p0s, p0e, p0 = self.one_pulse()
        p1s, p1e, p1 = self.one_pulse()

        dp0 = p0e - p0s
        dp1 = p1e - p1s

        dpdiff_percent = (abs(dp0 - dp1) * 100) / dp0

        if ( (p0 is None) or (p1 is None) or # must have 2 in a row
             (p0 != p1) or # Expect S0 or S1 pulse first
             (dpdiff_percent > 15) # pulse timing shoudl be pretty close
           ):
           self.state = 'START'
        else:
            self.ss = p0s
            self.es = p1e
            self.pulse_width = dp0

            self.state = 'DATA'
            bit0 = 1 if p0 > 0 else 0
            self.message_bit(bit0)
            self.putx([0, ['%d' % bit0 ]])

    def handle_data(self):
        self.state = 'START' # go back to start after this
        decoded_data_bits = 0
        while decoded_data_bits < 10: # 11 total but 1st one is the start bit
            s, e, b = self.one_bit()

            if b is None:
                break
            else:
                decoded_data_bits += 1
                self.ss = s
                self.es = e
                self.message_bit(b)
                self.putx([0, ['%d' % b ]])

        if decoded_data_bits > 2:
            (c2s, c2e, c2), (c1s, c1e, c1), (c0s, c0e, c0) = self.message[0:3]
            if c2 == 0:
                self.put(c2s, c0e, self.out_ann, [1, ["Data", "D"]])
                if c1 == 1:
                    self.put(c1s, c1e, self.out_ann, [2, ["End of record", "EOR", "E"]])
                if c0 == 1:
                    self.put(c1s, c1e, self.out_ann, [2, ["Service request", "SR", "S"]])
            elif c1 == 1:
                self.put(c2s, c0e, self.out_ann, [1, ["Identity", "Ident", "Id"]])
                if c0 == 1:
                    self.put(c1s, c1e, self.out_ann, [2, ["Service request", "SR", "S"]])
            elif c0 == 1:
                self.put(c2s, c0e, self.out_ann, [1, ["Command ready", "Ready", "Rd"]])

        if decoded_data_bits == 10:
            payload = self.message[3:]
            ps = payload[0][0]
            pe = payload[7][1]
            pbits = list(map(lambda s: s[2], payload))
            p = reduce(lambda a, b: (a << 1) | b, pbits)
            self.put(ps, pe, self.out_ann, [3, ["%d %s" % (p, ascii(chr(p)))]])


    def decode(self):
        while True:
            if self.state == 'START':
                self.handle_start()
            elif self.state == 'DATA':
                self.handle_data()
