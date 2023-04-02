import sigrokdecode as srd
import math
from functools import reduce

class Decoder(srd.Decoder):
    """
    ASK capture from Elegiant EOX-9906 weather station transmitter

    OOK pulses are captured from the pin 6 of the transmitter IC (unmarked).

    Transmits once per minute.

    0.48ms pulses of seemingly equal lengths, followed by either a short or a long pause.

    0.95ms pause (short)
    1.93ms pause (long)

    Preamble: 4 long pauses
    """
    api_version = 3
    id = 'elegiant-eox9906'
    name = 'elegiant-EOX-9906'
    longname = 'Elegiant EOX 9906'
    desc = 'Elegiant EOX 9906 decoder.'
    tags = ['Embedded/industrial']
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['eox9906']
    channels = (
        {'id': 'ook', 'name': 'OOK', 'desc': 'OnOffKey pulses'},
    )
    binary = (
        ('bits', 'EOX 9006 bits'),
    )
    annotations = (
        ('bits', 'EOX 9906 bits'),
        ('fields', 'EOX 9906 bit fields'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('bit_fields', 'Bit fields', (1,)),
    )

    def __init__(self, **kwargs):
        self.reset()

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            print(f"Sample rate: {self.samplerate}")

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def reset(self):
        self.state = 'START'

    def decode(self):
        while True:
            if self.state == 'START':
                self.decode_start()
            elif self.state == 'DATA':
                self.decode_data()
            else:
                raise Exception(f"Unexpected state {self.state}")

    def decode_start(self):
        ps = self.require_n_pulses(4)
        if ps:
          if all(val == 1 for val in map(lambda x: x[0], ps)):
            fs = ps[0][1]
            es = ps[-1][2]
            self.put(fs, es, self.out_ann, [1, ['SOF']])
            self.state = 'DATA'
          else:
            self.reset()
        else:
            self.reset()

    def decode_data(self):
        ps = self.decode_byte()
        while ps:
          ps = self.decode_byte()

        self.reset()

    def decode_byte(self):
        ps = self.require_n_pulses(8)
        if ps:
            fs = ps[0][1]
            es = ps[-1][2]
            b = 0
            i = 7
            for bit in map(lambda x: x[0], ps):
                b = b + (bit << i)
                i = i - 1

            self.put(fs, es, self.out_ann, [1, ['%d' % b]])
            return (b, fs, es)
        else:
            self.reset()

    def require_n_pulses(self, n):
        p = self.try_decode_pulse()
        i = 0
        result = []

        while p and i < n:
            (b, s, e) = p
            self.put(s, e, self.out_ann, [0, ['%d' % b]])
            result.append(p)
            i = i+1
            if i > 0 and i < n:
                p = self.try_decode_pulse()

        if len(result) == n:
            return result
        else:
            self.reset()

    def try_decode_pulse(self):
        self.wait([{0: 'h'}])
        self.wait([{0: 'n'}])
        start = self.samplenum
        self.wait([{0: 'f'}])
        fall = self.samplenum # On pulse, same lenghts
        self.wait([{0: 'r'}])
        self.samplenum = self.samplenum - 1
        end_of_pause = self.samplenum # pause after falling edge, long or short

        on_pulse_millis = self.duration_millis(start, fall)

        # expect same-ish ON pulse
        if math.fabs(on_pulse_millis - 0.5) > 0.2:
            return None

        pause_millis = self.duration_millis(fall, end_of_pause)

        print(f"{on_pulse_millis} / {pause_millis}")

        # expect either short or a long pause
        if math.fabs(pause_millis - 2.0) < 0.2:
            return (1, start, end_of_pause)
        elif math.fabs(pause_millis - 1.0) < 0.2:
            return (0, start, end_of_pause)
        else:
            return None

    def duration_millis(self, ss, es):
        return 1000 * (es - ss) / self.samplerate
