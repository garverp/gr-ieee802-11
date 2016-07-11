#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Sim Ricean
# Generated: Fri Jul  8 14:28:29 2016
##################################################

import os
import sys
sys.path.append(os.environ.get('GRC_HIER_PATH', os.path.expanduser('~/.grc_gnuradio')))

from gnuradio import blocks
from gnuradio import channels
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
from wifi_phy_hier import wifi_phy_hier  # grc-generated hier_block
import foo
import ieee802_11
import math
import pmt
import random


class sim_ricean(gr.top_block):

    def __init__(self, encoding=0, interval=200, messages=10, pdu_len=435, repetition=23, snr=10):
        gr.top_block.__init__(self, "Sim Ricean")

        ##################################################
        # Parameters
        ##################################################
        self.encoding = encoding
        self.interval = interval
        self.messages = messages
        self.pdu_len = pdu_len
        self.repetition = repetition
        self.snr = snr

        ##################################################
        # Variables
        ##################################################
        self.out_buf_size = out_buf_size = 96000

        ##################################################
        # Blocks
        ##################################################
        self.wifi_phy_hier_0 = wifi_phy_hier(
            chan_est=0,
            encoding=encoding,
            sensitivity=0.56,
        )
        self.ieee802_11_ofdm_mac_0 = ieee802_11.ofdm_mac(([0x23, 0x23, 0x23, 0x23, 0x23, 0x23]), ([0x42, 0x42, 0x42, 0x42, 0x42, 0x42]), ([0xff, 0xff, 0xff, 0xff, 0xff, 0xff]))
        self.foo_wireshark_connector_0 = foo.wireshark_connector(127, False)
        self.foo_periodic_msg_source_0 = foo.periodic_msg_source(pmt.intern("".join([chr(random.randint(0,255)) for x in (lambda: (random.seed(repetition) == None) and range(pdu_len-24))()])), interval, messages, True, False)
        self.foo_packet_pad2_0 = foo.packet_pad2(False, False, 0.001, 1000, 1000)
        (self.foo_packet_pad2_0).set_min_output_buffer(96000)
        self.channels_fading_model_0 = channels.fading_model( 8, 0, True, 5.0, repetition )
        self.channels_channel_model_0 = channels.channel_model(
        	noise_voltage=1,
        	frequency_offset=0,
        	epsilon=1.0,
        	taps=(1.0, ),
        	noise_seed=repetition,
        	block_tags=False
        )
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc(((10**(snr/10.0))**.5, ))
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_char*1, "" + os.path.dirname(os.path.realpath(__file__)) + "/results/sim_%d_%d_%.1f_.pcap" % (repetition, encoding, snr) + "", False)
        self.blocks_file_sink_0_0.set_unbuffered(True)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.foo_periodic_msg_source_0, 'out'), (self.foo_wireshark_connector_0, 'in'))    
        self.msg_connect((self.foo_periodic_msg_source_0, 'out'), (self.ieee802_11_ofdm_mac_0, 'app in'))    
        self.msg_connect((self.ieee802_11_ofdm_mac_0, 'phy out'), (self.wifi_phy_hier_0, 'mac_in'))    
        self.msg_connect((self.wifi_phy_hier_0, 'mac_out'), (self.foo_wireshark_connector_0, 'in'))    
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.channels_channel_model_0, 0))    
        self.connect((self.channels_channel_model_0, 0), (self.channels_fading_model_0, 0))    
        self.connect((self.channels_fading_model_0, 0), (self.wifi_phy_hier_0, 0))    
        self.connect((self.foo_packet_pad2_0, 0), (self.blocks_multiply_const_vxx_0, 0))    
        self.connect((self.foo_wireshark_connector_0, 0), (self.blocks_file_sink_0_0, 0))    
        self.connect((self.wifi_phy_hier_0, 1), (self.blocks_null_sink_0, 0))    
        self.connect((self.wifi_phy_hier_0, 0), (self.foo_packet_pad2_0, 0))    

    def get_encoding(self):
        return self.encoding

    def set_encoding(self, encoding):
        self.encoding = encoding
        self.blocks_file_sink_0_0.open("" + os.path.dirname(os.path.realpath(__file__)) + "/results/sim_%d_%d_%.1f_.pcap" % (self.repetition, self.encoding, self.snr) + "")
        self.wifi_phy_hier_0.set_encoding(self.encoding)

    def get_interval(self):
        return self.interval

    def set_interval(self, interval):
        self.interval = interval

    def get_messages(self):
        return self.messages

    def set_messages(self, messages):
        self.messages = messages

    def get_pdu_len(self):
        return self.pdu_len

    def set_pdu_len(self, pdu_len):
        self.pdu_len = pdu_len

    def get_repetition(self):
        return self.repetition

    def set_repetition(self, repetition):
        self.repetition = repetition
        self.blocks_file_sink_0_0.open("" + os.path.dirname(os.path.realpath(__file__)) + "/results/sim_%d_%d_%.1f_.pcap" % (self.repetition, self.encoding, self.snr) + "")

    def get_snr(self):
        return self.snr

    def set_snr(self, snr):
        self.snr = snr
        self.blocks_file_sink_0_0.open("" + os.path.dirname(os.path.realpath(__file__)) + "/results/sim_%d_%d_%.1f_.pcap" % (self.repetition, self.encoding, self.snr) + "")
        self.blocks_multiply_const_vxx_0.set_k(((10**(self.snr/10.0))**.5, ))

    def get_out_buf_size(self):
        return self.out_buf_size

    def set_out_buf_size(self, out_buf_size):
        self.out_buf_size = out_buf_size


def argument_parser():
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option(
        "-e", "--encoding", dest="encoding", type="intx", default=0,
        help="Set encoding [default=%default]")
    parser.add_option(
        "", "--interval", dest="interval", type="intx", default=200,
        help="Set interval [default=%default]")
    parser.add_option(
        "-n", "--messages", dest="messages", type="intx", default=10,
        help="Set messages [default=%default]")
    parser.add_option(
        "", "--pdu-len", dest="pdu_len", type="intx", default=435,
        help="Set pdu_len [default=%default]")
    parser.add_option(
        "", "--repetition", dest="repetition", type="intx", default=23,
        help="Set repetition [default=%default]")
    parser.add_option(
        "-s", "--snr", dest="snr", type="eng_float", default=eng_notation.num_to_str(10),
        help="Set snr [default=%default]")
    return parser


def main(top_block_cls=sim_ricean, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    tb = top_block_cls(encoding=options.encoding, interval=options.interval, messages=options.messages, pdu_len=options.pdu_len, repetition=options.repetition, snr=options.snr)
    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
