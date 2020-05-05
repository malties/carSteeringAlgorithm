#!/usr/bin/env python3

# Copyright (C) 2019 Christian Berger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import struct

# Default messages from libcluon providing Envelope and TimeStamp
import cluonDataStructures_pb2
# OpenDLV Standard Message Set
import opendlv_standard_message_set_v0_9_9_pb2
# Messages from individual message set
import example_pb2


################################################################################
# Extract and print an Envelope's payload based on it's dataType.
def extractAndPrintPayload(dataType, payload):
    # Example on how to extract a message from the OpenDLV Standard Message Set:
    if dataType == 19:   # opendlv.proxy.GeodeticWgs84Reading
        messageFromPayload = opendlv_standard_message_set_v0_9_9_pb2.opendlv_proxy_GeodeticWgs84Reading()
        messageFromPayload.ParseFromString(payload)
        print("Payload: %s" % (str(messageFromPayload)))

    # Example on how to extract a message from the individual example message set:
    if dataType == 1002: # TestMessage2
        messageFromPayload = example_pb2.odcore_testdata_TestMessage2()
        messageFromPayload.ParseFromString(payload)
        print("Payload: %s" % (str(messageFromPayload)))

    if dataType == 1005: # TestMessage5
        messageFromPayload = example_pb2.odcore_testdata_TestMessage5()
        messageFromPayload.ParseFromString(payload)
        print("Payload: %s" % (str(messageFromPayload)))


################################################################################
# Print an Envelope's meta information.
def printEnvelope(e):
    print("Envelope ID/senderStamp = %s/%s" % (str(e.dataType), str(e.senderStamp)))
    print(" - sent                 = %s.%s" % (str(e.sent.seconds), str(e.sent.microseconds)))
    print(" - received             = %s.%s" % (str(e.received.seconds), str(e.received.microseconds)))
    print(" - sample time          = %s.%s" % (str(e.sampleTimeStamp.seconds), str(e.sampleTimeStamp.microseconds)))
    extractAndPrintPayload(e.dataType, e.serializedData)
    print()


################################################################################
# Main entry point.
if len(sys.argv) != 2:
    print("Display Envelopes captured from OpenDLV.")
    print("  Usage: %s example.rec" % (str(sys.argv[0])))
    sys.exit()


# Read Envelopes from .rec file.
with open(sys.argv[1], "rb") as f:
    buf = b""
    bytesRead = 0
    expectedBytes = 0
    LENGTH_ENVELOPE_HEADER = 5
    consumedEnvelopeHeader = False

    byte = f.read(1)
    while byte != "":
        buf =  b"".join([buf, byte])
        bytesRead = bytesRead + 1

        if consumedEnvelopeHeader:
            if len(buf) >= expectedBytes:
                envelope = cluonDataStructures_pb2.cluon_data_Envelope()
                envelope.ParseFromString(buf)
                printEnvelope(envelope)
                # Start over and read next container.
                consumedEnvelopeHeader = False
                buf = buf[expectedBytes:]
                expectedBytes = 0

        if not consumedEnvelopeHeader:
            if len(buf) >= LENGTH_ENVELOPE_HEADER:
                consumedEnvelopeHeader = True
                byte0 = buf[0]
                byte1 = buf[1]

                # Check for Envelope header.
                if byte0 == 0x0D and byte1 == 0xA4:
                    v = struct.unpack('<L', buf[1:5]) # Read uint32_t and convert to little endian.
                    expectedBytes = v[0] >> 8 # The second byte belongs to the header of an Envelope.
                    buf = buf[5:] # Remove header.
                else:
                    print("Failed to consume header from Envelope.")

        # Read next byte.
        byte = f.read(1)

        # End processing at file's end.
        if not byte:
            break
f.close()
