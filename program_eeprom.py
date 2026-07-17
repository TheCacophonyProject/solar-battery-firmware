#!/usr/bin/env python3
"""Program the M24C02 EEPROM on the solar battery PCB from a Raspberry Pi.

Writes the 12-byte data block read by the firmware (see src/m24c02.h):

  offset 0:  u8  version    - layout version (EEPROM_DATA_VERSION)
  offset 1:  u32 timestamp  - unix epoch seconds when programmed
  offset 5:  u16 id         - battery box ID
  offset 7:  u8  pcb major
  offset 8:  u8  pcb minor
  offset 9:  u8  pcb patch
  offset 10: u16 crc        - CRC-16/CCITT-FALSE over bytes 0-9

All fields little-endian. The EEPROM write-protect (WC) pin is driven by
GPIO4 (BCM); it is held high (protected) except during the write.

Usage:
  program_eeprom.py --id 123 --pcb-version 1.0.0   # program
  program_eeprom.py --read                          # dump current contents
"""

import argparse
import struct
import sys
import time

import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg

EEPROM_ADDRESS = 0x50
EEPROM_DATA_ADDR = 0x00
EEPROM_DATA_LEN = 12
EEPROM_DATA_VERSION = 1
WRITE_CYCLE_TIME = 0.01  # M24C02 max write cycle is 5 ms

PIN_WRITE_PROTECT = 4  # BCM. High = write protected.


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Matches crc16CCITT() in src/util.h."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_block(box_id, pcb_version):
    major, minor, patch = pcb_version
    timestamp = int(time.time())
    payload = struct.pack("<BIHBBB", EEPROM_DATA_VERSION, timestamp, box_id, major, minor, patch)
    return payload + struct.pack("<H", crc16_ccitt(payload))


def decode_block(block):
    version, timestamp, box_id, major, minor, patch, crc = struct.unpack("<BIHBBBH", bytes(block))
    crc_ok = crc == crc16_ccitt(bytes(block[:-2]))
    print(f"  Layout version: {version}")
    print(f"  Programmed at:  {timestamp} ({time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime(timestamp))})")
    print(f"  Battery box ID: {box_id}")
    print(f"  PCB version:    {major}.{minor}.{patch}")
    print(f"  CRC:            0x{crc:04X} ({'OK' if crc_ok else 'MISMATCH'})")
    return crc_ok


def read_block(bus):
    write = i2c_msg.write(EEPROM_ADDRESS, [EEPROM_DATA_ADDR])
    read = i2c_msg.read(EEPROM_ADDRESS, EEPROM_DATA_LEN)
    bus.i2c_rdwr(write, read)
    return list(read)


def write_block(bus, block):
    # 12 bytes at offset 0 fits in a single 16-byte page write.
    bus.write_i2c_block_data(EEPROM_ADDRESS, EEPROM_DATA_ADDR, list(block))
    time.sleep(WRITE_CYCLE_TIME)


def parse_pcb_version(value):
    try:
        major, minor, patch = (int(p) for p in value.split("."))
        if not all(0 <= p <= 255 for p in (major, minor, patch)):
            raise ValueError
        return major, minor, patch
    except ValueError:
        raise argparse.ArgumentTypeError(f"invalid PCB version '{value}', expected MAJOR.MINOR.PATCH")


def main():
    parser = argparse.ArgumentParser(description="Program the solar battery M24C02 EEPROM")
    parser.add_argument("--read", action="store_true", help="only read and decode the current contents")
    parser.add_argument("--id", type=int, help="battery box ID (0-65535)")
    parser.add_argument("--pcb-version", type=parse_pcb_version, help="PCB version, e.g. 1.0.0")
    args = parser.parse_args()

    with SMBus(1) as bus:
        if args.read:
            print("EEPROM contents:")
            sys.exit(0 if decode_block(read_block(bus)) else 1)

        if args.id is None or args.pcb_version is None:
            parser.error("--id and --pcb-version are required to program")
        if not 0 <= args.id <= 0xFFFF:
            parser.error("--id must be 0-65535")

        block = build_block(args.id, args.pcb_version)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN_WRITE_PROTECT, GPIO.OUT, initial=GPIO.HIGH)
        try:
            GPIO.output(PIN_WRITE_PROTECT, GPIO.LOW)
            write_block(bus, block)
        finally:
            # No GPIO.cleanup(): that would float the pin, leave it driven high.
            GPIO.output(PIN_WRITE_PROTECT, GPIO.HIGH)

        readback = read_block(bus)
        if bytes(readback) != block:
            print("ERROR: readback does not match written data")
            print(f"  wrote: {block.hex()}")
            print(f"  read:  {bytes(readback).hex()}")
            sys.exit(1)

        print("EEPROM programmed OK:")
        decode_block(readback)


if __name__ == "__main__":
    main()
