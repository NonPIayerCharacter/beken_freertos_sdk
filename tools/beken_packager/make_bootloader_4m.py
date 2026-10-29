#!/usr/bin/env python3
"""Make a 4 MB-flash variant of a Beken BK7231/BK7238 bootloader.

The bootloader binary ends in its FAL partition table: three 64-byte entries
("01PE", name[24], flash device[24], offset, length, crc32 of the first 60
bytes). The shipped bootloader_bk7238_uart1_v1.0.15.bin carries the 2 MB table
(download at 0x132000). Rewriting that table for the SDK's 4 MB layout
(application 0x10000+0x250000, download 0x286000+0x15A000) gives the bootloader
a 4 MB chip needs. Checked against a Sonoff S61 (4 MB BK7238): its factory
bootloader is exactly this file with exactly this table.

  python3 make_bootloader_4m.py bootloader_bk7238_uart1_v1.0.15.bin bootloader_bk7238_uart1_v1.0.15_4M.bin
"""
import struct
import sys
import zlib

MAGIC = b"01PE"
ENTRY = 64
TABLE_4M = (
    (b"bootloader", b"beken_onchip_crc", 0x000000, 0x010000),
    (b"app",        b"beken_onchip_crc", 0x010000, 0x250000),
    (b"download",   b"beken_onchip",     0x286000, 0x15A000),
)


def entry(name, dev, offset, length):
    e = MAGIC + name.ljust(24, b"\0") + dev.ljust(24, b"\0") + struct.pack("<II", offset, length)
    return e + struct.pack("<I", zlib.crc32(e) & 0xFFFFFFFF)


def valid_entry(e):
    return len(e) == ENTRY and e[:4] == MAGIC and \
        struct.unpack_from("<I", e, 60)[0] == zlib.crc32(e[:60]) & 0xFFFFFFFF


def table_start(blob):
    """The partition table is the run of valid 64-byte entries that ends the file."""
    start = None
    pos = len(blob) - ENTRY
    while pos >= 0 and valid_entry(blob[pos:pos + ENTRY]):
        start = pos
        pos -= ENTRY
    if start is None:
        raise SystemExit("no partition table found at the end of the bootloader")
    return start


def convert(src, dst):
    blob = open(src, "rb").read()
    start = table_start(blob)
    table = b"".join(entry(*e) for e in TABLE_4M)
    out = blob[:start] + table
    open(dst, "wb").write(out)
    print("%s: table at 0x%x (%d entries) -> %s (%d bytes)" % (src, start, (len(blob) - start) // ENTRY, dst, len(out)))


def main():
    convert(sys.argv[1], sys.argv[2])


if __name__ == "__main__":
    main()
