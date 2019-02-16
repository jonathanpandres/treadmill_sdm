import struct
import sys


INPUT_FILE = r"../treadmill_sdm/pca10059/s212/ses/Output/Release/Exe/treadmill_sdm_s212_RBL.hex"
OUTPUT_FILE = r"../treadmill_sdm/pca10059/s212/ses/Output/Release/Exe/treadmill_sdm_s212_beltlength_%dmm.hex"


def calculateChecksum(line):
    lineBytes = bytearray.fromhex(line.strip()[1:-2])

    checksum = 0
    for b in lineBytes:
        checksum += b
        checksum &= 0xff
    checksum = -checksum
    checksum &= 0xff

    return line[0:2*len(lineBytes)+1] + ("%02X" % checksum) + line[2*len(lineBytes)+3:]


def main(beltLength):
    if beltLength > 0xffff or beltLength < 1:
        print("Belt Length can only be in the range 1-65535")
        return

    try:
        f = open(INPUT_FILE, "r")
    except FileNotFoundError:
        print("Unable to find treadmill_sdm_s212_RBL.hex")
        return

    if sys.version_info[0] < 3:
        replacement = struct.pack("<H", beltLength).encode('hex').upper()
    else:
        replacement = struct.pack("<H", beltLength).hex().upper()

    out = ""
    replaced = False

    for line in f.readlines():
        index = line.find("5A0C")  # 3162mm From My Belt

        if index != -1:
            replaced = True
            line = calculateChecksum(line[0:index] + replacement + line[index+4:])

        out += line

    if replaced:
        print("Belt Length updated to %dmm" % beltLength)

        try:
            outputFile = OUTPUT_FILE % beltLength
            with open(outputFile, "w") as f:
                f.write(out)
            print("Output file written as %s" % outputFile)
        except IOError:
            print("Error writing output file")
    else:
        print("Unable to find existing belt length to update.")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: %s <Belt Length In Millimeters>" % sys.argv[0])
    else:
        main(int(sys.argv[1], 10))
