import sys


def compute_hex(args):
    arr = args

    cksum = 0
    for i in range(3, len(arr)):
        cur_byte = bytes.fromhex(arr[i])
        cksum += cur_byte[0]

    cksum = cksum & 0xFF
    cksum = ~cksum & 0xFF
    print(f"sum of {len(arr)} bytes: {hex(cksum)}")
        



if __name__ == "__main__":
    # Check if an argument was passed from the terminal
    if len(sys.argv) > 1:
        arg = sys.argv[:] # all arguments after the script name
        compute_hex(arg)
    else:
        print("Please provide an argument from the terminal.")