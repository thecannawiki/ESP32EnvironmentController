import serial
import datetime
from collections import deque

PORT = "COM5"     # Change as needed
BAUD = 115200             # Change as needed
RESET_STRING = "ets"
PRE_LINES = 40
POST_LINES = 20

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    # Keeps rolling log of previous PRE_LINES lines
    history = deque(maxlen=PRE_LINES)

    post_capture_count = 0
    post_buffer = []

    while True:
        raw = ser.readline()
        if not raw:
            continue

        try:
            line = raw.decode(errors="replace")
        except:
            continue

        print(line,end="")  # Log live to terminal

        # If currently capturing post-reset lines
        if post_capture_count > 0:
            post_buffer.append(line)
            post_capture_count -= 1

            # If finished capturing POST_LINES, write file
            if post_capture_count == 0:
                save_capture(history, post_buffer)
                post_buffer = []
            continue

        # Add line to rolling history
        history.append(line)

        # Detect reset
        if RESET_STRING in line:
            print("\n--- RESET DETECTED ---\n")
            post_capture_count = POST_LINES


def save_capture(before, after):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"reset_captures/reset_capture_{timestamp}.log"

    with open(filename, "w") as f:
        f.write("=== PRECEDING LINES ===\n")
        for l in before:
            f.write(l + "\n")

        f.write("\n=== RESET TRIGGER LINE + FOLLOWING LINES ===\n")
        for l in after:
            f.write(l + "\n")

    print(f"\nSaved reset capture to {filename}\n")


if __name__ == "__main__":
    main()
