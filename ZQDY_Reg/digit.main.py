import os
from collections import deque

from maix import app, camera, display, image, nn, pinmap, time, uart


APP_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(APP_DIR, "digit_1to5.mud")
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
START_COMMAND = "START"
STOP_COMMAND = "STOP"
RESULT_SEND_INTERVAL_MS = 20
DETECT_CONFIDENCE = 0.45
VOTE_WINDOW = 12
VOTE_MIN_COUNT = 9

# The camera faces right, so the left-side cargo box is always out of view.
# The detector must recognize the other four physical boxes in this order.
ALL_DIGITS = ("1", "2", "3", "4", "5")
ALL_DIGIT_SET = set(ALL_DIGITS)


def read_uart_commands(serial, rx_buffer):
    """Return complete newline-terminated commands and the remaining data."""
    data = serial.read()
    if data:
        rx_buffer += data.decode("ascii", "ignore")

    commands = []
    while "\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split("\n", 1)
        command = line.strip().upper()
        if command:
            commands.append(command)

    if len(rx_buffer) > 128:
        rx_buffer = rx_buffer[-128:]
    return rx_buffer, commands


def find_five_position_digits(objects, labels):
    """Return (P1, P2, P3, P4, P5), with P1 inferred from four detections."""
    best_by_digit = {}
    for obj in objects:
        if obj.class_id < 0 or obj.class_id >= len(labels):
            continue
        digit = str(labels[obj.class_id])
        if digit not in ALL_DIGIT_SET:
            continue
        previous = best_by_digit.get(digit)
        if previous is None or obj.score > previous.score:
            best_by_digit[digit] = obj

    # Four different labels are required. Duplicate detections do not create
    # a second cargo-box result.
    if len(best_by_digit) != 4:
        return None, best_by_digit

    missing_digits = ALL_DIGIT_SET - set(best_by_digit)
    if len(missing_digits) != 1:
        return None, best_by_digit
    inferred_left_digit = missing_digits.pop()

    visible_left_to_right = sorted(
        best_by_digit.items(), key=lambda item: item[1].x + item[1].w // 2
    )
    visible_digits = tuple(item[0] for item in visible_left_to_right)
    position_digits = (inferred_left_digit,) + visible_digits

    if set(position_digits) != ALL_DIGIT_SET:
        return None, best_by_digit
    return position_digits, best_by_digit


def main():
    detector = nn.YOLO11(model=MODEL_PATH)
    cam = camera.Camera(
        detector.input_width(),
        detector.input_height(),
        detector.input_format(),
    )
    disp = display.Display()
    pinmap.set_pin_function("A18", "UART1_RX")
    pinmap.set_pin_function("A19", "UART1_TX")
    serial = uart.UART(UART_DEVICE, UART_BAUDRATE)

    candidate_history = deque(maxlen=VOTE_WINDOW)
    recognizing = False
    rx_buffer = ""
    first_frame = True
    result_message = None
    last_result_send_ms = 0
    result_send_count = 0

    print("Model ready:", MODEL_PATH)
    print("Labels must be:", ALL_DIGITS, "actual:", detector.labels)
    print("P1 is inferred; camera view is P2, P3, P4, P5 from left to right")
    print("UART output order: P1, P2, P3, P4, P5")
    print("Waiting for controller UART7 command: START")

    while not app.need_exit():
        rx_buffer, commands = read_uart_commands(serial, rx_buffer)
        for command in commands:
            if command == START_COMMAND:
                if not recognizing and result_message is None:
                    candidate_history.clear()
                    recognizing = True
                    first_frame = True
                    print("RX START: begin recognition")
            elif command == STOP_COMMAND:
                candidate_history.clear()
                recognizing = False
                result_message = None
                result_send_count = 0
                print("RX STOP: stop result streaming")

        if result_message is not None:
            now_ms = time.ticks_ms()
            if now_ms - last_result_send_ms >= RESULT_SEND_INTERVAL_MS:
                last_result_send_ms = now_ms
                sent = serial.write_str(result_message)
                if sent is not None and sent < 0:
                    if result_send_count % 50 == 0:
                        print("UART repeat send failed:", sent)
                elif result_send_count == 0:
                    print("TX to controller UART7:", result_message.strip())
                result_send_count += 1
            time.sleep_ms(1)
            continue

        img = cam.read()

        best_by_digit = {}
        if recognizing:
            objects = detector.detect(img, conf_th=DETECT_CONFIDENCE, iou_th=0.45)
            if first_frame:
                print("Camera and first inference ready")
                first_frame = False

            candidate_digits, best_by_digit = find_five_position_digits(
                objects, detector.labels
            )
            candidate_history.append(candidate_digits)

            if (
                candidate_digits is not None
                and sum(item == candidate_digits for item in candidate_history)
                >= VOTE_MIN_COUNT
            ):
                # P1 has already been inferred above, so send the complete
                # physical order P1, P2, P3, P4, P5 to the controller.
                message = ",".join(candidate_digits) + "\n"
                result_message = message
                last_result_send_ms = (
                    time.ticks_ms() - RESULT_SEND_INTERVAL_MS
                )
                result_send_count = 0
                print("Recognition complete:", message.strip())
                recognizing = False
                candidate_history.clear()

        for digit, obj in best_by_digit.items():
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED, thickness=2)
            img.draw_string(
                obj.x,
                max(0, obj.y - 20),
                "{} {:.2f}".format(digit, obj.score),
                color=image.COLOR_RED,
            )

        if recognizing:
            img.draw_string(
                5,
                5,
                "Recognizing... send after {}/{} stable frames".format(
                    VOTE_MIN_COUNT, VOTE_WINDOW
                ),
                color=image.COLOR_RED,
            )
        else:
            img.draw_string(5, 5, "Waiting UART7: START", color=image.COLOR_RED)

        disp.show(img)
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
