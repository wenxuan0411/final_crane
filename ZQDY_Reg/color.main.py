import os
from collections import deque

from maix import app, camera, display, image, nn, pinmap, time, uart


APP_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(APP_DIR, "green_white_yellow.mud")
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
START_COMMAND = "START"
STOP_COMMAND = "STOP"
RESULT_SEND_INTERVAL_MS = 20
DETECT_CONFIDENCE = 0.35
VOTE_WINDOW = 8
VOTE_MIN_COUNT = 6
SEND_INFERRED_TO_STM32 = True

COLOR_CODE = {
    "yellow": "01",
    "green": "02",
    "white": "03",
}
ALL_LABELS = set(COLOR_CODE)

# The fixed field positions from left to right are P2, P3, and P1.
POSITION_FROM_LEFT_TO_RIGHT = (2, 3, 1)


def read_uart_commands(serial, rx_buffer):
    """Return complete newline-terminated commands and remaining data."""
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


def find_position_beans(objects, labels, frame_width):
    """Return codes only when middle detection agrees with side inference."""
    best_by_region = {"left": None, "middle": None, "right": None}

    for obj in objects:
        label = labels[obj.class_id]
        if label not in COLOR_CODE:
            continue

        center_x = obj.x + obj.w // 2
        if center_x < frame_width // 3:
            region = "left"
        elif center_x > frame_width * 2 // 3:
            region = "right"
        else:
            region = "middle"

        previous = best_by_region[region]
        if previous is None or obj.score > previous.score:
            best_by_region[region] = obj

    left_obj = best_by_region["left"]
    middle_obj = best_by_region["middle"]
    right_obj = best_by_region["right"]
    if left_obj is None or right_obj is None:
        return None, best_by_region, False

    left_label = labels[left_obj.class_id]
    right_label = labels[right_obj.class_id]
    if left_label == right_label:
        return None, best_by_region, False

    missing = ALL_LABELS - {left_label, right_label}
    if len(missing) != 1:
        return None, best_by_region, None
    inferred_middle_label = missing.pop()

    if middle_obj is None:
        middle_label = inferred_middle_label
        middle_mode = "inferred"
    else:
        middle_label = labels[middle_obj.class_id]
        if middle_label != inferred_middle_label:
            print(
                "Middle mismatch: detected={}, inferred={}".format(
                    middle_label, inferred_middle_label
                )
            )
            return None, best_by_region, None
        middle_mode = "verified"

    left_to_right_labels = (left_label, middle_label, right_label)
    if set(left_to_right_labels) != ALL_LABELS:
        return None, best_by_region, False

    # UART output is always ordered by position number: P1, P2, P3.
    codes = (
        COLOR_CODE[right_label],
        COLOR_CODE[left_label],
        COLOR_CODE[middle_label],
    )
    return codes, best_by_region, middle_mode


def main():
    # Disable dual buffering: it can stall before the first inference on some devices.
    detector = nn.YOLO26(model=MODEL_PATH)
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
    print("Labels:", detector.labels)
    print("UART ready: A19(TX), A18(RX),", UART_DEVICE, UART_BAUDRATE)
    print("Waiting for controller UART10 command: START")

    while not app.need_exit():
        rx_buffer, commands = read_uart_commands(serial, rx_buffer)
        for command in commands:
            if command == START_COMMAND:
                if not recognizing and result_message is None:
                    candidate_history.clear()
                    recognizing = True
                    first_frame = True
                    print("RX START from controller UART10: begin recognition")
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
                    print("TX to controller UART10:", result_message.strip())
                result_send_count += 1
            time.sleep_ms(1)
            continue

        img = cam.read()

        objects = []
        position_beans = {"left": None, "middle": None, "right": None}
        candidate_codes = None
        middle_mode = None

        if recognizing:
            objects = detector.detect(
                img, conf_th=DETECT_CONFIDENCE, iou_th=0.45
            )
            if first_frame:
                print("Camera and first inference ready")
                first_frame = False

            candidate_codes, position_beans, middle_mode = find_position_beans(
                objects, detector.labels, detector.input_width()
            )
            candidate_key = (
                (candidate_codes, middle_mode)
                if candidate_codes is not None
                else None
            )
            candidate_history.append(candidate_key)

            vote_count = sum(
                item == candidate_key for item in candidate_history
            )
            if candidate_key is not None and vote_count >= VOTE_MIN_COUNT:
                allow_send = (
                    middle_mode != "inferred" or SEND_INFERRED_TO_STM32
                )
                if allow_send:
                    message = ",".join(candidate_codes) + "\n"
                    result_message = message
                    last_result_send_ms = (
                        time.ticks_ms() - RESULT_SEND_INTERVAL_MS
                    )
                    result_send_count = 0
                    print("Recognition complete:", message.strip())
                    recognizing = False
                    candidate_history.clear()

        for obj in objects:
            label = detector.labels[obj.class_id]
            img.draw_rect(
                obj.x,
                obj.y,
                obj.w,
                obj.h,
                color=image.COLOR_RED,
                thickness=2,
            )
            img.draw_string(
                obj.x,
                max(0, obj.y - 20),
                f"{label} {obj.score:.2f}",
                color=image.COLOR_RED,
            )

        if candidate_codes is not None:
            for region, obj in position_beans.items():
                if obj is None:
                    continue
                position = {
                    "left": POSITION_FROM_LEFT_TO_RIGHT[0],
                    "middle": POSITION_FROM_LEFT_TO_RIGHT[1],
                    "right": POSITION_FROM_LEFT_TO_RIGHT[2],
                }[region]
                img.draw_string(
                    obj.x,
                    min(detector.input_height() - 18, obj.y + obj.h + 2),
                    "P{} {}".format(position, candidate_codes[position - 1]),
                    color=image.COLOR_RED,
                )

            if middle_mode == "inferred":
                middle_code = candidate_codes[POSITION_FROM_LEFT_TO_RIGHT[1] - 1]
                img.draw_string(
                    5,
                    25,
                    "MIDDLE INFERRED: {}".format(middle_code),
                    color=image.COLOR_RED,
                )
            elif middle_mode == "verified":
                middle_code = candidate_codes[POSITION_FROM_LEFT_TO_RIGHT[1] - 1]
                img.draw_string(
                    5,
                    25,
                    "MIDDLE VERIFIED: {}".format(middle_code),
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
            img.draw_string(5, 5, "Waiting UART10: START", color=image.COLOR_RED)

        disp.show(img)
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
