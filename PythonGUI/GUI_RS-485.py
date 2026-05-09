import csv
from datetime import datetime
from pathlib import Path
import re
import time

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button
import serial

CONTROLLER_SERIAL_PORT = 'COM7'
CONTROLLER_BAUD_RATE = 921600

SET_SERIAL_PORT = 'COM8'
SET_BAUD_RATE = 115200

NILOTECH_DATALOG_ROOT = Path(r'C:\Users\kostas.caplinskas\Documents\Release\Datalog')
NILOTECH_POLL_INTERVAL_S = 0.2

OUTPUT_FILE = 'duomenys.csv'

DEFAULT_VISIBLE_POINTS = 512
MIN_VISIBLE_POINTS = 50
MAX_VISIBLE_POINTS = 50000

SEND_SEPARATOR = ', '
SEND_LINE_ENDING = '\r\n'

# Controller telemetry packet format:
# TEL,<counter>,<tick>,<dt_s>,<pt1000_c>,<lm35_c>,<sp_pt_c>,<sp_lm_c>,<pwm_pt_pct>,<pwm_lm_pct>,
#     <raw_pt>,<raw_lm>,<aux_raw>,
#     <pt_enable>,<pt_p>,<pt_i>,<pt_d>,
#     <lm_enable>,<lm_p>,<lm_i>,<lm_d>
TEL_TAG = 'TEL'
TEL_MIN_FIELDS = 21
IDX_COUNTER = 1
IDX_TICK = 2
IDX_DT = 3
IDX_PT1000 = 4
IDX_LM35 = 5
IDX_SP_PT = 6
IDX_SP_LM = 7
IDX_PWM_PT = 8
IDX_PWM_LM = 9
IDX_RAW_PT = 10
IDX_RAW_LM = 11
IDX_AUX_RAW = 12
IDX_PT_ENABLE = 13
IDX_PT_P = 14
IDX_PT_I = 15
IDX_PT_D = 16
IDX_LM_ENABLE = 17
IDX_LM_P = 18
IDX_LM_I = 19
IDX_LM_D = 20

FLOAT_PATTERN = re.compile(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?')

SET_PATTERN = re.compile(
    r'\bSET\s+PT=('
    r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
    r')\s+LM=('
    r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
    r')\b',
    re.IGNORECASE
)

# Buffers
time_data = []
pt_data = []
lm_data = []
pwm_pt_data = []
pwm_lm_data = []
nilotech_pt_data = []
nilotech_lm_data = []

start_time = time.time()
visible_points = DEFAULT_VISIBLE_POINTS

latest_controller_sample = None
latest_nilotech_sample = None
latest_com10_set_sample = None

latest_sent_top = None
latest_sent_bottom = None
latest_sent_command = ''

last_nilotech_poll = 0.0
last_nilotech_row_key = None
last_nilotech_warning = 0.0

manual_send_updated = False


def parse_tel_line(line):
    parts = [part.strip() for part in line.split(',')]
    if len(parts) < TEL_MIN_FIELDS or parts[0] != TEL_TAG:
        return None

    try:
        return {
            'counter': int(parts[IDX_COUNTER]),
            'tick': int(parts[IDX_TICK]),
            'dt_s': float(parts[IDX_DT]),
            'pt1000': float(parts[IDX_PT1000]),
            'lm35': float(parts[IDX_LM35]),
            'sp_pt': float(parts[IDX_SP_PT]),
            'sp_lm': float(parts[IDX_SP_LM]),
            'pwm_pt': float(parts[IDX_PWM_PT]),
            'pwm_lm': float(parts[IDX_PWM_LM]),
            'raw_pt': int(parts[IDX_RAW_PT]),
            'raw_lm': int(parts[IDX_RAW_LM]),
            'aux_raw': int(parts[IDX_AUX_RAW]),
            'pt_enable': int(parts[IDX_PT_ENABLE]),
            'pt_p': float(parts[IDX_PT_P]),
            'pt_i': float(parts[IDX_PT_I]),
            'pt_d': float(parts[IDX_PT_D]),
            'lm_enable': int(parts[IDX_LM_ENABLE]),
            'lm_p': float(parts[IDX_LM_P]),
            'lm_i': float(parts[IDX_LM_I]),
            'lm_d': float(parts[IDX_LM_D]),
        }
    except (ValueError, IndexError):
        return None


def parse_set_line(line):
    match = SET_PATTERN.search(line)
    if match is None:
        return None

    try:
        return {
            'set_pt': float(match.group(1)),
            'set_lm': float(match.group(2)),
        }
    except ValueError:
        return None


def open_serial_port(port, baud_rate, required=False):
    try:
        return serial.Serial(port, baud_rate, timeout=0)
    except serial.SerialException as exc:
        if required:
            raise

        print(f'Warning: {port} could not be opened ({exc}). Continuing without it.')
        return None


def parse_nilotech_csv_float(value):
    if value is None:
        return None

    match = FLOAT_PATTERN.search(value.replace(',', '.'))
    if match is None:
        return None

    return float(match.group())


def find_latest_nilotech_datalog():
    if not NILOTECH_DATALOG_ROOT.exists():
        return None

    files = list(NILOTECH_DATALOG_ROOT.rglob('datalog_*.csv'))
    if not files:
        return None

    return max(files, key=lambda file_path: file_path.stat().st_mtime)


def read_latest_nilotech_datalog_sample():
    latest_file = find_latest_nilotech_datalog()
    if latest_file is None:
        return None

    try:
        with latest_file.open('r', encoding='utf-8-sig', errors='ignore') as datalog_file:
            lines = datalog_file.readlines()
    except OSError as exc:
        print(f'Nilotech datalog read failed: {exc}')
        return None

    for line_number, line in reversed(list(enumerate(lines, start=1))):
        parts = [part.strip() for part in line.strip().split(';')]
        if len(parts) < 3:
            continue

        pt1000 = parse_nilotech_csv_float(parts[1])
        lm35 = parse_nilotech_csv_float(parts[2])
        if pt1000 is None or lm35 is None:
            continue

        row_key = (str(latest_file), line_number, line.strip())
        if row_key == last_nilotech_row_key:
            return None

        print(f'Nilotech datalog: {parts[0]} A/PT1000={pt1000} B/LM35={lm35}')
        return {
            'pt1000': pt1000,
            'lm35': lm35,
            'row_key': row_key,
        }

    return None


def parse_user_temperature(value):
    return float(value.strip().replace(',', '.'))


controller_ser = open_serial_port(CONTROLLER_SERIAL_PORT, CONTROLLER_BAUD_RATE, required=False)
set_ser = open_serial_port(SET_SERIAL_PORT, SET_BAUD_RATE, required=False)

file = open(OUTPUT_FILE, mode='w', newline='')
writer = csv.writer(file)
writer.writerow([
    'Timestamp',
    'Time_s',

    'Counter',
    'Tick',
    'Packet_dt_s',
    'PT1000',
    'LM35',
    'Setpoint_PT_Controller',
    'Setpoint_LM_Controller',
    'PWM_PT',
    'PWM_LM',
    'Raw_PT',
    'Raw_LM',
    'Aux_Raw',
    'PT_Enable',
    'PT_P',
    'PT_I',
    'PT_D',
    'LM_Enable',
    'LM_P',
    'LM_I',
    'LM_D',

    'COM10_SET_PT',
    'COM10_SET_LM',

    'Nilotech_A_PT1000',
    'Nilotech_B_LM35',

    'Sent_Top',
    'Sent_Bottom',
    'Sent_Command',
])

plt.ion()
fig, axs = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
fig.subplots_adjust(bottom=0.20, top=0.92)

line_pt, = axs[0].plot([], [], label='Controller PT1000')
axs[0].set_ylabel('PT1000 (C)')
axs[0].grid()

line_lm, = axs[1].plot([], [], label='Controller LM35', color='orange')
axs[1].set_ylabel('LM35 (C)')
axs[1].ticklabel_format(style='plain')
axs[1].grid()

line_pwm_pt, = axs[2].plot([], [], label='PWM PT1000', color='green')
axs[2].set_ylabel('PWM PT (%)')
axs[2].grid()

line_pwm_lm, = axs[3].plot([], [], label='PWM LM35', color='red')
axs[3].set_ylabel('PWM LM (%)')
axs[3].grid()

line_nilotech_pt, = axs[4].plot([], [], label='Nilotech A PT1000', color='purple')
axs[4].set_ylabel('Nilo A PT1000 (C)')
axs[4].grid()

line_nilotech_lm, = axs[5].plot([], [], label='Nilotech B LM35', color='brown')
axs[5].set_ylabel('Nilo B LM35 (C)')
axs[5].set_xlabel('Time (s)')
axs[5].grid()

for ax in axs:
    ax.legend(loc='upper left')

status_text = fig.text(
    0.5,
    0.965,
    'COM10 SET PT: ---   LM: ---   |   Last sent: ---',
    ha='center',
    va='top',
    fontsize=10,
)

slider_ax = fig.add_axes([0.15, 0.10, 0.7, 0.03])
points_slider = Slider(
    ax=slider_ax,
    label='Visible points',
    valmin=MIN_VISIBLE_POINTS,
    valmax=MAX_VISIBLE_POINTS,
    valinit=DEFAULT_VISIBLE_POINTS,
    valstep=10,
)

top_ax = fig.add_axes([0.15, 0.045, 0.18, 0.035])
bottom_ax = fig.add_axes([0.43, 0.045, 0.18, 0.035])
send_ax = fig.add_axes([0.68, 0.045, 0.17, 0.035])

top_textbox = TextBox(top_ax, 'Top', initial='37.2')
bottom_textbox = TextBox(bottom_ax, 'Bottom', initial='37.0')
send_button = Button(send_ax, 'Send COM10')


def update_status_text():
    if latest_com10_set_sample is None:
        set_pt_text = '---'
        set_lm_text = '---'
    else:
        set_pt_text = f"{latest_com10_set_sample.get('set_pt'):.2f}"
        set_lm_text = f"{latest_com10_set_sample.get('set_lm'):.2f}"

    if latest_sent_command:
        sent_text = latest_sent_command
    else:
        sent_text = '---'

    status_text.set_text(
        f'COM10 SET PT: {set_pt_text}   LM: {set_lm_text}   |   Last sent: {sent_text}'
    )


def update_visible_window():
    if not time_data:
        return

    visible_count = min(int(visible_points), len(time_data))
    start_idx = max(0, len(time_data) - visible_count)

    x_view = time_data[start_idx:]
    pt_view = pt_data[start_idx:]
    lm_view = lm_data[start_idx:]
    pwm_pt_view = pwm_pt_data[start_idx:]
    pwm_lm_view = pwm_lm_data[start_idx:]
    nilotech_pt_view = nilotech_pt_data[start_idx:]
    nilotech_lm_view = nilotech_lm_data[start_idx:]

    line_pt.set_data(x_view, pt_view)
    line_lm.set_data(x_view, lm_view)
    line_pwm_pt.set_data(x_view, pwm_pt_view)
    line_pwm_lm.set_data(x_view, pwm_lm_view)
    line_nilotech_pt.set_data(x_view, nilotech_pt_view)
    line_nilotech_lm.set_data(x_view, nilotech_lm_view)

    if len(x_view) == 1:
        x_min = x_view[0]
        x_max = x_view[0] + 1
    else:
        x_min = x_view[0]
        x_max = x_view[-1]
        if x_max <= x_min:
            x_max = x_min + 1

    axs[-1].set_xlim(x_min, x_max)

    for ax in axs:
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)


def on_slider_change(value):
    global visible_points
    visible_points = int(value)
    update_visible_window()
    fig.canvas.draw_idle()


def on_send_button_clicked(event):
    global latest_sent_top
    global latest_sent_bottom
    global latest_sent_command
    global manual_send_updated

    if set_ser is None:
        print(f'Cannot send: {SET_SERIAL_PORT} is not open.')
        return

    try:
        top = parse_user_temperature(top_textbox.text)
        bottom = parse_user_temperature(bottom_textbox.text)
    except ValueError:
        print('Invalid Top/Bottom value. Use numbers like 37.2 or 37,2.')
        return

    command = f'{top:.1f}{SEND_SEPARATOR}{bottom:.1f}{SEND_LINE_ENDING}'

    try:
        set_ser.write(command.encode('ascii'))
        set_ser.flush()
    except serial.SerialException as exc:
        print(f'COM10 send failed: {exc}')
        return

    latest_sent_top = top
    latest_sent_bottom = bottom
    latest_sent_command = command.replace('\n', '\\n').replace('\r', '\\r')
    manual_send_updated = True

    update_status_text()
    fig.canvas.draw_idle()

    print(f'Sent to {SET_SERIAL_PORT}: {latest_sent_command}')


points_slider.on_changed(on_slider_change)
send_button.on_clicked(on_send_button_clicked)

plt.tight_layout(rect=[0, 0.16, 1, 0.94])
plt.show(block=False)
fig.canvas.draw_idle()
fig.canvas.flush_events()

print(f'Real-time telemetry logging from controller {CONTROLLER_SERIAL_PORT}.')
print(f'Reading SET packets from {SET_SERIAL_PORT}, example: SET PT=25.20 LM=25.00')
print(f'Reading latest Nilotech datalog under: {NILOTECH_DATALOG_ROOT}')
print(f'Use Top/Bottom fields and Send COM10 button to send: 37.2,37.0\\n\\r')
print('Ctrl+C to stop.')

try:
    while True:
        data_updated = False

        controller_line = ''
        if controller_ser is not None:
            controller_line = controller_ser.readline().decode('utf-8', errors='ignore').strip()

        if controller_line:
            print(f'Controller: {controller_line}')
            sample = parse_tel_line(controller_line)
            if sample is not None:
                latest_controller_sample = sample
                data_updated = True

        set_line = ''
        if set_ser is not None:
            set_line = set_ser.readline().decode('utf-8', errors='ignore').strip()

        if set_line:
            print(f'COM10: {set_line}')
            sample = parse_set_line(set_line)
            if sample is not None:
                latest_com10_set_sample = sample
                update_status_text()
                data_updated = True

        if time.time() - last_nilotech_poll >= NILOTECH_POLL_INTERVAL_S:
            last_nilotech_poll = time.time()
            sample = read_latest_nilotech_datalog_sample()

            if sample is not None:
                last_nilotech_row_key = sample.pop('row_key')
                latest_nilotech_sample = sample
                data_updated = True
            elif latest_nilotech_sample is None and time.time() - last_nilotech_warning >= 10.0:
                last_nilotech_warning = time.time()
                print(
                    f'Waiting for Nilotech datalog rows under {NILOTECH_DATALOG_ROOT}. '
                    f'Start a datalog session in ProbeConfigurator.'
                )

        if manual_send_updated:
            data_updated = True
            manual_send_updated = False

        if not data_updated:
            plt.pause(0.001)
            continue

        now = time.time() - start_time
        controller = latest_controller_sample or {}
        nilotech = latest_nilotech_sample or {}
        com10_set = latest_com10_set_sample or {}

        writer.writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            now,

            controller.get('counter'),
            controller.get('tick'),
            controller.get('dt_s'),
            controller.get('pt1000'),
            controller.get('lm35'),
            controller.get('sp_pt'),
            controller.get('sp_lm'),
            controller.get('pwm_pt'),
            controller.get('pwm_lm'),
            controller.get('raw_pt'),
            controller.get('raw_lm'),
            controller.get('aux_raw'),
            controller.get('pt_enable'),
            controller.get('pt_p'),
            controller.get('pt_i'),
            controller.get('pt_d'),
            controller.get('lm_enable'),
            controller.get('lm_p'),
            controller.get('lm_i'),
            controller.get('lm_d'),

            com10_set.get('set_pt'),
            com10_set.get('set_lm'),

            nilotech.get('pt1000'),
            nilotech.get('lm35'),

            latest_sent_top,
            latest_sent_bottom,
            latest_sent_command,
        ])
        file.flush()

        time_data.append(now)
        pt_data.append(controller.get('pt1000', float('nan')))
        lm_data.append(controller.get('lm35', float('nan')))
        pwm_pt_data.append(controller.get('pwm_pt', float('nan')))
        pwm_lm_data.append(controller.get('pwm_lm', float('nan')))
        nilotech_pt_data.append(nilotech.get('pt1000', float('nan')))
        nilotech_lm_data.append(nilotech.get('lm35', float('nan')))

        update_status_text()
        update_visible_window()

        plt.pause(0.001)

except KeyboardInterrupt:
    print('Stop.')
finally:
    if controller_ser is not None:
        controller_ser.close()
    if set_ser is not None:
        set_ser.close()
    file.close()