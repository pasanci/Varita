import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

# Change this to your serial port (e.g., 'COM3' on Windows)
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

if len(sys.argv) > 1:
    SERIAL_PORT = sys.argv[1]

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

rolls, pitches, yaws = [], [], []
max_points = 200

fig, ax = plt.subplots()
ax.set_title('MPU6050 Roll, Pitch, Yaw')
ax.set_xlabel('Sample')
ax.set_ylabel('Angle (deg)')
line_roll, = ax.plot([], [], label='Roll')
line_pitch, = ax.plot([], [], label='Pitch')
line_yaw, = ax.plot([], [], label='Yaw')
ax.legend()


def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        try:
            roll, pitch, yaw = map(float, line.split(','))
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)
            if len(rolls) > max_points:
                rolls.pop(0)
                pitches.pop(0)
                yaws.pop(0)
        except ValueError:
            continue
    line_roll.set_data(range(len(rolls)), rolls)
    line_pitch.set_data(range(len(pitches)), pitches)
    line_yaw.set_data(range(len(yaws)), yaws)
    ax.relim()
    ax.autoscale_view()
    return line_roll, line_pitch, line_yaw

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.show()
ser.close()
