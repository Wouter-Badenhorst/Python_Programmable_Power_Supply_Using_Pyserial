import matplotlib.pyplot
import datetime
import pandas
import serial
import numpy
import time
import glob
import sys


# Power supply settings
# BAUD rate = 9600
# Connection type
# Interface USB
# Language GEN
# Address 06


def serial_ports():                                                             # Searches for available COM ports

    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []

    for port in ports:

        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


print('Available serial ports: ', serial_ports())                                   # Prints all available ports

with serial.Serial(port='COM3', baudrate=9600, bytesize=8, parity='N', stopbits=1,  # Initiates communication with PSU
                   xonxoff=False, rtscts=True, dsrdtr=False, timeout=1) as ser:

    ser.set_buffer_size(rx_size=12800, tx_size=12800)                               # Sets buffer sizes, arbitrary?

    ser.write_timeout = None                                                        # Some PySerial functions require
    ser.read_timeout = None                                                         # this to properly work

    if str(ser.is_open) == 'False':                                                 # Opens the port if closed
        print('\nOpening port', ser.name)
        ser.open()

    print('\nIs the port open: ', ser.is_open)                  # Checks if the port is open
    print('Is port writable: ', ser.writable())                 # Checks if the port is writable
    print('Is port readable: ', ser.readable())                 # Checks if the port is readable

    print('\nConnected device information: ', ser)              # Prints device information

    try:

        ser.write(b'ADR 06\r')                                  # Instantiates the power supply, for commands

        time.sleep(0.1)                                         # Delays the read of the data, to allow for response

        if ser.inWaiting() > 0:                                 # Validates whether input was delivered
            print('\nCommand message found in input buffer....')
        else:
            print('\nCommand message not found in input buffer')

        response = ser.read(2).decode()                         # Reads 2 bits and decodes into ASCII characters

        if response == 'OK':                                    # Checks if correct response is returned from the
            print('Instrument response:', response)             # PSU, 'OK' in this case
            print('\nInstrument ready for further commands')

        else:
            print('Instrument did not accept instantiation')    # Prints error if instantiation failed
            exit()

    except serial.SerialException as ex:
        print('Unable to write command to instrument on port', ser.name)
        exit()

    start_time = time.time()                                    # Records the initial time for drift correction
    end_time = start_time + 1 * 9                               # Sets the end time of the experiment (+ seconds)

    # Initiates all the arrays used to capture data as empty arrays, numpy.append will be used to increase their
    # length after each new data input

    date_time_list = numpy.array([])                            # Initiates the date time measurement list
    ampere_reading_list = numpy.array([])                       # Initiates the ampere measurement list
    voltage_reading_list = numpy.array([])                      # Initiates the vol measurement list
    date_time_list_seconds = numpy.array([])                    # Initiates the sec measurement list

    while time.time() < end_time:

        # The instrument does seem to flush the input and output buffer after a successful read
        # as such these two lines might be arbitrary additions, nonetheless they remain

        ser.flushInput()                                        # Flushes the input, for next command
        ser.flushOutput()                                       # Flushes the output, for next reading

        ser.write(b'MV?\r')                                     # Send the measure voltage command
        time.sleep(0.1)                                         # Allows for response from the instrument
        voltage_measurement = ser.read(10).decode()             # Reads the voltage reading from the instrument

        ser.flushInput()                                        # Flushes the input, for next command
        ser.flushOutput()                                       # Flushes the output, for next reading

        ser.write(b'MC?\r')                                     # Send the measure current command
        time.sleep(0.1)                                         # Allows for response from the instrument
        ampere_measurement = ser.read(10).decode()              # Reads the measured current reading from instrument

        # Captures all the relevant data points and appends them to the appropriate arrays
        # This might not be the optimal method for this as numpy.append creates a entirely
        # new array and copies the old data into the new one with the extra value
        # Switched over to classical python lists, occurs 10 times faster, however, still
        # using numpy for its append function

        date_time_list_seconds = numpy.append(date_time_list_seconds, (time.time()-start_time))
        date_time_list = numpy.append(date_time_list, time.asctime(time.localtime()))
        voltage_reading_list = numpy.append(voltage_reading_list, voltage_measurement)
        ampere_reading_list = numpy.append(ampere_reading_list, ampere_measurement)

        # Removal of carriage return added to values when reading from instrument (expected)
        # Increases readability and eases future use of data

        voltage_reading_list = [i.strip() for i in voltage_reading_list]
        ampere_reading_list = [i.strip() for i in ampere_reading_list]

        # As the carriage return was part of the data stored in the numpy array the values are now of the type str
        # for the values to be used and exported they should be converted back into a float (preserving decimals)

        for i in range(len(voltage_reading_list)):
            ampere_reading_list[i] = float(ampere_reading_list[i])
            voltage_reading_list[i] = float(voltage_reading_list[i])

        # Captures the real time data, this is used to determine if something has gone wrong in the internal system
        # through plotting of the data. As physically detecting problems with the run will be near impossible.

        matplotlib.pyplot.plot(date_time_list_seconds, voltage_reading_list)
        matplotlib.pyplot.ylabel('Voltage (V)')
        matplotlib.pyplot.xlabel('Time (s)')
        matplotlib.pyplot.pause(0.1)
        matplotlib.pyplot.show()

        time.sleep(3 - ((time.time() - start_time) % 3.0))      # Corrects for time drift due to code execution

    # Exporting of the data using the Pandas library, works more effortless than the csv module commonly used.
    # The index can be set to False, I however, will use it for time calculations and checking for drift of the
    # timer used in the program.

    data = pandas.DataFrame({'Time': date_time_list, 'Seconds': date_time_list_seconds,
                             'Current': ampere_reading_list, 'Volt': voltage_reading_list})
    data.to_csv(datetime.datetime.today().strftime("%d%m%Y")+'.csv', index=True)

# Indicates that the program has successfully finished and shows the name of the csv file generated containing
# all the data obtained during the run, which includes the time, amperes and the voltage applied.

print('\nRun has successfully been completed')
print('Output stored in:', datetime.datetime.today().strftime("%d%m%Y")+'.csv')
