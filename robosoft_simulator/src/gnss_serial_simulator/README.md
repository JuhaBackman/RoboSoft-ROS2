# GNSS serial simulator

`gnss_serial_simulator_node` converts simulated odometry into NMEA 0183
GGA/GST/VTG and supported attitude sentences. It writes them to a configured
serial endpoint and reconnects when that endpoint becomes available.

Measurement delay, output rate, initial WGS84 position and serial device are
parameters. A virtual test can use a `socat` pseudo-terminal pair while a
two-computer test can use physical serial adapters.
