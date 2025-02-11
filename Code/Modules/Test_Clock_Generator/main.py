from machine import Pin, Timer

# Initialize pins
clock_pin = Pin(2, Pin.OUT)
flag_pin = Pin(3, Pin.OUT)

# Counter to keep track of clock cycles
cycle_count = 0

# Function to toggle the clock and update the flag
def tick(timer):
    global cycle_count
    
    # Toggle the clock signal
    clock_pin.toggle()
    
    # If the clock is high, increment the cycle count
    if clock_pin.value() == 1:
        cycle_count += 1
        
        # Every 32 cycles, set the flag high for one clock cycle
        if cycle_count % 32 == 0:
            flag_pin.value(1)
        else:
            flag_pin.value(0)

# Set up a timer to generate the clock signal
# The timer will trigger every 500 microseconds (1 ms period for a 1 kHz clock)
timer = Timer()
timer.init(period=1, mode=Timer.PERIODIC, callback=tick)

# Main loop (optional, can be used for other tasks)
while True:
    pass
