import time
import network
import machine
import ubinascii
import ujson as json
from machine import Pin
import dht
from umqtt.simple import MQTTClient
import gc  # Garbage collection for memory management

# ------------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------------
WIFI_SSID = "GROUND"
WIFI_PASS = "RCA@2024"

MQTT_HOST = "broker.benax.rw"
MQTT_PORT = 1883

TOPIC_DATA = b"sensors_aaron/dht"
TOPIC_LED_CMD = b"control_aaron/led"
TOPIC_LED_STATE = b"control_aaron/led/status"

PUBLISH_INTERVAL_SEC = 10  # Increased from 5 to reduce load
UNIX_OFFSET = 946684800  # MicroPython → Unix time offset

# Connection stability settings
MAX_RECONNECT_ATTEMPTS = 3
WIFI_TIMEOUT_MS = 30000  # 30 seconds
MQTT_KEEPALIVE = 60      # Increased keepalive
# ------------------------------------------------------------------

CLIENT_ID = b"esp8266_aaron" + ubinascii.hexlify(machine.unique_id())
TOPIC_STATUS = b"iot/status/" + CLIENT_ID

# ------------------------------------------------------------------
# HARDWARE
# ------------------------------------------------------------------
sensor = dht.DHT11(Pin(5))   # D1 (GPIO5)
led = Pin(4, Pin.OUT)        # D2 (GPIO4)
led.value(0)

client = None

# ------------------------------------------------------------------
# NETWORK
# ------------------------------------------------------------------
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    # Disable power saving mode for better stability
    wlan.config(pm=0xa11140)  # Disable power management

    if not wlan.isconnected():
        print("Connecting to Wi-Fi:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASS)

        timeout = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), timeout) > WIFI_TIMEOUT_MS:
                raise RuntimeError("Wi-Fi connection timeout")
            print(".", end="")
            time.sleep(1)

    config = wlan.ifconfig()
    print("\nWi-Fi connected!")
    print("IP:", config[0])
    print("Signal strength:", wlan.status('rssi'), "dBm")
    return wlan


def check_wifi():
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        print("Wi-Fi disconnected, reconnecting...")
        wifi_connect()
    return wlan.isconnected()


def sync_time():
    try:
        import ntptime
        print("Syncing time...")
        ntptime.settime()
        print("Time synchronized")
        return True
    except Exception as e:
        print("NTP sync failed:", e)
        return False


def unix_time():
    return int(time.time() + UNIX_OFFSET)

# ------------------------------------------------------------------
# MQTT
# ------------------------------------------------------------------
def publish_led_state():
    try:
        if client and client.sock:  # Check if client is valid
            state = b"ON" if led.value() else b"OFF"
            client.publish(TOPIC_LED_STATE, state, retain=True)
            print("LED state published:", state.decode())
        else:
            print("Cannot publish LED state - no MQTT connection")
    except Exception as e:
        print("Failed to publish LED state:", e)


def mqtt_callback(topic, msg):
    try:
        print("MQTT message received:", topic, msg)
        if topic == TOPIC_LED_CMD:
            cmd = msg.decode().strip().upper()
            print("LED command:", cmd)
            
            if cmd in ("ON", "1", "TRUE"):
                led.value(1)
                print("LED turned ON")
            else:
                led.value(0)
                print("LED turned OFF")

            publish_led_state()
    except Exception as e:
        print("MQTT callback error:", e)


def mqtt_connect():
    global client
    
    for attempt in range(MAX_RECONNECT_ATTEMPTS):
        try:
            print(f"MQTT connection attempt {attempt + 1}/{MAX_RECONNECT_ATTEMPTS}")
            
            c = MQTTClient(
                client_id=CLIENT_ID,
                server=MQTT_HOST,
                port=MQTT_PORT,
                keepalive=MQTT_KEEPALIVE
            )

            c.set_last_will(TOPIC_STATUS, b"offline", retain=True)
            c.set_callback(mqtt_callback)
            
            c.connect()
            print("MQTT connected successfully")
            
            # Publish status and subscribe
            c.publish(TOPIC_STATUS, b"online", retain=True)
            c.subscribe(TOPIC_LED_CMD)
            print("Subscribed to:", TOPIC_LED_CMD.decode())
            
            return c
            
        except Exception as e:
            print(f"MQTT connection attempt {attempt + 1} failed:", e)
            if attempt < MAX_RECONNECT_ATTEMPTS - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
            else:
                raise Exception("All MQTT connection attempts failed")


def mqtt_ping():
    try:
        if client and client.sock:
            client.ping()
            return True
    except Exception as e:
        print("MQTT ping failed:", e)
        return False

# ------------------------------------------------------------------
# BOOT
# ------------------------------------------------------------------
print("=" * 50)
print("ESP8266 aaron Sensor Starting...")
print("Client ID:", CLIENT_ID.decode())
print("=" * 50)

# Initialize hardware
print("Initializing hardware...")
led.value(0)  # Ensure LED is off
print("LED initialized (OFF)")

# Connect to network
try:
    wlan = wifi_connect()
    sync_time()
    client = mqtt_connect()
    publish_led_state()
    print("✓ System initialized successfully")
    print("Publishing interval:", PUBLISH_INTERVAL_SEC, "seconds")
except Exception as e:
    print("✗ Initialization failed:", e)
    print("Restarting in 10 seconds...")
    time.sleep(10)
    machine.reset()

last_publish = time.ticks_ms()
last_ping = time.ticks_ms()
ping_interval = 30000  # Ping every 30 seconds

# ------------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------------
print("Starting main loop...")

while True:
    try:
        # Check MQTT messages
        if client and client.sock:
            client.check_msg()
        
        current_time = time.ticks_ms()
        
        # Periodic MQTT ping to maintain connection
        if time.ticks_diff(current_time, last_ping) >= ping_interval:
            last_ping = current_time
            if not mqtt_ping():
                print("MQTT ping failed, reconnecting...")
                raise Exception("MQTT ping failed")
        
        # Publish sensor data
        if time.ticks_diff(current_time, last_publish) >= PUBLISH_INTERVAL_SEC * 1000:
            last_publish = current_time
            
            # Check Wi-Fi before publishing
            if not check_wifi():
                raise Exception("Wi-Fi connection lost")

            try:
                print("Reading sensor...")
                sensor.measure()
                temp = sensor.temperature()
                hum = sensor.humidity()
                
                payload = {
                    "temperature": temp,
                    "humidity": hum,
                    "ts": unix_time()
                }

                if client and client.sock:
                    client.publish(TOPIC_DATA, json.dumps(payload), retain=True)
                    print(f"✓ Published: T={temp}°C, H={hum}%")
                    
                    # Memory management
                    gc.collect()
                    print("Free memory:", gc.mem_free(), "bytes")
                else:
                    print("✗ Cannot publish - no MQTT connection")
                    raise Exception("No MQTT connection")

            except OSError as e:
                print("✗ DHT sensor read error:", e)
            except Exception as e:
                print("✗ Publish error:", e)
                raise

        # Small delay to prevent tight loop
        time.sleep_ms(200)

    except Exception as e:
        print("✗ Main loop error:", e)
        print("Attempting recovery...")

        # Cleanup
        try:
            if client:
                client.disconnect()
        except Exception:
            pass
        
        client = None
        
        # Recovery delay
        time.sleep(5)
        
        # Reconnect
        try:
            if not check_wifi():
                wlan = wifi_connect()
            
            client = mqtt_connect()
            publish_led_state()
            print("✓ Recovery successful")
            
            # Reset timers
            last_publish = time.ticks_ms()
            last_ping = time.ticks_ms()
            
        except Exception as recovery_error:
            print("✗ Recovery failed:", recovery_error)
            print("Restarting system in 10 seconds...")
            time.sleep(10)
            machine.reset()