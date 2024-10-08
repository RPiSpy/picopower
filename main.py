# Import standard libraries
import time
import math
import network
import random
import neopixel
from machine import Pin, I2C, ADC, Timer

# Import installed packages
from ssd1306 import SSD1306_I2C
#from simple import MQTTClient
from umqtt.robust import MQTTClient
from writer import Writer

# Import local files
import config as c
import secrets as s
import functions as f

# Large Font for OLED display
import fonts.freesans20 as freesans20

#-------------------------------------------------------
# MQTT Function definitions
#-------------------------------------------------------

def mqtt_sub_cb(topic, msg):
    global batterySOC
    global batteryPower
    global solarPower
    global gridPower
    global sbatterySOC
    global sbatteryPower
    global ssolarPower
    global sgridPower

    topicText = topic.decode('utf-8')
    msgText = msg.decode('utf-8')
    print("New message on topic",topicText," : ",msgText)

    if topic==s.topic_sub_batt_soc:
        batterySOC=int(msgText)        
    elif topic==s.topic_sub_batt_power:
        batteryPower=int(msgText)
    elif topic==s.topic_sub_grid:
        gridPower=int(msgText)
    elif topic==s.topic_sub_solar:
        solarPower=int(msgText)
    else:
        print("Unknown MQTT topic")
        
    # Convert data to strings
    sbatterySOC = str(batterySOC)
    sbatteryPower = str(batteryPower)
    ssolarPower = str(solarPower)
    sgridPower  = str(gridPower)

def mqtt_connect():
    client = MQTTClient(s.mqtt_client_id, s.mqtt_server, port=1883, keepalive=120 ,user=s.mqtt_user, password=s.mqtt_password)
    client.set_callback(mqtt_sub_cb)
    client.set_last_will(s.mqtt_status_topic, 'offline', retain=True)
    client.connect()
    print('Connected to %s MQTT Broker'%(s.mqtt_server))
    return client

def mqtt_subscribeTopics():
    # Subscribe to MQTT topics
    client.subscribe(s.topic_sub_batt_soc)
    client.subscribe(s.topic_sub_grid)
    client.subscribe(s.topic_sub_solar)
    client.subscribe(s.topic_sub_batt_power)

#-------------------------------------------------------
# OLED Function definitions
#-------------------------------------------------------

def drawBatterySmall():
    # Draw Small Battery with 10 divisions
    # Draw Battery Outline
    oled.fill(0)
    oled.fill_rect(0, 0, 67, 32, 1)
    oled.fill_rect(2, 2, 63, 28, 0)
    oled.fill_rect(67, 7, 3, 18, 1)
    posX=4
    batterySteps = round(batterySOC/10)
    for step in range(batterySteps):
      posX = 4 + (6*step)
      oled.fill_rect(posX, 4, 5, 24, 1)
    batteryStatus = "B"+ "% 5s" % sbatterySOC + "%"
    solarStatus   = "S"+ "% 5s" % ssolarPower + "W"
    gridStatus    = "G"+ "% 5s" % sgridPower + "W"
    oled.text(batteryStatus, 72, 0)
    oled.text(solarStatus, 72, 10)
    oled.text(gridStatus, 72, 20)

    oled.show()

def drawBatteryLarge():
    # Draw Large Battery with 20 divisions
    # Draw Battery Outline
    oled.fill(0)
    oled.fill_rect(0, 0, 126, 32, 1)
    oled.fill_rect(2, 2, 122, 28, 0)
    oled.fill_rect(126, 7, 2, 18, 1)
    pixelX=4
    batterySteps = round(batterySOC/5)
    for step in range(batterySteps):
      pixelX = 4 + (6*step)
      oled.fill_rect(pixelX, 4, 4, 24, 1)
    if batteryPower>0:
        txtStatus = " D"
    elif batteryPower==0:
        txtStatus = " -"
    else:
        txtStatus = " C"
    txtSOC = "% 3s" % sbatterySOC + "%"
    txtPower = "% 4s" % str(abs(batteryPower)) + "W"
    txtStatus = txtSOC + " " + txtStatus + " " + txtPower
        
    oled.fill_rect(2, 9, 122, 14, 0)
    oled.text(txtStatus, 6, 12)
    
    oled.show()

def drawStatusSmall():
    # Display status text
    bSOC   = "% 3s" % sbatterySOC + "%"
    bPower = "% 5s" % sbatteryPower + "W"
    sPower = "% 5s" % ssolarPower + "W"
    gPower = "% 5s" % sgridPower + "W"
    line1  = "Batt "+bSOC + " " + bPower
    line2  = "Solar     " + sPower
    line3  = "Grid      " + gPower
    oled.fill(0)
    oled.text(line1, 0, 2)
    oled.text(line2, 0, 12)
    oled.text(line3, 0, 22)
    oled.show()

def drawStatusLarge(soc,power):

    sSOC   = "%3s" % soc + "%"
    sPower = "%4s" % abs(power) + "W"
    sState = f.getPowerState(power)
   
    sMessage = sSOC + " " + sPower
    xoffset=math.floor((128-f.getMessageWidth(sMessage))/2)
    
    oled.fill(0)
    wri = Writer(oled, freesans20, False)
    Writer.set_textpos(oled, 0, xoffset)  # verbose = False to suppress console output
    wri.printstring(sMessage)
    oled.text(sState.center(16), 2, 23)
    oled.show()

#-------------------------------------------------------
# Other Function definitions
#-------------------------------------------------------

def setBrightness(timer):
    global oled_off
    global oled
    global adc_obj
    global max_adc_val
    # Read ADC value to set OLED brightness
    adc_val = adc_obj.read_u16()
    # Calculate percentage based on max value
    contrast = round((adc_val/65535)*max_adc_val)
    if (contrast<=5):
        if (oled_off==False):
            oled_off=True
            oled.poweroff()
            print("OLED turned OFF")
    else:
        oled.contrast(contrast) # Brightness 0-255
        if (oled_off==True):
            oled_off=False
            oled.poweron()
            print("OLED turned ON")

def getColour(baseColour,brightness=20):
    # Take a colour tuple and adjust the values using the brightness value
    brightness=brightness/100
    if brightness>0 and brightness<1:
        adjustedColour=tuple([int(x*brightness) for x in baseColour])
    else:
        adjustedColour=baseColour
    return adjustedColour

def updateNeopixels(batteryPower,solarPower,gridPower,evPower):
    # Update the 4 neopixels
    
    # Battery Power
    if batteryPower==9999:
        pixels[0] = getColour(c.COLOUR_PURPLE,c.NEOPIXEL_BRIGHTNESS)
    elif batteryPower>100:
        pixels[0] = getColour(c.COLOUR_RED,c.NEOPIXEL_BRIGHTNESS)
    elif batteryPower>50:
        pixels[0] = getColour(c.COLOUR_ORANGE,c.NEOPIXEL_BRIGHTNESS)
    elif batteryPower==0:
        pixels[0] = getColour(c.COLOUR_YELLOW,c.NEOPIXEL_BRIGHTNESS)
    else:
        pixels[0] = getColour(c.COLOUR_GREEN,c.NEOPIXEL_BRIGHTNESS)
        
    # Solar Power
    if solarPower==9999:
        pixels[1] = getColour(c.COLOUR_PURPLE,c.NEOPIXEL_BRIGHTNESS)
    elif solarPower>1000:
        pixels[1] = getColour(c.COLOUR_GREEN,c.NEOPIXEL_BRIGHTNESS)
    elif solarPower>0:
        pixels[1] = getColour(c.COLOUR_ORANGE,c.NEOPIXEL_BRIGHTNESS)
    else:
        pixels[1] = getColour(c.COLOUR_YELLOW,c.NEOPIXEL_BRIGHTNESS)
        
    # Grid Power
    if gridPower==9999:
        pixels[2] = getColour(c.COLOUR_PURPLE,c.NEOPIXEL_BRIGHTNESS)
    elif gridPower>100:
        pixels[2] = getColour(c.COLOUR_RED,c.NEOPIXEL_BRIGHTNESS)
    elif gridPower>=50:
        pixels[2] = getColour(c.COLOUR_ORANGE,c.NEOPIXEL_BRIGHTNESS)
    elif gridPower<-50:
        pixels[2] = getColour(c.COLOUR_GREEN,c.NEOPIXEL_BRIGHTNESS)
    else:
        pixels[2] = getColour(c.COLOUR_YELLOW,c.NEOPIXEL_BRIGHTNESS)
    
    # EV Power
    pixels[3] = getColour(c.COLOUR_CLEAR,c.NEOPIXEL_BRIGHTNESS)
    
    pixels.write()

def reboot_pico():
    print('Rebooting ...')
    time.sleep(10)
    machine.reset()

#-------------------------------------------------------
# Start main script
#-------------------------------------------------------

# Setup OLED Screen
i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 32, i2c)
oled.contrast(c.OLED_BRIGHTNESS) # Brightness 0-255
oled.invert(c.OLED_INVERT)       # Invert 0 or 1
oled_off=False

# Setup Neopixel strip
#pixels = neopixel.NeoPixel(machine.Pin(c.NEOPIXEL_GPIO), 4)    # RGB
pixels = neopixel.NeoPixel(machine.Pin(c.NEOPIXEL_GPIO), 4, bpp=4)   # RGBW
pixels.fill(getColour(c.COLOUR_PURPLE,3))
pixels.write()

# Setup ADC
#--------------REMOVE IF NO ADC FITTED------------------
#adc_pin = Pin(c.ADC_GPIO, mode=Pin.IN)
#adc_obj = ADC(adc_pin)
#max_adc_val = 255  # value we want when ADC on 3.3V (65535)
#contrastTimer = Timer()
#contrastTimer.init(period=2000, mode=Timer.PERIODIC, callback=setBrightness)
#--------------REMOVE IF NO ADC FITTED------------------

oled.fill(0)
oled.text("Connecting to:", 0, 0)
oled.text(s.wifi_ssid, 0, 10)
oled.show()

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

print("Waiting for WiFi ", end="")

counter=1
while not wlan.isconnected():
    wlan.connect(s.wifi_ssid,s.wifi_password)
    print(".", end="")
    oled.text(">"*counter, 0, 20)
    oled.show()
    time.sleep(3)
    counter=counter+1
    if counter>10:
        print("Failed to connect to WiFi")
        oled.fill(0)
        oled.text("Failed to connect WiFi", 0, 20)
        oled.show()
        time.sleep(2)
        oled.fill(0)
        oled.show()
        counter=1
        #exit
print(".")

print(wlan.isconnected())
print(wlan.ifconfig())

ipConfig = wlan.ifconfig()
ipAddress = ipConfig[0]
ipRouter=ipConfig[2]
ipDNS=ipConfig[3]

oled.fill(0)
oled.text(ipAddress, 0, 0)
oled.text(ipRouter, 0, 10)
oled.text(ipDNS, 0, 20)
oled.show()

time.sleep(2)

# DEFAULT DATA
batterySOC = 99
batteryPower = 9999
solarPower = 9999
gridPower = 9999

# Convert data to strings
sbatterySOC   = str(batterySOC)
sbatteryPower = str(batteryPower)
ssolarPower   = str(solarPower)
sgridPower    = str(gridPower)

print("Try to connect to mqtt")    
try:
    client = mqtt_connect()
except OSError as e:
    reboot_pico()

# Subscribe to MQTT topics
mqtt_subscribeTopics()

time.sleep(1)

everyThingOk = True

loopCounter = 0
loopSleep = 1
screenCounter = 0

while everyThingOk:
    
    if wlan.isconnected() is False:
        print("No network connection!")
        oled.fill(0)
        oled.text("No network connection", 0, 20)
        oled.show()
        time.sleep(5)
        everyThingOk = False
    
    # Get random data
    #batterySOC,batteryPower,solarPower=f.getRandomData()
    
    if everyThingOk is True:
        
        # Update neopixels
        updateNeopixels(batteryPower,solarPower,gridPower,0)
             
        # Every 3 loops check for messages
        if loopCounter%3==0:
            try:
                print(" Check msg")
                client.check_msg()
            except:
                print("MQTT connection has failed!")
                oled.fill(0)
                oled.text("MQTT connection has failed!", 0, 20)
                oled.show()
                time.sleep(5)
                everyThingOk = False
        
        # Every 20 loops publish message
        if loopCounter%20==0:
                print(" Publish msg")
                client.publish(s.mqtt_status_topic,'online') 
               
        # Every 10 loops update screen
        if loopCounter%10==0:
            screenCounter+=1
            print(" Update Screen %s"%(str(screenCounter)))
            if screenCounter==1:
                drawStatusLarge(batterySOC,batteryPower)
            elif screenCounter==2:
                # Draw battery on screen
                drawBatteryLarge()
            elif screenCounter==3:
                drawStatusSmall()
                screenCounter=0

        # Every 20 loops reset counter
        if loopCounter%20==0:
            loopCounter=0

    else:
        pixels.fill(getColour(c.COLOUR_RED,10))
        pixels.write()        
        reboot_pico()

    loopCounter += 1
    print("%s Loop counter"%(str(loopCounter)))
    time.sleep(loopSleep)

# Required Libraries
# ============================================================================
# network  - using default library from firmware
# neopixel - using default library from firmware
#   
# micropython-ssd1306 by Stefan Lehmann installed via Thonny
#   https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
# Micropython umqttsimple library
#   https://mpython.readthedocs.io/en/master/library/mPython/umqtt.simple.html
