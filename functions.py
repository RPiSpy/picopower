#-------------------------------------------------------
# Other Function definitions
#-------------------------------------------------------

import random
import fonts.freesans20 as pythonfont

def getPowerState(power):
    # Takes an integer power value and returns a string
    # stating what the battery is doing
    if power>0:
        # positive power
        state = "Discharge"
    elif power==0:
        # no power
        state = "Idle"
    else:
        # negative power
        state = "Charge"
    return state
    
def getRandomData():
    # Generate some random data which can be used
    # when networking has been disabled.
    # Returns 3 values.
    rspower=random.randint(0,3999)
    rbpower=random.randint(-2999,2999)
    rbsoc=random.randint(0,100)
    return rbsoc,rbpower,rspower

def getMessageWidth(msg):
    # Calculates the pixel width of a string based
    # on the data available from the python font.
    # This can be used to position the text.
    totalwidth=0
    for char in msg:
        [x,y,width]=pythonfont.get_ch(char)
        totalwidth=totalwidth+width
    return totalwidth
