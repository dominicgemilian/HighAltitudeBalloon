import math
import location
import numpy as np
import navpy
from scipy.constants import c
#import matplotlib.pyplot as plt
from scipy.special import j1

# Function that calculates ECEF look vectors between source and target
def calculateLookVectorECEF(linkSourceLocation, linkTargetLocation, debugFlag = False):

    # Get dimensions for source, target look vectors based on location array dimensions
    xSize, ySize, zSize = len(linkSourceLocation), len(linkTargetLocation), 3

    # Create empty source and target ECEF look vectors
    sourceLookVectorECEF = np.empty((xSize, ySize, zSize))
    targetLookVectorECEF = np.empty((xSize, ySize, zSize))

    # Loop through all source and target locations
    for i in range(0, len(linkSourceLocation.xECEF)):
        
        for j in range(0, len(linkTargetLocation.xECEF)):

            # Create source ECEF array
            sourceECEF = np.array([linkSourceLocation.xECEF[i], linkSourceLocation.yECEF[i], linkSourceLocation.zECEF[i]])

            # Create target ECEF array
            targetECEF = np.array([linkTargetLocation.xECEF[j], linkTargetLocation.yECEF[j], linkTargetLocation.zECEF[j]])

            # Calculate the source-target look vector
            sourceLookVector = sourceECEF - targetECEF

            # Calculate the source-target look vector
            targetLookVector = targetECEF - sourceECEF
            
            # Calculate the magnitude of the look vector
            sourceMagnitude = np.linalg.norm(sourceLookVector)

            # Calculate the magnitude of the look vector
            targetMagnitude = np.linalg.norm(targetLookVector)

            # Loop through x, y, z components
            for k in range(3):
                
                # Normalize the look vector
                sourceLookVectorECEF[i][j][k] = sourceLookVector[k] / sourceMagnitude

                # Normalize the look vector
                targetLookVectorECEF[i][j][k] = targetLookVector[k] / targetMagnitude

    if (debugFlag):
        print("=========================== Look Vector Calculation =================================")
        print("Source ECEF look vector to target: ", sourceLookVectorECEF[0][0])
        print("Target ECEF look vector to source: ", targetLookVectorECEF[0][0])
        print("Source Magnitude: ", np.sqrt(math.pow(targetLookVectorECEF[0][0][0],2) + math.pow(targetLookVectorECEF[0][0][1],2) + math.pow(targetLookVectorECEF[0][0][2],2)))
        print("Target Magnitude: ", np.sqrt(math.pow(targetLookVectorECEF[0][0][0],2) + math.pow(targetLookVectorECEF[0][0][1],2) + math.pow(targetLookVectorECEF[0][0][2],2)))


    return sourceLookVectorECEF, targetLookVectorECEF


# Function that calculates ENU and NED look vectors between source and target
def calculateLookVectorENU(linkSourceLocation, linkTargetLocation,  debugFlag = False):

    # Get sizes of each vector
    xSize, ySize, zSize = len(linkSourceLocation.xECEF), len(linkTargetLocation.yECEF), 3

    # Empty array to store look vectors
    sourceLookVectorENU = np.empty((xSize, ySize, zSize))
    targetLookVectorNED = np.empty((xSize, ySize, zSize))

    # Loop through source and target locations
    for i in range(0, len(linkSourceLocation.xECEF)):
        
        for j in range(0, len(linkTargetLocation.xECEF)):

            # Get the NED look vectors using navpy lla2ned
            targetLookVectorNED[i][j] = navpy.lla2ned(linkSourceLocation.latitude[i], linkSourceLocation.longitude[i], linkSourceLocation.altitude[i],linkTargetLocation.latitude[j], linkTargetLocation.longitude[j], linkTargetLocation.altitude[j],latlon_unit='deg', alt_unit='m', model='wgs84')
            
            # Get ENU look vectors using simple equation
            sourceLookVectorENU[i][j] = [-1*targetLookVectorNED[i][j][1],-1*targetLookVectorNED[i][j][0], 1*targetLookVectorNED[i][j][2]]
            
            # Calculate the magnitude of the target NED look vector
            targetMagnitudeNED = np.linalg.norm(targetLookVectorNED[i][j])

            # Get target NED look vector
            targetLookVectorNED[i][j][0] = targetLookVectorNED[i][j][0]/targetMagnitudeNED
            targetLookVectorNED[i][j][1] = targetLookVectorNED[i][j][1]/targetMagnitudeNED
            targetLookVectorNED[i][j][2] = targetLookVectorNED[i][j][2]/targetMagnitudeNED

            # Get source ENU look vector
            sourceMagnitudeENU = np.linalg.norm(sourceLookVectorENU[i][j])
            sourceLookVectorENU[i][j][0] = sourceLookVectorENU[i][j][0]/sourceMagnitudeENU
            sourceLookVectorENU[i][j][1] = sourceLookVectorENU[i][j][1]/sourceMagnitudeENU
            sourceLookVectorENU[i][j][2] = sourceLookVectorENU[i][j][2]/sourceMagnitudeENU

    if (debugFlag):
        print("=========================== Look Vector Calculation =================================")
        print("Source ENU look vector to target: ", sourceLookVectorENU[0][0])
        print("Target NED look vector to source: ", targetLookVectorNED[0][0])
        print("Source Magnitude: ", np.sqrt(math.pow(sourceLookVectorENU[0][0][0],2) + math.pow(sourceLookVectorENU[0][0][1],2) + math.pow(sourceLookVectorENU[0][0][2],2)))
        print("Target Magnitude: ", np.sqrt(math.pow(targetLookVectorNED[0][0][0],2) + math.pow(targetLookVectorNED[0][0][1],2) + math.pow(targetLookVectorNED[0][0][2],2)))


    return sourceLookVectorENU, targetLookVectorNED

# Function that calculates internal angle between two ENU look vectors for victim link budget 
def calculateInternalAngle(sourceLookENU, victimLookENU, debugFlag = False):

    # Get intended source and victim look angles
    xSize, sizeSource, zSize = sourceLookENU.shape
    xSize, sizeTarget, zSize = victimLookENU.shape

    # Create empty array to store internal angle
    angleRadians = np.empty((sizeSource, sizeTarget, 1))
    angleDegrees = np.empty((sizeSource, sizeTarget, 1))

    # Loop through source and target locations
    for i in range(0, sizeSource):
        
        for j in range(0, sizeTarget):
        
            # Get source and victim look vectors at current source and victim indexes
            vec1 = [sourceLookENU[0][i][0], sourceLookENU[0][i][1], sourceLookENU[0][i][2]]
            vec2 = [victimLookENU[0][j][0], victimLookENU[0][j][1], victimLookENU[0][j][2]]
                                
            # Calculate the dot product of the vectors
            dot_product = np.dot(vec1, vec2)

            # Calculate the magnitudes (lengths) of the vectors
            magnitude_vec1 = np.linalg.norm(vec1)
            magnitude_vec2 = np.linalg.norm(vec2)

            # Calculate the cosine of the angle
            cos_theta = dot_product / (magnitude_vec1 * magnitude_vec2)

            # Ensure the cosine value is within the valid range for arccos
            cos_theta = np.clip(cos_theta, -1.0, 1.0)

            # Calculate the angle in radians and then convert to degrees
            angleRadians[i][j] = np.arccos(cos_theta)
            angleDegrees[i][j] = np.degrees(angleRadians[i][j])

    if (debugFlag):
        print("=========================== Off-axis Angle Calculation ==============================")
        print("Off-axis angle: ", round(float(angleDegrees[0][0]),2))

    return angleDegrees



# Calculate spherical coordinate system antenna angles
def calculateSphericalAngles(sourceLookVectorENU, targetLookVectorNED, debugFlag = False):

    sizeSource, ySize, zSize = sourceLookVectorENU.shape
    xSize, sizeTarget, zSize = targetLookVectorNED.shape

    thetaSource = np.empty((sizeSource, sizeTarget, 1))
    phiSource = np.empty((sizeSource, sizeTarget, 1))

    thetaTarget = np.empty((sizeSource, sizeTarget, 1))
    phiTarget = np.empty((sizeSource, sizeTarget, 1))
        
    for i in range(0, sizeSource):
        
        for j in range(0, sizeTarget):

            # Calculate the magnitude of the NED vector
            rNED = np.sqrt(targetLookVectorNED[i][j][0]**2 + targetLookVectorNED[i][j][1]**2 + targetLookVectorNED[i][j][2]**2)
            
            # Calculate theta (inclination angle)
            thetaNED = np.arccos(targetLookVectorNED[i][j][2] / rNED)
            
            # Calculate phi (azimuth angle)
            phiNED = np.arctan2(targetLookVectorNED[i][j][1], targetLookVectorNED[i][j][0])
            
            # Convert angles from radians to degrees if needed
            thetaTarget[i][j] = np.degrees(thetaNED)
            phiTarget[i][j] = np.degrees(phiNED)

            # Calculate the magnitude of the NED vector
            rENU = np.sqrt(sourceLookVectorENU[i][j][0]**2 + sourceLookVectorENU[i][j][1]**2 + sourceLookVectorENU[i][j][2]**2)
            
            # Calculate theta (inclination angle)
            thetaENU = np.arccos(sourceLookVectorENU[i][j][2] / rENU)
            
            # Calculate phi (azimuth angle)
            phiENU = np.arctan2(sourceLookVectorENU[i][j][1], sourceLookVectorENU[i][j][0])
            
            # Convert angles from radians to degrees if needed
            thetaSource[i][j] = np.degrees(thetaENU)
            phiSource[i][j] = np.degrees(phiENU)    

    if (debugFlag):
        print("=========================== Spherical Source Antenna Pointing Angles ================")
        print("Source Theta: ", round(float(thetaSource[0][0]),2), " degrees")
        print("Source Phi: ", round(float(phiSource[0][0]),2), " degrees")

        print("Target Theta: ", round(float(thetaTarget[0][0]),2), " degrees")
        print("Target Phi: ", round(float(phiTarget[0][0]),2), " degrees")

    return thetaSource, phiSource, thetaTarget, phiTarget

# Function that calculates slant range between two ECEF locations
def calculateSlantRange(linkSourceLocation, linkTargetLocation, debugFlag=False):

    # Calculate slant range between each location
    slantRange = np.empty((len(linkSourceLocation.xECEF), len(linkTargetLocation.xECEF), 1))

    # Loop through source locations
    for i in range(0, len(linkSourceLocation.xECEF)):
        
        # Loop through target locations
        for j in range(0, len(linkTargetLocation.xECEF)):

            # Create source ECEF array
            sourceECEF = np.array([linkSourceLocation.xECEF[i], linkSourceLocation.yECEF[i], linkSourceLocation.zECEF[i]])

            # Create target ECEF array
            targetECEF = np.array([linkTargetLocation.xECEF[j], linkTargetLocation.yECEF[j], linkTargetLocation.zECEF[j]])

            # Calculate the difference vector
            difference_vector = targetECEF - sourceECEF
            
            # Calculate the slant range as the Euclidean norm of the difference vector
            slantRange[i][j] = np.linalg.norm(difference_vector)

    if (debugFlag):
        print("=========================== Slant Range Calculation =================================")
        print("Slant Range: ", round(float(slantRange[0][0]),2), " meters")

    return slantRange

# Function that calculates path loss between two ECEF locations
def calculatePathLoss(linkSourceLocation, linkTargetLocation, wavelength, debugFlag=False):

    # Calculate slant range between each location
    slantRange = calculateSlantRange(linkSourceLocation, linkTargetLocation)

    # Create an empty variable to store path loss
    pathLoss = np.empty((len(linkSourceLocation.xECEF), len(linkTargetLocation.xECEF), 1))

    # Loop through source ECEF locations
    for i in range(0, len(linkSourceLocation.xECEF)):
        
        # Loop through target ECEF locations
        for j in range(0, len(linkTargetLocation.xECEF)):

            # Calculate the slant range as the Euclidean norm of the difference vector
            pathLoss[i][j] = 20*np.log10((4*math.pi*slantRange[i][j])/wavelength)

    if (debugFlag):
        print("=========================== Path Loss Calculation ===================================")
        print("Path Loss: ", round(float(pathLoss[0][0]),2), " dB")

    return pathLoss

# Function that calculates spreading loss between two ECEF locations
def calculateSpreadingLoss(linkSourceLocation, linkTargetLocation, debugFlag=False):

    # Calculate slant range between each location
    slantRange = calculateSlantRange(linkSourceLocation, linkTargetLocation)

    # Empty array to store spreading loss
    spreadingLoss = np.empty((len(linkSourceLocation.xECEF), len(linkTargetLocation.xECEF), 1))

    # Loop through source and target locations
    for i in range(0, len(linkSourceLocation.xECEF)):
        
        for j in range(0, len(linkTargetLocation.xECEF)):

            # Calculate the slant range as the Euclidean norm of the difference vector
            spreadingLoss[i][j] = 10*np.log10(4*math.pi*math.pow(slantRange[i][j],2))

    if (debugFlag):
        print("=========================== Spreading Loss Calculation  =============================")
        print("Slant Range: " , round(float(slantRange[0][0]),2), " meters")
        print("Spreading Loss: ", round(float(spreadingLoss[0][0]),2), " dB/m^2")

    return spreadingLoss

# Function that calculates the incident gain at a look angle from a parabolic dish
def calculateIncidentGain(sourceLookVectorENU, targetLookVectorNED, frequency, diameter, steeredFlag, debugFlag=False):

    # Get spherical angles at point of incidence
    thetaSource, phiSource, thetaTarget, phiTarget = calculateSphericalAngles(sourceLookVectorENU, targetLookVectorNED)

    # Get dimensions for gain array
    sizeSource, ySize, zSize = thetaSource.shape
    xSize, sizeTarget, zSize = thetaTarget.shape

    # Create empty array to store gain
    gain = np.empty((sizeSource, sizeTarget, 1))

    # Loop through source and target locations
    for i in range(0, sizeSource):
        
        for j in range(0, sizeTarget):
            
            # If array is steered use the peak gain
            if (steeredFlag == True):
                thetaSource[i][j] = 0

            # Get dish gain using Bessel function
            gain[i][j] = calculateBessel(frequency, diameter, thetaSource[i][j]) 

    if (debugFlag):
        print("===========================  Incident Gain Calculation  =============================")
        print("Transmit antenna incident theta toward receiving satelllite: ", round(float(thetaSource[0][0]),2), " degrees")
        print("Transmit antenna gain toward receiving satellite: ", round(float(gain[0][0]),2), " dBi")            

    return gain

# Function that calculates incident GEO zone gain using off-axis angle
def calculateIncidentGainITU(offAxisAngles, frequency, diameter, debugFlag=False):

    # Calculate wavelength for gain formula
    wavelength = c/frequency

    # Calculate peak gain
    Gm = calculatePeakGainDish(frequency, diameter)

    # Get dimensions for gain equation
    xSize, ySize, zSize = offAxisAngles.shape

    # Create empty array to store gain
    gain = np.empty((xSize, ySize, zSize))

    # Loop through source, target locations
    for i in range(0, xSize):
      for j in range(0, ySize): 

        # Get dish gain using Bessel function
        gain[i][j] = calculateBessel(frequency, diameter, offAxisAngles[i][j]) - Gm

    if (debugFlag):
        print("=========================== ITU Receive Satellite Mask  =============================")
        print("Transmit antenna incident theta toward receiving satelllite: ", round(offAxisAngles[0][0],2), " degrees")
        print("Transmit antenna gain toward receiving satellite: ", round(gain[0][0],2), " dBi")            

    return gain

# Function that gets the ITU-R S.672-4 receiver gain mask
def calculateITU672RxGain(targetLookVectorNED,  dishDiameter, L_s, frequency, debugFlag=False):

    # # Create empty array to store geo zone satellite NED look vectors
    # Simple, just calculate the internal angle between straight dow NED look vector and each target NED look vector
    geoZoneLookVectorNED = np.empty((1, 1, 3))

    ## Straight down is [0 0 1] in NED:

    # # North
    geoZoneLookVectorNED[0][0][0] = 0

    # # East
    geoZoneLookVectorNED[0][0][1] = 0

    # # Down
    geoZoneLookVectorNED[0][0][2] = 1

    # Calculate the spherical angles toward the mask 
    theta = calculateInternalAngle(geoZoneLookVectorNED, targetLookVectorNED)   

    # Get dimensions for gain array
    sizeSource, ySize, zSize = geoZoneLookVectorNED.shape
    xSize, sizeTarget, zSize = targetLookVectorNED.shape

    # Create empty array to store gain
    gain = np.empty((sizeSource, sizeTarget, 1))

    # Empty array to store mask gain
    G = []

    # Loop through source and target locations
    for i in range(0, sizeSource):
        
        for j in range(0, sizeTarget):
        
            # Calculate ITU gain
            gain[i][j] = calculateITU672MaskGain(theta[i][j], dishDiameter, L_s, frequency)
  
    if (debugFlag):
        print("=========================== ITU Receive Satellite Mask  =============================")
        print("ITU R.672 Mask Gain: ", round(float(gain[0][0]),2), " dBi")

    return gain

# Function that generates MIL-STD-188-164C ESD mask
def mil188MaskComparison(nSamples, txDiameter, debugFlag=False):

    # Create array of off-axis angles
    theta = np.linspace(-90,90, nSamples)

    # Set empty array for dish gain
    txDishGain = np.empty((len(theta)))

    # Loop through off-axis angles
    for i in range(0,len(theta)):

        txDishGain[i] = calculateBessel(30.5E9, txDiameter, theta[i]) 

    # Create an empty arry to store gain
    G = [] 

    # Create an empty array to store off-axis angle
    oA = []

    # Set count to 1
    count = 1

    # Set current index to 0 for mask gain delta calculation
    ind = 0

    # Loop through off-axis angles
    for thetaAtInd in theta:

        # Calculate mask gain
        if(0 <= abs(thetaAtInd) and abs(thetaAtInd) < 2):
            G.append(np.nan)
            oA.append(thetaAtInd)
        if(2 <= abs(thetaAtInd) and abs(thetaAtInd) < 20):
            G.append(-6.4 - 25*np.log10(abs(thetaAtInd)))
            oA.append(thetaAtInd)
        elif(abs(thetaAtInd) >= 20 and abs(thetaAtInd) <= 26.3):
            G.append(-38.9)
            oA.append(thetaAtInd)
        elif(abs(thetaAtInd) > 26.3 and abs(thetaAtInd) < 48):
            G.append(-3.4 - 25*np.log10(abs(thetaAtInd)))
            oA.append(thetaAtInd)
        elif(abs(thetaAtInd) >= 48):
            G.append(-45.4)
            oA.append(thetaAtInd)

        # Get difference between mask and txDish gain
        diff = G[len(G) - 1] - txDishGain[ind]

        # Calculate max gain difference to set tx ESD level
        if count == 1: 
            maxdiff = diff

        if diff < maxdiff:
            maxdiff = diff

        count = count + 1
        ind = ind + 1

    if (debugFlag):
        print("=========================== MIL-STD-188-164 ESD Mask  =============================")

        fig, ax = plt.subplots()
        ax.plot(oA, G)
        ax.plot(theta, txDishGain + maxdiff)

        ax.set(xlabel='Off-axis Angle [Degrees]', ylabel='ESD [dBW/Hz]')
               
        ax.legend(['MIL-STD-188-164 ESD Mask', '%.2f Meter Parabolic Dish Radiation Pettern'  % (txDiameter)])
        ax.grid()
        plt.xlim(-90, 90)
        plt.ylim(-60,0)

        plt.show()
        plt.savefig('plot.png')
        plt.close()

    return theta, G

# Function that calculates Bessel 1st order based far-field patterh
def calculateBessel(frequency, diameter, theta):
    
    # Calculate the wavelength for antenna gain formula
    wavelength = c/frequency

    # Calculate peak gain
    Gm = calculatePeakGainDish(frequency, diameter)

    # Calculate k for Bessel function
    k = 2 * np.pi / wavelength

    # Calculate x for Bessel function
    x = k * diameter * np.sin(math.radians(theta)) / 2

    # Avoid division by zero
    x = np.where(x == 0, 1e-20, x)

    # Calculate electric fields using Bessel function of first order
    eTheta = 2 * j1(x) / x

    # Calculate x for Bessel function
    x0 = k * diameter * np.sin(math.radians(0)) / 2

    # Avoid division by zero
    x0 = np.where(x0 == 0, 1e-20, x0)

    # Calculate electric fields using Bessel function of first order
    eTheta0 = 2 * j1(x0) / x0

    # Get the boresight pTheta for normalization
    pthetaBoresight = math.pow(eTheta0,2)

    # Convert electric fields to power
    pTheta = math.pow(eTheta,2)

    # Normalize power
    pThetaNorm = pTheta/pthetaBoresight

    # Add gain to tx gain array
    dishGain = 10*np.log10(pThetaNorm) + Gm

    return dishGain

def calculateITU672MaskGain(theta, dishDiameter, L_s, frequency):

    # Calculate wavelength for gain formula
    wavelength = c/frequency

    # Set the beamwidth of the dish
    bw = (70*wavelength)/dishDiameter

    # Set the dish gain
    Gm = 10*np.log10(math.pow(((math.pi*dishDiameter)/wavelength),2))

    # Set the 0 dBi gain point
    w1 = 30

    # Set the 3 dB beamwidth
    w0 = .775

    # Determine Ls based on a,b inputs
    if L_s == -20 or L_s == -10:
        a = 2.58
        b = 6.32
    if L_s == -25:
        a = 2.88
        b = 6.32
    if L_s == -30:
        a = 3.16
        b = 6.32

    # Set off-axis angle
    oA = abs(theta)

    # Determine Ls based on a,b inputs
    if L_s == -20 or L_s == -10:
        a = 2.58
        b = 6.32
    if L_s == -25:
        a = 2.88
        b = 6.32
    if L_s == -30:
        a = 3.16
        b = 6.32

    # Set off-axis angle
    oA = abs(theta)
            
    # Calculate the mask gain
    if(w0 <= oA and oA <= a*w0):
        gain = - 3*math.pow((oA/w0),2)
    elif(a*w0 < oA and oA <= b*w0):
        gain = L_s
    elif(b*w0 < oA and oA <= w1):
        gain = L_s - 25*np.log10(oA/w0) + 25*np.log10(b*.75)
    elif(w1 < oA and oA <= 90):
        gain = -Gm
    elif(90 < oA and oA <= 180):
        gain = max(15 + L_s + .25*Gm , 0)
    else: 
        gain = np.nan

    return gain

# Function that calculates Bessel 1st order based far-field patterh
def calculatePeakGainDish(frequency, diameter):

    # Calculate the wavelength for antenna gain formula
    wavelength = c/frequency

    # Calculate peak gain
    Gm = 10*np.log10(math.pow(((math.pi*diameter)/wavelength),2))

    return Gm

# Function that calculates tx power based on antenna gain and EIRP
def eirpBasedTxPower(txGain, EIRP):

    # Calculate tx power
    txPower = math.pow(10,(EIRP - txGain)/10)

    return txPower

# Function that calculates tx power based on antenna gain and EIRP
def G_T_BasedRxTemp(rxGain, GT):

    # Calculate tx power
    rxTemp = math.pow(10,(rxGain - GT)/10)

    return rxTemp
