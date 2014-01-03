#!/usr/bin/python  -tt
# Kalman.py

Q_angle = 0     # Process noise variance for the accelerometer
Q_bias  = 0     # Process noise variance for the gyro bias
R_measure = 0   # Measurement noise variance - \
angle = 0  # The angle calculated by the Kalman filter - \
               # part of the 2x1 state matrix
bias = 0  # The gyro bias calculated by the Kalman filter - \
                 #part of the 2x1 state matrix
#float(rate)  # Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    # Kalman filter variables #
          # this is actually the variance of the measurement noise
P = [[00,01],[10,11]]
K = [0,1]
#   P = [2,2],[2,2]  # Error covariance matrix - This is a 2x2 matrix
#   float(K[2])  # Kalman gain - This is a 2x1 matrix
#   float(y)  # Angle difference - 1x1 matrix
#   float(S)  # Estimate error - 1x1 matrix

#   accXangle = accYangle = 0#;  # Angle calculate using the accelerometer
#   gyroXangle =  gyroYangle = 0 # Angle calculate using the gyro
#   kalAngleX = kalAngleY = 0#;  # Calculate the angle using a Kalman filter

class Kalman():

    '''
    accXangle = accYangle = 0#; #// Angle calculate using the accelerometer
# #/* IMU Data #
    accX =  accY = accZ = 0#;
    tempRaw = 0#;
    gyroX =  gyroY = gyroZ = 0#;
    temp = 0#; #// Temperature
    gyroXangle =  gyroYangle = 0#; #// Angle calculate using the gyro
    compAngleX =  compAngleY = 0#; #// Calculate the angle using a complementary    filter
    kalAngleX = kalAngleY = 0#; #// Calculate the angle using a Kalman filter
'''
    timer = 0 #;

    def  Kalman(self):

# We will set the varibles like so, these can also be tuned by the user
        Q_angle = 0.001 # Process noise variance for the accelerometer
        Q_bias = 0.003  # Process noise variance for the gyro bias
        R_measure = 0.03 # Measurement noise variance - \
            #this is actually the variance  of the measurement noise
        P[0][0] = 0  # Since we assume tha the bias is 0 and we know the\
            # starting angle (use setAngle), the error covariance matrix \
            # is set like so - see: \
            # http:#en.wikipedia.org/wiki/Kalman_filter#Example_application.\
            # 2C_technical
        P[0][1] = 0
        P[1][0] = 0
        P[1][1] = 0

        # The angle should be in degrees and the rate should be in \
        # degrees per second and the delta time in seconds
    def getAngle(self,newAngle, newRate, dt):

        global bias
        global angle
        # KasBot V2 - Kalman filter module - http:#www.x-firm.com/?page_id=145
        # Modified by Kristian Lauszus
        # See my blog post for more information: \
            #http:#blog.tkjelectronics.dk/2012/09/a-practical-approach-to-\
            #kalman-filter-and-how-to-implement-it

        # Discrete Kalman filter time update equations - Time Update ("Predict")
        # Update xhat - Project the state ahead
            # Step 1
        rate = newRate - bias;
        angle += dt * rate

        # Update estimation error covariance - Project the \
                #error covariance ahead
           # Step 2
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
        P[0][1] -= dt * P[1][1]
        P[1][0] -= dt * P[1][1]
        P[1][1] += Q_bias * dt

        # Discrete Kalman filter measurement update equations - \
            # Measurement Update ("Correct")
        # Calculate Kalman gain - Compute the Kalman gain
            # Step 4
        S = P[0][0] + R_measure
           #  Step 5
        K[0] = P[0][0] / S
        K[1] = P[1][0] / S

        # Calculate angle and bias - Update estimate with measurement \
                    #zk (newAngle)
           # Step 3
        y = newAngle - angle
           # Step 6
        angle += K[0] * y
        bias += K[1] * y

        # Calculate estimation error covariance - Update the error covariance
           #  Step 7
        P[0][0] -= K[0] * P[0][0]
        P[0][1] -= K[0] * P[0][1]
        P[1][0] -= K[1] * P[0][0]
        P[1][1] -= K[1] * P[0][1]

        return angle

    def setAngle(self,newAngle):
        global angle
        angle = newAngle # Used to set angle, this should be set \
                        #as the starting angle
#def float(getRate()):
#    return rate   # Return the unbiased rate

      # These are used to tune the Kalman filter
    def  setQangle(self,newQ_angle): Q_angle = newQ_angle
    def  setQbias(self,newQ_bias):   Q_bias = newQ_bias
    def  setRmeasure(self,newR_measure):   R_measure = newR_measure


