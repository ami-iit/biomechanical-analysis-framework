import numpy as np
import pandas as pd

def loadKPIreferenceParameters():
    # Returns a struct of KPI-related parameters from the state-of-the-art (soa) as follows:
    
    #  | Parameter    | Description                                             |
    #  --------------------------------------------------------------------------
    #  | EDSS         | Extended Disability Status Scale                        |
    #  | cadence      | Number of steps performed in one minute [step/min]      |
    #  | velocity     | Distance travelled per unit of time [m/s]               |
    #  | strideLenght | Distance between successive points of initial contact
    #                   of the same foot [m]                                    |
    #  | strideWidth  | Distance between lines through the midline of the two
    #                   heels [m]                                               |
    #  | strideTime   | Time of a stride, i.e., the cycle duration [s]          |
    #  | stanceTime   | Time that a foot is in contact [s]                      |
    #  | swingTime    | Time that a foot is in the air [s]                      |
    #  | dsTime       | Time of the double support [s]                          |
    #  | ssTime       | Time of the single support [s]                          |
    #  | dsPercentage | Double support time percentage w.r.t. cycle duration    |
    #  | stepLength   | Distance from a point of the ground contact of one foot
    #                   to the point of the ground contact of the other foot [m]|
    #  | stepTime     | Period of time taken for one step [s]                   |
    
    #  Please note that each column of the tables contains:
    #  - the mean value
    #  - the standard deviation
    #  - the state-of-art reference
    
    # Define parameters from state-of-the-art (soa)
    soa = {
        'stancePercentageRefValue': 60,  # percentage of stance w.r.t. stride
        'swingPercentageRefValue': 40,   # percentage of swing w.r.t. stride
        'APpercentageRefValue': 20       # percentage of Antero Posterior (AP) force w.r.t. vertical force
    }
    
    # Define table for KPI w.r.t. EDSS 
    # EDSS = 1
    level = 1
    
    cadence = np.array([
        [99, 14.5, "Kalron, 2014"],
        [114.8, 8.2, "Escudero, 2018"],
        [113.31, 6.05, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [step/min]
    
    velocity = np.array([
        [1.55, 0.14, "Muller, 2023"],
        [0.90, 0.27, "Kalron, 2014"],
        [1.165, 0.19, "Alster, 2022"],
        [1.49, 0.2, "Shammas, 2018"],
        [1.55, 0.17, "Muller, 2021"],
        [1.32, 0.211, "Escudero, 2018"],
        [1.20, 0.24, "Zanotto, 2022"],
        [1.37, 0.22, "Feys, 2013"]
    ]) # [m/s]
    
    strideLength = np.array([
        [1.53, 0.27, "Muller, 2023"],
        [1.25, 0.15, "Alster, 2022"],
        [1.52, 0.13, "Muller, 2021"],
        [1.326, 0.18, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideWidth = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideTime = np.array([
        [0.99, 0.07, "Muller, 2023"],
        [1.24, 0.19, "Kalron, 2014"],
        [1.1, 0.1, "Alster, 2022"],
        [0.99, 0.07, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stanceTime = np.array([
        [0.53, 0.04, "Muller, 2023"],
        [0.53, 0.04, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    swingTime = np.array([
        [0.46, 0.03, "Muller, 2023"],
        [0.46, 0.03, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    dsTime = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    ssTime = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stancePercentage = np.array([
        [65.3, 1.95, "Kalron, 2014"],
        [62.5, 1.7, "Escudero, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    swingPercentage = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    dsPercentage = np.array([
        [30.6, 3.8, "Kalron, 2014"],
        [25.2, 3.4, "Escudero, 2018"],
        [26.25, 4.45, "Zanotto, 2022"],
        [46.62, 10, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    ssPercentage = np.array([
        [34.7, 1.9, "Kalron, 2014"],
        [37.1, 2.8, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    stepLength = np.array([
        [0.53, 0.1, "Kalron, 2014"],
        [0.688, 0.079, "Escudero, 2018"],
        [0.6521, 0.0928, "Zanotto, 2022"],
        [0.7240, 0.0855, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    stepTime = np.array([
        [0.62, 0.10, "Kalron, 2014"],
        [0.53, 0.04, "Escudero, 2018"],
        [0.65, 0.43, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]

    EDSS = np.full(cadence.shape[0], level)
    soa['EDSS1'] = pd.DataFrame({
        'EDSS': EDSS,
        'cadence': list(map(tuple, cadence)),
        'velocity': list(map(tuple, velocity)),
        'strideLength': list(map(tuple, strideLength)),
        'strideWidth': list(map(tuple, strideWidth)),
        'strideTime': list(map(tuple, strideTime)),
        'stanceTime': list(map(tuple, stanceTime)),
        'swingTime': list(map(tuple, swingTime)),
        'dsTime': list(map(tuple, dsTime)),
        'ssTime': list(map(tuple, ssTime)),
        'stancePercentage': list(map(tuple, stancePercentage)),
        'swingPercentage': list(map(tuple, swingPercentage)),
        'dsPercentage': list(map(tuple, dsPercentage)),
        'ssPercentage': list(map(tuple, ssPercentage)),
        'stepLength': list(map(tuple, stepLength)),
        'stepTime': list(map(tuple, stepTime))
    })
    
    
    # EDSS = 2
    level = 2
    
    cadence = np.array([
        [94.4, 2.1, "Givon, 2009"],
        [89.4, 25.6, "Sacco, 2011"],
        [104.32, 7.67, "Guner, 2015"],
        [110.91, 12.3, "Fernandez, 2023"],
        [113.79, 15.61, "Pau, 2016"],
        [94.4, 18.5, "Kalron, 2013"],
        [99, 14.5, "Kalron, 2014"],
        [99.9, 24.3, "Fraser, 2019"],
        [114.8, 8.2, "Escudero, 2018"],
        [113.31, 6.05, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [step/min]

    velocity = np.array([
        [0.85, 0.03, "Givon, 2009"],
        [1.35, 0.59, "Sacco, 2011"],
        [1.26, 0.34, "Zahn, 2023"],
        [0.91, 0.11, "Guner, 2015"],
        [1.21, 0.18, "Lizrova, 2015"],
        [1.16, 0.19, "Fernandez, 2023"],
        [1.14, 0.22, "Pau, 2016"],
        [0.96, 0.27, "Severini, 2017"],
        [1.38, 0.30, "Muller, 2023"],
        [0.90, 0.27, "Kalron, 2014"],
        [1.123, 0.22, "Alster, 2022"],
        [1.49, 0.2, "Shammas, 2018"],
        [1.47, 0.19, "Muller, 2021"],
        [1.32, 0.211, "Escudero, 2018"],
        [1.20, 0.24, "Zanotto, 2022"],
        [1.37, 0.22, "Feys, 2013"]
    ]) # [m/s]

    strideLength = np.array([
        [1.05, 0.08, "Guner, 2015"],
        [1.27, 0.13, "Fernandez, 2023"],
        [1.19, 0.19, "Pau, 2016"],
        [0.751, 0.29, "Kalron, 2013"],
        [1.11, 0.18, "Severini, 2017"],
        [1.38, 0.21, "Muller, 2023"],
        [1.213, 0.172, "Alster, 2022"],
        [1.04, 0.02, "Kaipust, 2012"],
        [1.47, 0.13, "Muller, 2021"],
        [1.326, 0.18, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]

    strideWidth = np.full((16, 3), [np.nan, np.nan, "-"]) # [m]

    strideTime = np.array([
        [1.57, 1.0, "Sacco, 2011"],
        [1.10, 0.18, "Zahn, 2023"],
        [1.20, 0.19, "Severini, 2017"],
        [1.01, 0.10, "Muller, 2023"],
        [1.24, 0.19, "Kalron, 2014"],
        [1.1, 0.1, "Alster, 2022"],
        [1.28, 0.365, "Fraser, 2019"],
        [1.01, 0.07, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]

    stanceTime = np.array([
        [1.1, 0.9, "Sacco, 2011"],
        [0.71, 0.14, "Zahn, 2023"],
        [0.56, 0.07, "Muller, 2023"],
        [0.55, 0.05, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    swingTime = np.array([
        [0.47, 0.1, "Sacco, 2011"],
        [0.39, 0.04, "Zahn, 2023"],
        [0.45, 0.04, "Muller, 2023"],
        [0.46, 0.03, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    dsTime = np.array([
        [0.61, 0.8, "Sacco, 2011"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    ssTime = np.array([
        [0.47, 0.1, "Sacco, 2011"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
        
    stancePercentage = np.array([
        [64.40, 3.99, "Pau, 2016"],
        [69.4, 5.5, "Kalron, 2013"],
        [63, 2.1, "Severini, 2017"],
        [65.3, 1.95, "Kalron, 2014"],
        [59.5, 5, "Fraser, 2019"],
        [62.5, 1.7, "Escudero, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
        
    swingPercentage = np.array([
        [35.59, 3.99, "Pau, 2016"],
        [14.36, 3.99, "Pau, 2016"],
        [37, 2.1, "Severini, 2017"],
        [40.5, 5, "Fraser, 2019"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    dsPercentage = np.array([
        [25.6, 3.3, "Lizrova, 2015"],
        [14.36, 3.99, "Pau, 2016"],
        [38.8, 10.5, "Kalron, 2013"],
        [30.6, 3.8, "Kalron, 2014"],
        [23.65, 7.7, "Fraser, 2019"],
        [25.2, 3.4, "Escudero, 2018"],
        [26.25, 4.45, "Zanotto, 2022"],
        [46.62, 10, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]

    ssPercentage = np.array([
        [30.6, 5.5, "Kalron, 2013"],
        [34.7, 1.9, "Kalron, 2014"],
        [40.5, 5.5, "Fraser, 2019"],
        [37.1, 2.8, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]

    stepLength = np.array([
        [0.45, 1, "Givon, 2009"],
        [0.67, 0.06, "Lizrova, 2015"],
        [0.377, 0.148, "Kalron, 2013"],
        [0.53, 0.1, "Kalron, 2014"],
        [0.688, 0.079, "Escudero, 2018"],
        [0.6521, 0.0928, "Zanotto, 2022"],
        [0.7240, 0.0855, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]

    stepTime = np.array([
        [0.66, 0.02, "Givon, 2009"],
        [0.55, 0.05, "Lizrova, 2015"],
        [0.66, 0.15, "Kalron, 2013"],
        [0.62, 0.10, "Kalron, 2014"],
        [0.53, 0.04, "Escudero, 2018"],
        [0.65, 0.43, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]


    EDSS = np.full(cadence.shape[0], level)
    soa['EDSS2'] = pd.DataFrame({
        'EDSS': EDSS,
        'cadence': list(map(tuple, cadence)),
        'velocity': list(map(tuple, velocity)),
        'strideLength': list(map(tuple, strideLength)),
        'strideWidth': list(map(tuple, strideWidth)),
        'strideTime': list(map(tuple, strideTime)),
        'stanceTime': list(map(tuple, stanceTime)),
        'swingTime': list(map(tuple, swingTime)),
        'dsTime': list(map(tuple, dsTime)),
        'ssTime': list(map(tuple, ssTime)),
        'stancePercentage': list(map(tuple, stancePercentage)),
        'swingPercentage': list(map(tuple, swingPercentage)),
        'dsPercentage': list(map(tuple, dsPercentage)),
        'ssPercentage': list(map(tuple, ssPercentage)),
        'stepLength': list(map(tuple, stepLength)),
        'stepTime': list(map(tuple, stepTime))
    })

    
    # EDSS = 3
    level = 3
    
    cadence = np.array([
        [101.97, 14.02, "Pau, 2018"],
        [99.9, 24.3, "Fraser, 2019"],
        [102.3, 11.1, "Escudero, 2018"],
        [101.83, 11.91, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [step/min]
    
    velocity = np.array([
        [0.842, 0.228, "Alster, 2022"],
        [0.88, 0.21, "Pau, 2018"],
        [1.13, 0.44, "Shammas, 2018"],
        [1.30, 0.25, "Muller, 2021"],
        [0.924, 0.159, "Escudero, 2018"],
        [0.84, 0.27, "Zanotto, 2022"],
        [1.01, 0.11, "Feys, 2013"]
    ]) # [m/s]
    
    strideLength = np.array([
        [1.021, 0.217, "Alster, 2022"],
        [1.12, 0.20, "Pau, 2018"],
        [1.28, 0.365, "Fraser, 2019"],
        [1.04, 0.02, "Kaipust, 2012"],
        [1.33, 0.20, "Muller, 2021"],
        [1.0283, 0.3427, "Zanotto, 2022"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideWidth = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideTime = np.array([
        [1.2, 0.1, "Alster, 2022"],
        [1.04, 0.09, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stanceTime = np.array([
        [0.58, 0.07, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    swingTime = np.array([
        [0.46, 0.05, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    dsTime = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    ssTime = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stancePercentage = np.array([
        [64.53, 3.40, "Pau, 2018"],
        [59.5, 5, "Fraser, 2019"],
        [66, 2.1, "Escudero, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    swingPercentage = np.array([
        [34.72, 2.78, "Pau, 2018"],
        [40.5, 5, "Fraser, 2019"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    dsPercentage = np.array([
        [15.04, 2.97, "Pau, 2018"],
        [23.65, 7.7, "Fraser, 2019"],
        [32, 4.2, "Escudero, 2018"],
        [32.45, 7.88, "Zanotto, 2022"],
        [31.625, 3.65, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    ssPercentage = np.array([
        [40.5, 5, "Fraser, 2019"],
        [34, 3.5, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    stepLength = np.array([
        [0.541, 0.065, "Escudero, 2018"],
        [0.5085, 0.1725, "Zanotto, 2022"],
        [0.5950, 0.0640, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    stepTime = np.array([
        [0.635, 0.195, "Fraser, 2019"],
        [0.59, 0.06, "Escudero, 2018"],
        [0.585, 0.065, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    EDSS = np.full(cadence.shape[0], level)
    
    soa['EDSS3'] = pd.DataFrame({
        'EDSS': EDSS,
        'cadence': list(map(tuple, cadence)),
        'velocity': list(map(tuple, velocity)),
        'strideLength': list(map(tuple, strideLength)),
        'strideWidth': list(map(tuple, strideWidth)),
        'strideTime': list(map(tuple, strideTime)),
        'stanceTime': list(map(tuple, stanceTime)),
        'swingTime': list(map(tuple, swingTime)),
        'dsTime': list(map(tuple, dsTime)),
        'ssTime': list(map(tuple, ssTime)),
        'stancePercentage': list(map(tuple, stancePercentage)),
        'swingPercentage': list(map(tuple, swingPercentage)),
        'dsPercentage': list(map(tuple, dsPercentage)),
        'ssPercentage': list(map(tuple, ssPercentage)),
        'stepLength': list(map(tuple, stepLength)),
        'stepTime': list(map(tuple, stepTime))
    })
    
    
    # EDSS = 4
    level = 4
    
    cadence = np.array([
        [94.4, 2.1, "Givon, 2009"],
        [89.4, 25.6, "Sacco, 2011"],
        [104.32, 7.67, "Guner, 2015"],
        [110.91, 12.3, "Fernandez, 2023"],
        [113.79, 15.61, "Pau, 2016"],
        [94.4, 18.5, "Kalron, 2013"],
        [101.97, 14.02, "Pau, 2018"],
        [101.38, 13.23, "Hadouiri, 2023"],
        [102.3, 11.1, "Escudero, 2018"],
        [101.83, 11.91, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [step/min]
    
    velocity = np.array([
        [0.85, 0.03, "Givon, 2009"],
        [1.35, 0.59, "Sacco, 2011"],
        [1.26, 0.34, "Zahn, 2023"],
        [0.91, 0.11, "Guner, 2015"],
        [1.03, 0.19, "Lizrova, 2015"],
        [1.16, 0.19, "Fernandez, 2023"],
        [1.14, 0.22, "Pau, 2016"],
        [0.70, 0.22, "Severini, 2017"],
        [1.38, 0.30, "Muller, 2023"],
        [0.88, 0.21, "Pau, 2018"],
        [1.13, 0.44, "Shammas, 2018"],
        [1, 0.27, "Hadouiri, 2023"],
        [1.30, 0.25, "Muller, 2021"],
        [0.924, 0.159, "Escudero, 2018"],
        [0.84, 0.27, "Zanotto, 2022"],
        [1.01, 0.11, "Feys, 2013"]
    ]) # [m/s]
    
    strideLength = np.array([
        [1.05, 0.08, "Guner, 2015"],
        [1.27, 0.13, "Fernandez, 2023"],
        [1.19, 0.19, "Pau, 2016"],
        [0.751, 0.29, "Kalron, 2013"],
        [0.933, 0.21, "Severini, 2017"],
        [1.12, 0.20, "Pau, 2018"],
        [0.85, 0.01, "Kaipust, 2012"],
        [1.17, 0.22, "Hadouiri, 2023"],
        [1.33, 0.20, "Muller, 2021"],
        [1.0283, 0.3427, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideWidth = np.array([
        [0.15, 0.04, "Hadouiri, 2023"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideTime = np.array([
        [1.57, 1, "Sacco, 2011"],
        [1.10, 0.18, "Zahn, 2023"],
        [1.33, 0.29, "Kalron, 2013"],
        [1.39, 0.20, "Severini, 2017"],
        [1.04, 0.09, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stanceTime = np.array([
        [1.1, 0.9, "Sacco, 2011"],
        [0.71, 0.14, "Zahn, 2023"],
        [0.58, 0.07, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    swingTime = np.array([
        [0.47, 0.1, "Sacco, 2011"],
        [0.39, 0.04, "Zahn, 2023"],
        [0.46, 0.05, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    dsTime = np.array([
        [0.61, 0.8, "Sacco, 2011"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    ssTime = np.array([
        [0.47, 0.1, "Sacco, 2011"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stancePercentage = np.array([
        [64.40, 3.99, "Pau, 2016"],
        [69.4, 5.5, "Kalron, 2013"],
        [65.8, 4, "Severini, 2017"],
        [64.53, 3.40, "Pau, 2018"],
        [66, 2.1, "Escudero, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    swingPercentage = np.array([
        [35.59, 3.99, "Pau, 2016"],
        [34.2, 4, "Severini, 2017"],
        [34.72, 2.78, "Pau, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    dsPercentage = np.array([
        [29.4, 3.9, "Lizrova, 2015"],
        [14.36, 3.99, "Pau, 2016"],
        [38.8, 10.5, "Kalron, 2013"],
        [15.04, 2.97, "Pau, 2018"],
        [33.71, 5.83, "Hadouiri, 2023"],
        [32, 4.2, "Escudero, 2018"],
        [32.45, 7.88, "Zanotto, 2022"],
        [31.625, 3.65, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    ssPercentage = np.array([
        [30.6, 5.5, "Kalron, 2013"],
        [34, 3.5, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    stepLength = np.array([
        [0.45, 1, "Givon, 2009"],
        [0.58, 0.06, "Lizrova, 2015"],
        [0.377, 0.148, "Kalron, 2013"],
        [0.541, 0.065, "Escudero, 2018"],
        [0.5085, 0.1725, "Zanotto, 2022"],
        [0.5950, 0.0640, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [
        
    stepTime = np.array([
        [0.66, 0.02, "Givon, 2009"],
        [0.57, 0.07, "Lizrova, 2015"],
        [0.66, 0.15, "Kalron, 2013"],
        [0.59, 0.06, "Escudero, 2018"],
        [0.585, 0.065, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    EDSS = np.full(cadence.shape[0], level)
    
    soa['EDSS4'] = pd.DataFrame({
        'EDSS': EDSS,
        'cadence': list(map(tuple, cadence)),
        'velocity': list(map(tuple, velocity)),
        'strideLength': list(map(tuple, strideLength)),
        'strideWidth': list(map(tuple, strideWidth)),
        'strideTime': list(map(tuple, strideTime)),
        'stanceTime': list(map(tuple, stanceTime)),
        'swingTime': list(map(tuple, swingTime)),
        'dsTime': list(map(tuple, dsTime)),
        'ssTime': list(map(tuple, ssTime)),
        'stancePercentage': list(map(tuple, stancePercentage)),
        'swingPercentage': list(map(tuple, swingPercentage)),
        'dsPercentage': list(map(tuple, dsPercentage)),
        'ssPercentage': list(map(tuple, ssPercentage)),
        'stepLength': list(map(tuple, stepLength)),
        'stepTime': list(map(tuple, stepTime))
    })
    
    
    
    # EDSS = 5
    level = 5
    cadence = np.array([
        [77.80, 13.60, "Monticone, 2014"],
        [101.38, 13.23, "Hadouiri, 2023"],
        [79.6, 9.3, "Escudero, 2018"],
        [101.83, 11.91, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [step/min]
    
    velocity = np.array([
        [0.912, 0.289, "Alster, 2022"],
        [1, 0.4, "Storm, 2018"],
        [0.576, 0.193, "Monticone, 2014"],
        [1.13, 0.44, "Shammas, 2018"],
        [1, 0.27, "Hadouiri, 2023"],
        [1.30, 0.25, "Muller, 2021"],
        [0.559, 0.123, "Escudero, 2018"],
        [0.46, 0.24, "Zanotto, 2022"],
        [1.01, 0.11, "Feys, 2013"]
    ]) # [m/s]
    
    strideLength = np.array([
        [1.055, 0.184, "Alster, 2022"],
        [0.879, 0.201, "Monticone, 2014"],
        [0.85, 0.01, "Kaipust, 2012"],
        [1.17, 0.22, "Hadouiri, 2023"],
        [1.33, 0.20, "Muller, 2021"],
        [0.8433, 0.322, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideWidth = np.array([
        [0.15, 0.04, "Hadouiri, 2023"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideTime = np.array([
        [1.2, 0.03, "Alster, 2022"],
        [1.59, 0.29, "Monticone, 2014"],
        [1.04, 0.09, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stanceTime = np.array([
        [0.58, 0.07, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    swingTime = np.array([
        [0.46, 0.05, "Muller, 2021"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    dsTime = np.array([
        [0.66, 0.25, "Monticone, 2014"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    ssTime = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stancePercentage = np.array([
        [70.17, 4.15, "Monticone, 2014"],
        [69.8, 2.3, "Escudero, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    swingPercentage = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    dsPercentage = np.array([
        [33.71, 5.83, "Hadouiri, 2023"],
        [30, 4.6, "Escudero, 2018"],
        [42, 15.88, "Zanotto, 2022"],
        [31.625, 3.65, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    ssPercentage = np.array([
        [29.84, 4.07, "Monticone, 2014"],
        [29.15, 7.75, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    stepLength = np.array([
        [0.439, 0.104, "Monticone, 2014"],
        [0.419, 0.064, "Escudero, 2018"],
        [0.4171, 0.1527, "Zanotto, 2022"],
        [0.5950, 0.0640, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    stepTime = np.array([
        [0.8, 0.15, "Monticone, 2014"],
        [0.77, 0.11, "Escudero, 2018"],
        [0.585, 0.065, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    EDSS = np.full(cadence.shape[0], level)
    
    soa['EDSS5'] = pd.DataFrame({
        'EDSS': EDSS,
        'cadence': list(map(tuple, cadence)),
        'velocity': list(map(tuple, velocity)),
        'strideLength': list(map(tuple, strideLength)),
        'strideWidth': list(map(tuple, strideWidth)),
        'strideTime': list(map(tuple, strideTime)),
        'stanceTime': list(map(tuple, stanceTime)),
        'swingTime': list(map(tuple, swingTime)),
        'dsTime': list(map(tuple, dsTime)),
        'ssTime': list(map(tuple, ssTime)),
        'stancePercentage': list(map(tuple, stancePercentage)),
        'swingPercentage': list(map(tuple, swingPercentage)),
        'dsPercentage': list(map(tuple, dsPercentage)),
        'ssPercentage': list(map(tuple, ssPercentage)),
        'stepLength': list(map(tuple, stepLength)),
        'stepTime': list(map(tuple, stepTime))
    })
    
    
    
    # EDSS = 6
    level = 6
    
    cadence = np.array([
        [94.4, 2.1, "Givon, 2009"],
        [89.4, 25.6, "Sacco, 2011"],
        [94.85, 16.2, "Guner, 2015"],
        [87.27, 35.57, "Pau, 2016"],
        [77.80, 13.60, "Monticone, 2014"],
        [80.1, 14.2, "Fraser, 2019"],
        [79.6, 9.3, "Escudero, 2018"],
        [73.83, 19.08, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [step/min]
    
    velocity = np.array([
        [0.85, 0.03, "Givon, 2009"],
        [1.35, 0.59, "Sacco, 2011"],
        [1.26, 0.34, "Zahn, 2023"],
        [0.59, 0.16, "Guner, 2015"],
        [0.57, 0.19, "Lizrova, 2015"],
        [0.90, 0.39, "Pau, 2016"],
        [0.33, 0.18, "Severini, 2017"],
        [0.661, 0.248, "Alster, 2022"],
        [0.7, 0.3, "Storm, 2018"],
        [0.576, 0.193, "Monticone, 2014"],
        [0.559, 0.123, "Escudero, 2018"],
        [0.46, 0.24, "Zanotto, 2022"],
        [0.56, 0.17, "Feys, 2013"]
    ]) # [m/s]
    
    strideLength = np.array([
        [0.76, 0.19, "Guner, 2015"],
        [1.22, 0.15, "Pau, 2016"],
        [0.667, 0.20, "Severini, 2017"],
        [0.863, 0.204, "Alster, 2022"],
        [0.879, 0.201, "Monticone, 2014"],
        [1.585, 0.365, "Fraser, 2019"],
        [0.8433, 0.322, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideWidth = np.array([
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    strideTime = np.array([
        [1.57, 1, "Sacco, 2011"],
        [1.10, 0.18, "Zahn, 2023"],
        [1.33, 0.29, "Kalron, 2013"],
        [2.41, 0.99, "Severini, 2017"],
        [1.4, 0.4, "Alster, 2022"],
        [1.59, 0.29, "Monticone, 2014"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stanceTime = np.array([
        [1.1, 0.9, "Sacco, 2011"],
        [0.71, 0.14, "Zahn, 2023"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    swingTime = np.array([
        [0.47, 0.1, "Sacco, 2011"],
        [0.39, 0.04, "Zahn, 2023"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    dsTime = np.array([
        [0.61, 0.8, "Sacco, 2011"],
        [0.66, 0.25, "Monticone, 2014"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    ssTime = np.array([
        [0.47, 0.1, "Sacco, 2011"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    stancePercentage = np.array([
        [67.27, 7.56, "Pau, 2016"],
        [70.9, 8.3, "Severini, 2017"],
        [70.17, 4.15, "Monticone, 2014"],
        [61.5, 5, "Fraser, 2019"],
        [69.8, 2.3, "Escudero, 2018"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    swingPercentage = np.array([
        [32.72, 7.56, "Pau, 2016"],
        [29.1, 8.3, "Severini, 2017"],
        [38.5, 5, "Fraser, 2019"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    dsPercentage = np.array([
        [41.2, 8, "Lizrova, 2015"],
        [16.98, 7.04, "Pau, 2016"],
        [28.75, 6.2, "Fraser, 2019"],
        [30, 4.6, "Escudero, 2018"],
        [42, 15.88, "Zanotto, 2022"],
        [26.8, 5, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    ssPercentage = np.array([
        [29.84, 4.07, "Monticone, 2014"],
        [38.6, 5.2, "Fraser, 2019"],
        [29.15, 7.75, "Zanotto, 2022"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [% of gait cycle]
    
    stepLength = np.array([
        [0.45, 1, "Givon, 2009"],
        [0.43, 0.09, "Lizrova, 2015"],
        [0.439, 0.104, "Monticone, 2014"],
        [0.419, 0.064, "Escudero, 2018"],
        [0.4171, 0.1527, "Zanotto, 2022"],
        [0.4559, 0.0546, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [m]
    
    stepTime = np.array([
        [0.66, 0.02, "Givon, 2009"],
        [1.09, 0.46, "Lizrova, 2015"],
        [0.8, 0.15, "Monticone, 2014"],
        [0.79, 0.205, "Fraser, 2019"],
        [0.77, 0.11, "Escudero, 2018"],
        [0.525, 0.045, "Feys, 2013"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"],
        [np.nan, np.nan, "-"]
    ]) # [s]
    
    EDSS = np.full(cadence.shape[0], level)
    
    soa['EDSS6'] = pd.DataFrame({
        'EDSS': EDSS,
        'cadence': list(map(tuple, cadence)),
        'velocity': list(map(tuple, velocity)),
        'strideLength': list(map(tuple, strideLength)),
        'strideWidth': list(map(tuple, strideWidth)),
        'strideTime': list(map(tuple, strideTime)),
        'stanceTime': list(map(tuple, stanceTime)),
        'swingTime': list(map(tuple, swingTime)),
        'dsTime': list(map(tuple, dsTime)),
        'ssTime': list(map(tuple, ssTime)),
        'stancePercentage': list(map(tuple, stancePercentage)),
        'swingPercentage': list(map(tuple, swingPercentage)),
        'dsPercentage': list(map(tuple, dsPercentage)),
        'ssPercentage': list(map(tuple, ssPercentage)),
        'stepLength': list(map(tuple, stepLength)),
        'stepTime': list(map(tuple, stepTime))
    })
    
    return soa