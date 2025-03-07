import numpy as np
import idyntree.bindings as iDynTree
import matplotlib.pyplot as plt


def computeNioshKPI(kinDynComp, joints_state, human_state, W_T_base, KPI, TIMESTAMPS):
    
    ABS_TIMESTAMPS = TIMESTAMPS - TIMESTAMPS[0]
    
    # Define constant metrics
    LC = 23 # kg
    FREQUENCIES = np.array([0.2, 0.5, 1, 2, 3, 4,
                                     5, 6, 7, 8, 9, 10, 11,
                                     12, 13, 14, 15, 15.5]) # min 0.2lifts/min, max 15lifts/min
    
    DURATIONS = [1, 2, 8] # 1:lifting < 1h, 2:lifitng 1~2h, 8:lifitng 2~8h
    CUTOFFS = {1: 14, 2: 12, 8: 10}
    FM_TABLE = {1: [1.00, 0.97, 0.94, 0.91, 0.88, 0.84,
                            0.80, 0.75, 0.70, 0.60, 0.52, 0.45,
                            0.41, 0.37, 0.34, 0.31, 0.28, 0.00],
                        2: [0.95, 0.92, 0.88, 0.84, 0.79, 0.72,
                            0.60, 0.50, 0.42, 0.35, 0.30, 0.26,
                            0.23, 0.21, 0.00, 0.00, 0.00, 0.00],
                        8: [0.85, 0.81, 0.75, 0.65, 0.55, 0.45,
                            0.35, 0.27, 0.22, 0.18, 0.15, 0.13,
                            0.00, 0.00, 0.00, 0.00, 0.00, 0.00]}
    
    # Coupling multiplier fo the quality of gripping
    COUPLINGS = {"Good": [1.0, 1.0], "Fair": [0.95, 1.00], "Poor": [0.9, 0.9]}
    
    # define NIOSH equation variables
    A = 0.0 # Angle of Asymmerty
    F = 0.2 # lifts per minute
    
    print("============================================")
    print("[INFO] Please check current payload weight: ")
    print("============================================")
    weightLoad = float(input())
    
    
    
    s = iDynTree.JointPosDoubleArray(kinDynComp.model())
    ds = iDynTree.JointDOFsDoubleArray(kinDynComp.model())
    base_vel = iDynTree.Twist()
    gravity = iDynTree.Vector3.FromPython([0.0, 0.0, -9.81])
    
    
    links_lhand, links_rhand, links_lfoot, links_rfoot = [], [], [], []
    
    for lenIdx in range(KPI['len']):
        # Set the joint positions
        s = joints_state['positions']['data'][lenIdx][0]
        ds = joints_state['velocities']['data'][lenIdx][0]
        lin_vel = human_state['base_linear_velocity']['data'][lenIdx][0]
        ang_vel = human_state['base_angular_velocity']['data'][lenIdx][0]
        base_vel = np.concatenate((lin_vel, ang_vel))        
        
        kinDynComp.setRobotState(W_T_base[lenIdx], s, base_vel, ds, gravity)
        
        # Get the desired link configuration wrt world frame for retrieving H, V and D
        # Left hand wrt world 
        H_link_lhand = kinDynComp.getWorldTransform("LeftHandCOM")
        H_link_lhand_np = H_link_lhand.asHomogeneousTransform().toNumPy()
        link_lhand_pos = H_link_lhand_np[0:3,3]
        links_lhand.append(link_lhand_pos)
        
        # Right hand wrt world
        H_link_rhand = kinDynComp.getWorldTransform("RightHandCOM")
        H_link_rhand_np = H_link_rhand.asHomogeneousTransform().toNumPy()
        link_rhand_pos = H_link_rhand_np[0:3,3]
        links_rhand.append(link_rhand_pos)
        
        # Left foot wrt world
        H_link_lfoot = kinDynComp.getWorldTransform("LeftFoot")  
        H_link_lfoot_np = H_link_lfoot.asHomogeneousTransform().toNumPy()
        link_lfoot_pos = H_link_lfoot_np[0:3,3]
        links_lfoot.append(link_lfoot_pos)
        
        # Right foot wrt world
        H_link_rfoot = kinDynComp.getWorldTransform("RightFoot")
        H_link_rfoot_np = H_link_rfoot.asHomogeneousTransform().toNumPy()
        link_rfoot_np = H_link_rfoot_np[0:3,3]
        links_rfoot.append(link_rfoot_np)
        
    # Distance between left hand and right hand for each timestamp     
    midpoint_hands = (np.array(links_lhand) + np.array(links_rhand))/2
    # Distance between left foot and right foot for each timestamp
    midpoint_feets = (np.array(links_lfoot) + np.array(links_rfoot))/2
            
        
    # Prepare NIOSH parameters and multipliers
    Hs, HMs, Vs, VMs, Ds, DMs, AMs, CMs, FMs, RWLs, LIs = ([] for i in range(11))
        
    V_max = findV(midpoint_feets[0][2], midpoint_hands[0][2])
    hands_zMax = midpoint_hands[0][2]
    
    
    for i in range(len(links_lhand)):
        # Horizontal distance H & HM 
        H = findH(midpoint_feets[i][0], midpoint_hands[i][0])
        
        Hs.append(H)
        HM = getHM(H)
        HMs.append(HM)
        
        # Vertical distance V & VM 
        V_now = findV(midpoint_feets[i][2], midpoint_hands[i][2])
        
        if V_now >= V_max:
            V, V_max = V_now, V_now
            hands_zMax = midpoint_hands[0][2]
        else:
            # V = V_max
            V = V_now
            
        Vs.append(V)
        VM = getVM(V)
        VMs.append(VM)
        
        # Vertical traveling distance D & DM
        D = findD(midpoint_hands[0][2], hands_zMax)
        Ds.append(D)
        DM = getDM(D)
        DMs.append(DM)
        
        # Asymmetry angle is as default 0
        AM = getAM(A)
        AMs.append(AM)
        
        # Coupling state default as 'Fair'
        CM = getCM(V, 'Fair', COUPLINGS)
        CMs.append(CM)
        
        # Lifting frequency as default 7 lifts/min
        FM = getFM(F, FREQUENCIES, V, CUTOFFS, FM_TABLE, duration=1)
        FMs.append(FM)
        
        # Prepare RWL 
        RWL = getRWL(HM, VM, DM, AM, CM, FM, LC)
        RWLs.append(RWL)
        
        # Prepare LI
        LI = getLI(weightLoad, RWL)
        LIs.append(LI)
    
    NioshKPI = {'RWLs': RWLs, 'LIs': LIs, 'Hs': Hs, 'Vs': Vs, 'Ds': Ds}
            
            
    # Plot NIOSH KPIs
    # plotNioshKPIs(RWLs, LIs, Hs, Vs, Ds, ABS_TIMESTAMPS)
    
            
    print("============================================")
    print("[INFO] NIOSH KPIs for the current payload: ")
    print("============================================")
    print("Mean RWL: ", np.mean(RWLs))
    print("Mean LI: ", np.mean(LIs))
    print("============================================")
    
    return NioshKPI
            
        
        
    

def computePosition(posl, posr, direction):
    left_pos = posl[direction]
    right_pos = posr[direction]
    
    return (left_pos + right_pos) / 2


# Find horizontal distance H
def findH(jL5S1_x0, jHand_x0):
    return np.abs(jHand_x0 - jL5S1_x0) * 100    

# Compute horizontal multiplier factor HM 
def getHM(H):
    if H <= 63: # cm
        return 25.0 / max(H, 25.0)
    else:
        return 0

# Find vertical distance V
def findV(ground, jHand_z0):
    return np.abs(jHand_z0 - ground) * 100
    
# Compute vertical multiplier factor VM
def getVM(V):
    if V <= 175: # cm
        return 1 - (0.003 * np.abs(max(0, min(175, V)) - 75))
    else:
        return 0
    
#  Find vertical traveling distance D
def findD(jHand_z0, jHand_zt):
    return np.abs(jHand_zt - jHand_z0) * 100

# Compute vertical traveling multiplier factor DM
def getDM(D):
    return 0.82 + 4.5 / min(max(25.0, D), 175)

# Compute frequency multiplier factor FM
def getFM(F, FREQUENCIES, V, CUTOFFS, FM_TABLE, duration):
    ind = np.argmin(np.abs(F - FREQUENCIES))
    multiplier = duration_multiplier(V, duration, ind, CUTOFFS)
    return FM_TABLE[duration][ind] * multiplier

# Compute asymmetry angle AM
def getAM(A):
    if A <= 135:
            return 1 - 0.0032 * A
    else:
        return 0
    
# Compute coupling multiplier factor CM
def getCM(V, coupling, COUPLINGS):
    if V >= 30:
        return COUPLINGS[coupling][1]
    else:
        return COUPLINGS[coupling][0]

# Compute RWL
def getRWL(HM, VM, DM, AM, CM, FM, LC):
    return LC * HM * VM * DM * AM * CM * FM

# Compute LI
def getLI(weightLoad, RWL):
    if RWL == 0:
        return 10
    else:
        return weightLoad / RWL
    
def duration_multiplier(V, duration, fmIndex, CUTOFFS):
    if V >= 30: # cm
        return 1
    else:
        return fmIndex <= CUTOFFS[duration]
    
    
def plotNioshKPIs(RWLs, LIs, Hs, Vs, Ds, time_steps):
    """
    Plot NIOSH KPIs (RWL, LI, H, V, D) over time.

    Parameters:
    - RWLs: List of Recommended Weight Limits
    - LIs: List of Lifting Indexes
    - Hs: Horizontal distances
    - Vs: Vertical distances
    - Ds: Vertical traveling distances
    - time_steps: Time steps corresponding to the data points
    """
    plt.figure(figsize=(12, 8))
    
    # Plot each KPI
    plt.plot(time_steps, RWLs, label="RWL (Recommended Weight Limit)", color="blue", linestyle="-", linewidth=2)
    plt.plot(time_steps, LIs, label="LI (Lifting Index)", color="orange", linestyle="-", linewidth=2)
    plt.plot(time_steps, Hs, label="H (Horizontal Distance)", color="green", linestyle="-", linewidth=2)
    plt.plot(time_steps, Vs, label="V (Vertical Distance)", color="red", linestyle="-", linewidth=2)
    plt.plot(time_steps, Ds, label="D (Vertical Traveling Distance)", color="purple", linestyle="-", linewidth=2)
    
    # Add labels, legend, and grid
    plt.xlabel("Time Steps", fontsize=14)
    plt.ylabel("Value", fontsize=14)
    plt.title("NIOSH KPIs Over Time", fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.tight_layout()
    plt.show()
    