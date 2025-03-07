import numpy as np
import matplotlib.pyplot as plt

def computeLiftingEnergyConsumption(KPI, m):
    
    LEC = {'Ek': np.zeros(KPI['len']), 'Ep': np.zeros(KPI['len']), 'Em': np.zeros(KPI['len'])}
    
    for i in range(KPI['len']):
        # Kinetic energy of the COM
        Ekx = 0.5 * m * KPI['COM']['velocity'][0,i]**2
        Eky = 0.5 * m * KPI['COM']['velocity'][1,i]**2
        Ekz = 0.5 * m * KPI['COM']['velocity'][2,i]**2
        
        Ek = Ekx + Eky + Ekz
        LEC['Ek'][i] = Ek
        
        # Potential energy of the COM
        Ep = m * 9.81 * KPI['COM']['position'][2,i]
        LEC['Ep'][i] = Ep
        
        # Mechanical energy of the COM
        Em = Ek + Ep
        LEC['Em'][i] = Em
        
    LEC['LECk'] = np.max(LEC['Ek']) - np.min(LEC['Ek'])
    LEC['LECp'] = np.max(LEC['Ep']) - np.min(LEC['Ep'])
    LEC['LECm'] = np.max(LEC['Em']) - np.min(LEC['Em'])
    
    plotLEC(LEC)
        
    return LEC
    
        
        
def plotLEC(LEC):
    
    # fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    
    # axes[0].plot(LEC['Ek'], label='Kinetic Energy')
    # axes[0].set_title('Kinetic Energy of the COM')
    # axes[0].set_xlabel('Time')
    # axes[0].set_ylabel('Energy')
    # axes[0].legend()
    
    # axes[1].plot(LEC['Ep'], label='Potential Energy')
    # axes[1].set_title('Potential Energy of the COM')
    # axes[1].set_xlabel('Time')
    # axes[1].set_ylabel('Energy')
    # axes[1].legend()
    
    # axes[2].plot(LEC['Em'], label='Mechanical Energy')
    # axes[2].set_title('Mechanical Energy of the COM')
    # axes[2].set_xlabel('Time')
    # axes[2].set_ylabel('Energy')
    # axes[2].legend()
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True)

    energies = ['Ek', 'Ep', 'Em']
    titles = ['Kinetic Energy', 'Potential Energy', 'Mechanical Energy']

    for idx, energy in enumerate(energies):
        axs[idx].plot(LEC[energy], label=f'{titles[idx]}', color='blue')
        axs[idx].axhline(y=np.max(LEC[energy]), color='red', linestyle='--', label='Max')
        axs[idx].axhline(y=np.min(LEC[energy]), color='green', linestyle='--', label='Min')
        axs[idx].set_title(titles[idx])
        axs[idx].set_ylabel('Energy (J)')
        axs[idx].legend()
        axs[idx].grid()

    axs[-1].set_xlabel('Index')
    plt.tight_layout()
    plt.show()
    
    
    return 0