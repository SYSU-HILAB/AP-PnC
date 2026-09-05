import sys
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "figure.figsize": (29.7/2.54, 21.0/2.54),
    "xtick.labelsize": 15,
    "ytick.labelsize": 15,
    "font.sans-serif": ["Helvetica"]})

def plot(filename, ax, style):
    data = np.genfromtxt(filename)
    x = data[:,0]
    
    # Plot deviations for the lowest three eigenvalues.
    for i in [1,2,3]:
        ax.plot(x, data[:, i], style)


if len(sys.argv) < 3:
    print("No folder and/or output-file specified.")
    sys.exit(1)
    
folder = sys.argv[1]
output_file = sys.argv[2]

double_eps = 2.2204460492503131e-16
quad_eps = 1.925929944387235853055977942584927319e-34

# Share both X and Y axes with all subplots
fig, axs = plt.subplots(2, 2, sharex='all', sharey='all', gridspec_kw=dict(width_ratios=[1,1], height_ratios=[1,1]))

axs[0,0].set_ylim(1.0e-40, 1.0)
axs[0,0].set_yscale('log')
axs[0,0].set_ylabel(r'$\left|\frac{\Delta E}{E}\right|$', fontsize=25)
axs[1,0].set_ylabel(r'$\left|\frac{\Delta E}{E}\right|$', fontsize=25)


axs[1,0].set_xlim(18, 2100)
axs[1,0].set_xscale('log')
axs[1,0].set_xlabel(r'$n$', fontsize=25)
axs[1,1].set_xlabel(r'$n$', fontsize=25)

axs[0,0].set_title('Spline Order: 2', fontsize=25)
axs[0,1].set_title('Spline Order: 5', fontsize=25)
axs[1,0].set_title('Spline Order: 10', fontsize=25)
axs[1,1].set_title('Spline Order: 15', fontsize=25)

for i, ax in enumerate(fig.axes):
    ax.grid(True)
    ax.axhline(y=double_eps, color='r', linestyle='-')
    ax.axhline(y=quad_eps, color='b', linestyle='-')
    
plot(folder + '/double_2.txt', axs[0,0], 'ro:')
plot(folder + '/double_5.txt', axs[0,1], 'ro:')
plot(folder + '/double_10.txt', axs[1,0], 'ro:')
plot(folder + '/double_15.txt', axs[1,1], 'ro:')

plot(folder + '/quad_2.txt', axs[0,0], 'b^:')
plot(folder + '/quad_5.txt', axs[0,1], 'b^:')
plot(folder + '/quad_10.txt', axs[1,0], 'b^:')
plot(folder + '/quad_15.txt', axs[1,1], 'b^:')



plt.tight_layout()
print("Saving to " + output_file)
plt.savefig(output_file, dpi=300.0)
#plt.show()
