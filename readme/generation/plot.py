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

if len(sys.argv) < 3:
    print("No input and/or output-file specified.")
    sys.exit(1)
    
input_file = sys.argv[1]
output_file = sys.argv[2]
red_splines = 0

if len(sys.argv) == 4:
    red_splines = int(sys.argv[3])

double_eps = 2.2204460492503131e-16
quad_eps = 1.925929944387235853055977942584927319e-34

fig = plt.figure()

data = np.genfromtxt(input_file)

(x,y) = data.shape

for i in range(1, y):
    if i <= red_splines:
       plt.plot(data[:,0], data[:, i], 'r-')
    else:
       plt.plot(data[:,0], data[:, i], 'b-')

plt.ylabel('$s(x)$', fontsize=25)
plt.xlabel('$x$', fontsize=25)
plt.grid(True)
plt.tight_layout()
print("Saving to " + output_file)
plt.savefig(output_file, dpi=300.0)
#plt.show()
