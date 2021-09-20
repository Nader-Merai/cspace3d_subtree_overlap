import sys
import matplotlib.pyplot as plt
import numpy as np
import statistics
import matplotlib

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
# if you have latex you may use the commented line
# matplotlib.rcParams['text.usetex'] = True
plt.rcParams.update({'font.size': 22})
matplotlib.rcParams.update({'errorbar.capsize': 2})

costs = []
times = []
expansions = []
for i in range(5):
    costs.append([])
    times.append([])
    with open(sys.argv[i+1], 'r') as f:
        for line in f:
            if line.startswith('total expands this call ='):
                times[i] += [float(line.split(' ')[9])]

error_times_mean = []
error_times_std = []
for i in range(5):
    error_times_mean.append(statistics.mean(times[i]))
    error_times_std.append(np.std(times[i]) / np.sqrt(len(times[i])))
fig1, ax1 = plt.subplots()
ax1.set_title('Varying $r$ for 3D Experiment')
x_labels = ["0.05", "0.075" , "0.1", "0.125", "0.15"]
ax1.set_ylabel('Planning Time (seconds)')
ax1.set_xlabel('Radius of overlap $r$')
plt.errorbar(x = x_labels, y = error_times_mean, yerr = error_times_std, marker='o', markersize=8, linewidth=2)
plt.grid(linestyle='--')
plt.gcf().set_size_inches([7, 6])
# plt.show()
plt.savefig("radius_time_errbar.pdf", format='pdf', bbox_inches='tight')
plt.savefig("radius_time_errbar.png", bbox_inches='tight')
