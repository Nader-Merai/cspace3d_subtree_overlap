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
for i in range(3):
    costs.append([])
    times.append([])
    with open(sys.argv[i+1], 'r') as f:
        for line in f:
            if line.startswith('total expands this call ='):
                cur_time = float(line.split(' ')[9])
                cur_time = cur_time if cur_time < 35 else 35
                times[i] += [cur_time]
            if line.startswith('post_process solution cost: '):
                costs[i] += [float(line.split(' ')[3])]


costs.append([])
times.append([])
with open(sys.argv[4], 'r') as f:
    for line in f:
        if line.startswith('Time: '):
            cur_time = float(line.split(' ')[1])
            cur_time = cur_time if cur_time < 35 else 35
            times[3] += [cur_time]
        if line.startswith('solution cost: '):
            costs[3] += [float(line.split(' ')[2])]
costs[0] += 18*[140000]
error_times_mean = []
error_costs_mean = []
error_times_std = []
error_costs_std = []
for i in range(4):
    error_times_mean.append(statistics.mean(times[i]))
    error_costs_mean.append(statistics.mean(costs[i]))
    error_times_std.append(np.std(times[i]) / np.sqrt(len(times[i])))
    error_costs_std.append(np.std(costs[i]) / np.sqrt(len(costs[i])))
fig1, ax1 = plt.subplots()
ax1.set_title('Planning Time for 3D Experiment')
x_labels = ["WA*", "Penalty" , "HashSubtree", "RRT"]
ax1.set_ylabel('Planning Time (seconds)')
plt.errorbar(x = x_labels, y = error_times_mean, yerr = error_times_std, marker='o', markersize=10, linewidth=3)
plt.grid(linestyle='--')
plt.gcf().set_size_inches([6, 5])
# plt.show()
plt.savefig("3d_time_errbar.pdf", format='pdf', bbox_inches='tight')
plt.savefig("3d_time_errbar.png", bbox_inches='tight')
# plt.clf()
fig1, ax1 = plt.subplots()
ax1.set_title('Solution Cost for 3D Experiment')
x_labels = ["WA*", "Penalty" , "HashSubtree", "RRT"]
ax1.set_ylabel('Solution Cost ($\\times 10^4$)')
plt.errorbar(x = x_labels, y = [y/10000 for y in error_costs_mean], yerr = [y/10000 for y in error_costs_std], marker='o', markersize=10, linewidth=3)
plt.grid(linestyle='--')
plt.gcf().set_size_inches([6, 5])
# plt.show()
plt.savefig("3d_solution_cost_errbar.pdf", format='pdf', bbox_inches='tight')
plt.savefig("3d_solution_cost_errbar.png", bbox_inches='tight')
