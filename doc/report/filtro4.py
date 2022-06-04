from decimal import Decimal
import sys
from matplotlib import pyplot as plt
import numpy as np

def plot_results():
    """ 
        Plot results calculated of: execution time with standard deviation, 
        maximum execution time, minimum execution time, average execution time,
        and total executions
    """
    fig = plt.figure()
    gs = fig.add_gridspec(len(tempi2), 1, hspace=0.6, wspace=0)
    axs = gs.subplots(sharex=True, sharey=False)
    i = 0
    for key in tempi2:
        average = []
        minimum = []
        maximum = []
        exec_num = []
        for j in range(0, executions[key]):
            #create y(value)
            average.append(avg[key])
            minimum.append(min[key])
            maximum.append(max[key])
            #enumerate executions
            exec_num.append(j)
            
        fig.suptitle('Statistics:')
        axs[i].plot(tempi2[key], label='tempi', color='r')
        axs[i].plot(average, color='b', label='media')
        axs[i].plot(maximum, color='m', label='max')
        axs[i].plot(minimum, color='g', label='min')   
        axs[i].errorbar(exec_num, tempi2[key], yerr=std_dev[key], xerr=0, linestyle='None', color='k')
        axs[i].set_title(key) 
        axs[i].legend()
        i+=1

    plt.show()


"""
extrapolates statistics: 
    from: 
        file argv[1]
    
    output options:
        print -> print to argv[3]
        printh -> print to argv[3] with header included
        plot -> plot graph
        all -> do every output redirection option

    to:
        file argv[3]
        graph
        stdout (default)
"""
if len(sys.argv) < 2:
    print("filtro3.py should be launched as: filtro4.py <data-file> <(optional) output-redirect-option> <(optional) output-file>")
    sys.exit(1)

data_file = sys.argv[1]
tempi = {}
executions = {}
min = {}
max = {}
std_dev = {}
avg = {}
i = 0
start = 0
end = 0
var = 0
time = 0
tempi2 = {}

with open(data_file) as f1:
    Lines = f1.readlines()    
    #print(Lines)
    for line in Lines:
        #print(line)
        line = line.strip()
        var = Decimal(line[(line.index(']') + 1) : line.index(':')])
        nome_funzione = line[(line.index('@') + 1) : line.index('#')]
        if i % 2 == 0:
            start = var
        else:
            end = var
            nome_funzione = line[(line.index('@') + 1) : line.index('#')]
            if nome_funzione not in tempi:
                tempi[nome_funzione] = 0

            if nome_funzione not in tempi2:
                tempi2[nome_funzione] = []

            if nome_funzione not in min:
                min[nome_funzione] = float('inf')

            if nome_funzione not in max:
                max[nome_funzione] = 0
            
            if nome_funzione not in avg:
                avg[nome_funzione] = 0

            if nome_funzione not in std_dev:
                std_dev[nome_funzione] = 0

            time = (end - start)
            tempi[nome_funzione] = tempi[nome_funzione] + time 
            tempi2[nome_funzione].append(time)
            if nome_funzione not in executions:
                executions[nome_funzione] = 1
            else:
                executions[nome_funzione] = executions[nome_funzione] + 1
            
            if time > max[nome_funzione]:
                max[nome_funzione] = time

            if time < min[nome_funzione]:
                min[nome_funzione] = time

        i = i + 1

    for key in tempi:
        avg[key] = tempi[key] / executions[key] 

    for key in tempi:
        std_dev[key] = np.std(tempi2[key]) 

do = True
if len(sys.argv) > 2 and (sys.argv[2] == "print" or sys.argv[2] == "printh" or sys.argv[2] == "all"):
    do = False
    if len(sys.argv) > 3:
        file = sys.argv[3]
    else:
        file = "output.csv"
    with open(file, 'a') as f2:
        if sys.argv[2] == "printh" or sys.argv[2] == "all":
            print("func_name,tot_exec_time,tot_exec,min_exec_time,max_exec_time,average,std_dev", file = f2)
        for key, value in tempi.items():
            print(str(key) + "," + str(value) + "," + str(executions[key]) + "," + str(min[key]) + "," +
                str(max[key]) + "," + str(avg[key]) + "," + str(std_dev[key]), file = f2)
if len(sys.argv) > 2 and (sys.argv[2] == "plot" or sys.argv[2] == "all"):
    do = False
    plot_results()

if do:
    for key, value in tempi.items():
        print(key + "," + value + "," + executions[key] + "," + min[key] + "," +
                max[key] + "," + avg[key] + "," + std_dev[key])