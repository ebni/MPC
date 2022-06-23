from decimal import Decimal
import simplejson as json
import matplotlib.pyplot as plt
import numpy as np
import statistics as stat
import sys

def check_json(json_dict):
    #spezzare condizione
    if "tempi" in json_dict and "executions" in json_dict and "min" in json_dict and "max" in json_dict and "std_dev" in json_dict and "avg" in json_dict and "tempi2" in json_dict and "stati" in json_dict and "matlab_msg" in json_dict and "it_cnts" in json_dict:
        return True
    else: 
        return False

def euclidean_norm(state):
    if len(state)!=12:
        return "Error(euclidean_norm): Wrong number of state elements"
    coef=[1,1,1,1,1,1,1,1,1,1,1,1]
    
    for i in range (0,11):
        state[i]=state[i]*coef[i]
    
    return np.linalg.norm(state)

def on_pick(event):
    # On the pick event, find the original line corresponding to the legend
    # proxy line, and toggle its visibility.
    legline = event.artist
    origline = ld[legline]
    visible = not origline.get_visible()
    origline.set_visible(visible)
    # Change the alpha on the line in the legend so we can see what lines
    # have been toggled.
    legline.set_alpha(1.0 if visible else 0.2)
    fig.canvas.draw()



def plot_graph(plt_dt):
    gs = fig.add_gridspec(3, 2, hspace=0.6, wspace=0.3)
    axs = gs.subplots(sharex='col', sharey=False)
    key = 'glp_simplex'
    fig.suptitle('Statistics: ' + key)
    execs = plt_dt['executions'][key]
    time = plt_dt['tempi2'][key]
    exec_num = [j for j in range(0, execs)]
    it_cnts = plt_dt['it_cnts']
    states = plt_dt['stati']
    e_norm = [euclidean_norm(states[k]) for k in range(0, len(states))]
                
    for i in range(0,6):        
        row = i % 3
        col = 0 if i < 3 else 1
        match i:
            case 0:
                xlabel = "indice iterazione"
                ylabel = "it_cnt"
                minimum = np.full(shape=execs, fill_value=min(it_cnts), dtype=int)
                maximum = np.full(shape=execs, fill_value=max(it_cnts), dtype=int)
                average = np.full(shape=execs, fill_value=stat.mean(it_cnts), dtype=Decimal)
                std_dev = np.full(shape=execs, fill_value=np.std(it_cnts), dtype=Decimal)
                l0, = axs[row][col].plot(exec_num, it_cnts, label='data', color='r', marker='*')
                l1, = axs[row][col].plot(exec_num, average, label='average', color='b', linestyle='--')
                l2, = axs[row][col].plot(exec_num, minimum, label='min', color='m', linestyle='--')
                l3, = axs[row][col].plot(exec_num, maximum, label='max', color='g', linestyle='--')
                axs[row][col].errorbar(exec_num, it_cnts, yerr=std_dev, xerr=0, linestyle='None', color='k', label='std')
                ls = [l0, l1, l2, l3]
                
                axs[row][col].set_title('0')
                #probably need to add matlab separators
            case 1:
                xlabel = "indice iterazione"
                ylabel = "tempo impiegato"
                minimum = np.full(shape=execs, fill_value=min(time), dtype=Decimal)
                maximum = np.full(shape=execs, fill_value=max(time), dtype=Decimal)
                average = np.full(shape=execs, fill_value=stat.mean(time), dtype=Decimal)
                std_dev = np.full(shape=execs, fill_value=np.std(time), dtype=Decimal)
                l0, = axs[row][col].plot(exec_num, time, label='data', color='r', marker='*')
                l1, = axs[row][col].plot(exec_num, average, label='average', color='b', linestyle='--')
                l2, = axs[row][col].plot(exec_num, minimum, label='min', color='m', linestyle='--')
                l3, = axs[row][col].plot(exec_num, maximum, label='max', color='g', linestyle='--')
                axs[row][col].errorbar(exec_num, time, yerr=std_dev, xerr=0, linestyle='None', color='k', label='std')
                ls = [l0, l1, l2, l3]
                
                axs[row][col].set_title('1')
                #probably need to add matlab separators        
            case 2:
                xlabel = "indice iterazione"
                ylabel = "norma euclidea stato"
                minimum = np.full(shape=execs, fill_value=min(e_norm), dtype=Decimal)
                maximum = np.full(shape=execs, fill_value=max(e_norm), dtype=Decimal)
                average = np.full(shape=execs, fill_value=stat.mean(e_norm), dtype=Decimal)
                std_dev = np.full(shape=execs, fill_value=np.std(e_norm), dtype=Decimal)
                l0, = axs[row][col].plot(exec_num, e_norm, label='data', color='r', marker='*')
                l1, = axs[row][col].plot(exec_num, average, label='average', color='b', linestyle='--')
                l2, = axs[row][col].plot(exec_num, minimum, label='min', color='m', linestyle='--')
                l3, = axs[row][col].plot(exec_num, maximum, label='max', color='g', linestyle='--')
                axs[row][col].errorbar(exec_num, e_norm, yerr=std_dev, xerr=0, linestyle='None', color='k', label='std')
                ls = [l0, l1, l2, l3]
                
                axs[row][col].set_title('2')
                #probably need to add matlab separators          
            case 3:
                xlabel = "norma euclidea stato"
                ylabel = "tempo impiegato"
                minimum = np.full(shape=execs, fill_value=min(time), dtype=Decimal)
                maximum = np.full(shape=execs, fill_value=max(time), dtype=Decimal)
                average = np.full(shape=execs, fill_value=stat.mean(time), dtype=Decimal)
                std_dev = np.full(shape=execs, fill_value=np.std(time), dtype=Decimal)
                l0, = axs[row][col].plot(e_norm, time, label='data', color='r', marker='*')
                l1, = axs[row][col].plot(e_norm, average, label='average', color='b', linestyle='--')
                l2, = axs[row][col].plot(e_norm, minimum, label='min', color='m', linestyle='--')
                l3, = axs[row][col].plot(e_norm, maximum, label='max', color='g', linestyle='--')
                axs[row][col].errorbar(e_norm, time, yerr=std_dev, xerr=0, linestyle='None', color='k', label='std')
                axs[row][col].set_title('3')
                ls = [l0, l1, l2, l3]
                
                #probably need to add matlab separators        
            case 4:
                xlabel = "norma euclidea stato"
                ylabel = "it_cnt"
                minimum = np.full(shape=execs, fill_value=min(it_cnts), dtype=int)
                maximum = np.full(shape=execs, fill_value=max(it_cnts), dtype=int)
                average = np.full(shape=execs, fill_value=stat.mean(it_cnts), dtype=Decimal)
                std_dev = np.full(shape=execs, fill_value=np.std(it_cnts), dtype=Decimal)
                l0, = axs[row][col].plot(e_norm, it_cnts, label='data', color='r', marker='*')
                l1, = axs[row][col].plot(e_norm, average, label='average', color='b', linestyle='--')
                l2, = axs[row][col].plot(e_norm, minimum, label='min', color='m', linestyle='--')
                l3, = axs[row][col].plot(e_norm, maximum, label='max', color='g', linestyle='--')
                axs[row][col].errorbar(e_norm, it_cnts, yerr=std_dev, xerr=0, linestyle='None', color='k', label='std')
                axs[row][col].set_title('4')
                ls = [l0, l1, l2, l3]
                
                #probably need to add matlab separators
            case 5:
                xlabel = "norma euclidea stato"
                ylabel = "indice iterazione"
                l0, = axs[row][col].plot(e_norm, exec_num, label='data', color='r', marker='*')
                axs[row][col].set_title('5')
                ls = [l0]        
            case _:
                axs[row][col].axis('off')       
        legend = axs[row][col].legend()
        for legline, origline in zip(legend.get_lines(), ls):
            legline.set_picker(True)  # Enable picking on the legend line.
            legline.set_pickradius(10)
            ld[legline] = origline 
        axs[row][col].set_xlabel(xlabel)
        axs[row][col].set_ylabel(ylabel)
        

    
    
    
    fig.canvas.mpl_connect('pick_event', on_pick)
    plt.show()

if len(sys.argv) < 2:
    print("plot.py should be launched as: plot.py <data-file.json>")
    sys.exit(1)   

file = sys.argv[1]
with open (file, 'r') as js:
    plot_data = json.load(js, use_decimal=True)
    if check_json(plot_data):
        ld = {}
        fig = plt.figure()
        plot_graph(plot_data)
    else:
        print('Error(): Invalid input json file content')
        sys.exit(1)           
