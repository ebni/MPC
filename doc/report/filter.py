from decimal import Decimal
import sys
import numpy as np
import simplejson as json

def matlab(line):
    if "MATLAB:" in line:
        matlab_msg.append(executions['glp_simplex'])        
        return False
    else:
        return True

if len(sys.argv) < 2:
    print("filter.py should be launched as: filter.py < data-file >  < csv | json >(optional) < output-file-rename >(optional)")
    sys.exit(1)

data_file = sys.argv[1]
tempi, executions, min, max, std_dev, avg, tempi2 = {}, {}, {}, {}, {}, {}, {}
graphs = {}
i, start, end, var, time = 0, 0, 0, 0, 0
stati, matlab_msg, it_cnts = [], [], []
with open(data_file) as f1:
    Lines = f1.readlines()    
    for line in Lines:
        line = line.strip()
        if matlab(line):
            var = Decimal(line[(line.index(']') + 1) : line.index(':')])
            nome_funzione = line[(line.index('@') + 1) : line.index('#')]
            
            if i % 2 == 0:
                start = var
            else:
                end = var

                nome_funzione = line[(line.index('@') + 1) : line.index('#')]
                #print(line)

                if nome_funzione == "glp_simplex" and "- end" in line:

                    stato_str = line[line.index('{') : line.index('}')].split('\t')
                    it_cnts.append(int(stato_str[27]))
                    stato_str = stato_str[1:13]
                    stato = []
                    for st in stato_str:
                        stato.append(Decimal(st))
                    stati.append(stato)
                
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

            i += 1

    for key in tempi:
        avg[key] = tempi[key] / executions[key] 

    for key in tempi:
        std_dev[key] = np.std(tempi2[key]) 


do = True 
if len(sys.argv) > 2:
    if len(sys.argv) > 3:
        file = sys.argv[3] + "." + sys.argv[2]
    else: 
        file = "output" + "." + sys.argv[2]
    
    if sys.argv[2] == "csv" or sys.argv[2] == "json":
        with open(file, 'a') as f2:
            if sys.argv[2] == "csv":
                for key, value in tempi.items():
                    print('{0!s},{1!s},{2!s},{3!s},{4!s},{5!s},{6!s}'.format(key, value, executions[key], min[key], max[key], avg[key], std_dev[key]), file=f2)

            elif sys.argv[2] == "json":
                
                plot_data = {
                             "tempi" : tempi, 
                             "executions" : executions, 
                             "min" : min, 
                             "max" : max, 
                             "std_dev" : std_dev, 
                             "avg" : avg, 
                             "tempi2" : tempi2,
                             "stati" : stati, 
                             "matlab_msg" : matlab_msg,
                             "it_cnts" : it_cnts
                            }
                json_str = json.dumps(plot_data, use_decimal=True)
                print(json_str, file=f2)
        do = False              
if do:
    for key, value in tempi.items():
        print("func:", key, "\ntot exec time:", value, "\ncalls:", executions[key], "\nmin:", min[key], "\nmax:", max[key], "\navg:", avg[key], "\nstd dev:", std_dev[key],"\n\n\n")


