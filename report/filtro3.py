from decimal import Decimal
import sys

if len(sys.argv) < 2:
    print("filtro3.py should be launched as: filtro3.py <data-file> <(optional) output-file>")
    sys.exit(1)

data_file = sys.argv[1]
tempi = {}
executions = {}
i = 0
start = 0
end = 0
var = 0
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

            tempi[nome_funzione] = tempi[nome_funzione] + (end - start)
            if nome_funzione not in executions:
                executions[nome_funzione] = 1
            else:
                executions[nome_funzione] = executions[nome_funzione] + 1

        i = i + 1


if len(sys.argv) > 2:
    file = sys.argv[2]
    with open(file, 'a') as f2:
        for key, value in tempi.items():
            print(key, ' : ', value, ' --> ', executions[key], file = f2)

        print('___________________________________________', file = f2) 
else:
    for key, value in tempi.items():
        print(key, ' : ', value, ' --> ', executions[key])


