//da modificare
questa cartella contiene le sottocartelle conteneti le analisi fatte con 
gpfrof delle esecuzioni del progetto con parametri diversi durante ogni 
esecuzione
parametri json di iniziali len_horizon=20 len_ctrl=5
t1 ha come parametri len_horizon=1 len_ctrl=5
t2 ha come parametri len_horizon=5 len_ctrl=5
t3 ha come parametri len_horizon=10 len_ctrl=5
t4 ha come parametri len_horizon=30 len_ctrl=5
t5 ha come parametri len_horizon=10 len_ctrl=10
t6 ha come parametri len_horizon=45 len_ctrl=5
t7 ha come parametri len_horizon=50 len_ctrl=10 -> iteration: 2622, time: 26.32

con len_ctrl che ha definisce il numero di input del problema MPC
con len_horizon che è il numero di passi che che il sistema calcola preventivamente

considerazini
* t1 e t2 falliscono perché il drone si muove nel verso opposto dell'asse x.
* t3 la simulazione funziona ma la traiettoria risulta meno lineare a quella prefista.
* t4 la simulazione sembra apparire più fluida.
* t5 la simulazione si comporta come t3
