import pandas as pd;
import numppy as np;

 rawgn_rx = pd.read_csv('all_ricean.csv',sep=';'); rawgn_rx = pd.read_csv('all_ricean.csv',sep=';');
a = rawgn_rx.groupby(['encoding','snr'])
b = a.agg({'received':np.mean}).reset_index();
b.to_csv('rawgn_qpskhalf.csv');

