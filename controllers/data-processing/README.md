## DATA PROCESSING

All Simulink controllers in the repo save data with the following convention:

- data are saved in `structure with time` format;

- data contains the tag `DATA` in their names.

Based on these rules, the script `processDataFromSimulink.m` automatically loads from workspace (or from a previously saved `.mat` file) all the data, and plots them according to the user-defined specifications.
