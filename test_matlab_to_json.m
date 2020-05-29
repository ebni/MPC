addpath jsonlab/jsonlab
addpath jsonlab

filename='json/MatlabExample.json'; % warning: folder json must exist! 
s = create_model();
write_json_file(filename,s);






