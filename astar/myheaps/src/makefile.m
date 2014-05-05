clc, clear, close all;
mex('-outdir','../toolbox','pq_create.cpp');
mex('-outdir','../toolbox','pq_delete.cpp');
mex('-outdir','../toolbox','pq_pop.cpp');
mex('-outdir','../toolbox','pq_push.cpp');
mex('-outdir','../toolbox','pq_size.cpp');
mex('-outdir','../toolbox','pq_top.cpp');
disp('compile done');