% Script: Generate_HDL.m
% Description: Automated MBD workflow for Pipelined CORDIC generation


cfg = coder.config('hdl');
cfg.TargetLanguage = 'Verilog';
cfg.SynthesisTool = 'Xilinx ISE';

cfg.DistributedPipelining = true; 
cfg.OutputPipeline = 13;         



ARGS = {fi(0, 1, 16, 13)};

codegen -config cfg CORDIC_HDL_Core -args ARGS
