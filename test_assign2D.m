clc
clear
close all

C=[7,   51,  52,  87,  38,  60,  74,  66,   0,   20;
   50,  12,   0,  64,   8,  53,   0,  46,  76,  42;
   27,  77,   0,  18,  22,  48,  44,  13,   0,  57;
   62,   0,   3,   8,   5,   6,  14,   0,  26,  39;
    0,  97,   0,   5,  13,   0,  41,  31,  62,  48;
   79,  68,   0,   0,  15,  12,  17,  47,  35,  43;
   76,  99,  48,  27,  34,   0,   0,   0,  28,   0;
    0,  20,   9,  27,  46,  15,  84,  19,   3,  24;
   56,  10,  45,  39,   0,  93,  67,  79,  19,  38;
   27,   0,  39,  53,  46,  24,  69,  46,  23,   1];
[col4row, row4col, gain]=assign2D(C);
N=size(C,1);%It is a square matrix.
tuples=zeros(2,N);
for curRow=1:N
    tuples(1,curRow)=curRow;
    tuples(2,curRow)=col4row(curRow);
end

gain1=0;
gain2=0;
gain3=0;
for k=1:N
    gain1=gain1+C(k,col4row(k));
    gain2=gain2+C(row4col(k),k);
    gain3=gain3+C(tuples(1,k),tuples(2,k));
end
[gain,gain1,gain2,gain3]
tuples