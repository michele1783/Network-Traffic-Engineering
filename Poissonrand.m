function [N] = Poissonrand(a)
% a is the mean of the Poisson r.v.
k = 0;
S = -log(rand);
while S <= a
  k = k+1;
  S = S-log(rand);
end
N = k;


rwnd_v = floor(rwnd_v);
a = [IW, cwnd_v];
aaa = floor(a);
b = []; 
for i=1:length(aaa)
    b = [b, min(aaa(i), rwnd_v(i))];
end
b