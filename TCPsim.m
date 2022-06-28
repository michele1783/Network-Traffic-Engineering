clear

Lbyte = 1000;  % byte
L = 8*Lbyte;
C = 10000;  % kbit/s bottleneck link capacity 
R = 100*C;  % line rate
T = 100;   % ms
Tf = T/2;   % si assume lo stesso valore per ritardo di base forward e backward
BDP = C*T/L;  % pacchetti

% TCP parameters
IW = 1;
ssthresh = 64;
rwnd = Inf;

% cross traffic at the bottleneck
rhoc = 0.5;
lambdac = C*rhoc/L;
Q = L*rhoc/2/(1-rhoc);
% questo e' il valore iniziale assegnato alla lunghezza di coda nel buffer del bottleneck


% Quello che segue e' il primo RTT, invio di IW pacchetti a velocita line rate della
% sorgente
% sndv e' vettore che contiene i sending times
% rcvv e' il vettore che contiene i tempi di arrivo al ricevitore
% ackv e' il vettore contiene i tempi di ricezione dell'ACK al sender
t = 1;
cwnd = IW;
wtx = min(rwnd,floor(cwnd));
wv(1) = wtx;
sndv(1) = 0;
rcvv(1) = L/C+Tf;
ackv(1) = L/C+T;
queuev(1) = Q;
offset = 0;
for kk=2:wtx
  sndv(kk) = (kk-1)*L/R;
  deltat = sndv(kk)-sndv(kk-1);
  ac = L*Poissonrand(lambdac*deltat);
  % la coda viene calcolata in tempini di bit. Il ritardo e' Q/C assumendo
  % politica FCFS.
  Q = max(0,Q+L+ac-C*deltat);
  queuev(kk) = Q;
  rcvv(kk) = sndv(kk)+Q/C+L/C+Tf;
  ackv(kk) = rcvv(kk)+Tf;
end
% i vettori uv,vv, av,qv, conterrano l'intera storia dei tempi di partenza,
% i tempi di arrivo al receiver, i tempi di ricezione dell'ack e le
% lunghezze di coda viste dai pacchetti della connessione
uv(offset+1:offset+wtx) = sndv(1:wtx);
vv(offset+1:offset+wtx) = rcvv(1:wtx);
av(offset+1:offset+wtx) = ackv(1:wtx);
qv(offset+1:offset+wtx) = queuev(1:wtx);
offset = offset+wtx;
tnext = sndv(wtx);  % questo e' il tempo di partenza dell'ultimo pacchetto della finestra

numt = 16;   % questo e' il numero di RTT che sono simulati
for t=2:numt
  wprec = wv(t-1);
  flightsize = wprec;  % si assume connessione greedy, sempre pacchetto pronto da inviare
  % attenzione agli indici:
  % jj scandisce i nuovi vettori di stato che vengono costruiti in questo
  % RTT
  % kk scandisce i valori di arrivo degli ack calcolati nel PRECEDENTE RTT
  jj = 0;
  for kk=1:wprec
    jj = jj+1;
    if jj==1
      sndv(jj) = max(tnext,ackv(kk))+L/R;
    else
      sndv(jj) = max(sndv(jj-1),ackv(kk))+L/R;
    end
    if cwnd < ssthresh
      cwnd = cwnd+1;
    else
      cwnd = cwnd+1/cwnd;
    end
    wtx = min(rwnd,floor(cwnd));
    while wtx > flightsize
      jj = jj+1;
      sndv(jj) = sndv(jj-1)+L/R;
      flightsize = flightsize+1;
    end
  end
  assert(jj==wtx,'Hai toppato!')
  wv(t) = wtx;
  for ii=1:wtx
    if ii==1
      deltat = sndv(1)-tnext;
    else
      deltat = sndv(ii)-sndv(ii-1);
    end
    ac = L*Poissonrand(lambdac*deltat);
    Q = max(0,Q+L+ac-C*deltat);
    queuev(ii) = Q;
    rcvv(ii) = sndv(ii)+Q/C+L/C+Tf;
    ackv(ii) = rcvv(ii)+Tf;
  end
  uv(offset+1:offset+wtx) = sndv(1:wtx);
  vv(offset+1:offset+wtx) = rcvv(1:wtx);
  av(offset+1:offset+wtx) = ackv(1:wtx);
  qv(offset+1:offset+wtx) = queuev(1:wtx);
  offset = offset+wtx;
  tnext = sndv(wtx); 
end

npacc = length(uv);
IATv = diff(vv);
nIAT = npacc-1;


clf
stem(IATv,'LineWidth',1.5)
ax = gca; % current axes
ax.FontSize = 24;
% xinf = 0;
% xsup = 20*ceil(max(xv)/20);
% yinf = 0;
% ysup = 1;
% axis([xinf xsup yinf ysup])
% grid on
xlabel('Samples')
ylabel('IAT (ms)')
% text(xinf+0.9*(xsup-xinf),yinf+0.5*(ysup-yinf),strcat('L = ',num2str(Lpackmed),' bytes'),'FontSize',24,'HorizontalAlignment','right');
% text(xinf+0.9*(xsup-xinf),yinf+0.3*(ysup-yinf),strcat('E[AoI] = ',num2str(EAoIopt*boslot,3),' ms'),'FontSize',24,'HorizontalAlignment','right');
% text(xinf+0.9*(xsup-xinf),yinf+0.1*(ysup-yinf),strcat('1/f = ',num2str(Tsendv(jAoI),3),' ms'),'FontSize',24,'HorizontalAlignment','right');
% nomefile=char(strcat('Poissonmodel_PC_CBR_vs_AoI_L',num2str(Lpackmed),'.eps'));
% saveas(gcf,nomefile,'eps2c')


pause

clf
plot(uv/T,qv/L,'b-','LineWidth',1.5)
ax = gca; % current axes
ax.FontSize = 24;
% xinf = 0;
% xsup = 20*ceil(max(xv)/20);
% yinf = 0;
% ysup = 1;
% axis([xinf xsup yinf ysup])
% grid on
xlabel('Time in base RTT')
ylabel('pkts')
% text(xinf+0.9*(xsup-xinf),yinf+0.5*(ysup-yinf),strcat('L = ',num2str(Lpackmed),' bytes'),'FontSize',24,'HorizontalAlignment','right');
% text(xinf+0.9*(xsup-xinf),yinf+0.3*(ysup-yinf),strcat('E[AoI] = ',num2str(EAoIopt*boslot,3),' ms'),'FontSize',24,'HorizontalAlignment','right');
% text(xinf+0.9*(xsup-xinf),yinf+0.1*(ysup-yinf),strcat('1/f = ',num2str(Tsendv(jAoI),3),' ms'),'FontSize',24,'HorizontalAlignment','right');
% nomefile=char(strcat('Poissonmodel_PC_CBR_vs_AoI_L',num2str(Lpackmed),'.eps'));
% saveas(gcf,nomefile,'eps2c')







