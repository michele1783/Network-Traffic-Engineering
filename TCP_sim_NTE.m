clear

Lbyte = 1000;  % byte
%Lbyte = input("How many byte in a packet? ")
L = 8*Lbyte;   %packet in bits

C = 1e4;  % bit/s bottleneck link capacity 
%C = input("What is in kbit/s the bottleneck link capacity? ")
R = 100*C;  % line rate [bit/s]
%R = input("What is in kbit/s the line rate? ")
T = 200;   % ms
Tf = T/2;   % si assume lo stesso valore per ritardo di base forward e backward
BDP = C*T/L;  % pacchetti = W*

% TCP parameters
IW = 1;
ssthresh = 128;
rwnd = Inf;

% cross traffic at the bottleneck
rhoc = 0.8;
%rhoc = input("What is the parameter rho for the cross traffic? ")
lambdac = C*rhoc/L;
Q = L*rhoc/(2*(1-rhoc));
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
  ac = L*poissrnd(lambdac*deltat);
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

%numt = input("How many RTT? ");   % questo e' il numero di RTT che sono simulati
numt = 50;
rwnd_v=rwnd;
wprec_v=[];
cwnd_v=[];
thrpt=[];
rtt=av(1);
for t=2:numt
  %wprec = wv(t-1)
  wprec = min(floor(rwnd), wv(t-1));
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

    wtx = min(floor(rwnd),floor(cwnd));
    while wtx > flightsize
      jj = jj+1;
      sndv(jj) = sndv(jj-1)+L/R;
      flightsize = flightsize+1;
    end
  end

  wv(t) = wtx;
  for ii=1:wtx
    if ii==1
      deltat = sndv(1)-tnext;
    else
      deltat = sndv(ii)-sndv(ii-1);
    end
    ac = L*poissrnd(lambdac*deltat);
    Q = max(0,Q+L+ac-C*deltat);
    queuev(ii) = Q;
    rcvv(ii) = sndv(ii)+Q/C+L/C+Tf;
    ackv(ii) = rcvv(ii)+Tf;
  end
  
  %stima c_bottleneck
  stima_c_bottleneck = 1/min(diff(rcvv(1:wtx)));

  %stima RTT
  if rcvv(1) - vv(offset) > 2 * min(diff(rcvv(1:wtx)))
    stima_RTT = rcvv(wtx) - vv(offset);
  end
  
  %throughput
  rtt_thrpt=wtx*L/stima_RTT;     
  
  %rwnd
  rwnd = stima_c_bottleneck * stima_RTT;
  rwnd_v=[rwnd_v rwnd];

  %wtx
  wprec_v=[wprec_v wprec];

  %cwnd
  cwnd_v=[cwnd_v cwnd];

  %th
  thrpt=[thrpt rtt_thrpt];

  %rtt
  rtt=[rtt stima_RTT];

  uv(offset+1:offset+wtx) = sndv(1:wtx);
  vv(offset+1:offset+wtx) = rcvv(1:wtx);
  av(offset+1:offset+wtx) = ackv(1:wtx);
  qv(offset+1:offset+wtx) = queuev(1:wtx);
  offset = offset+wtx;
  tnext = sndv(wtx); 

end

npacc = length(uv);
conn_time=vv(end)-uv(1);     % TCP connection lifetime [ms]
avg_thrpt=npacc*L/conn_time; % average throughput of the connection [bits/s]
% snd_time_axis=linspace(uv(1),uv(end),length(uv));
% q_time_axis=snd_time_axis+T/4;
% rec_time_axis=linspace(vv(1),vv(end),length(vv));
IATv = diff(vv);
nIAT = npacc-1;

%% TIME
figure()
stem(uv,'ob', ':');
hold on
stem(vv,'*r', ':');
hold off
xlabel('pkts','FontSize',20);
ylabel('Time [ms]','FontSize',20);
legend('Departures','Arrivals','Location','best');
title('Time Evolution', 'FontSize', 20);
str = {'Bottleneck Capacity = 1e3'};
text(10,3800,str);
str = {'\rho cross traffic = 0.8'};
text(10,3500,str);
axis tight;
%% QUEUE

%
figure()
q_time=uv+T/4;
q_time=linspace(q_time(1),q_time(end),length(qv));
stairs(q_time,qv/L,'LineWidth',1.3);
xlabel('Time [ms]', 'FontSize',15);
ylabel('Queue length [pkts]','FontSize',15);
title('Queue at the bottleneck','FontSize',18);
str = {'Bottleneck Capacity = 1e3 bps'};
text(2500,16,str);
str = {'xtraffic \rho = 0.8'};
text(2500,12,str);
%
%figure()
%stem(q_time,qv/L,'LineWidth', 2, 'Color', '#77AC30');
%xlabel('pkts','FontSize',20);
%ylabel('Queue Bottleneck [pkts]', 'FontSize',20);
%title('Queue in Bottleneck', 'FontSize', 20);
%axis tight;
%str = {'Bottleneck Capacity = 1e4'};
%text(50,120,str);
%str = {'\rho cross traffic = 0.3'};
%text(50,110,str);
% figure()
% plot(uv/T, qv/L,'LineWidth', 2, 'Color', '#77AC30');
% xlabel('Time in base RTT','FontSize',20);
% ylabel('Queue Bottleneck [pkts]', 'FontSize',20);
% title('Queue in Bottleneck', 'FontSize', 20);
% axis tight;
% str = {'Bottleneck Capacity = 1e3'};
% text(1000,70,str);
% str = {'\rho cross traffic = 0.3'};
% text(1000,66,str);
%% INTERARRIVALS
figure()
stem(IATv,'filled', 'LineWidth', 2, 'Color', '#7E2F8E');
xlabel('pkts', 'FontSize',20);
ylabel('InterArrivals [ms]', 'FontSize',20);
title('Interarrivals','FontSize',20);
axis tight;
str = {'Bottleneck Capacity = 1e3'};
text(40,200,str);
str = {'\rho cross traffic = 0.8'};
text(40,180,str);
%% WINDOWS

figure()
stem(rwnd_v,'og','LineWidth', 2, 'Color', '#0072BD');
xlabel('Time in base RTT','FontSize',20)
ylabel('Rwnd','FontSize',20);
title('Receiver Window','FontSize',20);
axis tight;
str = {'Bottleneck Capacity = 1e3'};
text(8,24,str);
str = {'\rho cross traffic = 0.8'};
text(8,20,str);


figure()
stem(wprec_v,'LineWidth', 2, 'Color', '#D95319');
ylabel('Rx window', 'FontSize',20);
xlabel('Time in base RTT', 'FontSize',20);
axis tight;
title('Trasmission Window', 'FontSize',20);
str = {'Bottleneck Capacity = 1e3'};
text(2,35,str);
str = {'\rho cross traffic = 0.8'};
text(2,30,str);
%
figure()
stem(cwnd_v,'LineWidth', 2, 'Color', '#EDB120');
ylabel('Cwnd','FontSize',20);
xlabel('Time in base RTT','FontSize',20);
title('Congestion Window','FontSize',20);
str = {'Bottleneck Capacity = 1e3'};
text(1,120,str);
str = {'\rho cross traffic = 0.8'};
text(1,110,str);


%% THROUGHPUT
% figure()
% plot(snd_time_axis,qv/L);
% title('Queue vs time');
x=1:1:length(thrpt);
figure()
stem(x,thrpt/L,'LineWidth',1.3);
hold on
plot(x,(avg_thrpt/L)*ones(1,length(thrpt)),'LineWidth',1.3);
hold off
xlabel("RTT",'FontSize',15);
ylabel('Throughput [kbyte/ms]','FontSize',15);
title('Connection Throughput','FontSize',18);
legend('TH','Avg TH','Location','best');
axis tight;
str = {'Bottleneck Capacity = 1e3 bps'};
text(2,0.16,str);
str = {'xtraffic \rho = 0.8'};
text(2,0.15,str);
%% ROUND TRIP TIME
figure()
stem(rtt,'LineWidth',1.3);
xlabel('RTT #','FontSize',15);
ylabel('RTT value [ms]','FontSize',15);
title('Round Trip Time','FontSize',18);
str = {'Bottleneck Capacity = 1e3 bps'};
text(2,300,str);
str = {'xtraffic \rho = 0.8'};
text(2,250,str);

